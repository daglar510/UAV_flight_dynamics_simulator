# UAV Longitudinal Flight Dynamics Simulator
# Adapted from Mattia Di Mauro (2025) + Dağlar Duman extensions
# Author: Mattia Di Mauro / Dağlar Duman
# Year: 2025

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import solve_ivp
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting

# --- Constants ---
g     = 9.80665      # gravity [m/s²]
rho0  = 1.225        # sea-level air density [kg/m³]
R     = 287.05       # specific gas constant for air [J/(kg·K)]
gamma = 1.4          # ratio of specific heats

# --- UAV database with parameters (all “assumed”) ---
def get_uav_parameters(name):
    UAV_DB = {
        "TB2": {
            "company":"Baykar","country":"Turkey",
            "mass":700,   "S":9.34,  "c":0.78,  "b":12.0,  "Iyy":2500,  "Mach":0.12,
            "CL_0":0.30,  "CL_alpha":5.5,  "CL_q":5.0,   "CL_deltae":0.4, "CL_u":0.0,
            "CD_0":0.05,  "CD_alpha":0.3,  "CD_q":0.0,   "CD_deltae":0.0, "CD_u":0.0,
            "Cm_0":0.0,   "Cm_alpha":-0.6, "Cm_q":-5.0,  "Cm_deltae":-1.0,"Cm_u":0.0
        },
        "Anka": {
            "company":"TUSAŞ","country":"Turkey",
            "mass":1700, "S":18.0,"c":1.05,"b":17.5,"Iyy":6000,"Mach":0.18,
            "CL_0":0.35,"CL_alpha":5.7,"CL_q":5.5,"CL_deltae":0.5,"CL_u":0.0,
            "CD_0":0.06,"CD_alpha":0.32,"CD_q":0.0,"CD_deltae":0.0,"CD_u":0.0,
            "Cm_0":0.0,"Cm_alpha":-0.8,"Cm_q":-6.0,"Cm_deltae":-1.2,"Cm_u":0.0
        },
        "Aksungur": {
            "company":"TUSAŞ","country":"Turkey",
            "mass":3300,"S":30.0,"c":1.25,"b":24.2,"Iyy":12000,"Mach":0.21,
            "CL_0":0.25,"CL_alpha":5.2,"CL_q":6.0,"CL_deltae":0.6,"CL_u":0.0,
            "CD_0":0.07,"CD_alpha":0.35,"CD_q":0.0,"CD_deltae":0.0,"CD_u":0.0,
            "Cm_0":0.0,"Cm_alpha":-0.7,"Cm_q":-7.0,"Cm_deltae":-1.5,"Cm_u":0.0
        },
        "Karayel": {
            "company":"Vestel","country":"Turkey",
            "mass":630,"S":9.0,"c":0.75,"b":13.0,"Iyy":2000,"Mach":0.11,
            "CL_0":0.30,"CL_alpha":5.5,"CL_q":4.5,"CL_deltae":0.4,"CL_u":0.0,
            "CD_0":0.05,"CD_alpha":0.3,"CD_q":0.0,"CD_deltae":0.0,"CD_u":0.0,
            "Cm_0":0.0,"Cm_alpha":-0.5,"Cm_q":-3.5,"Cm_deltae":-0.8,"Cm_u":0.0
        },
        "Predator": {
            "company":"GA-ASI","country":"USA",
            "mass":1020,"S":11.45,"c":0.78,"b":14.8,"Iyy":5000,"Mach":0.14,
            "CL_0":0.25,"CL_alpha":5.6,"CL_q":4.0,"CL_deltae":0.3,"CL_u":0.0,
            "CD_0":0.05,"CD_alpha":0.3,"CD_q":0.0,"CD_deltae":0.0,"CD_u":0.0,
            "Cm_0":0.0,"Cm_alpha":-0.5,"Cm_q":-4.0,"Cm_deltae":-0.5,"Cm_u":0.0
        },
        "Heron": {
            "company":"IAI","country":"Israel",
            "mass":1150,"S":12.9,"c":0.78,"b":16.6,"Iyy":4000,"Mach":0.18,
            "CL_0":0.40,"CL_alpha":5.7,"CL_q":5.5,"CL_deltae":0.5,"CL_u":0.0,
            "CD_0":0.06,"CD_alpha":0.33,"CD_q":0.0,"CD_deltae":0.0,"CD_u":0.0,
            "Cm_0":0.0,"Cm_alpha":-0.6,"Cm_q":-5.0,"Cm_deltae":-1.0,"Cm_u":0.0
        }
    }
    if name not in UAV_DB:
        raise KeyError(f"Unknown UAV: {name}")
    return UAV_DB[name]

# --- Build the A, B, U0 state-space from the chosen UAV’s dict ---
def build_state_space(uav):
    # Unpack geometry/inertia/trim Mach
    m, S, c, Iyy, Mach = (uav[k] for k in ("mass","S","c","Iyy","Mach"))
    # Atmosphere & trim speed
    a  = math.sqrt(gamma * R * 288.15)  # speed of sound [m/s]
    U0 = Mach * a
    rho = rho0

    # Dimensionless scaling
    m1  = m   / (0.5 * rho * U0 * S)
    c1  = c   / (2   * U0)
    Jy1 = Iyy / (0.5 * rho * U0**2 * S * c)

    # Unpack aero derivatives
    CL0, CLa, CLq, CLde, CLu = (uav[k] for k in ("CL_0","CL_alpha","CL_q","CL_deltae","CL_u"))
    CD0, CDa, CDq, CDde, CDu = (uav[k] for k in ("CD_0","CD_alpha","CD_q","CD_deltae","CD_u"))
    Cm0, Cma, Cmq, Cmde, Cmu = (uav[k] for k in ("Cm_0","Cm_alpha","Cm_q","Cm_deltae","Cm_u"))

    # Force/moment coefficients
    CXu  = -2*CD0   - CDu
    CXa  = -CDa     + CL0
    CXq  = -CDq
    CZu  = -2*CL0   - CLu
    CZa  = -CLa     - CD0
    CZq  = -CLq
    Cmu  =  2*Cm0   + Cmu
    Cma  =  Cma     + Cm0
    Cmq  =  Cmq
    CXde = -CDde
    CZde = -CLde
    Cmde =  Cmde

    # M matrix (mass + zero alpha-dot here)
    M = np.array([
        [ m1,    0,     0,  0],
        [  0,   m1,     0,  0],
        [  0,    0,   Jy1,  0],
        [  0,    0,     0,  1],
    ])

    # K matrix (system dynamics)
    K = np.array([
        [-CXu,    -CXa,       -CXq,   m1*(g/U0)],
        [-CZu,    -CZa,  -c1*CZq - m1,   0     ],
        [-Cmu,    -Cma,       -c1*Cmq,    0     ],
        [  0,       0,         -1,       0     ],
    ])

    # B matrix (thrust, elevator)
    B = np.array([
        [   0,   CXde],
        [   0,   CZde],
        [   0,   Cmde],
        [   0,     0 ],
    ])

    # Solve for A = -M⁻¹K, B_total = M⁻¹B
    A  = np.linalg.solve(-M, K)
    Bt = np.linalg.solve( M, B)
    return A, Bt, U0

# --- simulate_response: given A,B,U0 & list of (t0,t1,deg) pulses ---
def simulate_response(A, B, U0, pulses, duration):
    t_eval = np.linspace(0, duration, 1000)

    def delta_e(t):
        for t0,t1,deg in pulses:
            if t >= t0 and t <= t1:
             return deg * np.pi/180
        return 0

    def delta_T(t):
        return 0  # thrust held at trim

    def f(t, y):
        δ = np.array([delta_T(t), delta_e(t)])
        return A @ y + B @ δ

    y0 = np.zeros(4)  # [u, α, q, θ]

    # --- DEBUG PRINTS: inspect trim and first/last states ---
    print(f">>> Trim speed U0 = {U0:.2f} m/s")
    print(">>> A matrix:\n", A)
    print(">>> B matrix:\n", B)

    sol = solve_ivp(f, [0,duration], y0, t_eval=t_eval, rtol=1e-8, atol=1e-8)

    print(">>> First 5 state columns:\n", sol.y[:, :5])
    print(">>> Last 5 state columns:\n", sol.y[:, -5:])

    de = np.array([delta_e(tt) for tt in sol.t])
    return sol.t, sol.y, de

# --- plot_response: standardized 5-panel figure ---
def plot_response(t, y, de, name):
    plt.figure(figsize=(15,10))
    plt.subplot(3,2,1)
    plt.plot(t,    y[0]);    plt.title(f"{name} – forward speed u");    plt.grid()
    plt.subplot(3,2,2)
    plt.plot(t, np.degrees(y[1])); plt.title("AoA α [deg]");         plt.grid()
    plt.subplot(3,2,3)
    plt.plot(t, np.degrees(y[2])); plt.title("Pitch rate q [deg/s]"); plt.grid()
    plt.subplot(3,2,4)
    plt.plot(t, np.degrees(y[3])); plt.title("Pitch θ [deg]");        plt.grid()
    plt.subplot(3,2,5)
    plt.plot(t, np.degrees(de));    plt.title("Elevator δe [deg]");    plt.grid()
    plt.tight_layout()
    plt.show()

# --- plot_3D_trajectory: plot (alpha, q, theta) as a 3D parametric curve ---
def plot_3D_trajectory(y, name):
    alpha = np.degrees(y[1])
    q     = np.degrees(y[2])
    theta = np.degrees(y[3])

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(alpha, q, theta, label='Trajectory', lw=2)
    ax.set_xlabel('AoA α [deg]')
    ax.set_ylabel('Pitch rate q [deg/s]')
    ax.set_zlabel('Pitch θ [deg]')
    ax.set_title(f'{name} – Trajectory in (α, q, θ) space')
    ax.legend()
    plt.show()

# --- Main driver ---
if __name__=="__main__":
    name     = input("Select UAV (TB2, Anka, Aksungur, Karayel, Predator, Heron): ")
    duration = float(input("Simulation duration [s]: "))
    n_p      = int(  input("Number of elevator pulses: ") )

    pulses = []
    for i in range(n_p):
        t0  = float(input(f" Pulse {i+1} start [s]: "))
        dt  = float(input(f" Pulse {i+1} duration [s]: "))
        deg = float(input(f" Pulse {i+1} elevator [deg]: "))
        pulses.append((t0, t0+dt, deg))

    uav = get_uav_parameters(name)
    A, B, U0 = build_state_space(uav)
    t, y, de = simulate_response(A, B, U0, pulses, duration)
    plot_response(t, y, de, name)
    plot_3D_trajectory(y, name)   # <<< 3D plot here!
    plt.show()
    