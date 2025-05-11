# UAV Longitudinal Flight Dynamics Simulator
# Adapted from Mattia Di Mauro's original simulator (2025)
# Updated by Dağlar Duman for Turkish and foreign UAVs
# Includes full English comments and 'assumed' indicators

import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.integrate import solve_ivp

# --- Constants ---
g = 9.80665  # gravitational acceleration [m/s^2]
ft_to_m = 0.3048
lbs_to_kg = 0.45359237
slug_ft2_to_kgm2 = 1.35581795

# --- UAV database with parameters (some assumed) ---
def get_uav_parameters(uav_name):
    """
    Returns geometric, inertial, and aerodynamic parameters for selected UAV.
    Also includes company and country of origin for documentation purposes.
    """
    UAV_DB = {
        "TB2": {
            "company": "Baykar", "country": "Turkey",
            "mass": 700, "S": 9.34, "c": 0.78, "b": 12.0, "Iyy": 2500, "Mach": 0.12,
            # Aerodynamic coefficients – assumed
            "CL_0": 0.30, "CL_alpha": 5.5, "CL_q": 5.0, "CL_deltae": 0.4, "CL_u": 0.0,
            "CD_0": 0.05, "CD_alpha": 0.3, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0
        },
        "Anka": {
            "company": "TUSAŞ", "country": "Turkey",
            "mass": 1700, "S": 18.0, "c": 1.05, "b": 17.5, "Iyy": 6000, "Mach": 0.18,
            # All coefficients – assumed
            "CL_0": 0.35, "CL_alpha": 5.7, "CL_q": 5.5, "CL_deltae": 0.5, "CL_u": 0.0,
            "CD_0": 0.06, "CD_alpha": 0.32, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.8, "Cm_q": -6.0, "Cm_deltae": -1.2, "Cm_u": 0.0
        },
        "Aksungur": {
            "company": "TUSAŞ", "country": "Turkey",
            "mass": 3300, "S": 30.0, "c": 1.25, "b": 24.2, "Iyy": 12000, "Mach": 0.21,
            # All values – assumed
            "CL_0": 0.25, "CL_alpha": 5.2, "CL_q": 6.0, "CL_deltae": 0.6, "CL_u": 0.0,
            "CD_0": 0.07, "CD_alpha": 0.35, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.7, "Cm_q": -7.0, "Cm_deltae": -1.5, "Cm_u": 0.0
        },
        "Karayel": {
            "company": "Vestel", "country": "Turkey",
            "mass": 630, "S": 9.0, "c": 0.75, "b": 13.0, "Iyy": 2000, "Mach": 0.11,
            # All values – assumed
            "CL_0": 0.30, "CL_alpha": 5.5, "CL_q": 4.5, "CL_deltae": 0.4, "CL_u": 0.0,
            "CD_0": 0.05, "CD_alpha": 0.3, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.5, "Cm_q": -3.5, "Cm_deltae": -0.8, "Cm_u": 0.0
        },
        "Predator": {
            "company": "General Atomics", "country": "USA",
            "mass": 1020, "S": 11.45, "c": 0.78, "b": 14.8, "Iyy": 5000, "Mach": 0.14,
            # All values – assumed
            "CL_0": 0.25, "CL_alpha": 5.6, "CL_q": 4.0, "CL_deltae": 0.3, "CL_u": 0.0,
            "CD_0": 0.05, "CD_alpha": 0.3, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.5, "Cm_q": -4.0, "Cm_deltae": -0.5, "Cm_u": 0.0
        },
        "Heron": {
            "company": "IAI", "country": "Israel",
            "mass": 1150, "S": 12.9, "c": 0.78, "b": 16.6, "Iyy": 4000, "Mach": 0.18,
            # All values – assumed
            "CL_0": 0.40, "CL_alpha": 5.7, "CL_q": 5.5, "CL_deltae": 0.5, "CL_u": 0.0,
            "CD_0": 0.06, "CD_alpha": 0.33, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0
        }
    }
    return UAV_DB[uav_name]

# --- Build A, B matrices based on UAV parameters ---
def build_state_space(uav):
    """Builds the state-space matrices (A and B) for the selected UAV"""
    mass, S, c, Iyy, Mach = uav["mass"], uav["S"], uav["c"], uav["Iyy"], uav["Mach"]
    rho = 1.225  # air density at sea level [kg/m^3]
    a = math.sqrt(1.4 * 287.05 * 288.15)  # speed of sound at sea level
    U_0 = Mach * a  # trim airspeed

    m_1 = mass / (0.5 * rho * U_0 * S)
    c_1 = c / (2 * U_0)
    Jy_1 = Iyy / (0.5 * rho * U_0**2 * S * c)

    # Aerodynamic force/moment derivatives (longitudinal)
    CX_alpha = -uav["CD_alpha"] + uav["CL_0"]
    CX_u = -2 * uav["CD_0"] - uav["CD_u"]
    CX_q = -uav["CD_q"]

    CZ_alpha = -uav["CL_alpha"] - uav["CD_0"]
    CZ_u = -2 * uav["CL_0"] - uav["CL_u"]
    CZ_q = -uav["CL_q"]

    CM_alpha = uav["Cm_alpha"] + uav["Cm_0"]
    CM_u = 2 * uav["Cm_0"] + uav["Cm_u"]
    CM_q = uav["Cm_q"]

    CX_deltae = -uav["CD_deltae"]
    CZ_deltae = -uav["CL_deltae"]
    CM_deltae = uav["Cm_deltae"]

    # Mass/inertia matrix
    M = np.array([
        [m_1, 0, 0, 0],
        [0, m_1, 0, 0],
        [0, 0, Jy_1, 0],
        [0, 0, 0, 1],
    ])

    # System matrix (A) and input matrix (B)
    K = np.array([
        [-CX_u, -CX_alpha, -CX_q, m_1 * (g / U_0)],
        [-CZ_u, -CZ_alpha, -c_1 * CZ_q - m_1, 0],
        [-CM_u, -CM_alpha, -c_1 * CM_q, 0],
        [0, 0, -1, 0]
    ])

    Delta = np.array([
        [0, CX_deltae],
        [0, CZ_deltae],
        [0, CM_deltae],
        [0, 0]
    ])

    A = np.linalg.solve(-M, K)
    B = np.linalg.solve(M, Delta)
    return A, B, U_0

# --- Simulation of the time response ---
def simulate_response(A, B, U_0, pulses, duration):
    t_eval = np.linspace(0, duration, 1000)

    def elevator_input(t):
        for start, end, angle in pulses:
            if start <= t < end:
                return angle * np.pi / 180  # convert to radians
        return 0

    def thrust_input(t):
        return 0  # constant thrust (assumed constant trim condition)

    def state_derivative(t, y):
        delta = np.array([thrust_input(t), elevator_input(t)])
        return A @ y + B @ delta

    y0 = np.zeros(4)  # initial conditions: [u, alpha, q, theta]
    sol = solve_ivp(state_derivative, [0, duration], y0, t_eval=t_eval, method='RK45')
    return sol.t, sol.y, np.array([elevator_input(t) for t in sol.t])

# --- Plotting function ---
def plot_response(t, y, delta, uav_name):
    plt.figure(figsize=(15, 10))

    plt.subplot(3, 2, 1)
    plt.plot(t, y[0])
    plt.title(f'{uav_name}: Forward Speed u [m/s]')
    plt.grid(True)

    plt.subplot(3, 2, 2)
    plt.plot(t, np.degrees(y[1]))
    plt.title('Angle of Attack α [deg]')
    plt.grid(True)

    plt.subplot(3, 2, 3)
    plt.plot(t, np.degrees(y[2]))
    plt.title('Pitch Rate q [deg/s]')
    plt.grid(True)

    plt.subplot(3, 2, 4)
    plt.plot(t, np.degrees(y[3]))
    plt.title('Pitch Angle θ [deg]')
    plt.grid(True)

    plt.subplot(3, 2, 5)
    plt.plot(t, np.degrees(delta))
    plt.title('Elevator Input [deg]')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# --- Main Execution ---
if __name__ == "__main__":
    uav_choice = input("Select UAV (e.g., TB2, Anka, Aksungur, Karayel, Predator, Heron): ")
    duration = int(input("Simulation duration (seconds): "))
    num_inputs = int(input("Number of elevator inputs: "))

    pulses = []
    for i in range(num_inputs):
        start = float(input(f"Input {i+1} start time (s): "))
        duration_pulse = float(input(f"Input {i+1} duration (s): "))
        angle = float(input(f"Input {i+1} angle (deg): "))
        pulses.append((start, start + duration_pulse, angle))

    uav = get_uav_parameters(uav_choice)
    A, B, U_0 = build_state_space(uav)
    t, y, delta = simulate_response(A, B, U_0, pulses, duration)
    plot_response(t, y, delta, uav_choice)
