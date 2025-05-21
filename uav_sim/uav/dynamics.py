"""
Fixed-wing UAV dynamics - state-space modeling and analysis.
"""

import numpy as np
import math
from uav_sim.constants import GRAVITY, AIR_DENSITY, GAS_CONSTANT, GAMMA

def build_state_space(uav):
    """
    Build the state-space model for a fixed-wing UAV.
    
    Args:
        uav (dict): UAV parameters
        
    Returns:
        tuple: (A, B, U0) - state matrix, input matrix, and trim speed
    """
    m, S, c, Iyy, Mach = (uav[k] for k in ("mass", "S", "c", "Iyy", "Mach"))
    a = math.sqrt(GAMMA * GAS_CONSTANT * 288.15)  # speed of sound
    U0 = Mach * a
    rho = AIR_DENSITY

    m1 = m / (0.5 * rho * U0 * S)
    c1 = c / (2 * U0)
    Jy1 = Iyy / (0.5 * rho * U0**2 * S * c)

    CL0, CLa, CLq, CLde, CLu = (uav[k] for k in ("CL_0", "CL_alpha", "CL_q", "CL_deltae", "CL_u"))
    CD0, CDa, CDq, CDde, CDu = (uav[k] for k in ("CD_0", "CD_alpha", "CD_q", "CD_deltae", "CD_u"))
    Cm0, Cma, Cmq, Cmde, Cmu = (uav[k] for k in ("Cm_0", "Cm_alpha", "Cm_q", "Cm_deltae", "Cm_u"))

    CXu = -2*CD0 - CDu
    CXa = -CDa + CL0
    CXq = -CDq
    CZu = -2*CL0 - CLu
    CZa = -CLa - CD0
    CZq = -CLq
    Cmu = 2*Cm0 + Cmu
    Cma = Cma + Cm0
    Cmq = Cmq
    CXde = -CDde
    CZde = -CLde
    Cmde = Cmde

    M = np.array([
        [m1, 0, 0, 0],
        [0, m1, 0, 0],
        [0, 0, Jy1, 0],
        [0, 0, 0, 1],
    ])
    
    K = np.array([
        [-CXu, -CXa, -CXq, m1*(GRAVITY/U0)],
        [-CZu, -CZa, -c1*CZq - m1, 0],
        [-Cmu, -Cma, -c1*Cmq, 0],
        [0, 0, -1, 0],
    ])
    
    B = np.array([
        [0, CXde],
        [0, CZde],
        [0, Cmde],
        [0, 0],
    ])
    
    A = np.linalg.solve(-M, K)
    Bt = np.linalg.solve(M, B)
    
    return A, Bt, U0

def analyze_modes(A):
    """
    Analyze the flight dynamics modes based on eigenvalues.
    
    Args:
        A (np.ndarray): State matrix
        
    Returns:
        list: List of mode information dictionaries
    """
    eigvals, _ = np.linalg.eig(A)
    modes = []
    
    for i, l in enumerate(eigvals):
        wn = np.abs(l)
        sigma = l.real
        wd = l.imag
        zeta = -sigma / wn if wn > 0 else 0
        
        mode_info = {
            'mode_num': i+1,
            'eigenvalue': l,
            'natural_frequency': wn,
            'damping_ratio': zeta
        }
        
        if zeta < 0:
            mode_info['stability'] = "UNSTABLE"
            mode_info['description'] = "UNSTABLE (Danger: positive real part!)"
        elif zeta < 0.2:
            mode_info['stability'] = "POORLY_DAMPED"
            mode_info['description'] = "Poorly damped (will oscillate a lot)"
        elif zeta < 0.7:
            mode_info['stability'] = "GOOD"
            mode_info['description'] = "Good aircraft mode"
        else:
            mode_info['stability'] = "HIGHLY_DAMPED"
            mode_info['description'] = "Highly damped (probably not oscillatory)"
            
        modes.append(mode_info)
        
    return modes

def print_modes_analysis(modes):
    """
    Print the flight dynamics modes analysis.
    
    Args:
        modes (list): List of mode information dictionaries
    """
    print("\n=== Flight Dynamics Modes (Eigenvalues of A) ===")
    for mode in modes:
        l = mode['eigenvalue']
        wn = mode['natural_frequency']
        zeta = mode['damping_ratio']
        print(f"Mode {mode['mode_num']}: lambda = {l:.4f} | wn = {wn:.4f} rad/s | zeta = {zeta:.3f}")
        print(f"  --> {mode['description']}") 