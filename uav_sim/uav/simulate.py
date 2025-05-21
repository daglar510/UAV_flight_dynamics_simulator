"""
Fixed-wing UAV simulation module.
Contains classes for simulating fixed-wing UAV dynamics.
"""

import numpy as np
from scipy.integrate import solve_ivp
from uav_sim.utils.common import rad2deg

def simulate_response(A, B, U0, pulses, duration):
    """
    Simulate the response of a fixed-wing UAV to elevator pulses.
    
    Args:
        A (np.ndarray): State matrix
        B (np.ndarray): Input matrix
        U0 (float): Trim speed
        pulses (list): List of tuples (start_time, end_time, elevator_angle_degrees)
        duration (float): Simulation duration in seconds
        
    Returns:
        tuple: (t, y, de) - time, state, and elevator deflection arrays
    """
    t_eval = np.linspace(0, duration, 1000)
    
    def delta_e(t):
        for t0, t1, deg in pulses:
            if t >= t0 and t <= t1:
                return deg * np.pi/180
        return 0
    
    def delta_T(t):
        return 0
    
    def f(t, y):
        δ = np.array([delta_T(t), delta_e(t)])
        return A @ y + B @ δ
    
    y0 = np.zeros(4)
    sol = solve_ivp(f, [0, duration], y0, t_eval=t_eval, rtol=1e-8, atol=1e-8)
    de = np.array([delta_e(tt) for tt in sol.t])
    
    return sol.t, sol.y, de

class FixedWingSimulation:
    """
    Class to manage fixed-wing UAV simulation.
    """
    
    def __init__(self, uav_params, A, B, U0):
        """
        Initialize the simulation.
        
        Args:
            uav_params (dict): UAV parameters
            A (np.ndarray): State matrix
            B (np.ndarray): Input matrix
            U0 (float): Trim speed
        """
        self.uav_params = uav_params
        self.A = A
        self.B = B
        self.U0 = U0
        self.pulses = []
        self.duration = 0
        self.results = None
    
    def add_elevator_pulse(self, start_time, duration, angle_deg):
        """
        Add an elevator pulse to the simulation.
        
        Args:
            start_time (float): Start time in seconds
            duration (float): Pulse duration in seconds
            angle_deg (float): Elevator deflection angle in degrees
        """
        self.pulses.append((start_time, start_time + duration, angle_deg))
    
    def set_simulation_duration(self, duration):
        """
        Set the simulation duration.
        
        Args:
            duration (float): Simulation duration in seconds
        """
        self.duration = duration
    
    def run(self):
        """
        Run the simulation.
        
        Returns:
            tuple: (t, y, de) - time, state, and elevator deflection arrays
        """
        if not self.pulses:
            raise ValueError("No elevator pulses defined")
        
        if self.duration <= 0:
            raise ValueError("Invalid simulation duration")
        
        t, y, de = simulate_response(self.A, self.B, self.U0, self.pulses, self.duration)
        self.results = {
            'time': t,
            'state': y,
            'elevator': de,
            'uav_name': self.uav_params.get('company', 'Unknown UAV')
        }
        
        return t, y, de
    
    def get_results(self):
        """
        Get the simulation results.
        
        Returns:
            dict: Simulation results
            
        Raises:
            RuntimeError: If the simulation has not been run
        """
        if self.results is None:
            raise RuntimeError("Simulation has not been run")
        
        return self.results 