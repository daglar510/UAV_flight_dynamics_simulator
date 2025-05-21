"""
Quadcopter simulation implementation.
"""

import numpy as np
import time
from uav_sim.quadcopter.dynamics import QuadcopterDynamics
from uav_sim.quadcopter.controller import QuadcopterController

class QuadcopterSimulation:
    """
    Class to manage quadcopter simulation.
    """
    
    def __init__(self, quad_params, dt=0.01, noise_level=0.0):
        """
        Initialize the quadcopter simulation.
        
        Args:
            quad_params (dict): Quadcopter parameters
            dt (float): Time step (s)
            noise_level (float): Level of noise to add to simulation (0.0 to 1.0)
        """
        self.dt = dt
        self.noise_level = noise_level
        
        # Create dynamics and controller objects
        self.dynamics = QuadcopterDynamics(quad_params)
        self.dynamics.dt = dt
        self.controller = QuadcopterController(quad_params, dt)
        
        # Simulation data
        self.time = 0.0
        self.history = {
            'time': [],
            'position': [],
            'velocity': [],
            'attitude': [],
            'angular_velocity': [],
            'motor_speeds': [],
            'setpoints': []
        }
        
        # Current setpoint
        self.setpoint = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    
    def reset(self):
        """
        Reset the simulation.
        """
        self.time = 0.0
        self.dynamics.reset_state()
        self.controller.reset()
        
        # Clear history
        for key in self.history:
            self.history[key] = []
        
        # Reset setpoint
        self.setpoint = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    
    def set_initial_state(self, position=None, velocity=None, attitude=None, angular_velocity=None):
        """
        Set the initial state of the quadcopter.
        
        Args:
            position (np.ndarray, optional): 3D position [x, y, z] (m)
            velocity (np.ndarray, optional): 3D velocity [vx, vy, vz] (m/s)
            attitude (np.ndarray, optional): 3D attitude [roll, pitch, yaw] (rad)
            angular_velocity (np.ndarray, optional): 3D angular velocity [p, q, r] (rad/s)
        """
        self.dynamics.set_state(position, velocity, attitude, angular_velocity)
    
    def set_setpoint(self, x=None, y=None, z=None, yaw=None):
        """
        Set the setpoint for the controller.
        
        Args:
            x (float, optional): Desired x position (m)
            y (float, optional): Desired y position (m)
            z (float, optional): Desired z position (m)
            yaw (float, optional): Desired yaw angle (rad)
        """
        if x is not None:
            self.setpoint['x'] = x
        if y is not None:
            self.setpoint['y'] = y
        if z is not None:
            self.setpoint['z'] = z
        if yaw is not None:
            self.setpoint['yaw'] = yaw
    
    def update(self):
        """
        Update the simulation for one time step.
        
        Returns:
            dict: Current state of the quadcopter
        """
        # Get current state
        state = self.dynamics.get_state()
        
        # Add noise to measurements (if enabled)
        if self.noise_level > 0:
            noisy_state = self._add_noise(state)
        else:
            noisy_state = state
        
        # Compute control commands
        motor_commands = self.controller.compute_control(noisy_state, self.setpoint, self.dt)
        
        # Apply motor commands to the dynamics
        self.dynamics.set_motor_speeds(motor_commands)
        
        # Update dynamics
        next_state = self.dynamics.update(self.dt)
        
        # Update time
        self.time += self.dt
        
        # Record history
        self.history['time'].append(self.time)
        self.history['position'].append(state['position'].copy())
        self.history['velocity'].append(state['velocity'].copy())
        self.history['attitude'].append(state['attitude'].copy())
        self.history['angular_velocity'].append(state['angular_velocity'].copy())
        self.history['motor_speeds'].append(state['motor_speeds'].copy())
        self.history['setpoints'].append(self.setpoint.copy())
        
        return next_state
    
    def _add_noise(self, state):
        """
        Add noise to the state measurements.
        
        Args:
            state (dict): Current state of the quadcopter
            
        Returns:
            dict: State with added noise
        """
        noisy_state = state.copy()
        
        # Position noise: few cm
        pos_noise_std = 0.03 * self.noise_level
        noisy_state['position'] = state['position'] + np.random.normal(0, pos_noise_std, 3)
        
        # Velocity noise: few cm/s
        vel_noise_std = 0.05 * self.noise_level
        noisy_state['velocity'] = state['velocity'] + np.random.normal(0, vel_noise_std, 3)
        
        # Attitude noise: small angle (few degrees)
        att_noise_std = 0.01 * self.noise_level
        noisy_state['attitude'] = state['attitude'] + np.random.normal(0, att_noise_std, 3)
        
        # Angular velocity noise: few deg/s
        angvel_noise_std = 0.02 * self.noise_level
        noisy_state['angular_velocity'] = state['angular_velocity'] + np.random.normal(0, angvel_noise_std, 3)
        
        return noisy_state
    
    def run(self, duration, realtime=False):
        """
        Run the simulation for a specified duration.
        
        Args:
            duration (float): Simulation duration (s)
            realtime (bool): Whether to run in real-time
            
        Returns:
            dict: Simulation history
        """
        num_steps = int(duration / self.dt)
        
        for _ in range(num_steps):
            start_time = time.time()
            
            # Update simulation
            self.update()
            
            # If running in real-time, wait until dt has elapsed
            if realtime:
                elapsed = time.time() - start_time
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)
        
        return self.get_history()
    
    def get_history(self):
        """
        Get the simulation history.
        
        Returns:
            dict: Simulation history
        """
        # Convert lists to numpy arrays for easier analysis
        history_array = {}
        for key, value in self.history.items():
            if key == 'setpoints':
                # Handle dict of setpoints
                setpoint_arr = np.zeros((len(value), 4))
                for i, sp in enumerate(value):
                    setpoint_arr[i, 0] = sp.get('x', 0)
                    setpoint_arr[i, 1] = sp.get('y', 0)
                    setpoint_arr[i, 2] = sp.get('z', 0)
                    setpoint_arr[i, 3] = sp.get('yaw', 0)
                history_array[key] = setpoint_arr
            else:
                history_array[key] = np.array(value)
        
        return history_array

class TrajectoryGenerator:
    """
    Generate various trajectories for the quadcopter to follow.
    """
    
    @staticmethod
    def hover(duration, height=1.0, dt=0.01):
        """
        Generate a hover trajectory.
        
        Args:
            duration (float): Duration of the trajectory (s)
            height (float): Hover height (m)
            dt (float): Time step (s)
            
        Returns:
            dict: Trajectory setpoints
        """
        num_points = int(duration / dt)
        trajectory = {
            'time': np.linspace(0, duration, num_points),
            'x': np.zeros(num_points),
            'y': np.zeros(num_points),
            'z': np.ones(num_points) * height,
            'yaw': np.zeros(num_points)
        }
        return trajectory
    
    @staticmethod
    def square(duration, side_length=2.0, height=1.0, dt=0.01):
        """
        Generate a smooth square trajectory with acceleration and deceleration phases.
        
        Args:
            duration (float): Duration of the trajectory (s)
            side_length (float): Side length of the square (m)
            height (float): Flight height (m)
            dt (float): Time step (s)
            
        Returns:
            dict: Trajectory setpoints
        """
        num_points = int(duration / dt)
        time = np.linspace(0, duration, num_points)
        
        # Time for each side
        side_time = duration / 4
        
        # Time for ramping up/down (20% of each side)
        ramp_time = side_time * 0.2
        
        # Generate x, y coordinates
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        yaw = np.zeros(num_points)
        
        half_side = side_length / 2
        corners = [
            (-half_side, -half_side),  # Bottom-left
            (half_side, -half_side),   # Bottom-right
            (half_side, half_side),    # Top-right
            (-half_side, half_side)    # Top-left
        ]
        
        for i, t in enumerate(time):
            # Determine which side of the square we're on
            side_idx = int((t % duration) / side_time)
            side_progress = (t % side_time) / side_time  # 0 to 1 for current side
            
            # Current and next corner
            current_corner = corners[side_idx]
            next_corner = corners[(side_idx + 1) % 4]
            
            # Calculate interpolation factor with smooth acceleration/deceleration
            if side_progress < 0.2:  # Acceleration phase
                # Smooth start using sine function (0 to π/2 maps to 0 to 1)
                accel_factor = np.sin(side_progress * np.pi / 0.4) ** 2
                factor = 0.2 * accel_factor
            elif side_progress > 0.8:  # Deceleration phase
                # Smooth stop using sine function (0 to π/2 maps to 0 to 1)
                decel_factor = np.sin((1.0 - side_progress) * np.pi / 0.4) ** 2
                factor = 0.8 + 0.2 * (1.0 - decel_factor)
            else:  # Constant speed phase
                # Linear interpolation in the middle
                factor = 0.2 + (side_progress - 0.2) * 0.75  # Adjust for 60% of the path
            
            # Interpolate between corners
            x[i] = current_corner[0] + (next_corner[0] - current_corner[0]) * factor
            y[i] = current_corner[1] + (next_corner[1] - current_corner[1]) * factor
            
            # Set yaw to point in the direction of motion
            if side_idx == 0:  # Moving right
                yaw[i] = 0
            elif side_idx == 1:  # Moving up
                yaw[i] = np.pi/2
            elif side_idx == 2:  # Moving left
                yaw[i] = np.pi
            else:  # Moving down
                yaw[i] = -np.pi/2
        
        trajectory = {
            'time': time,
            'x': x,
            'y': y,
            'z': np.ones(num_points) * height,
            'yaw': yaw
        }
        
        return trajectory
    
    @staticmethod
    def circle(duration, radius=1.0, height=1.0, dt=0.01):
        """
        Generate a circular trajectory.
        
        Args:
            duration (float): Duration of the trajectory (s)
            radius (float): Circle radius (m)
            height (float): Flight height (m)
            dt (float): Time step (s)
            
        Returns:
            dict: Trajectory setpoints
        """
        num_points = int(duration / dt)
        time = np.linspace(0, duration, num_points)
        
        # Angular velocity
        omega = 2 * np.pi / duration
        
        # Generate x, y coordinates
        x = radius * np.cos(omega * time)
        y = radius * np.sin(omega * time)
        yaw = omega * time + np.pi/2  # Point to the center
        
        trajectory = {
            'time': time,
            'x': x,
            'y': y,
            'z': np.ones(num_points) * height,
            'yaw': yaw
        }
        
        return trajectory