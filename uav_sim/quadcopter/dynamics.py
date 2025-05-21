"""
Quadcopter dynamics module implementing 6DOF rigid body dynamics.
Based on the quadcopter dynamic model from various open-source references.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from uav_sim.constants import GRAVITY

def safe_cross(a, b):
    """
    Compute cross product with numerical stability checks.
    
    Args:
        a (np.ndarray): First vector
        b (np.ndarray): Second vector
        
    Returns:
        np.ndarray: Cross product of a and b
    """
    result = np.zeros(3)
    try:
        result = np.cross(a, b)
        # Check for NaN or Inf values
        if np.any(np.isnan(result)) or np.any(np.isinf(result)):
            return np.zeros(3)
    except:
        pass
    return result

class QuadcopterDynamics:
    """
    Implements the dynamics of a quadcopter in 6 degrees of freedom.
    
    This model represents the quadcopter as a rigid body with 4 rotors
    in a symmetric cross configuration.
    """
    
    def __init__(self, params):
        """
        Initialize the quadcopter dynamics model.
        
        Args:
            params (dict): Quadcopter parameters
        """
        # Extract parameters
        self.mass = params['mass']
        self.arm_length = params['arm_length']
        self.Ixx = params['Ixx']
        self.Iyy = params['Iyy']
        self.Izz = params['Izz']
        self.thrust_coef = params['thrust_coefficient']
        self.drag_coef = params['drag_coefficient']
        
        # Determine motor limits
        self.max_motor_speed = params.get('max_rpm', 10000) * (2 * np.pi / 60)  # rad/s
        self.min_motor_speed = params.get('min_rpm', 1000) * (2 * np.pi / 60)  # rad/s
        self.motor_time_constant = params.get('motor_time_constant', 0.02)  # s
        
        # Initial state
        self.reset_state()
        
        # Simulation parameters
        self.dt = 0.01  # Default time step for integration (s)
        
        # Motor configuration (position matters for dynamics)
        # Motors are numbered counter-clockwise: 1=front-right, 2=front-left, 3=back-left, 4=back-right
        # For a quadcopter in X configuration: (FR, FL, BL, BR)
        # Motor 1 and 3 rotate in the same direction, Motor 2 and 4 rotate in the opposite direction
        # CCW motor produces CW torque reaction and vice versa
        self.motor_dirs = np.array([1, -1, 1, -1])  # Direction of motor rotation (CW=1, CCW=-1)
        
        # Moment inertia matrix
        self.I = np.array([
            [self.Ixx, 0, 0],
            [0, self.Iyy, 0],
            [0, 0, self.Izz]
        ])
        
        # Inverse of moment inertia matrix
        self.I_inv = np.linalg.inv(self.I)
        
        # Numerical stability parameters
        self.max_velocity = 10.0  # m/s
        self.max_angular_velocity = 10.0  # rad/s
        self.max_acceleration = 30.0  # m/s²
        self.max_angular_acceleration = 30.0  # rad/s²
        self.safety_factor = 0.95  # Apply this factor to maximum limits for safety
    
    def reset_state(self):
        """
        Reset the state of the quadcopter to initial conditions.
        """
        # State vectors - in inertial frame
        self.position = np.zeros(3)       # x, y, z positions (m)
        self.velocity = np.zeros(3)       # velocity in x, y, z (m/s)
        self.attitude = np.zeros(3)       # roll, pitch, yaw angles (rad)
        self.angular_velocity = np.zeros(3)  # angular velocity around x, y, z (rad/s)
        
        # Motor states
        self.motor_speeds = np.ones(4) * self.min_motor_speed  # Current motor speeds (rad/s)
        self.motor_speeds_desired = np.copy(self.motor_speeds)  # Desired motor speeds (rad/s)
        
        # Forces and moments
        self.forces = np.zeros(3)         # Forces in x, y, z (N)
        self.moments = np.zeros(3)        # Moments around x, y, z (N*m)
    
    def set_state(self, position=None, velocity=None, attitude=None, angular_velocity=None):
        """
        Set the state of the quadcopter.
        
        Args:
            position (np.ndarray, optional): 3D position [x, y, z] (m)
            velocity (np.ndarray, optional): 3D velocity [vx, vy, vz] (m/s)
            attitude (np.ndarray, optional): 3D attitude [roll, pitch, yaw] (rad)
            angular_velocity (np.ndarray, optional): 3D angular velocity [p, q, r] (rad/s)
        """
        if position is not None:
            self.position = np.array(position)
        if velocity is not None:
            self.velocity = np.array(velocity)
        if attitude is not None:
            self.attitude = np.array(attitude)
        if angular_velocity is not None:
            self.angular_velocity = np.array(angular_velocity)
    
    def set_motor_speeds(self, motor_speeds):
        """
        Set the desired motor speeds.
        
        Args:
            motor_speeds (np.ndarray): Desired motor speeds (rad/s)
        """
        # Clip motor speeds to valid range
        self.motor_speeds_desired = np.clip(
            motor_speeds, 
            self.min_motor_speed, 
            self.max_motor_speed
        )
    
    def update_motor_speeds(self, dt):
        """
        Update motor speeds using first-order dynamics.
        
        Args:
            dt (float): Time step (s)
        """
        # First-order motor dynamics: dw/dt = (w_desired - w) / tau
        motor_speed_derivative = (self.motor_speeds_desired - self.motor_speeds) / self.motor_time_constant
        self.motor_speeds += motor_speed_derivative * dt
        
        # Ensure motor speeds stay within limits
        self.motor_speeds = np.clip(self.motor_speeds, self.min_motor_speed, self.max_motor_speed)
    
    def rotation_matrix_body_to_inertial(self):
        """
        Calculate the rotation matrix from body to inertial frame.
        Uses ZYX convention (yaw, pitch, roll) for Euler angles.
        
        Returns:
            np.ndarray: 3x3 rotation matrix
        """
        # Limit attitude angles to prevent gimbal lock and numerical issues
        phi = np.clip(self.attitude[0], -np.pi/2 * 0.99, np.pi/2 * 0.99)  # Roll
        theta = np.clip(self.attitude[1], -np.pi/2 * 0.99, np.pi/2 * 0.99)  # Pitch
        psi = self.attitude[2]  # Yaw
        
        # Calculate trigonometric functions once to save computation
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta, stheta = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        # Rotation matrix from body to inertial frame (ZYX convention)
        # This transforms vectors from body frame to inertial frame
        R = np.array([
            [ctheta*cpsi, sphi*stheta*cpsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi],
            [ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi],
            [-stheta, sphi*ctheta, cphi*ctheta]
        ])
        
        return R
    
    def rotation_matrix_inertial_to_body(self):
        """
        Calculate the rotation matrix from inertial to body frame.
        
        Returns:
            np.ndarray: 3x3 rotation matrix
        """
        # Transpose of body-to-inertial rotation matrix
        return self.rotation_matrix_body_to_inertial().T
    
    def euler_rates_from_angular_velocity(self):
        """
        Convert body angular velocity to Euler angle rates.
        
        Returns:
            np.ndarray: Euler angle rates [phi_dot, theta_dot, psi_dot] (rad/s)
        """
        # Limit attitude angles to prevent gimbal lock
        phi = np.clip(self.attitude[0], -np.pi/2 * 0.99, np.pi/2 * 0.99)  # Roll
        theta = np.clip(self.attitude[1], -np.pi/2 * 0.99, np.pi/2 * 0.99)  # Pitch
        
        p, q, r = self.angular_velocity  # Body rates
        
        # Calculate trigonometric functions once to save computation
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta = np.cos(theta)
        
        # Ensure cos(theta) is not too close to zero to avoid division by zero
        if abs(ctheta) < 1e-6:
            ctheta = 1e-6 if ctheta > 0 else -1e-6
        
        ttheta = np.tan(theta)
        
        # Transformation matrix from body rates to Euler rates
        euler_rates = np.array([
            p + q*sphi*ttheta + r*cphi*ttheta,
            q*cphi - r*sphi,
            q*sphi/ctheta + r*cphi/ctheta
        ])
        
        # Check for NaN values and replace with zeros
        euler_rates = np.nan_to_num(euler_rates, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Limit rates to avoid numerical issues
        max_rate = 5.0  # rad/s, reasonable maximum
        euler_rates = np.clip(euler_rates, -max_rate, max_rate)
        
        return euler_rates
    
    def calculate_forces_and_moments(self):
        """
        Calculate the forces and moments acting on the quadcopter.
        
        Updates the forces and moments attributes.
        
        For a quadcopter in X configuration, the motors are arranged as:
        
        Motor 1: Front-right
        Motor 2: Front-left
        Motor 3: Back-left
        Motor 4: Back-right
        
        Positive roll means right side down, left side up (rotate about x-axis)
        Positive pitch means front down, back up (rotate about y-axis)
        Positive yaw is counter-clockwise rotation when viewed from above (rotate about z-axis)
        """
        # Calculate individual motor thrust forces - thrust is proportional to square of motor speed
        motor_thrusts = self.thrust_coef * self.motor_speeds**2
        
        # Ensure motor thrusts are within reasonable limits
        max_single_thrust = 2.0 * self.mass * GRAVITY  # Reasonable maximum
        motor_thrusts = np.clip(motor_thrusts, 0, max_single_thrust)
        
        # Total thrust force (along body z-axis, positive upward)
        thrust = np.sum(motor_thrusts)
        
        # Limit total thrust to reasonable values
        max_thrust = 4.0 * self.mass * GRAVITY  # Maximum possible thrust
        thrust = min(thrust, max_thrust)
        
        # The thrust force is in the body frame (aligned with body z-axis)
        thrust_body = np.array([0, 0, thrust])
        
        # Calculate moment arms for each motor (distance * direction)
        # For X configuration, motors are at 45° from axes
        # The arm_length is the distance from center to motor
        # We need to project this onto the x and y axes with sin(45°) = cos(45°) = 1/√2
        motor_moment_arms = self.arm_length * np.sin(np.pi/4)
        
        # Calculate moments from motor thrusts
        # Each motor contributes to roll and pitch moments based on its position
        # Roll moment (x-axis): positive roll is right side down
        # Motor 2 and 3 (left side) produce positive roll moment
        # Motor 1 and 4 (right side) produce negative roll moment
        roll_moment = motor_moment_arms * (motor_thrusts[1] + motor_thrusts[2] - motor_thrusts[0] - motor_thrusts[3])
        
        # Pitch moment (y-axis): positive pitch is nose down
        # Motor 3 and 4 (back) produce positive pitch moment
        # Motor 1 and 2 (front) produce negative pitch moment
        pitch_moment = motor_moment_arms * (motor_thrusts[2] + motor_thrusts[3] - motor_thrusts[0] - motor_thrusts[1])
        
        # Yaw moment (z-axis): from counter-torque reaction
        # Multiply motor direction by drag coefficient and square of motor speed
        # If motor spins CW, it creates CCW reaction torque on the frame and vice versa
        yaw_moment = self.drag_coef * np.sum(self.motor_dirs * self.motor_speeds**2)
        
        # Combine moments in body frame
        moments_body = np.array([roll_moment, pitch_moment, yaw_moment])
        
        # Limit moments to reasonable values to prevent numerical issues
        max_moment = 2.0 * self.mass * GRAVITY * self.arm_length  # Reasonable maximum
        moments_body = np.clip(moments_body, -max_moment, max_moment)
        
        # Gravity force is always in the inertial frame (pointing down)
        gravity_inertial = np.array([0, 0, -self.mass * GRAVITY])
        
        # Transform thrust from body to inertial frame
        R_body_to_inertial = self.rotation_matrix_body_to_inertial()
        thrust_inertial = R_body_to_inertial @ thrust_body
        
        # Total forces in inertial frame
        self.forces = thrust_inertial + gravity_inertial
        
        # Limit forces to reasonable values
        max_force = 5.0 * self.mass * GRAVITY  # Reasonable maximum
        self.forces = np.clip(self.forces, -max_force, max_force)
        
        # Store moments in body frame
        self.moments = moments_body
    
    def update(self, dt=None):
        """
        Update the state of the quadcopter for one time step.
        Uses a simple Euler integration method.
        
        Args:
            dt (float, optional): Time step in seconds. If None, use the default dt.
            
        Returns:
            dict: Current state of the quadcopter
        """
        if dt is None:
            dt = self.dt
        
        # Limit dt to prevent numerical instability
        dt = min(dt, 0.05)  # Maximum of 50 milliseconds
        
        # Update motor speeds according to first-order dynamics
        self.update_motor_speeds(dt)
        
        # Calculate forces (inertial frame) and moments (body frame)
        self.calculate_forces_and_moments()
        
        # LINEAR MOTION: Update position and velocity in inertial frame
        # Force = mass * acceleration -> acceleration = force / mass
        acceleration = self.forces / self.mass
        
        # Limit acceleration to prevent numerical instability
        acceleration = np.clip(acceleration, -self.max_acceleration, self.max_acceleration)
        
        # Integrate velocity and position (Euler integration)
        self.velocity += acceleration * dt
        
        # Limit velocity to prevent numerical instability
        self.velocity = np.clip(self.velocity, -self.max_velocity, self.max_velocity)
        
        self.position += self.velocity * dt
        
        # ANGULAR MOTION: Update attitude and angular velocity in body frame
        
        # Calculate angular acceleration in body frame using rigid body dynamics
        # Euler's rotation equations: I * α + ω × (I * ω) = M
        # Where α is angular acceleration, ω is angular velocity, I is inertia tensor, M is moment
        omega = self.angular_velocity
        
        # Cross product of angular velocity and angular momentum
        # Use safe cross to prevent numerical issues
        omega_cross_I_omega = safe_cross(omega, self.I @ omega)
        
        # Apply Euler's equation: angular_acceleration = I^(-1) * (M - ω × (I * ω))
        angular_acceleration = self.I_inv @ (self.moments - omega_cross_I_omega)
        
        # Limit angular acceleration to prevent numerical instability
        angular_acceleration = np.clip(
            angular_acceleration, 
            -self.max_angular_acceleration, 
            self.max_angular_acceleration
        )
        
        # Integrate angular velocity (body frame)
        self.angular_velocity += angular_acceleration * dt
        
        # Limit angular velocity to prevent numerical instability
        self.angular_velocity = np.clip(
            self.angular_velocity, 
            -self.max_angular_velocity, 
            self.max_angular_velocity
        )
        
        # Convert angular velocity (body frame) to Euler angle rates (inertial frame)
        euler_rates = self.euler_rates_from_angular_velocity()
        
        # Integrate Euler angles
        self.attitude += euler_rates * dt
        
        # Normalize yaw angle to [-pi, pi]
        self.attitude[2] = ((self.attitude[2] + np.pi) % (2 * np.pi)) - np.pi
        
        # Return current state
        return self.get_state()
    
    def get_state(self):
        """
        Get the current state of the quadcopter.
        
        Returns:
            dict: Current state including position, velocity, attitude, etc.
        """
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'attitude': self.attitude.copy(),
            'angular_velocity': self.angular_velocity.copy(),
            'motor_speeds': self.motor_speeds.copy(),
            'forces': self.forces.copy(),
            'moments': self.moments.copy()
        } 