"""
Quadcopter PID controller implementation.
"""

import numpy as np
from collections import deque

class PIDController:
    """
    PID controller with anti-windup and filtering.
    """
    
    def __init__(self, kp, ki, kd, dt=0.01, windup_limit=10.0, differential_filter=True):
        """
        Initialize a PID controller.
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
            dt (float): Time step
            windup_limit (float): Maximum allowed integral term
            differential_filter (bool): Whether to use a filter for derivative term
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.windup_limit = windup_limit
        self.differential_filter = differential_filter
        
        # Internal state
        self.last_error = 0.0
        self.integral = 0.0
        
        # For filtered derivative
        self.derivative_filter_window = 5
        self.error_history = deque(maxlen=self.derivative_filter_window)
    
    def reset(self):
        """
        Reset the controller state.
        """
        self.last_error = 0.0
        self.integral = 0.0
        self.error_history.clear()
    
    def update(self, setpoint, measurement, dt=None):
        """
        Update the controller and compute the control output.
        
        Args:
            setpoint (float): Target value
            measurement (float): Current value
            dt (float, optional): Time step (if None, use the default)
            
        Returns:
            float: Control output
        """
        if dt is None:
            dt = self.dt
        
        # Calculate error
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.windup_limit, self.windup_limit)
        i_term = self.ki * self.integral
        
        # Derivative term (with optional filtering)
        if self.differential_filter and len(self.error_history) >= self.derivative_filter_window:
            # Use filtered derivative (average of recent derivatives)
            self.error_history.append(error)
            derivative = (error - self.error_history[0]) / (dt * self.derivative_filter_window)
        else:
            # Use simple derivative
            derivative = (error - self.last_error) / dt
            self.error_history.append(error)
        
        d_term = self.kd * derivative
        
        # Store error for next iteration
        self.last_error = error
        
        # Compute total control output
        output = p_term + i_term + d_term
        
        return output
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """
        Set new PID gains.
        
        Args:
            kp (float, optional): New proportional gain
            ki (float, optional): New integral gain
            kd (float, optional): New derivative gain
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

class QuadcopterController:
    """
    Implements a hierarchical controller for a quadcopter.
    
    The controller has a cascaded structure:
    - Position controller (outer loop) produces desired attitude
    - Attitude controller (inner loop) produces desired angular rates
    - Rate controller produces motor commands
    """
    
    def __init__(self, quad_params, dt=0.01):
        """
        Initialize the quadcopter controller.
        
        Args:
            quad_params (dict): Quadcopter parameters including PID gains
            dt (float): Time step
        """
        self.quad_params = quad_params
        self.dt = dt
        
        # Extract controller gains
        pid_gains = quad_params.get('pid_gains', {})
        
        # Inner loop controllers (attitude and angular rate)
        roll_gains = pid_gains.get('roll', {'kp': 10.0, 'ki': 0.0, 'kd': 3.0})
        self.roll_controller = PIDController(
            roll_gains['kp'], roll_gains['ki'], roll_gains['kd'], dt)
        
        pitch_gains = pid_gains.get('pitch', {'kp': 10.0, 'ki': 0.0, 'kd': 3.0})
        self.pitch_controller = PIDController(
            pitch_gains['kp'], pitch_gains['ki'], pitch_gains['kd'], dt)
        
        yaw_gains = pid_gains.get('yaw', {'kp': 5.0, 'ki': 0.0, 'kd': 1.0})
        self.yaw_controller = PIDController(
            yaw_gains['kp'], yaw_gains['ki'], yaw_gains['kd'], dt)
        
        # Outer loop controllers (position)
        altitude_gains = pid_gains.get('altitude', {'kp': 20.0, 'ki': 5.0, 'kd': 8.0})
        self.altitude_controller = PIDController(
            altitude_gains['kp'], altitude_gains['ki'], altitude_gains['kd'], dt)
        
        x_gains = pid_gains.get('x', {'kp': 1.5, 'ki': 0.0, 'kd': 0.5})
        self.x_controller = PIDController(
            x_gains['kp'], x_gains['ki'], x_gains['kd'], dt)
        
        y_gains = pid_gains.get('y', {'kp': 1.5, 'ki': 0.0, 'kd': 0.5})
        self.y_controller = PIDController(
            y_gains['kp'], y_gains['ki'], y_gains['kd'], dt)
        
        # Control limits
        self.max_angle = 0.5  # rad (~28 degrees)
        self.max_yaw_rate = 2.0  # rad/s
        
        # Physical parameters
        self.mass = quad_params['mass']
        self.g = GRAVITY if 'GRAVITY' in globals() else 9.81  # Use common gravity constant if available
        self.arm_length = quad_params['arm_length']
        self.thrust_coef = quad_params['thrust_coefficient']
        self.drag_coef = quad_params['drag_coefficient']
        self.max_motor_speed = quad_params.get('max_rpm', 10000) * (2 * np.pi / 60)
        self.min_motor_speed = quad_params.get('min_rpm', 1000) * (2 * np.pi / 60)
        
        # Calculate hover thrust and corresponding motor speeds
        self.hover_thrust = self.mass * self.g
        self.hover_motor_thrust = self.hover_thrust / 4.0  # Equal distribution among 4 motors
        self.hover_motor_speed = np.sqrt(self.hover_motor_thrust / self.thrust_coef)
    
    def reset(self):
        """
        Reset all controllers.
        """
        self.roll_controller.reset()
        self.pitch_controller.reset()
        self.yaw_controller.reset()
        self.altitude_controller.reset()
        self.x_controller.reset()
        self.y_controller.reset()
    
    def compute_control(self, state, setpoints, dt=None):
        """
        Compute control commands for the quadcopter.
        
        Args:
            state (dict): Current state of the quadcopter
            setpoints (dict): Desired setpoints (position, yaw)
            dt (float, optional): Time step (if None, use the default)
            
        Returns:
            np.ndarray: Motor speed commands (rad/s)
        """
        if dt is None:
            dt = self.dt
        
        # Extract current state
        position = state['position']
        velocity = state['velocity']
        attitude = state['attitude']
        angular_velocity = state['angular_velocity']
        yaw = attitude[2]
        
        # Extract setpoints
        desired_position = np.array([
            setpoints.get('x', position[0]),
            setpoints.get('y', position[1]),
            setpoints.get('z', position[2])
        ])
        
        desired_yaw = setpoints.get('yaw', yaw)
        
        # ======== Altitude Control ========
        # PID controller for altitude - output is a thrust command
        # We add the hover thrust to maintain altitude without error
        altitude_error = desired_position[2] - position[2]
        altitude_cmd = self.altitude_controller.update(
            desired_position[2], position[2], dt)
        
        # Compute total thrust - start with hover thrust and adjust based on control
        thrust_cmd = self.hover_thrust + altitude_cmd
        thrust_cmd = max(0, thrust_cmd)  # Ensure non-negative thrust
        
        # ======== Position Control (X-Y plane) ========
        # Apply position control to get desired acceleration in horizontal plane
        x_accel = self.x_controller.update(desired_position[0], position[0], dt)
        y_accel = self.y_controller.update(desired_position[1], position[1], dt)
        
        # Add velocity damping (proportional to negative velocity)
        x_accel -= 0.8 * velocity[0]
        y_accel -= 0.8 * velocity[1]
        
        # ======== Convert desired accelerations to desired attitude ========
        # Rotate commands to align with quadcopter yaw orientation
        c_yaw, s_yaw = np.cos(yaw), np.sin(yaw)
        x_accel_body = x_accel * c_yaw + y_accel * s_yaw
        y_accel_body = -x_accel * s_yaw + y_accel * c_yaw
        
        # Convert acceleration commands to desired roll and pitch angles
        # Small angle approximation: roll ~ y_accel / g, pitch ~ -x_accel / g
        # The negative sign for pitch is because positive pitch makes the quad move backward (negative x)
        desired_roll = np.clip(-y_accel_body / self.g, -self.max_angle, self.max_angle)
        desired_pitch = np.clip(x_accel_body / self.g, -self.max_angle, self.max_angle)
        
        # ======== Attitude Control ========
        # Compute control commands for roll, pitch, and yaw
        roll_cmd = self.roll_controller.update(desired_roll, attitude[0], dt)
        pitch_cmd = self.pitch_controller.update(desired_pitch, attitude[1], dt)
        yaw_cmd = self.yaw_controller.update(desired_yaw, yaw, dt)
        
        # Add damping using angular velocity feedback
        roll_cmd -= 0.3 * angular_velocity[0]
        pitch_cmd -= 0.3 * angular_velocity[1]
        yaw_cmd -= 0.3 * angular_velocity[2]
        
        # ======== Motor Mixing ========
        # Convert thrust, roll, pitch, yaw commands to individual motor controls
        motor_speeds = self.motor_mixing(thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd)
        
        return motor_speeds
    
    def motor_mixing(self, thrust, roll_torque, pitch_torque, yaw_torque):
        """
        Mix control commands to individual motor speeds.
        
        For a quadcopter in X configuration:
        - Motor 1: Front-right
        - Motor 2: Front-left
        - Motor 3: Back-left
        - Motor 4: Back-right
        
        The directions are determined from a top-down view with the
        front of the quadcopter facing upward.
        
        Args:
            thrust (float): Total thrust command (N)
            roll_torque (float): Roll torque command (Nm)
            pitch_torque (float): Pitch torque command (Nm)
            yaw_torque (float): Yaw torque command (Nm)
            
        Returns:
            np.ndarray: Motor speeds (rad/s)
        """
        # Calculate factor for X configuration (45° from axes)
        arm_scale = np.sin(np.pi/4) * self.arm_length
        
        # Individual thrust components
        # Each motor contributes to roll and pitch based on its position
        # Positive roll: increase thrust on left side, decrease on right side
        # Positive pitch: increase thrust on back, decrease on front
        # Positive yaw: increase CW motors, decrease CCW motors
        
        # Calculate individual motor thrusts
        motor_thrust_1 = thrust/4 - roll_torque/(4*arm_scale) - pitch_torque/(4*arm_scale) - yaw_torque/(4*self.drag_coef)  # Front-right (CW)
        motor_thrust_2 = thrust/4 + roll_torque/(4*arm_scale) - pitch_torque/(4*arm_scale) + yaw_torque/(4*self.drag_coef)  # Front-left (CCW)
        motor_thrust_3 = thrust/4 + roll_torque/(4*arm_scale) + pitch_torque/(4*arm_scale) - yaw_torque/(4*self.drag_coef)  # Back-left (CW)
        motor_thrust_4 = thrust/4 - roll_torque/(4*arm_scale) + pitch_torque/(4*arm_scale) + yaw_torque/(4*self.drag_coef)  # Back-right (CCW)
        
        # Combine into array and ensure non-negative thrusts
        motor_thrusts = np.maximum(np.array([
            motor_thrust_1, motor_thrust_2, motor_thrust_3, motor_thrust_4
        ]), 0)
        
        # Convert thrusts to motor speeds: F = k * ω²  ->  ω = sqrt(F/k)
        motor_speeds = np.sqrt(motor_thrusts / self.thrust_coef)
        
        # Clip motor speeds to valid range
        motor_speeds = np.clip(motor_speeds, self.min_motor_speed, self.max_motor_speed)
        
        return motor_speeds 