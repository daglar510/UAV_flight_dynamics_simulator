"""
Structured Quadcopter Simulation.

This module provides a non-interactive quadcopter simulator that takes
all inputs upfront, runs the complete simulation, and displays the results.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import time
import os
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from uav_sim.quadcopter.models import get_quadcopter_parameters, get_available_quadcopter_models
from uav_sim.quadcopter.dynamics import QuadcopterDynamics
from uav_sim.constants import GRAVITY

class StructuredQuadcopterSim:
    """
    Quadcopter simulator that runs a complete simulation with predefined inputs.
    
    This simulator takes all flight parameters upfront, runs the full simulation,
    and then displays the results. No interactive control during simulation.
    """
    
    def __init__(self, model_name="default", dt=0.01, duration=30.0):
        """
        Initialize the structured quadcopter simulation.
        
        Args:
            model_name (str): Name of the quadcopter model
            dt (float): Simulation time step (seconds)
            duration (float): Total simulation duration (seconds)
        """
        # Load quadcopter parameters
        self.params = get_quadcopter_parameters(model_name)
        self.dt = dt
        self.duration = duration
        
        # Calculate number of time steps
        self.n_steps = int(duration / dt)
        
        # Create quadcopter dynamics model
        self.quad = QuadcopterDynamics(self.params)
        
        # Set initial position slightly above ground
        self.quad.set_state(position=np.array([0.0, 0.0, 0.1]))
        
        # Initialize history storage
        self.time_history = np.zeros(self.n_steps)
        self.position_history = np.zeros((self.n_steps, 3))
        self.attitude_history = np.zeros((self.n_steps, 3))
        self.angular_velocity_history = np.zeros((self.n_steps, 3))
        self.linear_velocity_history = np.zeros((self.n_steps, 3))
        self.motor_speeds_history = np.zeros((self.n_steps, 4))
        
        # Calculate hover motor speed
        mass = self.params['mass']
        g = GRAVITY
        thrust_coef = self.params['thrust_coefficient']
        self.hover_motor_speed = np.sqrt(mass * g / (4 * thrust_coef))
        
        # Motor speed limits
        self.min_speed = self.quad.min_motor_speed
        self.max_speed = self.quad.max_motor_speed
        
        # RPM conversion factor
        self.rpm_conversion = 60 / (2 * np.pi)
        
        # Pre-defined motor speed profile (will be set by the user)
        self.motor_speed_profile = None
    
    def set_step_input(self, start_time, duration, motor_speeds):
        """
        Set a step input for motor speeds.
        
        Args:
            start_time (float): When to apply the step input (seconds)
            duration (float): How long to apply the input (seconds)
            motor_speeds (list): List of 4 motor speeds in RPM
        
        Returns:
            self: For method chaining
        """
        if self.motor_speed_profile is None:
            # Initialize with hover speed if not already set
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.hover_motor_speed * 0.95
        
        # Convert start time and duration to indices
        start_idx = int(start_time / self.dt)
        end_idx = min(int((start_time + duration) / self.dt), self.n_steps)
        
        # Convert RPM to rad/s
        speeds_rad_s = np.array(motor_speeds) * (2 * np.pi / 60)
        
        # Apply limits
        speeds_rad_s = np.clip(speeds_rad_s, self.min_speed, self.max_speed)
        
        # Set the motor speeds for the specified duration
        for i in range(start_idx, end_idx):
            if i < self.n_steps:
                self.motor_speed_profile[i] = speeds_rad_s
            
        return self
    
    def set_ramp_input(self, start_time, duration, start_speeds, end_speeds):
        """
        Set a ramp input (linear transition) for motor speeds.
        
        Args:
            start_time (float): When to start the ramp (seconds)
            duration (float): How long the ramp lasts (seconds)
            start_speeds (list): Initial speeds in RPM (list of 4)
            end_speeds (list): Final speeds in RPM (list of 4)
        
        Returns:
            self: For method chaining
        """
        if self.motor_speed_profile is None:
            # Initialize with hover speed if not already set
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.hover_motor_speed * 0.95
            
        # Convert start time and duration to indices
        start_idx = int(start_time / self.dt)
        end_idx = min(int((start_time + duration) / self.dt), self.n_steps)
        
        # Convert RPM to rad/s
        start_rad_s = np.array(start_speeds) * (2 * np.pi / 60)
        end_rad_s = np.array(end_speeds) * (2 * np.pi / 60)
        
        # Apply limits
        start_rad_s = np.clip(start_rad_s, self.min_speed, self.max_speed)
        end_rad_s = np.clip(end_rad_s, self.min_speed, self.max_speed)
        
        # Create a linear ramp
        n_ramp_steps = end_idx - start_idx
        if n_ramp_steps > 0:
            for motor_idx in range(4):
                ramp = np.linspace(start_rad_s[motor_idx], end_rad_s[motor_idx], n_ramp_steps)
                if start_idx < self.n_steps:
                    end = min(end_idx, self.n_steps)
                    self.motor_speed_profile[start_idx:end, motor_idx] = ramp[:end-start_idx]
                
        return self
    
    def set_hover_mode(self, start_time, duration):
        """
        Set motors to hover speed for a specified duration.
        """
        hover_speeds = [self.hover_motor_speed * self.rpm_conversion] * 4
        return self.set_step_input(start_time, duration, hover_speeds)
    
    def set_takeoff_profile(self, start_time=0.0, duration=5.0):
        """
        Set a standard takeoff profile.
        """
        # If no profile exists, create one with minimum speeds
        if self.motor_speed_profile is None:
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.min_speed
        
        # Start motors at 80% hover
        start_speeds = [self.hover_motor_speed * 0.8 * self.rpm_conversion] * 4
        
        # Ramp up to 110% of hover speed to gain altitude
        end_speeds = [self.hover_motor_speed * 1.1 * self.rpm_conversion] * 4
        
        # First stage: initial ramp up
        first_stage = duration * 0.3
        self.set_ramp_input(start_time, first_stage, start_speeds, end_speeds)
        
        # Second stage: hover at slightly above hover speed
        hover_speeds = [self.hover_motor_speed * 1.05 * self.rpm_conversion] * 4
        self.set_step_input(start_time + first_stage, duration - first_stage, hover_speeds)
        
        return self
    
    def set_landing_profile(self, start_time, duration=5.0):
        """
        Set a standard landing profile.
        """
        # Start from current motor speeds or hover if not set
        if self.motor_speed_profile is None:
            start_speeds = [self.hover_motor_speed * self.rpm_conversion] * 4
        else:
            idx = int(start_time / self.dt)
            if idx < len(self.motor_speed_profile):
                start_speeds = [s * self.rpm_conversion for s in self.motor_speed_profile[idx]]
            else:
                start_speeds = [self.hover_motor_speed * self.rpm_conversion] * 4
        
        # First stage: gentle descent (90% hover)
        first_stage = duration * 0.6
        mid_speeds = [self.hover_motor_speed * 0.9 * self.rpm_conversion] * 4
        self.set_ramp_input(start_time, first_stage, start_speeds, mid_speeds)
        
        # Second stage: final descent to minimum speed
        end_speeds = [self.min_speed * self.rpm_conversion * 1.2] * 4
        self.set_ramp_input(start_time + first_stage, duration - first_stage, mid_speeds, end_speeds)
        
        # Stop motors at the end
        min_speeds = [self.min_speed * self.rpm_conversion] * 4
        self.set_step_input(start_time + duration, 0.5, min_speeds)
        
        return self
    
    def set_roll_maneuver(self, start_time, duration, direction="right", intensity=0.2):
        """
        Set a roll maneuver.
        """
        # Ensure we have a motor profile
        if self.motor_speed_profile is None:
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.hover_motor_speed
        
        # Get current speeds at start time
        idx = int(start_time / self.dt)
        if idx < len(self.motor_speed_profile):
            current_speeds = self.motor_speed_profile[idx].copy()
        else:
            current_speeds = np.ones(4) * self.hover_motor_speed
        
        # Calculate speed difference for roll
        diff = self.hover_motor_speed * intensity
        
        if direction == "right":
            # To roll right, increase left motors (2,3) and decrease right motors (1,4)
            motor_speeds = current_speeds.copy()
            motor_speeds[1] += diff  # Increase motor 2 (FL)
            motor_speeds[2] += diff  # Increase motor 3 (BL)
            motor_speeds[0] -= diff  # Decrease motor 1 (FR)
            motor_speeds[3] -= diff  # Decrease motor 4 (BR)
        else:
            # To roll left, increase right motors (1,4) and decrease left motors (2,3)
            motor_speeds = current_speeds.copy()
            motor_speeds[0] += diff  # Increase motor 1 (FR)
            motor_speeds[3] += diff  # Increase motor 4 (BR)
            motor_speeds[1] -= diff  # Decrease motor 2 (FL)
            motor_speeds[2] -= diff  # Decrease motor 3 (BL)
        
        # Convert to RPM for the step input
        rpm_speeds = [s * self.rpm_conversion for s in motor_speeds]
        self.set_step_input(start_time, duration, rpm_speeds)
        
        # Return to hover after maneuver
        hover_rpm = [self.hover_motor_speed * self.rpm_conversion] * 4
        self.set_step_input(start_time + duration, 1.0, hover_rpm)
        
        return self
    
    def set_pitch_maneuver(self, start_time, duration, direction="forward", intensity=0.2):
        """
        Set a pitch maneuver.
        """
        # Ensure we have a motor profile
        if self.motor_speed_profile is None:
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.hover_motor_speed
        
        # Get current speeds at start time
        idx = int(start_time / self.dt)
        if idx < len(self.motor_speed_profile):
            current_speeds = self.motor_speed_profile[idx].copy()
        else:
            current_speeds = np.ones(4) * self.hover_motor_speed
        
        # Calculate speed difference for pitch
        diff = self.hover_motor_speed * intensity
        
        if direction == "forward":
            # To pitch forward, increase back motors (3,4) and decrease front motors (1,2)
            motor_speeds = current_speeds.copy()
            motor_speeds[2] += diff  # Increase motor 3 (BL)
            motor_speeds[3] += diff  # Increase motor 4 (BR)
            motor_speeds[0] -= diff  # Decrease motor 1 (FR)
            motor_speeds[1] -= diff  # Decrease motor 2 (FL)
        else:
            # To pitch backward, increase front motors (1,2) and decrease back motors (3,4)
            motor_speeds = current_speeds.copy()
            motor_speeds[0] += diff  # Increase motor 1 (FR)
            motor_speeds[1] += diff  # Increase motor 2 (FL)
            motor_speeds[2] -= diff  # Decrease motor 3 (BL)
            motor_speeds[3] -= diff  # Decrease motor 4 (BR)
        
        # Convert to RPM for the step input
        rpm_speeds = [s * self.rpm_conversion for s in motor_speeds]
        self.set_step_input(start_time, duration, rpm_speeds)
        
        # Return to hover after maneuver
        hover_rpm = [self.hover_motor_speed * self.rpm_conversion] * 4
        self.set_step_input(start_time + duration, 1.0, hover_rpm)
        
        return self
    
    def set_yaw_maneuver(self, start_time, duration, direction="clockwise", intensity=0.3):
        """
        Set a yaw maneuver.
        """
        # Ensure we have a motor profile
        if self.motor_speed_profile is None:
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.hover_motor_speed
        
        # Get current speeds at start time
        idx = int(start_time / self.dt)
        if idx < len(self.motor_speed_profile):
            current_speeds = self.motor_speed_profile[idx].copy()
        else:
            current_speeds = np.ones(4) * self.hover_motor_speed
        
        # Calculate speed difference for yaw
        diff = self.hover_motor_speed * intensity
        
        if direction == "clockwise":
            # To yaw clockwise, increase CCW motors (2,3) and decrease CW motors (1,4)
            motor_speeds = current_speeds.copy()
            motor_speeds[1] += diff  # Increase motor 2 (FL) - CCW
            motor_speeds[2] += diff  # Increase motor 3 (BL) - CCW
            motor_speeds[0] -= diff  # Decrease motor 1 (FR) - CW
            motor_speeds[3] -= diff  # Decrease motor 4 (BR) - CW
        else:
            # To yaw counterclockwise, increase CW motors (1,4) and decrease CCW motors (2,3)
            motor_speeds = current_speeds.copy()
            motor_speeds[0] += diff  # Increase motor 1 (FR) - CW
            motor_speeds[3] += diff  # Increase motor 4 (BR) - CW
            motor_speeds[1] -= diff  # Decrease motor 2 (FL) - CCW
            motor_speeds[2] -= diff  # Decrease motor 3 (BL) - CCW
        
        # Convert to RPM for the step input
        rpm_speeds = [s * self.rpm_conversion for s in motor_speeds]
        self.set_step_input(start_time, duration, rpm_speeds)
        
        # Return to hover after maneuver
        hover_rpm = [self.hover_motor_speed * self.rpm_conversion] * 4
        self.set_step_input(start_time + duration, 1.0, hover_rpm)
        
        return self
    
    def run_simulation(self):
        """
        Run the complete simulation with the predefined motor speed profile.
        """
        print("Running simulation...")
        start_time = time.time()
        
        # Ensure we have a motor profile
        if self.motor_speed_profile is None:
            print("No motor speed profile defined. Using hover speed.")
            self.motor_speed_profile = np.ones((self.n_steps, 4)) * self.hover_motor_speed
            
        # Loop through all time steps
        for i in range(self.n_steps):
            # Set motor speeds for this step
            self.quad.set_motor_speeds(self.motor_speed_profile[i])
            
            # Update simulation for one time step
            self.quad.update(self.dt)
            
            # Get current state
            state = self.quad.get_state()
            
            # Store in history arrays
            self.time_history[i] = i * self.dt
            self.position_history[i] = state['position']
            self.attitude_history[i] = state['attitude']
            self.angular_velocity_history[i] = state['angular_velocity']
            self.linear_velocity_history[i] = state['velocity']
            self.motor_speeds_history[i] = self.motor_speed_profile[i]
            
            # Print progress every 10% completion
            if i % (self.n_steps // 10) == 0 and i > 0:
                print(f"  {i / self.n_steps * 100:.0f}% complete")
        
        elapsed = time.time() - start_time
        print(f"Simulation completed in {elapsed:.2f} seconds")
    
    def visualize_results(self):
        """
        Visualize the simulation results with enhanced 3D plots and clear data visualization.
        """
        # Create main figure with standard plots for data analysis
        fig_main = plt.figure(figsize=(15, 9))
        fig_main.suptitle(f"Quadcopter Data - Model: {self.params['name']}", fontsize=14)
        
        # Standard 2D plots for data analysis
        # Position plot
        ax_pos = fig_main.add_subplot(2, 2, 1)
        ax_pos.set_title('Position')
        ax_pos.plot(self.time_history, self.position_history[:, 0], 'r-', label='X')
        ax_pos.plot(self.time_history, self.position_history[:, 1], 'g-', label='Y')
        ax_pos.plot(self.time_history, self.position_history[:, 2], 'b-', label='Z')
        ax_pos.set_xlabel('Time [s]')
        ax_pos.set_ylabel('Position [m]')
        ax_pos.grid(True)
        ax_pos.legend()
        
        # Attitude plot
        ax_att = fig_main.add_subplot(2, 2, 2)
        ax_att.set_title('Attitude')
        ax_att.plot(self.time_history, np.degrees(self.attitude_history[:, 0]), 'r-', label='Roll')
        ax_att.plot(self.time_history, np.degrees(self.attitude_history[:, 1]), 'g-', label='Pitch')
        ax_att.plot(self.time_history, np.degrees(self.attitude_history[:, 2]), 'b-', label='Yaw')
        ax_att.set_xlabel('Time [s]')
        ax_att.set_ylabel('Angle [deg]')
        ax_att.grid(True)
        ax_att.legend()
        
        # Motor speeds plot
        ax_mot = fig_main.add_subplot(2, 2, 3)
        ax_mot.set_title('Motor Speeds')
        rpm_conversion = 60 / (2 * np.pi)
        ax_mot.plot(self.time_history, self.motor_speeds_history[:, 0] * rpm_conversion, 'r-', label='M1 (FR)')
        ax_mot.plot(self.time_history, self.motor_speeds_history[:, 1] * rpm_conversion, 'g-', label='M2 (FL)')
        ax_mot.plot(self.time_history, self.motor_speeds_history[:, 2] * rpm_conversion, 'b-', label='M3 (BL)')
        ax_mot.plot(self.time_history, self.motor_speeds_history[:, 3] * rpm_conversion, 'y-', label='M4 (BR)')
        ax_mot.axhline(y=self.hover_motor_speed * rpm_conversion, color='k', linestyle='--', alpha=0.5, label='Hover')
        ax_mot.set_xlabel('Time [s]')
        ax_mot.set_ylabel('Speed [RPM]')
        ax_mot.grid(True)
        ax_mot.legend()
        
        # Velocity plot
        ax_vel = fig_main.add_subplot(2, 2, 4)
        ax_vel.set_title('Velocity')
        ax_vel.plot(self.time_history, self.linear_velocity_history[:, 0], 'r-', label='Vx')
        ax_vel.plot(self.time_history, self.linear_velocity_history[:, 1], 'g-', label='Vy')
        ax_vel.plot(self.time_history, self.linear_velocity_history[:, 2], 'b-', label='Vz')
        # Calculate and plot total velocity magnitude
        vel_mag = np.linalg.norm(self.linear_velocity_history, axis=1)
        ax_vel.plot(self.time_history, vel_mag, 'k-', label='|V|')
        ax_vel.set_xlabel('Time [s]')
        ax_vel.set_ylabel('Velocity [m/s]')
        ax_vel.grid(True)
        ax_vel.legend()
        
        # Adjust layout and save
        plt.tight_layout()
        plt.savefig('quadcopter_data.png', dpi=300, bbox_inches='tight')
        
        # Create dedicated 3D trajectory figure (larger, more detailed)
        fig_3d = plt.figure(figsize=(16, 12))
        ax_3d = fig_3d.add_subplot(111, projection='3d')
        ax_3d.set_title(f'3D Trajectory - {self.params["name"]} Quadcopter', fontsize=18)
        
        # Add a subtle ground plane with grid
        max_x = np.max(np.abs(self.position_history[:, 0]))
        max_y = np.max(np.abs(self.position_history[:, 1]))
        ground_size = max(max_x, max_y) * 1.5
        ground_size = max(ground_size, 3.0)  # Minimum size
        
        # Create ground grid
        xx = np.linspace(-ground_size, ground_size, 20)
        yy = np.linspace(-ground_size, ground_size, 20)
        XX, YY = np.meshgrid(xx, yy)
        ZZ = np.zeros_like(XX)
        
        # Plot ground grid
        ax_3d.plot_surface(XX, YY, ZZ, color='gray', alpha=0.2, antialiased=True, shade=False)
        
        # Add gridlines on the ground
        for x in xx[::2]:
            ax_3d.plot([x, x], [-ground_size, ground_size], [0, 0], 'k-', alpha=0.1)
        for y in yy[::2]:
            ax_3d.plot([-ground_size, ground_size], [y, y], [0, 0], 'k-', alpha=0.1)
            
        # Calculate color map for trajectory based on time - using plasma colormap for vivid colors
        time_normalized = self.time_history / self.time_history[-1]
        cmap = plt.cm.plasma
        colors = cmap(time_normalized)
        
        # Create a smooth trajectory line with gradient coloring
        points = np.array([self.position_history[:, 0], 
                           self.position_history[:, 1], 
                           self.position_history[:, 2]]).T.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Create a line collection for the trajectory
        lc = Line3DCollection(segments, cmap=cmap, norm=plt.Normalize(0, 1), lw=2.5)
        lc.set_array(time_normalized[:-1])
        line = ax_3d.add_collection3d(lc)
        
        # Add a colorbar for time reference
        cbar = fig_3d.colorbar(line, ax=ax_3d, pad=0.05)
        cbar.set_label('Time Progress', fontsize=12)
        
        # Add points along trajectory with color gradient (sparser for performance)
        point_interval = max(1, int(self.n_steps / 300))  # Limit to around 300 points for performance
        ax_3d.scatter(
            self.position_history[::point_interval, 0],
            self.position_history[::point_interval, 1],
            self.position_history[::point_interval, 2],
            c=time_normalized[::point_interval],
            cmap=cmap,
            s=15,
            alpha=0.8,
            depthshade=True
        )
        
        # Add start and end markers with enhanced styling
        ax_3d.scatter(
            self.position_history[0, 0],
            self.position_history[0, 1],
            self.position_history[0, 2],
            color='lime',
            s=150,
            marker='o',
            label='Start',
            edgecolor='black',
            linewidth=1.5,
            zorder=100
        )
        
        ax_3d.scatter(
            self.position_history[-1, 0],
            self.position_history[-1, 1],
            self.position_history[-1, 2],
            color='red',
            s=150,
            marker='o',
            label='End',
            edgecolor='black',
            linewidth=1.5,
            zorder=100
        )
        
        # Add timestamp markers at regular intervals
        marker_interval = int(2.0 / self.dt)  # every 2 seconds
        for i in range(0, self.n_steps, marker_interval):
            if i > 0 and i < self.n_steps - 1:  # Skip start/end points
                ax_3d.scatter(
                    self.position_history[i, 0],
                    self.position_history[i, 1],
                    self.position_history[i, 2],
                    color='white',
                    s=35,
                    alpha=0.8,
                    edgecolor='black',
                    linewidth=0.5,
                    zorder=90
                )
                
                # Add timestamp labels every 4 seconds for readability
                if i % (4 * int(1.0 / self.dt)) == 0:
                    ax_3d.text(
                        self.position_history[i, 0] + 0.1,
                        self.position_history[i, 1] + 0.1,
                        self.position_history[i, 2] + 0.1,
                        f'{self.time_history[i]:.1f}s',
                        color='white',
                        fontsize=9,
                        bbox=dict(facecolor='black', alpha=0.6, boxstyle='round,pad=0.2'),
                        zorder=95
                    )
        
        # Add velocity vectors at regular intervals
        arrow_interval = int(3.0 / self.dt)  # every 3 seconds
        for i in range(0, self.n_steps, arrow_interval):
            pos = self.position_history[i]
            vel = self.linear_velocity_history[i]
            # Scale velocity for visualization
            vel_mag = np.linalg.norm(vel)
            if vel_mag > 0.1:  # Only draw significant velocity vectors
                vel_scaled = vel * 0.5  # Scale factor for arrow length
                ax_3d.quiver(
                    pos[0], pos[1], pos[2],
                    vel_scaled[0], vel_scaled[1], vel_scaled[2],
                    color='orange',
                    alpha=0.7,
                    arrow_length_ratio=0.15,
                    linewidth=1.5,
                    zorder=80
                )
        
        # Add quadcopter orientation markers at regular intervals
        orientation_interval = int(3.0 / self.dt)  # every 3 seconds
        arm_length = self.params['arm_length']
        
        # Define a function to draw quadcopter at a specific position and orientation
        def draw_quadcopter(position, attitude, index):
            # Create rotation matrix from Euler angles
            roll, pitch, yaw = attitude
            
            # Calculate rotation matrix
            sa, sb, sg = np.sin(roll), np.sin(pitch), np.sin(yaw)
            ca, cb, cg = np.cos(roll), np.cos(pitch), np.cos(yaw)
            
            # Full rotation matrix
            R = np.array([
                [cb*cg, sa*sb*cg-ca*sg, ca*sb*cg+sa*sg],
                [cb*sg, sa*sb*sg+ca*cg, ca*sb*sg-sa*cg],
                [-sb, sa*cb, ca*cb]
            ])
            
            # Draw quadcopter arms and motors with X configuration
            scaled_arm = arm_length * 0.8  # Scale for visualization
            
            # Motor positions in body frame (X configuration)
            fr = np.array([scaled_arm, -scaled_arm, 0])  # Front-right
            fl = np.array([scaled_arm, scaled_arm, 0])   # Front-left
            bl = np.array([-scaled_arm, scaled_arm, 0])  # Back-left
            br = np.array([-scaled_arm, -scaled_arm, 0]) # Back-right
            
            # Center body
            center = position
            
            # Transform to world frame
            fr_world = position + R @ fr
            fl_world = position + R @ fl
            bl_world = position + R @ bl
            br_world = position + R @ br
            
            # Draw arms with gradient color based on time
            time_color = cmap(time_normalized[index])
            
            # Draw arms with enhanced styling
            arm_width = 2.0
            ax_3d.plot([center[0], fr_world[0]], [center[1], fr_world[1]], [center[2], fr_world[2]], 
                       color=time_color, linewidth=arm_width, alpha=0.8, zorder=70)
            ax_3d.plot([center[0], fl_world[0]], [center[1], fl_world[1]], [center[2], fl_world[2]], 
                       color=time_color, linewidth=arm_width, alpha=0.8, zorder=70)
            ax_3d.plot([center[0], bl_world[0]], [center[1], bl_world[1]], [center[2], bl_world[2]], 
                       color=time_color, linewidth=arm_width, alpha=0.8, zorder=70)
            ax_3d.plot([center[0], br_world[0]], [center[1], br_world[1]], [center[2], br_world[2]], 
                       color=time_color, linewidth=arm_width, alpha=0.8, zorder=70)
            
            # Draw central body as a slightly larger sphere
            ax_3d.scatter([center[0]], [center[1]], [center[2]], 
                          color=time_color, s=60, alpha=0.9, edgecolor='black', linewidth=0.5, zorder=75)
            
            # Add propeller positions as small spheres with color indicating motor number
            motor_sizes = 30
            ax_3d.scatter([fr_world[0]], [fr_world[1]], [fr_world[2]], 
                          color='r', s=motor_sizes, alpha=0.8, edgecolor='black', linewidth=0.5, zorder=75)
            ax_3d.scatter([fl_world[0]], [fl_world[1]], [fl_world[2]], 
                          color='g', s=motor_sizes, alpha=0.8, edgecolor='black', linewidth=0.5, zorder=75)
            ax_3d.scatter([bl_world[0]], [bl_world[1]], [bl_world[2]], 
                          color='b', s=motor_sizes, alpha=0.8, edgecolor='black', linewidth=0.5, zorder=75)
            ax_3d.scatter([br_world[0]], [br_world[1]], [br_world[2]], 
                          color='y', s=motor_sizes, alpha=0.8, edgecolor='black', linewidth=0.5, zorder=75)
            
        # Draw quadcopters at specified intervals
        for i in range(0, self.n_steps, orientation_interval):
            if i < self.n_steps:
                draw_quadcopter(self.position_history[i], self.attitude_history[i], i)
        
        # Draw final quadcopter position with higher visibility
        if self.n_steps > 0:
            draw_quadcopter(self.position_history[-1], self.attitude_history[-1], -1)
        
        # Draw coordinate frame at origin with enhanced styling
        origin = np.zeros(3)
        length = 1.0  # Longer axis for better visibility
        linewidth = 3.0  # Thicker lines
        
        # X axis (forward)
        ax_3d.quiver(*origin, length, 0, 0, color='red', linewidth=linewidth, label='X (Forward)', zorder=105)
        # Y axis (left)
        ax_3d.quiver(*origin, 0, length, 0, color='green', linewidth=linewidth, label='Y (Left)', zorder=105)
        # Z axis (up)
        ax_3d.quiver(*origin, 0, 0, length, color='blue', linewidth=linewidth, label='Z (Up)', zorder=105)
        
        # Draw origin marker
        ax_3d.scatter([0], [0], [0], color='black', s=80, marker='o', alpha=0.7, zorder=105)
        
        # Set reasonable axis limits with equal scaling for all axes
        max_range = np.max([
            self.position_history[:, 0].max() - self.position_history[:, 0].min(),
            self.position_history[:, 1].max() - self.position_history[:, 1].min(),
            self.position_history[:, 2].max() - self.position_history[:, 2].min()
        ])
        
        # Add some margin to the max range
        max_range *= 1.3
        
        mid_x = (self.position_history[:, 0].max() + self.position_history[:, 0].min()) / 2
        mid_y = (self.position_history[:, 1].max() + self.position_history[:, 1].min()) / 2
        mid_z = (self.position_history[:, 2].max() + self.position_history[:, 2].min()) / 2
        
        # Ensure Z starts from ground level (0) for better perspective
        ax_3d.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax_3d.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        ax_3d.set_zlim(0, mid_z + max_range/2)
        
        # Make the grid and axes more visible
        ax_3d.grid(True, linestyle='--', alpha=0.4)
        
        # Make panes semi-transparent for better visibility
        ax_3d.xaxis.pane.fill = True
        ax_3d.yaxis.pane.fill = True
        ax_3d.zaxis.pane.fill = True
        ax_3d.xaxis.pane.set_alpha(0.1)
        ax_3d.yaxis.pane.set_alpha(0.1)
        ax_3d.zaxis.pane.set_alpha(0.1)
        
        # Add labels
        ax_3d.set_xlabel('X [m]', fontsize=14)
        ax_3d.set_ylabel('Y [m]', fontsize=14)
        ax_3d.set_zlabel('Z [m]', fontsize=14)
        ax_3d.legend(fontsize=12, loc='upper right')
        
        # Set optimal viewing angle
        ax_3d.view_init(elev=30, azim=30)
        
        # Make tick labels larger
        ax_3d.tick_params(axis='both', which='major', labelsize=10)
        
        # Add model info and flight duration
        flight_info = f"Model: {self.params['name']}  |  Duration: {self.duration:.1f}s  |  Mass: {self.params['mass']:.2f}kg"
        plt.figtext(0.5, 0.01, flight_info, ha='center', fontsize=12, 
                   bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.5'))
        
        # Adjust layout and save 3D plot
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.05)  # Make room for the info text
        plt.savefig('quadcopter_trajectory_3d.png', dpi=300, bbox_inches='tight')
        
        # Show both figures
        plt.show()

def run_trajectory_simulation(model_name="default", duration=30.0, maneuver_type="square"):
    """
    Run a predefined trajectory simulation.
    
    Args:
        model_name (str): Name of the quadcopter model
        duration (float): Simulation duration in seconds
        maneuver_type (str): Type of trajectory to simulate
    
    Returns:
        StructuredQuadcopterSim: The simulator with results
    """
    # Create simulator
    sim = StructuredQuadcopterSim(model_name=model_name, duration=duration)
    
    # Set up the trajectory based on maneuver type
    if maneuver_type == "hover":
        sim.set_takeoff_profile(0.0, 5.0)
        sim.set_hover_mode(5.0, duration - 10.0)
        sim.set_landing_profile(duration - 5.0, 5.0)
    
    elif maneuver_type == "square":
        # Square trajectory
        sim.set_takeoff_profile(0.0, 5.0)
        sim.set_hover_mode(5.0, 2.0)
        sim.set_pitch_maneuver(7.0, 3.0, "forward", 0.15)
        sim.set_hover_mode(10.0, 2.0)
        sim.set_roll_maneuver(12.0, 3.0, "right", 0.15)
        sim.set_hover_mode(15.0, 2.0)
        sim.set_pitch_maneuver(17.0, 3.0, "backward", 0.15)
        sim.set_hover_mode(20.0, 2.0)
        sim.set_roll_maneuver(22.0, 3.0, "left", 0.15)
        sim.set_hover_mode(25.0, 2.0)
        sim.set_landing_profile(duration - 5.0, 5.0)
    
    elif maneuver_type == "figure8":
        # Figure-8 trajectory
        sim.set_takeoff_profile(0.0, 5.0)
        sim.set_hover_mode(5.0, 2.0)
        sim.set_pitch_maneuver(7.0, 2.0, "forward", 0.12)
        sim.set_roll_maneuver(9.0, 3.0, "right", 0.12)
        sim.set_pitch_maneuver(12.0, 2.0, "forward", 0.12)
        sim.set_roll_maneuver(14.0, 3.0, "left", 0.12)
        sim.set_pitch_maneuver(17.0, 2.0, "forward", 0.12)
        sim.set_roll_maneuver(19.0, 3.0, "right", 0.12)
        sim.set_hover_mode(22.0, 3.0)
        sim.set_landing_profile(duration - 5.0, 5.0)
    
    elif maneuver_type == "yaw_test":
        # Test yaw control
        sim.set_takeoff_profile(0.0, 5.0)
        sim.set_hover_mode(5.0, 2.0)
        sim.set_yaw_maneuver(7.0, 4.0, "clockwise", 0.3)
        sim.set_hover_mode(11.0, 3.0)
        sim.set_yaw_maneuver(14.0, 4.0, "counterclockwise", 0.3)
        sim.set_hover_mode(18.0, 3.0)
        sim.set_landing_profile(duration - 5.0, 5.0)
    
    elif maneuver_type == "spiral":
        # Create an impressive 3D spiral trajectory
        sim.set_takeoff_profile(0.0, 4.0)
        sim.set_hover_mode(4.0, 1.0)
        
        # Start ascending spiral
        start_time = 5.0
        segment_duration = 2.5
        
        # First create a rising spiral by alternating roll and pitch commands
        for i in range(4):
            # Ascending phase: increase altitude while spiraling
            yaw_direction = "clockwise" if i % 2 == 0 else "counterclockwise"
            
            # Yaw while pitching forward slightly to create spiral motion
            sim.set_yaw_maneuver(start_time, segment_duration, yaw_direction, 0.25)
            sim.set_pitch_maneuver(start_time, segment_duration, "forward", 0.1)
            
            # Brief hover between segments
            start_time += segment_duration
            sim.set_hover_mode(start_time, 0.5)
            start_time += 0.5
        
        # Hold position briefly
        sim.set_hover_mode(start_time, 2.0)
        start_time += 2.0
        
        # Now create a descending spiral
        for i in range(3):
            # Descending phase
            yaw_direction = "counterclockwise" if i % 2 == 0 else "clockwise"
            
            # Yaw while pitching at an angle
            if i % 2 == 0:
                sim.set_yaw_maneuver(start_time, segment_duration, yaw_direction, 0.25)
                sim.set_roll_maneuver(start_time, segment_duration, "right", 0.1)
            else:
                sim.set_yaw_maneuver(start_time, segment_duration, yaw_direction, 0.25)
                sim.set_roll_maneuver(start_time, segment_duration, "left", 0.1)
                
            # Brief hover between segments
            start_time += segment_duration
            sim.set_hover_mode(start_time, 0.5)
            start_time += 0.5
        
        # Final hover and landing
        sim.set_hover_mode(start_time, 2.0)
        sim.set_landing_profile(duration - 5.0, 5.0)
        
    else:
        # Default to hover if unknown maneuver type
        print(f"Unknown maneuver: {maneuver_type}, using hover")
        sim.set_takeoff_profile(0.0, 5.0)
        sim.set_hover_mode(5.0, duration - 10.0)
        sim.set_landing_profile(duration - 5.0, 5.0)
    
    # Run simulation
    sim.run_simulation()
    
    return sim

def main():
    """Main function for running trajectory simulations."""
    parser = argparse.ArgumentParser(description="Quadcopter Trajectory Simulator")
    parser.add_argument("--model", choices=get_available_quadcopter_models(), default="default",
                       help="Quadcopter model to simulate")
    parser.add_argument("--duration", type=float, default=30.0,
                       help="Simulation duration in seconds")
    parser.add_argument("--maneuver", choices=["hover", "square", "figure8", "yaw_test", "spiral"], default="square",
                       help="Type of maneuver to simulate")
    
    args = parser.parse_args()
    
    # Run simulation
    sim = run_trajectory_simulation(
        model_name=args.model,
        duration=args.duration,
        maneuver_type=args.maneuver
    )
    
    # Visualize results
    sim.visualize_results()

if __name__ == "__main__":
    main() 