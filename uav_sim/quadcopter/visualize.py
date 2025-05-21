"""
Visualization utilities for quadcopter simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.collections import LineCollection
from matplotlib import cm
from mpl_toolkits.mplot3d.art3d import Line3DCollection

def plot_3d_trajectory(history):
    """
    Plot the 3D trajectory of a quadcopter with enhanced visualization.
    
    Args:
        history (dict): Simulation history
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Get position and setpoint data, with safety checks
    if 'position' not in history or 'setpoints' not in history:
        # Handle missing data
        ax.text(0, 0, 0, "No trajectory data available", color='red')
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(-1, 1)
        ax.set_title('Quadcopter 3D Trajectory (No Data)')
        return fig
    
    position = history['position']
    setpoints = history['setpoints']
    time = history.get('time', np.arange(position.shape[0]))  # Use time if available or create sequence
    
    # Clean position data - remove any NaN or Inf values
    position_clean = np.copy(position)
    is_valid = ~(np.isnan(position_clean) | np.isinf(position_clean))
    
    # If all values in a row are invalid, set the entire row to zeros
    all_valid_rows = np.all(is_valid, axis=1)
    position_clean = np.nan_to_num(position_clean, nan=0.0, posinf=0.0, neginf=0.0)
    
    # Clean setpoint data - remove any NaN or Inf values
    setpoints_clean = np.copy(setpoints)
    is_valid_sp = ~(np.isnan(setpoints_clean) | np.isinf(setpoints_clean))
    all_valid_sp_rows = np.all(is_valid_sp, axis=1)
    setpoints_clean = np.nan_to_num(setpoints_clean, nan=0.0, posinf=0.0, neginf=0.0)
    
    # Plot only if there's valid data
    if np.any(all_valid_rows):
        # Add subtle ground plane for reference
        max_x = np.max(np.abs(position_clean[:, 0]))
        max_y = np.max(np.abs(position_clean[:, 1]))
        ground_size = max(max_x, max_y) * 1.2
        ground_size = max(ground_size, 2.0)  # Minimum size
        
        # Create ground grid
        xx = np.linspace(-ground_size, ground_size, 15)
        yy = np.linspace(-ground_size, ground_size, 15)
        XX, YY = np.meshgrid(xx, yy)
        ZZ = np.zeros_like(XX)
        
        # Plot ground grid
        ax.plot_surface(XX, YY, ZZ, color='gray', alpha=0.1, antialiased=True, shade=False)
        
        # Add gridlines on the ground
        for x in xx[::2]:
            ax.plot([x, x], [-ground_size, ground_size], [0, 0], 'k-', alpha=0.1)
        for y in yy[::2]:
            ax.plot([-ground_size, ground_size], [y, y], [0, 0], 'k-', alpha=0.1)
            
        # Create color map based on time for dynamic coloring
        time_normalized = (time - time.min()) / (time.max() - time.min()) if time.max() > time.min() else np.zeros_like(time)
        cmap = cm.viridis
        colors = cmap(time_normalized)
        
        # Create smooth trajectory with gradient coloring using line segments
        points = np.array([position_clean[:, 0], 
                          position_clean[:, 1], 
                          position_clean[:, 2]]).T.reshape(-1, 1, 3)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        lc = Line3DCollection(segments, cmap=cmap, norm=plt.Normalize(0, 1), linewidth=2.5)
        lc.set_array(time_normalized[:-1])
        trajectory = ax.add_collection3d(lc)
        
        # Add colorbar for time reference
        cbar = fig.colorbar(trajectory, ax=ax, pad=0.05)
        cbar.set_label('Time Progress', fontsize=10)
        
        # Add points along the trajectory (thinned for performance)
        point_interval = max(1, len(time) // 50)  # Show up to 50 points
        ax.scatter(position_clean[::point_interval, 0], 
                  position_clean[::point_interval, 1], 
                  position_clean[::point_interval, 2],
                  s=15, c=colors[::point_interval], alpha=0.7, depthshade=True)
        
        # Plot start and end points
        ax.scatter(position_clean[0, 0], position_clean[0, 1], position_clean[0, 2], 
                  color='lime', s=120, marker='o', label='Start', edgecolor='black', linewidth=1.5, zorder=100)
        
        ax.scatter(position_clean[-1, 0], position_clean[-1, 1], position_clean[-1, 2], 
                 color='red', s=120, marker='o', label='End', edgecolor='black', linewidth=1.5, zorder=100)
        
        # Add setpoint trajectory with dashed line and lower alpha for less visual domination
        if np.any(all_valid_sp_rows):
            ax.plot(setpoints_clean[:, 0], setpoints_clean[:, 1], setpoints_clean[:, 2], 
                   'r--', alpha=0.5, linewidth=1.5, label='Setpoint')
        
        # Add velocity vectors if available
        if 'velocity' in history:
            velocity = history['velocity']
            velocity_clean = np.nan_to_num(velocity, nan=0.0)
            
            # Add vectors at regular intervals
            vector_interval = max(1, len(time) // 10)  # Show up to 10 velocity vectors
            
            for i in range(0, len(time), vector_interval):
                if all_valid_rows[i]:
                    pos = position_clean[i]
                    vel = velocity_clean[i]
                    # Only show significant velocity
                    vel_magnitude = np.linalg.norm(vel)
                    if vel_magnitude > 0.1:
                        # Scale for better visualization
                        vel_scaled = vel * 0.3  
                        ax.quiver(pos[0], pos[1], pos[2], 
                                 vel_scaled[0], vel_scaled[1], vel_scaled[2],
                                 color='orange', alpha=0.7, arrow_length_ratio=0.15)
    else:
        ax.text(0, 0, 0, "No valid trajectory data", color='red')
    
    # Draw coordinate system at origin with improved styling
    origin = np.zeros(3)
    ax.quiver(*origin, 1, 0, 0, color='r', linewidth=3, label='X axis')
    ax.quiver(*origin, 0, 1, 0, color='g', linewidth=3, label='Y axis')
    ax.quiver(*origin, 0, 0, 1, color='b', linewidth=3, label='Z axis')
    
    # Add a sphere at origin
    ax.scatter([0], [0], [0], color='black', s=50)
    
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_zlabel('Z [m]', fontsize=12)
    ax.set_title('Quadcopter 3D Trajectory', fontsize=14)
    ax.legend(fontsize=10)
    
    # Make axes equal scale with reasonable limits
    try:
        # Calculate limits only from valid data points
        valid_x = position_clean[all_valid_rows, 0]
        valid_y = position_clean[all_valid_rows, 1]
        valid_z = position_clean[all_valid_rows, 2]
        
        # If no valid data, use defaults
        if len(valid_x) == 0 or len(valid_y) == 0 or len(valid_z) == 0:
            raise ValueError("No valid position data for axis limits")
        
        # Calculate ranges with extra check for numerical stability
        x_min, x_max = np.min(valid_x), np.max(valid_x)
        y_min, y_max = np.min(valid_y), np.max(valid_y)
        z_min, z_max = np.min(valid_z), np.max(valid_z)
        
        # Verify the limits aren't identical (would cause zero range)
        if x_min == x_max: 
            x_min, x_max = x_min - 1.0, x_max + 1.0
        if y_min == y_max:
            y_min, y_max = y_min - 1.0, y_max + 1.0
        if z_min == z_max:
            z_min, z_max = z_min - 1.0, z_max + 1.0
            
        # Calculate ranges
        x_range = x_max - x_min
        y_range = y_max - y_min
        z_range = z_max - z_min
        
        # Set minimum range to avoid tiny plots
        min_range = 2.0  # Minimum size of 2 meters in any direction
        x_range = max(x_range, min_range)
        y_range = max(y_range, min_range)
        z_range = max(z_range, min_range)
        
        # Use the largest range to make the plot more cubic
        max_range = max(x_range, y_range, z_range)
        max_range = min(max_range, 10.0)  # Limit maximum range to avoid extreme values
        
        # Calculate midpoints
        mid_x = (x_min + x_max) / 2
        mid_y = (y_min + y_max) / 2
        mid_z = (z_min + z_max) / 2
        
        # Final check for NaN or Inf in the calculated limits
        if (np.isnan(mid_x) or np.isnan(mid_y) or np.isnan(mid_z) or
            np.isnan(max_range) or np.isinf(mid_x) or np.isinf(mid_y) or
            np.isinf(mid_z) or np.isinf(max_range)):
            raise ValueError("NaN or Inf in calculated plot limits")
        
        # Set axis limits with safety checks
        ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        
        # Make sure Z starts from ground (0) or slightly below for better perspective
        z_min = min(0, z_min)  
        ax.set_zlim(z_min, mid_z + max_range/2)
        
    except Exception as e:
        # If there's any error calculating limits, set default ranges
        print(f"Warning: Using default plot limits: {e}")
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(0, 2)
    
    # Enhance visual appearance 
    ax.grid(True, linestyle='--', alpha=0.4)
    
    # Set optimal viewing angle
    ax.view_init(elev=25, azim=45)
    
    # Make panes semi-transparent for better visibility
    ax.xaxis.pane.fill = True
    ax.yaxis.pane.fill = True
    ax.zaxis.pane.fill = True
    ax.xaxis.pane.set_alpha(0.1)
    ax.yaxis.pane.set_alpha(0.1)
    ax.zaxis.pane.set_alpha(0.1)
    
    plt.tight_layout()
    return fig

def plot_position(history):
    """
    Plot the position of a quadcopter over time.
    
    Args:
        history (dict): Simulation history
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    # Check if required data exists
    if 'time' not in history or 'position' not in history or 'setpoints' not in history:
        for ax in axes:
            ax.text(0.5, 0.5, "No position data available", 
                    horizontalalignment='center', verticalalignment='center',
                    transform=ax.transAxes, color='red')
        plt.suptitle('Quadcopter Position (No Data)')
        return fig
    
    time = history['time']
    position = history['position']
    setpoints = history['setpoints']
    
    # Clean data and create masks for valid values
    time_clean = np.nan_to_num(time, nan=0.0)
    is_valid_time = ~np.isnan(time) & ~np.isinf(time)
    
    position_clean = np.nan_to_num(position, nan=0.0)
    is_valid_pos = ~np.isnan(position) & ~np.isinf(position)
    
    setpoints_clean = np.nan_to_num(setpoints, nan=0.0)
    is_valid_sp = ~np.isnan(setpoints) & ~np.isinf(setpoints)
    
    # Enhance position plots with colored areas
    alpha_fill = 0.15  # Transparency for filled areas
    
    # X position
    axes[0].set_ylabel('X [m]', fontsize=11)
    axes[0].grid(True, alpha=0.5, linestyle='--')
    if np.any(is_valid_pos[:, 0] & is_valid_time):
        valid_mask = is_valid_pos[:, 0] & is_valid_time
        axes[0].plot(time_clean[valid_mask], position_clean[valid_mask, 0], 'b-', linewidth=2, label='Actual')
        # Add subtle fill below the line
        axes[0].fill_between(time_clean[valid_mask], 0, position_clean[valid_mask, 0], 
                           color='blue', alpha=alpha_fill)
    if np.any(is_valid_sp[:, 0] & is_valid_time):
        valid_mask = is_valid_sp[:, 0] & is_valid_time
        axes[0].plot(time_clean[valid_mask], setpoints_clean[valid_mask, 0], 'r--', linewidth=1.5, label='Setpoint')
    axes[0].legend(loc='upper right')
    
    # Y position
    axes[1].set_ylabel('Y [m]', fontsize=11)
    axes[1].grid(True, alpha=0.5, linestyle='--')
    if np.any(is_valid_pos[:, 1] & is_valid_time):
        valid_mask = is_valid_pos[:, 1] & is_valid_time
        axes[1].plot(time_clean[valid_mask], position_clean[valid_mask, 1], 'g-', linewidth=2, label='Actual')
        # Add subtle fill below the line
        axes[1].fill_between(time_clean[valid_mask], 0, position_clean[valid_mask, 1], 
                           color='green', alpha=alpha_fill)
    if np.any(is_valid_sp[:, 1] & is_valid_time):
        valid_mask = is_valid_sp[:, 1] & is_valid_time
        axes[1].plot(time_clean[valid_mask], setpoints_clean[valid_mask, 1], 'r--', linewidth=1.5, label='Setpoint')
    axes[1].legend(loc='upper right')
    
    # Z position
    axes[2].set_ylabel('Z [m]', fontsize=11)
    axes[2].set_xlabel('Time [s]', fontsize=11)
    axes[2].grid(True, alpha=0.5, linestyle='--')
    if np.any(is_valid_pos[:, 2] & is_valid_time):
        valid_mask = is_valid_pos[:, 2] & is_valid_time
        axes[2].plot(time_clean[valid_mask], position_clean[valid_mask, 2], 'r-', linewidth=2, label='Actual')
        # Add subtle fill below the line
        axes[2].fill_between(time_clean[valid_mask], 0, position_clean[valid_mask, 2], 
                           color='red', alpha=alpha_fill)
    if np.any(is_valid_sp[:, 2] & is_valid_time):
        valid_mask = is_valid_sp[:, 2] & is_valid_time
        axes[2].plot(time_clean[valid_mask], setpoints_clean[valid_mask, 2], 'r--', linewidth=1.5, label='Setpoint')
    axes[2].legend(loc='upper right')
    
    # Set reasonable y-axis limits for each subplot
    for i, ax in enumerate(axes):
        try:
            # Get current y limits
            y_min, y_max = ax.get_ylim()
            
            # Check if limits contain NaN or Inf (matplotlib might not catch this)
            if np.isnan(y_min) or np.isnan(y_max) or np.isinf(y_min) or np.isinf(y_max):
                raise ValueError("Invalid y limits")
                
            # If limits are too narrow, expand them
            if abs(y_max - y_min) < 0.1:
                if i == 2:  # For Z, expand around positive value (height)
                    ax.set_ylim(0, 2.0)
                else:  # For X and Y, expand around zero
                    ax.set_ylim(-1.0, 1.0)
        except Exception:
            # Set default limits if there's any issue
            if i == 2:  # For Z, expand around positive value (height)
                ax.set_ylim(0, 2.0)
            else:  # For X and Y, expand around zero
                ax.set_ylim(-1.0, 1.0)
    
    plt.suptitle('Quadcopter Position', fontsize=14)
    plt.tight_layout()
    return fig

def plot_attitude(history):
    """
    Plot the attitude of a quadcopter over time with enhanced styling.
    
    Args:
        history (dict): Simulation history
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    # Check if required data exists
    if 'time' not in history or 'attitude' not in history:
        for ax in axes:
            ax.text(0.5, 0.5, "No attitude data available", 
                    horizontalalignment='center', verticalalignment='center',
                    transform=ax.transAxes, color='red')
        plt.suptitle('Quadcopter Attitude (No Data)')
        return fig
    
    time = history['time']
    attitude = history['attitude']
    
    # Get setpoints if available
    setpoints = history.get('setpoints', None)
    
    # Clean data and create masks for valid values
    time_clean = np.nan_to_num(time, nan=0.0)
    is_valid_time = ~np.isnan(time) & ~np.isinf(time)
    
    attitude_clean = np.nan_to_num(attitude, nan=0.0)
    is_valid_att = ~np.isnan(attitude) & ~np.isinf(attitude)
    
    # Enhanced styling
    alpha_fill = 0.15  # Transparency for filled areas
    
    # Roll angle
    axes[0].set_ylabel('Roll [deg]', fontsize=11)
    axes[0].grid(True, alpha=0.5, linestyle='--')
    if np.any(is_valid_att[:, 0] & is_valid_time):
        valid_mask = is_valid_att[:, 0] & is_valid_time
        roll_deg = np.degrees(attitude_clean[valid_mask, 0])
        axes[0].plot(time_clean[valid_mask], roll_deg, 'r-', linewidth=2, label='Actual')
        # Add subtle fill below the line
        axes[0].fill_between(time_clean[valid_mask], 0, roll_deg, 
                            color='red', alpha=alpha_fill)
    axes[0].legend(loc='upper right')
    
    # Pitch angle
    axes[1].set_ylabel('Pitch [deg]', fontsize=11)
    axes[1].grid(True, alpha=0.5, linestyle='--')
    if np.any(is_valid_att[:, 1] & is_valid_time):
        valid_mask = is_valid_att[:, 1] & is_valid_time
        pitch_deg = np.degrees(attitude_clean[valid_mask, 1])
        axes[1].plot(time_clean[valid_mask], pitch_deg, 'g-', linewidth=2, label='Actual')
        # Add subtle fill below the line
        axes[1].fill_between(time_clean[valid_mask], 0, pitch_deg, 
                            color='green', alpha=alpha_fill)
    axes[1].legend(loc='upper right')
    
    # Yaw angle and setpoint
    axes[2].set_ylabel('Yaw [deg]', fontsize=11)
    axes[2].set_xlabel('Time [s]', fontsize=11)
    axes[2].grid(True, alpha=0.5, linestyle='--')
    if np.any(is_valid_att[:, 2] & is_valid_time):
        valid_mask = is_valid_att[:, 2] & is_valid_time
        yaw_deg = np.degrees(attitude_clean[valid_mask, 2])
        axes[2].plot(time_clean[valid_mask], yaw_deg, 'b-', linewidth=2, label='Actual')
        # Add subtle fill below the line
        axes[2].fill_between(time_clean[valid_mask], 0, yaw_deg, 
                            color='blue', alpha=alpha_fill)
    
    # Plot yaw setpoint if available and valid
    if setpoints is not None and setpoints.shape[1] > 3:
        setpoints_clean = np.nan_to_num(setpoints, nan=0.0)
        is_valid_sp = ~np.isnan(setpoints) & ~np.isinf(setpoints)
        
        if np.any(is_valid_sp[:, 3] & is_valid_time):
            valid_mask = is_valid_sp[:, 3] & is_valid_time
            axes[2].plot(time_clean[valid_mask], np.degrees(setpoints_clean[valid_mask, 3]), 
                        'r--', linewidth=1.5, label='Setpoint')
    axes[2].legend(loc='upper right')
    
    # Set reasonable y-axis limits for each subplot
    for ax in axes:
        try:
            # Get current y limits
            y_min, y_max = ax.get_ylim()
            
            # Check if limits contain NaN or Inf
            if np.isnan(y_min) or np.isnan(y_max) or np.isinf(y_min) or np.isinf(y_max):
                raise ValueError("Invalid y limits")
                
            # If limits are too narrow, expand them
            if abs(y_max - y_min) < 5.0:  # At least 5 degrees range
                mid = (y_min + y_max) / 2
                ax.set_ylim(mid - 15.0, mid + 15.0)
        except Exception:
            # Set default attitude limits if there's any issue
            ax.set_ylim(-30.0, 30.0)
    
    plt.suptitle('Quadcopter Attitude', fontsize=14)
    plt.tight_layout()
    return fig

def plot_motors(history):
    """
    Plot the motor speeds of a quadcopter over time with enhanced styling.
    
    Args:
        history (dict): Simulation history
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Check if required data exists
    if 'time' not in history or 'motor_speeds' not in history:
        ax.text(0.5, 0.5, "No motor data available", 
                horizontalalignment='center', verticalalignment='center',
                transform=ax.transAxes, color='red')
        ax.set_title('Quadcopter Motor Speeds (No Data)')
        return fig
    
    time = history['time']
    motor_speeds = history['motor_speeds']
    
    # Clean data and create masks for valid values
    time_clean = np.nan_to_num(time, nan=0.0)
    is_valid_time = ~np.isnan(time) & ~np.isinf(time)
    
    motor_speeds_clean = np.nan_to_num(motor_speeds, nan=0.0)
    is_valid_motors = ~np.isnan(motor_speeds) & ~np.isinf(motor_speeds)
    
    # Convert from rad/s to RPM
    rpm_conversion = 60 / (2 * np.pi)
    motor_speeds_rpm = motor_speeds_clean * rpm_conversion
    
    # Plot each motor with enhanced styling
    motor_names = ['Motor 1 (Front-Right)', 'Motor 2 (Front-Left)', 
                   'Motor 3 (Back-Left)', 'Motor 4 (Back-Right)']
    
    colors = ['#e41a1c', '#377eb8', '#4daf4a', '#ff7f00']  # Distinct colors
    linewidths = 2.5
    
    for i in range(min(4, motor_speeds.shape[1])):
        if np.any(is_valid_motors[:, i] & is_valid_time):
            valid_mask = is_valid_motors[:, i] & is_valid_time
            ax.plot(time_clean[valid_mask], motor_speeds_rpm[valid_mask, i], 
                   color=colors[i], linewidth=linewidths, label=motor_names[i])
    
    # Add hover speed reference if available
    hover_speed = None
    # Common hover speed is usually around 4000-5000 RPM for medium quadcopters
    if np.any(is_valid_motors):
        hover_speed = np.mean(motor_speeds_rpm[is_valid_motors]) 
        if hover_speed > 0:
            ax.axhline(y=hover_speed, color='k', linestyle='--', 
                      alpha=0.5, linewidth=1.5, label='Avg Speed')
    
    # Set reasonable y-axis limits
    try:
        # Get current y limits
        y_min, y_max = ax.get_ylim()
        
        # Check if limits contain NaN or Inf
        if np.isnan(y_min) or np.isnan(y_max) or np.isinf(y_min) or np.isinf(y_max):
            raise ValueError("Invalid y limits")
            
        # If minimum is negative or limits too narrow, fix them
        if y_min < 0 or (y_max - y_min) < 100:
            ax.set_ylim(0, max(1000, y_max))
    except Exception:
        # Set default motor speed limits if there's any issue
        ax.set_ylim(0, 3000)  # Default range from 0 to 3000 RPM
    
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Motor Speed [RPM]', fontsize=12)
    ax.set_title('Quadcopter Motor Speeds', fontsize=14)
    ax.grid(True, alpha=0.5, linestyle='--')
    ax.legend(loc='upper right')
    
    # Add background shading for readability
    ax.set_facecolor('#f8f8f8')
    
    plt.tight_layout()
    return fig

class QuadcopterVisualizer:
    """
    Class to handle quadcopter visualization.
    """
    
    def __init__(self, history=None):
        """
        Initialize the visualizer.
        
        Args:
            history (dict, optional): Simulation history
        """
        self.history = history
        self.figures = {}
    
    def set_history(self, history):
        """
        Set the simulation history.
        
        Args:
            history (dict): Simulation history
        """
        self.history = history
    
    def plot_all(self, show=True):
        """
        Create all available plots.
        
        Args:
            show (bool): Whether to display the plots
            
        Returns:
            dict: Dictionary of created figures
        """
        if self.history is None:
            raise ValueError("No simulation history to visualize")
        
        self.figures['trajectory_3d'] = plot_3d_trajectory(self.history)
        self.figures['position'] = plot_position(self.history)
        self.figures['attitude'] = plot_attitude(self.history)
        self.figures['motors'] = plot_motors(self.history)
        
        if show:
            plt.show()
        
        return self.figures
    
    def plot_trajectory_3d(self, show=True):
        """
        Plot the 3D trajectory.
        
        Args:
            show (bool): Whether to display the plot
            
        Returns:
            matplotlib.figure.Figure: The created figure
        """
        if self.history is None:
            raise ValueError("No simulation history to visualize")
        
        fig = plot_3d_trajectory(self.history)
        self.figures['trajectory_3d'] = fig
        
        if show:
            plt.show()
        
        return fig
    
    def plot_position(self, show=True):
        """
        Plot the position over time.
        
        Args:
            show (bool): Whether to display the plot
            
        Returns:
            matplotlib.figure.Figure: The created figure
        """
        if self.history is None:
            raise ValueError("No simulation history to visualize")
        
        fig = plot_position(self.history)
        self.figures['position'] = fig
        
        if show:
            plt.show()
        
        return fig
    
    def plot_attitude(self, show=True):
        """
        Plot the attitude over time.
        
        Args:
            show (bool): Whether to display the plot
            
        Returns:
            matplotlib.figure.Figure: The created figure
        """
        if self.history is None:
            raise ValueError("No simulation history to visualize")
        
        fig = plot_attitude(self.history)
        self.figures['attitude'] = fig
        
        if show:
            plt.show()
        
        return fig
    
    def plot_motors(self, show=True):
        """
        Plot the motor speeds over time.
        
        Args:
            show (bool): Whether to display the plot
            
        Returns:
            matplotlib.figure.Figure: The created figure
        """
        if self.history is None:
            raise ValueError("No simulation history to visualize")
        
        fig = plot_motors(self.history)
        self.figures['motors'] = fig
        
        if show:
            plt.show()
        
        return fig
    
    def save_figures(self, prefix="quadcopter_"):
        """
        Save all generated figures.
        
        Args:
            prefix (str): Prefix for filenames
        """
        for name, fig in self.figures.items():
            filename = f"{prefix}{name}.png"
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Saved {filename}") 