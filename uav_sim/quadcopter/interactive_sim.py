"""
Interactive Quadcopter Simulation.

This module provides an interactive simulator that responds to keyboard inputs.
"""

import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib
import keyboard
import argparse
import os
import sys

from uav_sim.quadcopter.models import get_quadcopter_parameters, get_available_quadcopter_models
from uav_sim.quadcopter.dynamics import QuadcopterDynamics
from uav_sim.constants import GRAVITY

class InteractiveQuadcopterSim:
    """Interactive quadcopter simulation with manual motor control."""
    
    def __init__(self, model_name="default", dt=0.02, history_length=500):
        """
        Initialize the interactive quadcopter simulation.
        
        Args:
            model_name (str): Name of the quadcopter model
            dt (float): Simulation time step (seconds)
            history_length (int): Number of history points to store
        """
        # Load quadcopter parameters
        self.params = get_quadcopter_parameters(model_name)
        self.dt = dt
        self.history_length = history_length
        
        # Create quadcopter dynamics model
        self.quad = QuadcopterDynamics(self.params)
        
        # Set initial position slightly above ground
        self.quad.set_state(position=np.array([0.0, 0.0, 0.1]))
        
        # Initialize history storage
        self.time_history = np.zeros(history_length)
        self.position_history = np.zeros((history_length, 3))
        self.attitude_history = np.zeros((history_length, 3))
        self.motor_speeds_history = np.zeros((history_length, 4))
        
        # Index for current position in history arrays
        self.history_idx = 0
        self.sim_time = 0.0
        
        # Calculate hover motor speed
        mass = self.params['mass']
        g = GRAVITY
        thrust_coef = self.params['thrust_coefficient']
        self.hover_motor_speed = np.sqrt(mass * g / (4 * thrust_coef))
        
        # Initialize motor speeds at just below hover (to start on the ground)
        self.motor_speeds = np.ones(4) * self.hover_motor_speed * 0.95
        
        # Motor speed increment (as percentage of hover speed)
        self.speed_increment = 0.05 * self.hover_motor_speed
        
        # Control flags
        self.running = False
        self.paused = False
        
        # Visualization
        self.fig = None
        self.animation = None
        
        # Motor speed limits
        self.min_speed = self.quad.min_motor_speed
        self.max_speed = self.quad.max_motor_speed
        
        # Key bindings for motor control
        self.key_bindings = {
            # Motor 1 (Front-Right): 1/Q keys
            '1': {'motor': 0, 'change': self.speed_increment, 'description': 'Increase Motor 1 (Front-Right)'},
            'q': {'motor': 0, 'change': -self.speed_increment, 'description': 'Decrease Motor 1 (Front-Right)'},
            
            # Motor 2 (Front-Left): 2/W keys
            '2': {'motor': 1, 'change': self.speed_increment, 'description': 'Increase Motor 2 (Front-Left)'},
            'w': {'motor': 1, 'change': -self.speed_increment, 'description': 'Decrease Motor 2 (Front-Left)'},
            
            # Motor 3 (Back-Left): 3/E keys
            '3': {'motor': 2, 'change': self.speed_increment, 'description': 'Increase Motor 3 (Back-Left)'},
            'e': {'motor': 2, 'change': -self.speed_increment, 'description': 'Decrease Motor 3 (Back-Left)'},
            
            # Motor 4 (Back-Right): 4/R keys
            '4': {'motor': 3, 'change': self.speed_increment, 'description': 'Increase Motor 4 (Back-Right)'},
            'r': {'motor': 3, 'change': -self.speed_increment, 'description': 'Decrease Motor 4 (Back-Right)'},
            
            # Common controls
            'space': {'special': 'pause', 'description': 'Pause/Resume simulation'},
            'h': {'special': 'hover', 'description': 'Set all motors to hover speed'},
            'z': {'special': 'zero', 'description': 'Set all motors to minimum speed'},
            'r': {'special': 'reset', 'description': 'Reset simulation'},
            'esc': {'special': 'quit', 'description': 'Quit simulation'}
        }
    
    def update_simulation(self):
        """Update simulation state for one time step."""
        # Skip if paused
        if self.paused:
            return
            
        # Set motor speeds in the quadcopter
        self.quad.set_motor_speeds(self.motor_speeds)
        
        # Update simulation for one time step
        self.quad.update(self.dt)
        
        # Update simulation time
        self.sim_time += self.dt
        
        # Get current state
        state = self.quad.get_state()
        
        # Store in history arrays
        idx = self.history_idx % self.history_length
        self.time_history[idx] = self.sim_time
        self.position_history[idx] = state['position']
        self.attitude_history[idx] = state['attitude']
        self.motor_speeds_history[idx] = self.quad.motor_speeds
        
        # Increment history index
        self.history_idx += 1
    
    def setup_visualization(self):
        """Set up the interactive visualization."""
        # Create figure and subplots
        self.fig = plt.figure(figsize=(15, 8))
        
        # 3D trajectory subplot
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_3d.set_title('Quadcopter Trajectory')
        self.ax_3d.set_xlabel('X [m]')
        self.ax_3d.set_ylabel('Y [m]')
        self.ax_3d.set_zlabel('Z [m]')
        self.ax_3d.set_xlim(-2, 2)
        self.ax_3d.set_ylim(-2, 2)
        self.ax_3d.set_zlim(0, 4)
        
        # Position subplot
        self.ax_pos = self.fig.add_subplot(222)
        self.ax_pos.set_title('Position')
        self.ax_pos.set_xlabel('Time [s]')
        self.ax_pos.set_ylabel('Position [m]')
        self.ax_pos.grid(True)
        
        # Attitude subplot
        self.ax_att = self.fig.add_subplot(223)
        self.ax_att.set_title('Attitude')
        self.ax_att.set_xlabel('Time [s]')
        self.ax_att.set_ylabel('Angle [deg]')
        self.ax_att.grid(True)
        
        # Motor speeds subplot
        self.ax_mot = self.fig.add_subplot(224)
        self.ax_mot.set_title('Motor Speeds')
        self.ax_mot.set_xlabel('Time [s]')
        self.ax_mot.set_ylabel('Speed [RPM]')
        self.ax_mot.grid(True)
        
        # Initial empty plots
        self.traj_plot, = self.ax_3d.plot([], [], [], 'b-', label='Trajectory')
        self.quad_scatter = self.ax_3d.scatter([], [], [], s=100, c='r', marker='o')
        
        # Position plots
        self.pos_x_plot, = self.ax_pos.plot([], [], 'r-', label='X')
        self.pos_y_plot, = self.ax_pos.plot([], [], 'g-', label='Y')
        self.pos_z_plot, = self.ax_pos.plot([], [], 'b-', label='Z')
        self.ax_pos.legend()
        
        # Attitude plots
        self.att_roll_plot, = self.ax_att.plot([], [], 'r-', label='Roll')
        self.att_pitch_plot, = self.ax_att.plot([], [], 'g-', label='Pitch')
        self.att_yaw_plot, = self.ax_att.plot([], [], 'b-', label='Yaw')
        self.ax_att.legend()
        
        # Motor speed plots
        self.mot1_plot, = self.ax_mot.plot([], [], 'r-', label='M1 (FR)')
        self.mot2_plot, = self.ax_mot.plot([], [], 'g-', label='M2 (FL)')
        self.mot3_plot, = self.ax_mot.plot([], [], 'b-', label='M3 (BL)')
        self.mot4_plot, = self.ax_mot.plot([], [], 'y-', label='M4 (BR)')
        self.ax_mot.legend()
        
        # Text display for motor speeds
        self.motor_text = self.fig.text(0.01, 0.01, '', fontsize=10)
        
        # Status text
        self.status_text = self.fig.text(0.5, 0.01, '', fontsize=10, ha='center')
        
        # Add controls text
        controls_text = "Controls:\n"
        controls_text += "1/Q: Motor 1 (FR) up/down | 2/W: Motor 2 (FL) up/down\n"
        controls_text += "3/E: Motor 3 (BL) up/down | 4/R: Motor 4 (BR) up/down\n"
        controls_text += "H: Hover | Z: Zero | Space: Pause | Esc: Quit"
        self.fig.text(0.5, 0.95, controls_text, fontsize=10, ha='center')
        
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    
    def update_visualization(self, frame):
        """Update visualization with current simulation state."""
        # Skip a few frames if we're at the beginning
        if self.history_idx < 10:
            return self.traj_plot, self.quad_scatter, self.status_text
        
        # Get valid indices for plotting
        if self.history_idx <= self.history_length:
            # We haven't filled the buffer yet
            valid_indices = slice(0, self.history_idx)
            last_idx = self.history_idx - 1
        else:
            # Buffer is full and wrapping around
            newest_idx = (self.history_idx - 1) % self.history_length
            if newest_idx < self.history_length - 1:
                # Data is in two segments
                valid_indices = list(range(newest_idx + 1, self.history_length))
                valid_indices.extend(range(0, newest_idx + 1))
            else:
                # Data is contiguous
                valid_indices = slice(0, self.history_length)
            last_idx = newest_idx
        
        # Get current position
        current_pos = self.position_history[last_idx]
        
        # Update trajectory plot
        positions = self.position_history[valid_indices]
        # Filter out NaN/Inf values
        valid_mask = ~np.isnan(positions).any(axis=1) & ~np.isinf(positions).any(axis=1)
        if np.any(valid_mask):
            clean_positions = positions[valid_mask]
            self.traj_plot.set_data(clean_positions[:, 0], clean_positions[:, 1])
            self.traj_plot.set_3d_properties(clean_positions[:, 2])
            
            # Update quadcopter position marker
            if not (np.isnan(current_pos).any() or np.isinf(current_pos).any()):
                self.quad_scatter._offsets3d = ([current_pos[0]], [current_pos[1]], [current_pos[2]])
                
                # Adjust axis limits if needed
                self.ax_3d.set_xlim(min(-2, current_pos[0] - 2), max(2, current_pos[0] + 2))
                self.ax_3d.set_ylim(min(-2, current_pos[1] - 2), max(2, current_pos[1] + 2))
                self.ax_3d.set_zlim(max(0, current_pos[2] - 2), max(4, current_pos[2] + 2))
        
        # Update time plots
        times = self.time_history[valid_indices]
        
        # Position plot
        self.pos_x_plot.set_data(times, positions[:, 0])
        self.pos_y_plot.set_data(times, positions[:, 1])
        self.pos_z_plot.set_data(times, positions[:, 2])
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()
        
        # Attitude plot
        attitudes = self.attitude_history[valid_indices]
        self.att_roll_plot.set_data(times, np.degrees(attitudes[:, 0]))
        self.att_pitch_plot.set_data(times, np.degrees(attitudes[:, 1]))
        self.att_yaw_plot.set_data(times, np.degrees(attitudes[:, 2]))
        self.ax_att.relim()
        self.ax_att.autoscale_view()
        
        # Motor speeds plot
        motor_speeds = self.motor_speeds_history[valid_indices]
        rpm_conversion = 60 / (2 * np.pi)
        self.mot1_plot.set_data(times, motor_speeds[:, 0] * rpm_conversion)
        self.mot2_plot.set_data(times, motor_speeds[:, 1] * rpm_conversion)
        self.mot3_plot.set_data(times, motor_speeds[:, 2] * rpm_conversion)
        self.mot4_plot.set_data(times, motor_speeds[:, 3] * rpm_conversion)
        self.ax_mot.relim()
        self.ax_mot.autoscale_view()
        
        # Update motor speeds text
        motor_text = f"Motor Speeds (RPM):\n"
        motor_text += f"M1 (FR): {self.motor_speeds[0] * rpm_conversion:.0f}\n"
        motor_text += f"M2 (FL): {self.motor_speeds[1] * rpm_conversion:.0f}\n"
        motor_text += f"M3 (BL): {self.motor_speeds[2] * rpm_conversion:.0f}\n"
        motor_text += f"M4 (BR): {self.motor_speeds[3] * rpm_conversion:.0f}\n"
        motor_text += f"Hover: {self.hover_motor_speed * rpm_conversion:.0f}"
        self.motor_text.set_text(motor_text)
        
        # Update status text
        status = "Running" if not self.paused else "PAUSED"
        status_text = f"Status: {status} | Time: {self.sim_time:.1f}s"
        self.status_text.set_text(status_text)
        
        return self.traj_plot, self.quad_scatter, self.status_text
    
    def handle_key(self, event):
        """Handle keyboard input for interactive control."""
        key = event.key.lower()
        
        if key in self.key_bindings:
            action = self.key_bindings[key]
            
            # Handle special actions
            if 'special' in action:
                if action['special'] == 'pause':
                    self.paused = not self.paused
                    print(f"Simulation {'paused' if self.paused else 'resumed'}")
                elif action['special'] == 'hover':
                    self.motor_speeds = np.ones(4) * self.hover_motor_speed
                    print("Motors set to hover speed")
                elif action['special'] == 'zero':
                    self.motor_speeds = np.ones(4) * self.min_speed
                    print("Motors set to minimum speed")
                elif action['special'] == 'reset':
                    self.reset_simulation()
                    print("Simulation reset")
                elif action['special'] == 'quit':
                    self.running = False
                    plt.close(self.fig)
                    print("Quitting simulation")
            
            # Handle motor speed changes
            elif 'motor' in action:
                motor_idx = action['motor']
                change = action['change']
                self.motor_speeds[motor_idx] += change
                self.motor_speeds[motor_idx] = np.clip(
                    self.motor_speeds[motor_idx], 
                    self.min_speed, 
                    self.max_speed
                )
                print(f"Motor {motor_idx+1} speed: {self.motor_speeds[motor_idx] * 60 / (2 * np.pi):.0f} RPM")
    
    def keyboard_control_thread(self):
        """Thread function for keyboard control handling."""
        print("Keyboard control thread started")
        print("Press Esc to quit")
        
        # Register key event handlers
        for key in self.key_bindings:
            if len(key) == 1:  # Skip special keys like 'esc'
                keyboard.add_hotkey(key, lambda k=key: self.process_key_press(k))
        
        # Special keys
        keyboard.add_hotkey('space', lambda: self.process_key_press('space'))
        keyboard.add_hotkey('esc', lambda: self.process_key_press('esc'))
        
        # Wait until running is False
        while self.running:
            time.sleep(0.1)
    
    def process_key_press(self, key):
        """Process a key press from the keyboard library."""
        if key in self.key_bindings:
            action = self.key_bindings[key]
            
            # Handle special actions
            if 'special' in action:
                if action['special'] == 'pause':
                    self.paused = not self.paused
                    print(f"Simulation {'paused' if self.paused else 'resumed'}")
                elif action['special'] == 'hover':
                    self.motor_speeds = np.ones(4) * self.hover_motor_speed
                    print("Motors set to hover speed")
                elif action['special'] == 'zero':
                    self.motor_speeds = np.ones(4) * self.min_speed
                    print("Motors set to minimum speed")
                elif action['special'] == 'reset':
                    self.reset_simulation()
                    print("Simulation reset")
                elif action['special'] == 'quit':
                    self.running = False
                    print("Quitting simulation")
            
            # Handle motor speed changes
            elif 'motor' in action:
                motor_idx = action['motor']
                change = action['change']
                self.motor_speeds[motor_idx] += change
                self.motor_speeds[motor_idx] = np.clip(
                    self.motor_speeds[motor_idx], 
                    self.min_speed, 
                    self.max_speed
                )
                print(f"Motor {motor_idx+1} speed: {self.motor_speeds[motor_idx] * 60 / (2 * np.pi):.0f} RPM")
    
    def reset_simulation(self):
        """Reset the simulation to initial state."""
        self.quad.reset_state()
        self.quad.set_state(position=np.array([0.0, 0.0, 0.1]))
        self.sim_time = 0.0
        self.history_idx = 0
        
        # Reset history arrays
        self.time_history = np.zeros(self.history_length)
        self.position_history = np.zeros((self.history_length, 3))
        self.attitude_history = np.zeros((self.history_length, 3))
        self.motor_speeds_history = np.zeros((self.history_length, 4))
        
        # Reset motor speeds to just below hover
        self.motor_speeds = np.ones(4) * self.hover_motor_speed * 0.95
    
    def simulation_thread(self):
        """Run the simulation in a separate thread."""
        print("Simulation thread started")
        while self.running:
            self.update_simulation()
            time.sleep(self.dt / 2)  # Sleep for less than dt to ensure real-time performance
    
    def print_instructions(self):
        """Print control instructions to console."""
        print("\n=== Interactive Quadcopter Simulator ===")
        print(f"Quadcopter Model: {self.params['name']}")
        print(f"Mass: {self.params['mass']} kg")
        print(f"Hover motor speed: {self.hover_motor_speed * 60 / (2 * np.pi):.0f} RPM")
        print("\nControl Instructions:")
        print("- Number keys (1-4): Increase individual motor speeds")
        print("- Letter keys (Q,W,E,R): Decrease individual motor speeds")
        print("- H: Set all motors to hover speed")
        print("- Z: Set all motors to minimum speed")
        print("- Space: Pause/Resume simulation")
        print("- Esc: Quit simulation")
        print("\nMotor Configuration (Top View):")
        print("     Front")
        print("  2(FL) (FR)1")
        print("       X")
        print("  3(BL) (BR)4")
        print("     Back")
        print("\nWatch the plots to see how the quadcopter responds!\n")
    
    def run(self):
        """Start and run the interactive simulation."""
        self.print_instructions()
        
        # Set up visualization
        self.setup_visualization()
        
        # Start simulation
        self.running = True
        
        # Create and start simulation thread
        sim_thread = threading.Thread(target=self.simulation_thread)
        sim_thread.daemon = True
        sim_thread.start()
        
        # Create and start keyboard control thread
        keyboard_thread = threading.Thread(target=self.keyboard_control_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        # Start animation
        self.animation = FuncAnimation(
            self.fig, self.update_visualization,
            interval=50, blit=False
        )
        
        # Display the plot (this will block until the window is closed)
        plt.show()
        
        # When plot is closed, stop simulation
        self.running = False
        
        # Wait for threads to finish
        sim_thread.join(timeout=1.0)
        keyboard_thread.join(timeout=1.0)
        
        print("Simulation ended")

def main():
    """Main function to run the interactive quadcopter simulation."""
    parser = argparse.ArgumentParser(description="Interactive Quadcopter Simulator")
    parser.add_argument("--model", choices=get_available_quadcopter_models(), default="default",
                      help="Quadcopter model to simulate")
    
    args = parser.parse_args()
    
    # Create and run simulation
    sim = InteractiveQuadcopterSim(model_name=args.model)
    sim.run()

if __name__ == "__main__":
    main() 