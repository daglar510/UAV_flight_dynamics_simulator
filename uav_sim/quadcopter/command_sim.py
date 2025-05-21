"""
Command-line Interactive Quadcopter Simulation.

This module provides a simple command-line interface to control
motor speeds and observe quadcopter dynamics without keyboard dependencies.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import threading
import argparse
import cmd
import os
import re

from uav_sim.quadcopter.models import get_quadcopter_parameters, get_available_quadcopter_models
from uav_sim.quadcopter.dynamics import QuadcopterDynamics
from uav_sim.constants import GRAVITY

class QuadcopterCommandSim:
    """Command-line interactive quadcopter simulation."""
    
    def __init__(self, model_name="default", dt=0.02, history_length=500):
        """
        Initialize the command-line interactive quadcopter simulation.
        
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
        
        # Control flags
        self.running = False
        self.paused = False
        
        # Visualization
        self.fig = None
        self.animation = None
        
        # Motor speed limits
        self.min_speed = self.quad.min_motor_speed
        self.max_speed = self.quad.max_motor_speed
        
        # RPM conversion factor
        self.rpm_conversion = 60 / (2 * np.pi)
    
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
        """Set up the visualization window."""
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
        
        # Add info text
        info_text = f"Quadcopter Model: {self.params['name']} | " 
        info_text += f"Mass: {self.params['mass']:.2f} kg | "
        info_text += f"Hover: {self.hover_motor_speed * self.rpm_conversion:.0f} RPM"
        self.fig.text(0.5, 0.95, info_text, fontsize=10, ha='center')
        
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
        
        try:
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
            self.mot1_plot.set_data(times, motor_speeds[:, 0] * self.rpm_conversion)
            self.mot2_plot.set_data(times, motor_speeds[:, 1] * self.rpm_conversion)
            self.mot3_plot.set_data(times, motor_speeds[:, 2] * self.rpm_conversion)
            self.mot4_plot.set_data(times, motor_speeds[:, 3] * self.rpm_conversion)
            self.ax_mot.relim()
            self.ax_mot.autoscale_view()
            
            # Update motor speeds text
            motor_text = f"Motor Speeds (RPM):\n"
            motor_text += f"M1 (FR): {self.motor_speeds[0] * self.rpm_conversion:.0f}\n"
            motor_text += f"M2 (FL): {self.motor_speeds[1] * self.rpm_conversion:.0f}\n"
            motor_text += f"M3 (BL): {self.motor_speeds[2] * self.rpm_conversion:.0f}\n"
            motor_text += f"M4 (BR): {self.motor_speeds[3] * self.rpm_conversion:.0f}"
            self.motor_text.set_text(motor_text)
        except Exception as e:
            print(f"Visualization update error: {e}")
            
        # Update status text
        status = "Running" if not self.paused else "PAUSED"
        status_text = f"Status: {status} | Time: {self.sim_time:.1f}s"
        self.status_text.set_text(status_text)
        
        return self.traj_plot, self.quad_scatter, self.status_text
    
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
        print("\n=== Command-line Quadcopter Simulator ===")
        print(f"Quadcopter Model: {self.params['name']}")
        print(f"Mass: {self.params['mass']} kg")
        print(f"Hover motor speed: {self.hover_motor_speed * self.rpm_conversion:.0f} RPM")
        print("\nAvailable Commands:")
        print("  m1 <rpm> - Set Motor 1 (Front-Right) speed in RPM")
        print("  m2 <rpm> - Set Motor 2 (Front-Left) speed in RPM")
        print("  m3 <rpm> - Set Motor 3 (Back-Left) speed in RPM")
        print("  m4 <rpm> - Set Motor 4 (Back-Right) speed in RPM")
        print("  all <rpm> - Set all motors to the same speed in RPM")
        print("  batch <m1> <m2> <m3> <m4> - Set all motor speeds at once in RPM")
        print("  hover - Set all motors to hover speed")
        print("  zero - Set all motors to minimum speed")
        print("  pause - Pause simulation")
        print("  resume - Resume simulation")
        print("  reset - Reset simulation")
        print("  state - Print current quadcopter state")
        print("  run <seconds> - Run simulation for specified number of seconds")
        print("  script <filename> - Run commands from a script file")
        print("  quit - Quit simulation")
        print("\nMotor Configuration (Top View):")
        print("     Front")
        print("  2(FL) (FR)1")
        print("       X")
        print("  3(BL) (BR)4")
        print("     Back")
        print("\nWatch the plots to see how the quadcopter responds!\n")
    
    def start(self):
        """Start the simulation threads and visualization."""
        self.print_instructions()
        
        # Set up visualization
        self.setup_visualization()
        
        # Start simulation
        self.running = True
        
        # Create and start simulation thread
        sim_thread = threading.Thread(target=self.simulation_thread)
        sim_thread.daemon = True
        sim_thread.start()
        
        # Start animation
        self.animation = FuncAnimation(
            self.fig, self.update_visualization,
            interval=50, blit=False, save_count=100
        )
        
        # Show the plot in non-blocking mode
        plt.ion()  # Turn on interactive mode
        plt.show()
        
        # Return the simulation thread
        return sim_thread
    
    def stop(self):
        """Stop the simulation."""
        self.running = False
        plt.close(self.fig)
        print("Simulation ended")
    
    def set_motor_speed_rpm(self, motor_idx, rpm):
        """Set motor speed in RPM with checks and conversion."""
        # Convert RPM to rad/s
        rad_per_sec = rpm * (2 * np.pi / 60)
        
        # Apply limits
        speed = np.clip(rad_per_sec, self.min_speed, self.max_speed)
        
        # Set the motor speed
        self.motor_speeds[motor_idx] = speed
        
        # Return the actual RPM (might be different due to limits)
        return speed * self.rpm_conversion

class QuadcopterCommandShell(cmd.Cmd):
    """Interactive command shell for quadcopter simulation."""
    
    intro = 'Welcome to the quadcopter simulator. Type help or ? to list commands.\n'
    prompt = 'quad> '
    
    def __init__(self, sim):
        """
        Initialize the command shell.
        
        Args:
            sim (QuadcopterCommandSim): The simulation object
        """
        super().__init__()
        self.sim = sim
        
        # Start simulation
        self.sim_thread = self.sim.start()
    
    def do_m1(self, arg):
        """Set Motor 1 (Front-Right) speed in RPM: m1 <rpm>"""
        try:
            rpm = float(arg)
            actual_rpm = self.sim.set_motor_speed_rpm(0, rpm)
            print(f"Motor 1 (Front-Right) set to {actual_rpm:.0f} RPM")
        except ValueError:
            print("Error: Please provide a valid RPM value")
    
    def do_m2(self, arg):
        """Set Motor 2 (Front-Left) speed in RPM: m2 <rpm>"""
        try:
            rpm = float(arg)
            actual_rpm = self.sim.set_motor_speed_rpm(1, rpm)
            print(f"Motor 2 (Front-Left) set to {actual_rpm:.0f} RPM")
        except ValueError:
            print("Error: Please provide a valid RPM value")
    
    def do_m3(self, arg):
        """Set Motor 3 (Back-Left) speed in RPM: m3 <rpm>"""
        try:
            rpm = float(arg)
            actual_rpm = self.sim.set_motor_speed_rpm(2, rpm)
            print(f"Motor 3 (Back-Left) set to {actual_rpm:.0f} RPM")
        except ValueError:
            print("Error: Please provide a valid RPM value")
    
    def do_m4(self, arg):
        """Set Motor 4 (Back-Right) speed in RPM: m4 <rpm>"""
        try:
            rpm = float(arg)
            actual_rpm = self.sim.set_motor_speed_rpm(3, rpm)
            print(f"Motor 4 (Back-Right) set to {actual_rpm:.0f} RPM")
        except ValueError:
            print("Error: Please provide a valid RPM value")
    
    def do_all(self, arg):
        """Set all motors to the same speed in RPM: all <rpm>"""
        try:
            rpm = float(arg)
            rad_per_sec = rpm * (2 * np.pi / 60)
            speed = np.clip(rad_per_sec, self.sim.min_speed, self.sim.max_speed)
            self.sim.motor_speeds = np.ones(4) * speed
            actual_rpm = speed * self.sim.rpm_conversion
            print(f"All motors set to {actual_rpm:.0f} RPM")
        except ValueError:
            print("Error: Please provide a valid RPM value")
    
    def do_batch(self, arg):
        """Set all motor speeds at once: batch <m1_rpm> <m2_rpm> <m3_rpm> <m4_rpm>"""
        try:
            values = arg.split()
            if len(values) != 4:
                print("Error: Please provide exactly 4 RPM values")
                return
            
            rpms = [float(val) for val in values]
            actual_rpms = []
            
            for i, rpm in enumerate(rpms):
                actual_rpm = self.sim.set_motor_speed_rpm(i, rpm)
                actual_rpms.append(actual_rpm)
            
            print(f"Motor speeds set to: M1={actual_rpms[0]:.0f}, M2={actual_rpms[1]:.0f}, M3={actual_rpms[2]:.0f}, M4={actual_rpms[3]:.0f} RPM")
        except ValueError:
            print("Error: Invalid RPM values")
    
    def do_hover(self, arg):
        """Set all motors to hover speed"""
        hover_rpm = self.sim.hover_motor_speed * (60 / (2 * np.pi))
        self.sim.motor_speeds = np.ones(4) * self.sim.hover_motor_speed
        print(f"All motors set to hover speed ({hover_rpm:.0f} RPM)")
    
    def do_zero(self, arg):
        """Set all motors to minimum speed"""
        min_rpm = self.sim.min_speed * (60 / (2 * np.pi))
        self.sim.motor_speeds = np.ones(4) * self.sim.min_speed
        print(f"All motors set to minimum speed ({min_rpm:.0f} RPM)")
    
    def do_pause(self, arg):
        """Pause the simulation"""
        self.sim.paused = True
        print("Simulation paused")
    
    def do_resume(self, arg):
        """Resume the simulation"""
        self.sim.paused = False
        print("Simulation resumed")
    
    def do_reset(self, arg):
        """Reset the simulation"""
        self.sim.reset_simulation()
        print("Simulation reset")
    
    def do_run(self, arg):
        """Run simulation for specified number of seconds: run <seconds>"""
        try:
            seconds = float(arg)
            if seconds <= 0:
                print("Error: Run time must be positive")
                return
                
            print(f"Running simulation for {seconds:.1f} seconds...")
            self.sim.paused = False
            
            # Store the current time
            start_time = self.sim.sim_time
            target_time = start_time + seconds
            
            # Wait until the simulation reaches the target time
            while self.sim.running and self.sim.sim_time < target_time:
                print(f"  Time: {self.sim.sim_time:.1f}/{target_time:.1f} seconds", end='\r')
                time.sleep(0.1)
            
            print(f"\nCompleted {self.sim.sim_time - start_time:.1f} seconds of simulation")
        except ValueError:
            print("Error: Please provide a valid number of seconds")
    
    def do_script(self, arg):
        """Run commands from a script file: script <filename>"""
        try:
            with open(arg, 'r') as f:
                print(f"Running commands from {arg}...")
                for line in f:
                    line = line.strip()
                    
                    # Skip empty lines and comments
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse the command
                    print(f">>> {line}")
                    self.onecmd(line)
                    
                    # Small delay between commands
                    time.sleep(0.1)
                    
                print(f"Script {arg} completed")
        except FileNotFoundError:
            print(f"Error: File '{arg}' not found")
        except Exception as e:
            print(f"Error executing script: {e}")
    
    def do_state(self, arg):
        """Print current quadcopter state"""
        state = self.sim.quad.get_state()
        print("\nCurrent Quadcopter State:")
        print(f"Time: {self.sim.sim_time:.2f} seconds")
        print(f"Position (x,y,z): [{state['position'][0]:.2f}, {state['position'][1]:.2f}, {state['position'][2]:.2f}] m")
        
        # Convert attitude to degrees for display
        roll = np.degrees(state['attitude'][0])
        pitch = np.degrees(state['attitude'][1])
        yaw = np.degrees(state['attitude'][2])
        print(f"Attitude (roll,pitch,yaw): [{roll:.2f}, {pitch:.2f}, {yaw:.2f}] degrees")
        
        # Convert angular velocity to degrees/s for display
        roll_rate = np.degrees(state['angular_velocity'][0])
        pitch_rate = np.degrees(state['angular_velocity'][1])
        yaw_rate = np.degrees(state['angular_velocity'][2])
        print(f"Angular Velocity: [{roll_rate:.2f}, {pitch_rate:.2f}, {yaw_rate:.2f}] deg/s")
        
        # Display motor speeds in RPM
        rpm_conversion = 60 / (2 * np.pi)
        print("\nMotor Speeds (RPM):")
        print(f"Motor 1 (Front-Right): {state['motor_speeds'][0] * rpm_conversion:.0f}")
        print(f"Motor 2 (Front-Left): {state['motor_speeds'][1] * rpm_conversion:.0f}")
        print(f"Motor 3 (Back-Left): {state['motor_speeds'][2] * rpm_conversion:.0f}")
        print(f"Motor 4 (Back-Right): {state['motor_speeds'][3] * rpm_conversion:.0f}")
        print(f"Hover speed: {self.sim.hover_motor_speed * rpm_conversion:.0f}")
    
    def do_quit(self, arg):
        """Quit the simulator"""
        self.sim.stop()
        print("Quitting simulator...")
        return True
    
    def do_exit(self, arg):
        """Exit the simulator (alias for quit)"""
        return self.do_quit(arg)
    
    def default(self, line):
        """Handle unknown commands."""
        # Check if it's a motor speed pattern like "m1=5000 m2=4500"
        motor_pattern = re.compile(r'm([1-4])=(\d+(\.\d+)?)')
        matches = motor_pattern.findall(line)
        
        if matches:
            # Process each motor setting
            for match in matches:
                motor_num = int(match[0])
                rpm = float(match[1])
                
                # Call the appropriate motor command
                if 1 <= motor_num <= 4:
                    cmd = f"m{motor_num} {rpm}"
                    print(f"Executing: {cmd}")
                    self.onecmd(cmd)
            return
            
        print(f"Unknown command: {line}")
        print("Type 'help' or '?' to see available commands.")

def run_command_sim(model_name="default"):
    """
    Run the command-line interactive quadcopter simulator.
    
    Args:
        model_name (str): Name of the quadcopter model
    """
    # Create simulation object
    sim = QuadcopterCommandSim(model_name=model_name)
    
    # Create and run command shell
    shell = QuadcopterCommandShell(sim)
    shell.cmdloop()

def main():
    """Main function to run the command-line interactive quadcopter simulator."""
    parser = argparse.ArgumentParser(description="Command-line Interactive Quadcopter Simulator")
    parser.add_argument("--model", choices=get_available_quadcopter_models(), default="default",
                      help="Quadcopter model to simulate")
    
    args = parser.parse_args()
    
    run_command_sim(model_name=args.model)

if __name__ == "__main__":
    main() 