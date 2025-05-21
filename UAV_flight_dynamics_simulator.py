#!/usr/bin/env python3
"""
UAV Flight Dynamics Simulator - Unified Interface

Main entry point for all UAV flight simulators:
- Quadcopter Command Simulator: Interactive motor control with command interface
- Quadcopter Trajectory Simulator: Predefined flight patterns with parameters set upfront
- Fixed-Wing UAV Simulator: Elevator input response for various aircraft models
"""

import os
import sys
import argparse

def run_quadcopter_command_simulator(model="default", script=None):
    """Run interactive quadcopter simulator with direct motor control."""
    try:
        from uav_sim.quadcopter.models import get_quadcopter_parameters
        from uav_sim.quadcopter.command_sim import run_command_sim, QuadcopterCommandSim, QuadcopterCommandShell
        
        # Get model parameters for more detailed information
        params = get_quadcopter_parameters(model)
        
        print("\n=== Command-line Quadcopter Simulator ===")
        print(f"Model: {model} ({params['name']})")
        print(f"Mass: {params['mass']:.2f} kg")
        print(f"Arm length: {params['arm_length']:.3f} m")
        
        if script:
            # If a script file is provided, run it directly
            if not os.path.exists(script):
                print(f"Error: Script file '{script}' not found.")
                return False
                
            print(f"\nRunning commands from script: {script}")
            
            # Create simulation object and command shell
            sim = QuadcopterCommandSim(model_name=model)
            shell = QuadcopterCommandShell(sim)
            
            # Process the script file
            try:
                print("\nExecuting script commands...")
                shell.do_script(script)
                print("\nScript execution completed.")
                print("Simulator will remain open. Type 'quit' to exit.")
                shell.cmdloop()
            except Exception as e:
                print(f"Error executing script: {e}")
                return False
        else:
            # Normal interactive mode
            print("\nThis simulator lets you type commands to directly control motor speeds.")
            print("A visualization window will open to show the quadcopter's behavior.")
            print("\nUse commands like 'm1 5000' to set motor 1 to 5000 RPM.")
            print("Type 'help' for a full list of available commands.\n")
            
            # Run the simulator
            run_command_sim(model_name=model)
        
        return True
    except ImportError as e:
        print(f"Error: {e}")
        return False

def run_quadcopter_trajectory_simulator(model="default", duration=30.0, maneuver="square"):
    """Run structured quadcopter simulator with predefined maneuvers."""
    try:
        from uav_sim.quadcopter.models import get_quadcopter_parameters
        from uav_sim.quadcopter.trajectory_sim import run_trajectory_simulation
        
        # Get model parameters for more detailed information
        params = get_quadcopter_parameters(model)
        
        print("\n=== Quadcopter Trajectory Simulator ===")
        print(f"Model: {model} ({params['name']})")
        print(f"Mass: {params['mass']:.2f} kg")
        print(f"Maneuver: {maneuver}")
        print(f"Duration: {duration} seconds")
        
        # Run simulation with visualization
        sim = run_trajectory_simulation(
            model_name=model, 
            duration=duration, 
            maneuver_type=maneuver
        )
        sim.visualize_results()
        return True
    except ImportError as e:
        print(f"Error: {e}")
        return False

def run_fixed_wing_simulator(model="TB2", duration=60.0, pulse_start=5.0, pulse_duration=10.0, pulse_angle=2.0):
    """Run fixed-wing UAV simulator with elevator inputs."""
    try:
        from uav_sim.uav.models import get_fixed_wing_parameters
        from uav_sim.uav.run_simulation import run_fixed_wing_simulation, visualize_results
        
        print("\n=== Fixed-Wing UAV Simulator ===")
        print(f"Model: {model}")
        print(f"Duration: {duration} seconds")
        print(f"Elevator pulse: {pulse_angle}° at t={pulse_start}s for {pulse_duration}s")
        
        # Define elevator pulse inputs
        pulses = [(pulse_start, pulse_duration, pulse_angle)]
        
        # Run simulation and visualize
        results = run_fixed_wing_simulation(model, pulses, duration)
        visualize_results(results, show_plots=True, save_plots=False)
        print("\nSimulation complete!")
        
        return True
    except ImportError as e:
        print(f"Error: {e}")
        return False
    except Exception as e:
        print(f"Error running fixed-wing simulator: {e}")
        return False

def show_main_menu():
    """Display interactive menu for simulator selection."""
    while True:
        print("\n=== UAV Flight Dynamics Simulator ===")
        print("1. Quadcopter Command Simulator (interactive)")
        print("2. Quadcopter Trajectory Simulator (predefined)")
        print("3. Fixed-Wing UAV Simulator")
        print("4. Exit")
        
        choice = input("\nSelect option (1-4): ")
        
        if choice == '1':
            # Select quadcopter model
            try:
                from uav_sim.quadcopter.models import get_available_quadcopter_models
                models = get_available_quadcopter_models()
                
                print("\nAvailable models:")
                for i, model in enumerate(models, 1):
                    print(f"{i}. {model}")
                
                model_choice = input(f"\nSelect model (1-{len(models)}) or Enter for default: ")
                model = 'default'
                if model_choice.strip():
                    try:
                        idx = int(model_choice) - 1
                        if 0 <= idx < len(models):
                            model = models[idx]
                    except ValueError:
                        pass
                
                # Ask about script file
                script_path = input("\nScript file path (optional, press Enter to skip): ")
                if script_path and not os.path.exists(script_path):
                    print(f"Warning: Script file '{script_path}' not found, proceeding without script.")
                    script_path = None
                
                run_quadcopter_command_simulator(model, script_path)
            except ImportError:
                print("Error loading quadcopter models")
                
        elif choice == '2':
            # Configure trajectory simulator
            try:
                from uav_sim.quadcopter.models import get_available_quadcopter_models
                models = get_available_quadcopter_models()
                maneuvers = ["hover", "square", "figure8", "yaw_test"]
                
                # Select model
                print("\nAvailable models:")
                for i, model in enumerate(models, 1):
                    print(f"{i}. {model}")
                model_choice = input(f"\nSelect model (1-{len(models)}) or Enter for default: ")
                model = 'default'
                if model_choice.strip():
                    try:
                        idx = int(model_choice) - 1
                        if 0 <= idx < len(models):
                            model = models[idx]
                    except ValueError:
                        pass
                
                # Select maneuver
                print("\nAvailable maneuvers:")
                for i, maneuver in enumerate(maneuvers, 1):
                    print(f"{i}. {maneuver}")
                maneuver_choice = input(f"\nSelect maneuver (1-{len(maneuvers)}) or Enter for square: ")
                maneuver = 'square'
                if maneuver_choice.strip():
                    try:
                        idx = int(maneuver_choice) - 1
                        if 0 <= idx < len(maneuvers):
                            maneuver = maneuvers[idx]
                    except ValueError:
                        pass
                
                # Select duration
                duration = 30.0
                duration_choice = input("\nSimulation duration in seconds (Enter for 30s): ")
                if duration_choice.strip():
                    try:
                        duration = float(duration_choice)
                        if duration <= 0:
                            duration = 30.0
                    except ValueError:
                        pass
                
                run_quadcopter_trajectory_simulator(model, duration, maneuver)
                
            except ImportError:
                print("Error loading quadcopter models")
                
        elif choice == '3':
            # Configure fixed-wing simulator
            try:
                from uav_sim.uav.models import get_available_fixed_wing_models
                models = get_available_fixed_wing_models()
                
                # Select model
                print("\nAvailable fixed-wing models:")
                for i, model in enumerate(models, 1):
                    print(f"{i}. {model}")
                model_choice = input(f"\nSelect model (1-{len(models)}) or Enter for TB2: ")
                model = 'TB2'
                if model_choice.strip():
                    try:
                        idx = int(model_choice) - 1
                        if 0 <= idx < len(models):
                            model = models[idx]
                    except ValueError:
                        pass
                
                # Set duration
                duration = 60.0
                duration_choice = input("\nSimulation duration in seconds (Enter for 60s): ")
                if duration_choice.strip():
                    try:
                        duration = float(duration_choice)
                        if duration <= 0:
                            duration = 60.0
                    except ValueError:
                        pass
                
                # Set elevator pulse parameters
                pulse_start = 5.0
                pulse_start_choice = input("\nElevator pulse start time (Enter for 5s): ")
                if pulse_start_choice.strip():
                    try:
                        pulse_start = float(pulse_start_choice)
                        if pulse_start < 0:
                            pulse_start = 5.0
                    except ValueError:
                        pass
                
                pulse_duration = 10.0
                pulse_duration_choice = input("\nElevator pulse duration (Enter for 10s): ")
                if pulse_duration_choice.strip():
                    try:
                        pulse_duration = float(pulse_duration_choice)
                        if pulse_duration <= 0:
                            pulse_duration = 10.0
                    except ValueError:
                        pass
                
                pulse_angle = 2.0
                pulse_angle_choice = input("\nElevator pulse angle in degrees (Enter for 2°): ")
                if pulse_angle_choice.strip():
                    try:
                        pulse_angle = float(pulse_angle_choice)
                    except ValueError:
                        pass
                
                run_fixed_wing_simulator(model, duration, pulse_start, pulse_duration, pulse_angle)
                
            except ImportError as e:
                print(f"Error loading fixed-wing models: {e}")
            
        elif choice == '4':
            print("Exiting simulator")
            return
            
        else:
            print("Invalid option")

def main():
    """Main entry function with unified CLI interface."""
    parser = argparse.ArgumentParser(description="UAV Flight Dynamics Simulator")
    
    # Simulator selection
    sim_group = parser.add_mutually_exclusive_group()
    sim_group.add_argument("--quadcopter-command", action="store_true", help="run interactive command simulator")
    sim_group.add_argument("--quadcopter-trajectory", action="store_true", help="run trajectory simulator")
    sim_group.add_argument("--fixed-wing", action="store_true", help="run fixed-wing simulator")
    
    # Common parameters
    parser.add_argument("--model", help="UAV model")
    parser.add_argument("--duration", type=float, help="simulation duration in seconds")
    
    # Command simulator parameters
    parser.add_argument("--script", type=str, help="path to command script file")
    
    # Trajectory simulator parameters
    parser.add_argument("--maneuver", choices=["hover", "square", "figure8", "yaw_test"], 
                      help="trajectory maneuver (default: square)")
    
    # Fixed-wing simulator parameters
    parser.add_argument("--pulse-start", type=float, help="elevator pulse start time in seconds")
    parser.add_argument("--pulse-duration", type=float, help="elevator pulse duration in seconds")  
    parser.add_argument("--pulse-angle", type=float, help="elevator pulse angle in degrees")
    
    args = parser.parse_args()
    
    # Direct simulator selection
    if args.quadcopter_command:
        model = args.model if args.model else "default"
        run_quadcopter_command_simulator(model, args.script)
        
    elif args.quadcopter_trajectory:
        model = args.model if args.model else "default"
        duration = args.duration if args.duration else 30.0
        maneuver = args.maneuver if args.maneuver else "square"
        run_quadcopter_trajectory_simulator(model, duration, maneuver)
        
    elif args.fixed_wing:
        model = args.model if args.model else "TB2"
        duration = args.duration if args.duration else 60.0
        pulse_start = args.pulse_start if args.pulse_start is not None else 5.0
        pulse_duration = args.pulse_duration if args.pulse_duration is not None else 10.0
        pulse_angle = args.pulse_angle if args.pulse_angle is not None else 2.0
        run_fixed_wing_simulator(model, duration, pulse_start, pulse_duration, pulse_angle)
        
    else:
        # Show interactive menu
        show_main_menu()

if __name__ == "__main__":
    main()
