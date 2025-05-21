"""
Fixed-wing UAV simulation runner.
"""

import numpy as np
from uav_sim.uav.models import get_fixed_wing_parameters
from uav_sim.uav.dynamics import build_state_space, analyze_modes, print_modes_analysis
from uav_sim.uav.simulate import FixedWingSimulation
from uav_sim.uav.visualize import FixedWingVisualizer

def run_fixed_wing_simulation(uav_name, pulses, duration):
    """
    Run a complete fixed-wing UAV simulation.
    
    Args:
        uav_name (str): Name of the UAV model
        pulses (list): List of tuples (start_time, duration, elevator_angle_degrees)
        duration (float): Simulation duration in seconds
        
    Returns:
        dict: Simulation results
    """
    # Get UAV parameters
    uav_params = get_fixed_wing_parameters(uav_name)
    
    # Build state-space model
    A, B, U0 = build_state_space(uav_params)
    
    # Analyze modes
    modes = analyze_modes(A)
    print_modes_analysis(modes)
    print(f">>> Trim speed U0 = {U0:.2f} m/s")
    
    # Create and set up simulation
    sim = FixedWingSimulation(uav_params, A, B, U0)
    sim.set_simulation_duration(duration)
    
    # Add elevator pulses
    for start_time, pulse_duration, angle_deg in pulses:
        sim.add_elevator_pulse(start_time, pulse_duration, angle_deg)
    
    # Run simulation
    sim.run()
    
    # Get results
    results = sim.get_results()
    
    return results

def visualize_results(results, show_plots=True, save_plots=False):
    """
    Visualize simulation results.
    
    Args:
        results (dict): Simulation results
        show_plots (bool): Whether to display the plots
        save_plots (bool): Whether to save the plots
    """
    vis = FixedWingVisualizer(results)
    
    # Create plots
    vis.plot_time_response(show=show_plots)
    vis.plot_3d_trajectory(show=show_plots)
    
    # Save plots if requested
    if save_plots:
        vis.save_figures()

def main():
    """
    Main function to run from command line.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Run fixed-wing UAV simulation")
    parser.add_argument("--uav", type=str, default="TB2", help="UAV model name")
    parser.add_argument("--duration", type=float, default=60.0, help="Simulation duration in seconds")
    parser.add_argument("--pulse-start", type=float, default=5.0, help="Elevator pulse start time")
    parser.add_argument("--pulse-duration", type=float, default=10.0, help="Elevator pulse duration")
    parser.add_argument("--pulse-angle", type=float, default=2.0, help="Elevator pulse angle in degrees")
    parser.add_argument("--save-plots", action="store_true", help="Save plots to files")
    
    args = parser.parse_args()
    
    pulses = [(args.pulse_start, args.pulse_duration, args.pulse_angle)]
    
    results = run_fixed_wing_simulation(args.uav, pulses, args.duration)
    visualize_results(results, show_plots=True, save_plots=args.save_plots)

if __name__ == "__main__":
    main() 