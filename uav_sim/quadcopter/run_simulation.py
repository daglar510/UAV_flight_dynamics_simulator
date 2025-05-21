"""
Quadcopter simulation runner.
"""

import numpy as np
import matplotlib.pyplot as plt
from uav_sim.quadcopter.models import get_quadcopter_parameters, get_available_quadcopter_models
from uav_sim.quadcopter.dynamics import QuadcopterDynamics
from uav_sim.quadcopter.controller import QuadcopterController
from uav_sim.quadcopter.simulate import QuadcopterSimulation, TrajectoryGenerator
from uav_sim.quadcopter.visualize import QuadcopterVisualizer

def run_quadcopter_simulation(model_name, duration, trajectory_type, setpoint=None, noise_level=0.05):
    """
    Run a complete quadcopter simulation.
    
    Args:
        model_name (str): Name of the quadcopter model
        duration (float): Simulation duration
        trajectory_type (str): Type of trajectory (hover, square, circle, custom)
        setpoint (list, optional): Custom setpoint [x, y, z]
        noise_level (float, optional): Level of noise in the simulation
        
    Returns:
        dict: Simulation history
    """
    # Get quadcopter parameters
    quad_params = get_quadcopter_parameters(model_name)
    print(f"Using quadcopter model: {quad_params['name']}")
    
    # Create simulator
    dt = 0.01  # 100 Hz simulation
    sim = QuadcopterSimulation(quad_params, dt=dt, noise_level=noise_level)
    
    # Set initial position slightly off the ground
    sim.set_initial_state(position=np.array([0.0, 0.0, 0.1]))
    
    # Generate trajectory
    if trajectory_type == "hover":
        height = setpoint[2] if setpoint else 1.0
        trajectory = TrajectoryGenerator.hover(duration, height=height, dt=dt)
    elif trajectory_type == "square":
        height = setpoint[2] if setpoint else 1.0
        trajectory = TrajectoryGenerator.square(duration, side_length=2.0, height=height, dt=dt)
    elif trajectory_type == "circle":
        height = setpoint[2] if setpoint else 1.0
        trajectory = TrajectoryGenerator.circle(duration, radius=1.0, height=height, dt=dt)
    elif trajectory_type == "custom" and setpoint:
        # Custom setpoint
        trajectory = TrajectoryGenerator.hover(duration, height=setpoint[2], dt=dt)
        # Override x, y coordinates
        trajectory['x'] = np.ones_like(trajectory['x']) * setpoint[0]
        trajectory['y'] = np.ones_like(trajectory['y']) * setpoint[1]
    else:
        raise ValueError(f"Invalid trajectory type: {trajectory_type}")
    
    # Print simulation parameters
    print(f"Running {quad_params['name']} quadcopter simulation:")
    print(f"  - Duration: {duration} seconds")
    print(f"  - Trajectory: {trajectory_type}")
    if trajectory_type == "custom" or trajectory_type == "hover":
        print(f"  - Setpoint: [{setpoint[0]}, {setpoint[1]}, {setpoint[2]}]")
    print(f"  - Noise level: {noise_level}")
    
    # Run the simulation
    print("Simulating...")
    
    # Follow trajectory
    num_steps = len(trajectory['time'])
    for i in range(num_steps):
        # Set setpoint for current time
        sim.set_setpoint(
            x=trajectory['x'][i],
            y=trajectory['y'][i],
            z=trajectory['z'][i],
            yaw=trajectory['yaw'][i]
        )
        
        # Update simulation
        sim.update()
    
    print("Simulation complete!")
    
    # Get simulation history
    history = sim.get_history()
    
    return history

def visualize_results(history, show_plots=True, save_plots=False):
    """
    Visualize simulation results.
    
    Args:
        history (dict): Simulation history
        show_plots (bool): Whether to display the plots
        save_plots (bool): Whether to save the plots
    """
    # Create visualizer
    vis = QuadcopterVisualizer(history)
    
    # Create individual plots instead of using plot_all
    figures = {}
    
    # Generate each plot
    print("Generating plots...")
    figures['trajectory_3d'] = vis.plot_trajectory_3d(show=False)
    figures['position'] = vis.plot_position(show=False)
    figures['attitude'] = vis.plot_attitude(show=False)
    figures['motors'] = vis.plot_motors(show=False)
    
    # Save plots if requested
    if save_plots:
        print("Saving plots...")
        for name, fig in figures.items():
            filename = f"quadcopter_{name}.png"
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"  Saved {filename}")
    
    # Show all plots at once
    if show_plots:
        print("Displaying plots...")
        plt.show(block=True)  # Block until all windows are closed

def main():
    """
    Main function to run from command line.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Run quadcopter simulation")
    parser.add_argument("--model", choices=get_available_quadcopter_models(), default="default",
                        help="Quadcopter model name")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Simulation duration in seconds")
    parser.add_argument("--trajectory", choices=["hover", "square", "circle", "custom"], default="hover",
                        help="Trajectory to follow")
    parser.add_argument("--setpoint-x", type=float, default=0.0,
                        help="Target x position in meters")
    parser.add_argument("--setpoint-y", type=float, default=0.0,
                        help="Target y position in meters")
    parser.add_argument("--setpoint-z", type=float, default=1.0,
                        help="Target z position in meters")
    parser.add_argument("--noise", type=float, default=0.05,
                        help="Noise level (0.0 to 1.0)")
    parser.add_argument("--save-plots", action="store_true",
                        help="Save plots to files")
    
    args = parser.parse_args()
    
    setpoint = [args.setpoint_x, args.setpoint_y, args.setpoint_z]
    
    history = run_quadcopter_simulation(
        args.model,
        args.duration,
        args.trajectory,
        setpoint,
        args.noise
    )
    
    visualize_results(history, show_plots=True, save_plots=args.save_plots)

if __name__ == "__main__":
    main() 