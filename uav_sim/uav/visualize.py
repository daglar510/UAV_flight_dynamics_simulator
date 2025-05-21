"""
Fixed-wing UAV visualization module.
Contains functions for visualizing fixed-wing UAV simulation results.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from uav_sim.utils.common import rad2deg, set_plotting_style

def plot_response(t, y, de, name):
    """
    Plot the time-domain response of a fixed-wing UAV.
    
    Args:
        t (np.ndarray): Time array
        y (np.ndarray): State array
        de (np.ndarray): Elevator deflection array
        name (str): UAV name for plot title
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    fig = plt.figure(figsize=(15, 10))
    
    plt.subplot(3, 2, 1)
    plt.plot(t, y[0])
    plt.title(f"{name} – forward speed u")
    plt.grid()
    
    plt.subplot(3, 2, 2)
    plt.plot(t, np.degrees(y[1]))
    plt.title("AoA α [deg]")
    plt.grid()
    
    plt.subplot(3, 2, 3)
    plt.plot(t, np.degrees(y[2]))
    plt.title("Pitch rate q [deg/s]")
    plt.grid()
    
    plt.subplot(3, 2, 4)
    plt.plot(t, np.degrees(y[3]))
    plt.title("Pitch θ [deg]")
    plt.grid()
    
    plt.subplot(3, 2, 5)
    plt.plot(t, np.degrees(de))
    plt.title("Elevator δe [deg]")
    plt.grid()
    
    plt.tight_layout()
    return fig

def plot_3d_trajectory(y, name):
    """
    Plot a 3D trajectory of a fixed-wing UAV in (α, q, θ) space.
    
    Args:
        y (np.ndarray): State array
        name (str): UAV name for plot title
        
    Returns:
        matplotlib.figure.Figure: The created figure
    """
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(np.degrees(y[1]), np.degrees(y[2]), np.degrees(y[3]), label="Trajectory")
    ax.set_xlabel("AoA α [deg]")
    ax.set_ylabel("Pitch rate q [deg/s]")
    ax.set_zlabel("Pitch θ [deg]")
    ax.set_title(f"{name} – Trajectory in (α, q, θ) space")
    ax.legend()
    
    plt.tight_layout()
    return fig

class FixedWingVisualizer:
    """
    Class to handle fixed-wing UAV visualization.
    """
    
    def __init__(self, simulation_results=None):
        """
        Initialize the visualizer.
        
        Args:
            simulation_results (dict, optional): Simulation results from FixedWingSimulation
        """
        self.results = simulation_results
        self.figures = {}
    
    def set_results(self, simulation_results):
        """
        Set simulation results.
        
        Args:
            simulation_results (dict): Simulation results from FixedWingSimulation
        """
        self.results = simulation_results
    
    def plot_time_response(self, show=True):
        """
        Plot the time-domain response.
        
        Args:
            show (bool): Whether to display the plot
            
        Returns:
            matplotlib.figure.Figure: The created figure
            
        Raises:
            ValueError: If simulation results are not set
        """
        if self.results is None:
            raise ValueError("No simulation results to visualize")
        
        t = self.results['time']
        y = self.results['state']
        de = self.results['elevator']
        name = self.results['uav_name']
        
        fig = plot_response(t, y, de, name)
        self.figures['time_response'] = fig
        
        if show:
            plt.show()
        
        return fig
    
    def plot_3d_trajectory(self, show=True):
        """
        Plot the 3D trajectory.
        
        Args:
            show (bool): Whether to display the plot
            
        Returns:
            matplotlib.figure.Figure: The created figure
            
        Raises:
            ValueError: If simulation results are not set
        """
        if self.results is None:
            raise ValueError("No simulation results to visualize")
        
        y = self.results['state']
        name = self.results['uav_name']
        
        fig = plot_3d_trajectory(y, name)
        self.figures['3d_trajectory'] = fig
        
        if show:
            plt.show()
        
        return fig
    
    def save_figures(self, prefix="fixed_wing_"):
        """
        Save all generated figures.
        
        Args:
            prefix (str): Prefix for filenames
        """
        for name, fig in self.figures.items():
            filename = f"{prefix}{name}.png"
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Saved {filename}") 