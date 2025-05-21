"""
Common utility functions shared between fixed-wing and quadcopter simulations.
"""

import numpy as np
import matplotlib.pyplot as plt
import os

def create_output_directory(directory="output"):
    """
    Create an output directory if it doesn't exist.
    
    Args:
        directory (str): Directory name
        
    Returns:
        str: Path to the created directory
    """
    if not os.path.exists(directory):
        os.makedirs(directory)
    return directory

def save_data_to_csv(data_dict, filename, directory="output"):
    """
    Save simulation data to a CSV file.
    
    Args:
        data_dict (dict): Dictionary of data arrays
        filename (str): Output filename
        directory (str): Output directory
        
    Returns:
        str: Path to the saved file
    """
    import pandas as pd
    
    # Create output directory
    directory = create_output_directory(directory)
    
    # Convert data to pandas DataFrame
    df_dict = {}
    
    # Process each array in the dictionary
    for key, value in data_dict.items():
        if isinstance(value, np.ndarray):
            if value.ndim == 1:
                # 1D array
                df_dict[key] = value
            elif value.ndim == 2:
                # 2D array: create columns for each dimension
                for i in range(value.shape[1]):
                    df_dict[f"{key}_{i}"] = value[:, i]
    
    # Create DataFrame and save to CSV
    df = pd.DataFrame(df_dict)
    output_path = os.path.join(directory, filename)
    df.to_csv(output_path, index=False)
    
    return output_path

def set_plotting_style():
    """
    Set a consistent style for all plots.
    """
    plt.style.use('seaborn-v0_8-whitegrid')
    
    # Font sizes
    plt.rc('font', size=12)
    plt.rc('axes', titlesize=14, labelsize=12)
    plt.rc('legend', fontsize=11)
    
    # Line widths
    plt.rc('lines', linewidth=2.0)
    
    # Figure size
    plt.rc('figure', figsize=(10, 6))
    
    # Grid style
    plt.rc('grid', alpha=0.3)

def rad2deg(angle):
    """
    Convert radians to degrees.
    
    Args:
        angle (float or np.ndarray): Angle in radians
        
    Returns:
        float or np.ndarray: Angle in degrees
    """
    return angle * 180.0 / np.pi

def deg2rad(angle):
    """
    Convert degrees to radians.
    
    Args:
        angle (float or np.ndarray): Angle in degrees
        
    Returns:
        float or np.ndarray: Angle in radians
    """
    return angle * np.pi / 180.0 