"""
Quadcopter parameter database.
"""

import json
import os
from pathlib import Path
import numpy as np
from uav_sim.constants import GRAVITY

# Default quadcopter parameters
DEFAULT_QUAD_PARAMS = {
    "name": "default",
    "mass": 0.5,  # kg
    "arm_length": 0.25,  # m (distance from center to motor)
    "Ixx": 0.0023,  # kg*m^2 (moment of inertia around x-axis)
    "Iyy": 0.0023,  # kg*m^2 (moment of inertia around y-axis)
    "Izz": 0.0046,  # kg*m^2 (moment of inertia around z-axis)
    "thrust_coefficient": 1.91e-6,  # N/(rad/s)^2
    "drag_coefficient": 2.75e-7,  # N*m/(rad/s)^2
    "max_rpm": 10000,  # maximum motor speed in RPM
    "min_rpm": 1000,   # minimum motor speed in RPM
    "motor_time_constant": 0.02,  # s (motor response time)
    # Default PID controller gains - tuned for good performance
    "pid_gains": {
        "roll": {"kp": 12.0, "ki": 0.1, "kd": 4.0},     # Attitude control
        "pitch": {"kp": 12.0, "ki": 0.1, "kd": 4.0},    # Attitude control
        "yaw": {"kp": 8.0, "ki": 0.1, "kd": 2.0},       # Heading control
        "altitude": {"kp": 25.0, "ki": 3.0, "kd": 10.0}, # Height control
        "x": {"kp": 2.0, "ki": 0.1, "kd": 1.0},         # Position control
        "y": {"kp": 2.0, "ki": 0.1, "kd": 1.0}          # Position control
    }
}

# Database of predefined quadcopter models
QUAD_DB = {
    "default": DEFAULT_QUAD_PARAMS,
    "small": {
        "name": "small",
        "mass": 0.3,
        "arm_length": 0.15,
        "Ixx": 0.0015,
        "Iyy": 0.0015,
        "Izz": 0.003,
        "thrust_coefficient": 1.5e-6,
        "drag_coefficient": 2.0e-7,
        "max_rpm": 15000,
        "min_rpm": 1500,
        "motor_time_constant": 0.015,
        "pid_gains": {
            "roll": {"kp": 10.0, "ki": 0.1, "kd": 3.0},
            "pitch": {"kp": 10.0, "ki": 0.1, "kd": 3.0},
            "yaw": {"kp": 6.0, "ki": 0.1, "kd": 1.5},
            "altitude": {"kp": 18.0, "ki": 2.0, "kd": 7.0},
            "x": {"kp": 1.8, "ki": 0.1, "kd": 0.8},
            "y": {"kp": 1.8, "ki": 0.1, "kd": 0.8}
        }
    },
    "medium": {
        "name": "medium",
        "mass": 1.0,
        "arm_length": 0.3,
        "Ixx": 0.004,
        "Iyy": 0.004,
        "Izz": 0.008,
        "thrust_coefficient": 2.5e-6,
        "drag_coefficient": 3.0e-7,
        "max_rpm": 8000,
        "min_rpm": 800,
        "motor_time_constant": 0.025,
        "pid_gains": {
            "roll": {"kp": 15.0, "ki": 0.2, "kd": 4.5},
            "pitch": {"kp": 15.0, "ki": 0.2, "kd": 4.5},
            "yaw": {"kp": 8.0, "ki": 0.2, "kd": 2.0},
            "altitude": {"kp": 30.0, "ki": 4.0, "kd": 12.0},
            "x": {"kp": 2.5, "ki": 0.2, "kd": 1.2},
            "y": {"kp": 2.5, "ki": 0.2, "kd": 1.2}
        }
    },
    "large": {
        "name": "large",
        "mass": 2.0,
        "arm_length": 0.4,
        "Ixx": 0.008,
        "Iyy": 0.008,
        "Izz": 0.016,
        "thrust_coefficient": 3.5e-6,
        "drag_coefficient": 4.0e-7,
        "max_rpm": 6000,
        "min_rpm": 600,
        "motor_time_constant": 0.03,
        "pid_gains": {
            "roll": {"kp": 18.0, "ki": 0.3, "kd": 5.0},
            "pitch": {"kp": 18.0, "ki": 0.3, "kd": 5.0},
            "yaw": {"kp": 10.0, "ki": 0.3, "kd": 2.5},
            "altitude": {"kp": 35.0, "ki": 5.0, "kd": 15.0},
            "x": {"kp": 3.0, "ki": 0.3, "kd": 1.5},
            "y": {"kp": 3.0, "ki": 0.3, "kd": 1.5}
        }
    }
}

def load_quadcopter_params_from_file(filename):
    """
    Load quadcopter parameters from a JSON file.
    
    Args:
        filename (str): Path to the JSON file
        
    Returns:
        dict: Quadcopter parameters
        
    Raises:
        FileNotFoundError: If the file doesn't exist
        json.JSONDecodeError: If the file isn't valid JSON
    """
    with open(filename, 'r') as f:
        params = json.load(f)
    
    # Validate required fields
    required_fields = ["mass", "arm_length", "Ixx", "Iyy", "Izz"]
    for field in required_fields:
        if field not in params:
            raise ValueError(f"Missing required field: {field}")
    
    # Set defaults for missing optional fields
    if "pid_gains" not in params:
        params["pid_gains"] = DEFAULT_QUAD_PARAMS["pid_gains"]
    
    if "name" not in params:
        params["name"] = os.path.splitext(os.path.basename(filename))[0]
    
    return params

def get_quadcopter_parameters(model_name):
    """
    Get parameters for a specific quadcopter model.
    
    Args:
        model_name (str): Name of the model or path to a JSON file
        
    Returns:
        dict: Quadcopter parameters
        
    Raises:
        ValueError: If the model name is not found and is not a file
    """
    # Check if it's a predefined model
    if model_name in QUAD_DB:
        return QUAD_DB[model_name]
    
    # Check if it's a file path
    if os.path.exists(model_name):
        return load_quadcopter_params_from_file(model_name)
    
    # Check if it's a JSON file in current directory
    if not model_name.endswith('.json'):
        json_path = f"{model_name}.json"
        if os.path.exists(json_path):
            return load_quadcopter_params_from_file(json_path)
    
    raise ValueError(f"Unknown quadcopter model: {model_name}")

def get_available_quadcopter_models():
    """
    Get a list of all available predefined quadcopter models.
    
    Returns:
        list: List of model names
    """
    return list(QUAD_DB.keys())

def save_quadcopter_parameters(params, filename):
    """
    Save quadcopter parameters to a JSON file.
    
    Args:
        params (dict): Quadcopter parameters
        filename (str): Path to save the JSON file
        
    Returns:
        str: Path to the saved file
    """
    with open(filename, 'w') as f:
        json.dump(params, f, indent=2)
    
    return filename 