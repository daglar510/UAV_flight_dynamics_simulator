"""
UAV parameter database for fixed-wing aircraft.
"""

# --- Constants ---
GRAVITY = 9.80665      # gravity [m/s²]
AIR_DENSITY = 1.225    # sea-level air density [kg/m³]
GAS_CONSTANT = 287.05  # specific gas constant for air [J/(kg·K)]
GAMMA = 1.4            # ratio of specific heats

# --- UAV database (some values ASSUMED, see Markdown doc) ---
def get_fixed_wing_parameters(name):
    """
    Get parameters for a specific fixed-wing UAV model.
    
    Args:
        name (str): Name of the UAV model
        
    Returns:
        dict: Dictionary of UAV parameters
        
    Raises:
        KeyError: If the UAV name is not found in the database
    """
    UAV_DB = {
        "TB2": {
            "company": "Baykar", "country": "Turkey",
            "mass": 700, "S": 9.34, "c": 0.78, "b": 12.0, "Iyy": 2500, "Mach": 0.12,
            # Aerodynamic coefficients (see doc for details, *assumed* marked)
            "CL_0": 0.30, "CL_alpha": 5.5, "CL_q": 5.0, "CL_deltae": 0.4, "CL_u": 0.0,
            "CD_0": 0.05, "CD_alpha": 0.3, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0
        },
        "Anka": {
            "company": "TUSAŞ", "country": "Turkey",
            "mass": 1700, "S": 18.0, "c": 1.05, "b": 17.5, "Iyy": 6000, "Mach": 0.18,
            # All coefficients – assumed
            "CL_0": 0.35, "CL_alpha": 5.7, "CL_q": 5.5, "CL_deltae": 0.5, "CL_u": 0.0,
            "CD_0": 0.06, "CD_alpha": 0.32, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.8, "Cm_q": -6.0, "Cm_deltae": -1.2, "Cm_u": 0.0
        },
        "Aksungur": {
            "company": "TUSAŞ", "country": "Turkey",
            "mass": 3300, "S": 30.0, "c": 1.25, "b": 24.2, "Iyy": 12000, "Mach": 0.21,
            # All values – assumed
            "CL_0": 0.25, "CL_alpha": 5.2, "CL_q": 6.0, "CL_deltae": 0.6, "CL_u": 0.0,
            "CD_0": 0.07, "CD_alpha": 0.35, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.7, "Cm_q": -7.0, "Cm_deltae": -1.5, "Cm_u": 0.0
        },
        "Karayel": {
            "company": "Vestel", "country": "Turkey",
            "mass": 630, "S": 9.0, "c": 0.75, "b": 13.0, "Iyy": 2000, "Mach": 0.11,
            # All values – assumed
            "CL_0": 0.30, "CL_alpha": 5.5, "CL_q": 4.5, "CL_deltae": 0.4, "CL_u": 0.0,
            "CD_0": 0.05, "CD_alpha": 0.3, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.5, "Cm_q": -3.5, "Cm_deltae": -0.8, "Cm_u": 0.0
        },
        "Predator": {
            "company": "General Atomics", "country": "USA",
            "mass": 1020, "S": 11.45, "c": 0.78, "b": 14.8, "Iyy": 5000, "Mach": 0.14,
            # All values – assumed
            "CL_0": 0.25, "CL_alpha": 5.6, "CL_q": 4.0, "CL_deltae": 0.3, "CL_u": 0.0,
            "CD_0": 0.05, "CD_alpha": 0.3, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.5, "Cm_q": -4.0, "Cm_deltae": -0.5, "Cm_u": 0.0
        },
        "Heron": {
            "company": "IAI", "country": "Israel",
            "mass": 1150, "S": 12.9, "c": 0.78, "b": 16.6, "Iyy": 4000, "Mach": 0.18,
            # All values – assumed
            "CL_0": 0.40, "CL_alpha": 5.7, "CL_q": 5.5, "CL_deltae": 0.5, "CL_u": 0.0,
            "CD_0": 0.06, "CD_alpha": 0.33, "CD_q": 0.0, "CD_deltae": 0.0, "CD_u": 0.0,
            "Cm_0": 0.0, "Cm_alpha": -0.6, "Cm_q": -5.0, "Cm_deltae": -1.0, "Cm_u": 0.0
        }
    }
    if name not in UAV_DB:
        raise KeyError(f"Unknown UAV: {name}")
    return UAV_DB[name]

def get_available_fixed_wing_models():
    """
    Get a list of all available fixed-wing models.
    
    Returns:
        list: List of model names
    """
    return ["TB2", "Anka", "Aksungur", "Karayel", "Predator", "Heron"] 