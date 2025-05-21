"""
Quadcopter simulation module for UAV simulation package.
"""

# Import key components for easier access
from uav_sim.quadcopter.models import get_quadcopter_parameters, get_available_quadcopter_models
from uav_sim.quadcopter.dynamics import QuadcopterDynamics
from uav_sim.quadcopter.controller import QuadcopterController
from uav_sim.quadcopter.run_simulation import run_quadcopter_simulation, visualize_results

# Interactive simulators
try:
    from uav_sim.quadcopter.interactive_sim import InteractiveQuadcopterSim
except ImportError:
    # The interactive simulator requires the keyboard module, which may not be installed
    pass

try:
    from uav_sim.quadcopter.command_sim import run_command_sim, QuadcopterCommandSim
except ImportError:
    # In case the command simulator module has issues
    pass 