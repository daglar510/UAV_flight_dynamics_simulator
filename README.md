# UAV Flight Dynamics Simulator

A lightweight, powerful simulator for exploring UAV flight dynamics with both quadcopter and fixed-wing aircraft models.

## Project Structure

```
UAV_flight_dynamics_simulator/
├── UAV_flight_dynamics_simulator.py   # Main entry point for all simulations
├── uav_sim/                           # Core simulation package
│   ├── constants.py                   # Physics constants
│   ├── quadcopter/                    # Quadcopter simulation modules
│   │   ├── command_sim.py             # Interactive command-based simulator
│   │   ├── controller.py              # PID controller implementation
│   │   ├── dynamics.py                # Quadcopter physical dynamics
│   │   ├── interactive_sim.py         # Interactive keyboard simulator
│   │   ├── models.py                  # Quadcopter model parameters
│   │   ├── run_simulation.py          # Simulation runner for quadcopters
│   │   ├── simulate.py                # Simulation implementation
│   │   ├── trajectory_sim.py          # Predefined trajectory simulation
│   │   ├── visualize.py               # Visualization tools
│   │   └── __init__.py
│   ├── uav/                           # Fixed-wing simulation modules
│   │   ├── dynamics.py                # Fixed-wing physical dynamics
│   │   ├── models.py                  # Fixed-wing aircraft parameters
│   │   ├── run_simulation.py          # Simulation runner for fixed-wing
│   │   ├── simulate.py                # Fixed-wing simulation implementation
│   │   ├── visualize.py               # Fixed-wing visualization tools
│   │   └── __init__.py
│   ├── utils/                         # Utility functions
│   │   ├── common.py                  # Common utilities
│   │   └── __init__.py
│   └── __init__.py
└── demo_script.txt                    # Example commands for the quadcopter simulator
```

## Running the Simulator

### Command Line Interface

The simulator provides a unified command-line interface for all simulator types:

```bash
# Fixed-wing simulator
python UAV_flight_dynamics_simulator.py --fixed-wing --model TB2 --duration 60

# Quadcopter command simulator (interactive)
python UAV_flight_dynamics_simulator.py --quadcopter-command

# Quadcopter trajectory simulator
python UAV_flight_dynamics_simulator.py --quadcopter-trajectory --maneuver square --duration 30
```

### Interactive Menu

Run without arguments to use the interactive menu:

```bash
python UAV_flight_dynamics_simulator.py
```

## Available Models

### Quadcopter Models
- default: Standard quadcopter (0.5kg)
- small: Lightweight quadcopter (0.3kg)
- medium: Medium quadcopter (1.0kg)
- large: Heavy quadcopter (2.0kg)

### Fixed-wing Models
- TB2: Bayraktar TB2 medium-altitude long-endurance UAV
- Anka: TAI Anka reconnaissance UAV
- Aksungur: High-altitude long-endurance UAV
- Karayel: Tactical UAV
- Predator: General Atomics MQ-1 Predator
- Heron: IAI Heron surveillance UAV

## Maneuver Types (Quadcopter)

- hover: Maintain position at specified height
- square: Fly in a square pattern
- figure8: Fly in a figure-8 pattern
- yaw_test: Test yaw control

## Command Simulator

The command simulator allows direct control of motor speeds:

```
m1 5000    # Set motor 1 (Front-Right) to 5000 RPM
m2 5000    # Set motor 2 (Front-Left) to 5000 RPM
hover      # Set all motors to hover speed
script demo_script.txt  # Run commands from script file
```

## Features

- **Unified Command Interface**: Single entry point for all simulation modes
- **Quadcopter Simulator**: Direct motor control with real-time visualization
- **Fixed-Wing Simulator**: Study aircraft response to elevator inputs
- **Multiple UAV Models**: Various pre-defined aircraft configurations

## Project Organization

```
UAV_flight_dynamics_simulator/
├── uav_sim/                   # Main simulation package
│   ├── quadcopter/            # Quadcopter simulation modules
│   │   ├── command_sim.py     # Interactive command-based simulator
│   │   ├── controller.py      # PID controller implementation
│   │   ├── dynamics.py        # Quadcopter physical dynamics
│   │   ├── models.py          # Quadcopter model parameters
│   │   ├── run_simulation.py  # Entry point for quadcopter simulations
│   │   ├── simulate.py        # Simulation components and trajectory generation
│   │   ├── trajectory_sim.py  # Structured trajectory simulator
│   │   └── visualize.py       # Visualization functions for quadcopter
│   │
│   ├── uav/                   # Fixed-wing simulation modules
│   │   ├── dynamics.py        # Fixed-wing flight dynamics
│   │   ├── models.py          # Fixed-wing aircraft parameters
│   │   ├── run_simulation.py  # Entry point for fixed-wing simulations
│   │   ├── simulate.py        # Flight simulation components
│   │   └── visualize.py       # Visualization for fixed-wing simulations
│   │
│   └── utils/                 # Shared utility functions
│       └── common.py          # Common utilities for both simulators
│
└── UAV_flight_dynamics_simulator.py  # Main entry point with unified CLI
```

## Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/UAV_flight_dynamics_simulator.git
cd UAV_flight_dynamics_simulator

# Create a virtual environment
python -m venv .venv

# Activate the virtual environment
# On Windows:
.\.venv\Scripts\activate
# On Linux/Mac:
# source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Understanding the Output

### Quadcopter Visualization:

- **3D Trajectory**: Quadcopter's path in 3D space
- **Position Plot**: X, Y, Z position over time
- **Attitude Plot**: Roll, pitch, yaw angles in degrees
- **Motor Speeds Plot**: Individual motor speeds in RPM with hover speed reference

### Fixed-Wing Visualization:

- **Time Response**: Changes in forward speed, angle of attack, pitch rate, and pitch angle
- **3D Trajectory**: Path in (α, q, θ) space (Angle of Attack, Pitch Rate, Pitch)
- **Elevator Input**: Visualization of elevator deflection over time

## Applications

This simulator can be used for:

1. **Education**: Learning flight dynamics principles
2. **Research**: Testing control algorithms
3. **Design**: Evaluating UAV configurations
4. **Testing**: Verifying expected behavior
5. **Analysis**: Understanding aircraft stability and response

## Coming Soon

- **Fixed-Wing UAV Simulator**: Support for fixed-wing aircraft models and dynamics
