# Flight Dynamics Simulator

A comprehensive web-based application for simulating and analyzing aircraft longitudinal flight dynamics.

![Flight Dynamics Simulator](assets/favicon.ico)

## Overview

This Flight Dynamics Simulator is an interactive tool designed for aerospace engineering education and analysis. It allows users to:

- Select from a library of UAV (Unmanned Aerial Vehicle) models
- Customize aircraft parameters and aerodynamic coefficients
- Define elevator control inputs as time-based pulses
- Run simulations of aircraft longitudinal dynamics
- Analyze aircraft stability through eigenvalue analysis
- Visualize time-domain responses and state-space trajectories

The simulator focuses on the longitudinal dynamics of fixed-wing aircraft, analyzing the short period and phugoid modes, along with their stability characteristics.

## Features

### UAV Model Selection and Customization

- Library of pre-defined UAV models with realistic parameters
- Full customization of all aircraft parameters:
  - Physical properties (mass, wing area, chord length, etc.)
  - Aerodynamic coefficients (lift, drag, and moment derivatives)
- Save custom configurations for future use

### Elevator Control Inputs

- Define multiple elevator deflection pulses
- Configure start time, duration, and deflection angle for each pulse
- Add or remove pulses as needed

### Simulation Capabilities

- Time-domain simulation of aircraft response to control inputs
- Configurable simulation duration
- State-space model construction and analysis
- Automatic calculation of trim airspeed

### Stability Analysis

- Eigenvalue analysis of the state matrix
- Identification of natural modes (Short Period, Phugoid, Subsidence)
- Calculation of natural frequency and damping ratio for each mode
- Stability assessment and interpretation

### Visualization

- Interactive time-domain response plots showing:
  - Forward speed perturbation
  - Angle of attack
  - Pitch rate
  - Pitch angle
  - Elevator deflection
- 3D trajectory visualization in state space

## Technology Stack

- **Frontend**: Reflex UI framework with Tailwind CSS
- **Backend**: Python with NumPy, SciPy, and Plotly
- **Scientific Computing**:
  - NumPy for matrix operations
  - SciPy for ODE integration
  - Plotly for interactive data visualization

## Getting Started

### Prerequisites

- Python 3.8 or higher
- Required Python packages:
  ```
  reflex>=0.7.8
  numpy
  matplotlib
  scipy
  plotly
  ```

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/daglar510/UAV_flight_dynamics_simulator.git
   cd UAV_flight_dynamics_simulator
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Run the application:
   ```bash
   python -m reflex run
   ```

4. Open your browser and navigate to http://localhost:3000

## Usage Guide

### Step 1: Select a UAV Model

Choose a predefined UAV model from the dropdown menu in the "UAV Parameters" section. This will load the default parameters for that aircraft.

### Step 2: Customize Parameters (Optional)

You can modify any of the UAV parameters by entering new values in the corresponding input fields. Parameters include:

- **Physical Properties**:
  - Mass (kg)
  - Wing area (S) (m²)
  - Mean chord (c) (m)
  - Wingspan (b) (m)
  - Moment of inertia (Iyy) (kg·m²)
  - Cruise Mach number

- **Aerodynamic Coefficients**:
  - Lift coefficients (CL_0, CL_alpha, CL_q, CL_deltae, CL_u)
  - Drag coefficients (CD_0, CD_alpha, CD_q, CD_deltae, CD_u)
  - Moment coefficients (Cm_0, Cm_alpha, Cm_q, Cm_deltae, Cm_u)

### Step 3: Define Elevator Pulses

In the "Elevator Pulses" section:
1. Configure existing pulses by setting:
   - Start time (seconds)
   - Duration (seconds)
   - Angle (degrees)
2. Add additional pulses using the "Add Pulse" button
3. Remove unwanted pulses using the "-" button next to each pulse

### Step 4: Set Simulation Parameters

In the "Simulation Setup" section:
1. Set the desired simulation duration (in seconds)

### Step 5: Run the Simulation

Click the "Run Simulation" button to execute the flight dynamics simulation.

### Step 6: Analyze Results

The "Simulation Results" section will display:

1. **Simulation Inputs**: A summary of the inputs used in the simulation
2. **Trim Speed**: The calculated trim airspeed for the selected UAV
3. **Eigenvalue Analysis**: Natural modes with frequency and damping ratio
4. **Time Domain Response**: Interactive plots of aircraft state variables
5. **3D Trajectory**: Visualization of the aircraft trajectory in state space

### Step 7: Save Configuration (Optional)

Click "Save UAV Config" to save your customized UAV parameters to the database for future use.

## Theory and Background

### Longitudinal Flight Dynamics

The simulator models the longitudinal dynamics of aircraft using a 4-state linear model:
- Forward speed perturbation (u)
- Angle of attack (α)
- Pitch rate (q)
- Pitch angle (θ)

### State-Space Representation

The aircraft dynamics are represented in state-space form:
```
ẋ = Ax + Bu
```
where:
- x = [u, α, q, θ]ᵀ is the state vector
- u = [δₜ, δₑ]ᵀ is the control input vector (thrust and elevator)
- A is the state matrix
- B is the control matrix

### Natural Modes

The eigenvalues of the state matrix A determine the natural modes of the aircraft:

1. **Short Period Mode**:
   - Higher frequency oscillations (typically >0.5 rad/s)
   - Involves primarily angle of attack and pitch rate
   - Critical for handling qualities

2. **Phugoid Mode**:
   - Lower frequency oscillations (typically <0.5 rad/s)
   - Involves primarily speed and pitch angle
   - Represents exchange between kinetic and potential energy

3. **Subsidence Modes** (if present):
   - Non-oscillatory decay or growth
   - Can be associated with speed or height subsidence

### Stability Assessment

Stability is assessed through the damping ratio (ζ) of each mode:
- ζ < 0: Unstable (positive real part)
- 0 < ζ < 0.02: Very poorly damped
- 0.02 < ζ < 0.2: Poorly damped
- 0.2 < ζ < 0.7: Good aircraft response
- ζ > 0.7: Heavily damped (potentially non-oscillatory)

## Contributing

Contributions to improve the Flight Dynamics Simulator are welcome! Please feel free to submit a pull request or open an issue to discuss potential improvements.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Reflex framework for the reactive UI components
- NumPy, SciPy, and Plotly for scientific computing and visualization
- The aerospace engineering community for theoretical foundations 

## Author and Creator

This project was developed by Daglar Duman.

- **Email**: [Your Email](mailto:daglarduman510@gmail.com)

Feel free to reach out for any questions or contributions! 