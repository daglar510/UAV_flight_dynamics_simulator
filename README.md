# UAV Flight Dynamics Simulator

A comprehensive web-based application for simulating and analyzing the flight dynamics of fixed-wing UAVs. This tool allows aerospace engineers, researchers, and students to study how control inputs affect UAV dynamic behavior through an intuitive interface.

![Flight Dynamics Simulator](assets/favicon.ico)

## 🌟 Key Features

- **Dual Simulation Modes**: Supports both 4DOF (longitudinal) and 6DOF (full motion) simulation
- **Interactive UI**: Built with Reflex and Plotly for a responsive, dynamic experience
- **Real-time Visualization**: Comprehensive plotting of all state variables and control responses
- **Stability Analysis**: Eigenvalue analysis with mode identification and stability assessment
- **Customizable UAV Parameters**: Edit and save aircraft parameters for different vehicles
- **Multiple Control Inputs**: Define pulse inputs for elevator, aileron, rudder, and throttle

## ✈️ Supported UAV Models

| UAV Name   | Manufacturer                | Country |
| ---------- | --------------------------- | ------- |
| TB2        | Baykar                      | Turkey  |
| Anka       | TUSAŞ                       | Turkey  |
| Aksungur   | TUSAŞ                       | Turkey  |
| Karayel    | Vestel                      | Turkey  |
| Predator   | General Atomics             | USA     |
| Heron MK1  | Israel Aerospace Industries | Israel  |
| Heron MK2  | Israel Aerospace Industries | Israel  |

> ⚠️ **Some aerodynamic and inertial values are estimated or assumed based on academic literature, public data, or similar vehicles.** Lateral-directional derivatives for 6DOF simulation are particularly subject to estimation.

## 📐 Simulation Theory & Implementation

### 4DOF Simulation (Longitudinal Motion)

The 4DOF simulator models longitudinal motion using a linearized state-space system:

* **State Vector:** `[u, α, q, θ]`  
   * `u` — Forward speed deviation \[m/s\]  
   * `α` — Angle of attack \[radians\]  
   * `q` — Pitch rate \[radians/sec\]  
   * `θ` — Pitch angle \[radians\]
* **Input Vector:** `[thrust, elevator deflection]`  
* **System Equation:** ẋ = **A**·x + **B**·u

The A and B matrices are computed from aerodynamic and inertial properties, including:
- Mass, wing area, and chord length
- Lift and drag coefficients (CL, CD)
- Moment coefficients (Cm)
- Forward speed dependencies (CL_u, CD_u, Cm_u)

### 6DOF Simulation (Full Motion)

The 6DOF simulator implements full aircraft dynamics with:

* **State Vector:** `[u, v, w, p, q, r, φ, θ, ψ, x, y, z]`
   * `u, v, w` — Body-axis velocities \[m/s\]
   * `p, q, r` — Body-axis angular rates \[rad/s\]
   * `φ, θ, ψ` — Euler angles (roll, pitch, yaw) \[rad\]
   * `x, y, z` — NED position coordinates \[m\]
* **Input Vector:** `[roll, pitch, yaw, throttle]`
* **Additional Parameters Required:**
   * Moments of inertia (Ixx, Iyy, Izz, Ixz)
   * Lateral-directional derivatives (CYb, Clp, Cnr, etc.)
   * Control derivatives for all surfaces

Implementation details:
- Non-linear 6DOF equations of motion
- Inertial coupling through the full inertia tensor
- Position integration in North-East-Down (NED) reference frame
- Rotation matrix transformations between body and inertial frames

Both simulations use scipy.integrate.solve_ivp with Runge-Kutta 4(5) integration for accurate time-domain response.

## 🎮 Control Input System

The simulator supports defining multiple pulse inputs:

### 4DOF Mode
- **Elevator pulses**: Control the pitch motion
- Configure start time, duration, and deflection angle (degrees)

### 6DOF Mode
- **Roll control**: Aileron/roll input (degrees)
- **Pitch control**: Elevator/pitch input (degrees)
- **Yaw control**: Rudder/yaw input (degrees)
- **Throttle control**: Engine power setting (0-1)
- All inputs can be configured with start time and duration

The control input system handles multiple overlapping pulses, calculating the combined effect at each time step during simulation.

## 🎯 User Interface Components

### UAV Parameter Editor
- Select from predefined UAV models
- View/edit all physical and aerodynamic parameters
- Different parameters displayed based on simulation mode
- Save modified parameters for future use

### Pulse Editor
- Add, remove, and configure control pulses
- Adaptive interface changes based on simulation mode
- Real-time validation of input values

### Simulation Controls
- Switch between 4DOF and 6DOF simulation modes
- Set simulation duration
- Run simulation with current parameters
- Reset to saved configuration

### Results Display
- Trim speed calculation and display
- Eigenvalue analysis with mode identification
- Time-domain response plots
- 3D trajectory visualization
- Summary of simulation inputs

## 📊 Output Visualization

### 4DOF Simulation Results

After 4DOF simulation, you'll see:

**1. Time-Domain Response**
* **u**: Forward speed deviation \[m/s\]
* **α**: Angle of attack (degrees)
* **q**: Pitch rate (degrees/sec)
* **θ**: Pitch angle (degrees)
* **Elevator Input**: Commanded elevator angle (degrees)

![4DOF Time Domain Plot](assets/4dof_2dplot.png)

**2. 3D Trajectory in State-Space**
* Visualizes the relationship between α, q, and θ
* Helps identify dynamic coupling and stability patterns

![4DOF 3D Trajectory](assets/4dof_3dplot.PNG)

### 6DOF Simulation Results

After 6DOF simulation, the visualization includes:

**1. Comprehensive Time-Domain Response**
* **Velocities**: u, v, w (body-axis velocities)
* **Angular Rates**: p, q, r (roll, pitch, yaw rates)
* **Attitude**: φ, θ, ψ (roll, pitch, yaw angles)
* **Control Inputs**: Roll, pitch, yaw inputs and throttle
* **Derived Values**: 
  * Angle of attack (α) and sideslip (β)
  * Airspeed
  * Energy components (kinetic, potential)
  * Position (x, y, z in NED frame)

![6DOF Time Domain Plot](assets/6dof_2dplot.png)

**2. 3D Trajectory Visualization**
* Position trajectory (x, y, z)
* Euler angle trajectory (φ, θ, ψ)
* Shows the complete spatial motion of the UAV

![6DOF 3D Trajectory](assets/6dof_3dplot.png)

Each plot includes clear explanations of what the variable represents, making the results accessible to users with different levels of aerospace knowledge.

## 💡 Stability Analysis

The eigenvalue analysis provides insights into the dynamic stability of the aircraft:

### 4DOF Mode Analysis
1. **Short Period Mode**:
   * Higher frequency oscillations (typically >0.5 rad/s)
   * Involves primarily angle of attack and pitch rate
   * Critical for handling qualities

2. **Phugoid Mode**:
   * Lower frequency oscillations (typically <0.5 rad/s)
   * Involves primarily speed and pitch angle
   * Represents exchange between kinetic and potential energy

3. **Subsidence Modes** (if present):
   * Non-oscillatory decay or growth
   * Can be associated with speed or height subsidence

### Stability Assessment
The application automatically evaluates stability through the damping ratio (ζ) of each mode:
* ζ < 0: Unstable (positive real part)
* 0 < ζ < 0.02: Very poorly damped
* 0.02 < ζ < 0.2: Poorly damped
* 0.2 < ζ < 0.7: Good aircraft response
* ζ > 0.7: Heavily damped (potentially non-oscillatory)

## 📋 Example Simulation Results

### 4DOF Example

**Input Configuration:**
```
Simulation Duration: 90 seconds

Elevator Pulses:
- Start: 10s, Duration: 10s, Angle: 2°
- Start: 35s, Duration: 15s, Angle: -1°
```

**Results Summary:**
```
Trim Speed: U0 = 37.43 m/s (72.76 knots)

Eigenvalue Analysis:
- Short Period Mode: lambda = -0.9866+0.8354j | wn = 1.2928 rad/s | zeta = 0.763
  --> Highly damped (probably not oscillatory)
- Phugoid Mode: lambda = 0.0105+0.2110j | wn = 0.2112 rad/s | zeta = -0.050
  --> UNSTABLE (Danger: positive real part!)
```

**Interpretation:**
- The short period mode is well-damped, indicating good handling qualities for quick maneuvers
- The phugoid mode shows slight instability (positive real part), which means the aircraft will gradually diverge from equilibrium in speed and pitch over long periods
- The time domain response shows an initial pitch-up response to the 2° elevator input, followed by an oscillatory motion that gradually increases in amplitude due to the unstable phugoid

### 6DOF Example

**Input Configuration:**
```
Simulation Duration: 90 seconds

Control Pulses:
- Start: 10s, Duration: 10s, Roll: 2°, Pitch: 2°, Yaw: -3°, Throttle: 1
- Start: 30s, Duration: 15s, Roll: -1°, Pitch: -1°, Yaw: 0°, Throttle: 1
- Start: 60s, Duration: 10s, Roll: 0°, Pitch: 0°, Yaw: 2°, Throttle: 1
```

**Results Summary:**
```
Trim Speed: U0 = 37.43 m/s (72.76 knots)

Eigenvalue Analysis:
6DOF simulation: Eigen analysis not implemented.
```

**Interpretation of 6DOF Outputs:**
- **The Time Domain Response plots** show 21 different variables to provide a comprehensive view of the aircraft's behavior:
  - The top rows show the basic velocities (u, v, w) and how they respond to control inputs
  - The angular rates (p, q, r) show the rotation speeds around each axis
  - The Euler angles (roll, pitch, yaw) display the attitude of the aircraft
  - Control inputs are shown alongside their corresponding responses (as dotted lines)
  - Derived quantities like angle of attack (α), sideslip (β), and airspeed show aerodynamic conditions
  - Energy metrics help understand the trade-offs between potential and kinetic energy
  - Position coordinates (x, y, z) track the aircraft's path through space

- **The 3D Trajectory plot** shows two important aspects:
  - The blue line shows the physical path of the aircraft through space (x, y, z coordinates)
  - The red dashed line shows how the attitude (roll, pitch, yaw angles) changes over time
  - This visualization helps understand how control inputs affect both position and orientation

The 6DOF simulation provides a much more complete picture of aircraft behavior than the 4DOF model, capturing cross-coupling effects between longitudinal and lateral-directional dynamics.

## ⚙️ Technical Implementation

### Architecture
- **UI Framework**: Reflex for reactive web components
- **State Management**: Centralized FlightSimState class handling all simulation logic
- **Rendering**: Plotly for interactive plots with subplot capabilities
- **Mathematical Core**: NumPy for matrix operations, SciPy for ODE integration

### Key Implementation Features
- **Dynamic UI**: UI components adapt to the selected simulation mode
- **Error Handling**: Input validation with visual feedback
- **Asynchronous Processing**: Simulation runs in the background to prevent UI freezing
- **Responsive Layout**: Adapts to different screen sizes with Tailwind CSS
- **Plot Customization**: Comprehensive labels, legends, and visual styling

## 🚀 Installation & Usage

### Prerequisites
* Python 3.8 or higher
* Required Python packages:
```
reflex>=0.7.8
numpy
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

### Typical Workflow

1. **Select a UAV** from the dropdown menu
2. **Choose simulation mode** (4DOF or 6DOF)
3. **Adjust UAV parameters** if needed
4. **Set up control pulses**:
   - For 4DOF: Define elevator pulses
   - For 6DOF: Define roll, pitch, yaw, and throttle pulses
5. **Set simulation duration**
6. **Run the simulation**
7. **Analyze results** in the visualization section

## ⚠️ Limitations

* **Linear Assumptions**: 4DOF mode uses linearized aerodynamics
* **Parameter Estimation**: Some UAV parameters are estimated from limited public data
* **Environmental Effects**: No wind, turbulence, or atmospheric variation models
* **Computational Efficiency**: Complex simulations may be slower in the web environment
* **Not Flight-Certified**: For educational and pre-design purposes only

## 🧪 Internal Code Structure

The application consists of the following major components:

```
flight_dynamics_simulator_ui_design/
├── components/                     # UI components
│   ├── pulse_editor.py            # Control pulse definition interface
│   ├── results_display.py         # Visualization of simulation results
│   ├── simulation_controls.py     # Mode selection and run controls
│   └── uav_editor.py              # UAV parameter editing interface
├── states/
│   └── flight_sim_state.py        # Core simulation logic and state management
└── utils/
    ├── constants.py               # Physical constants
    ├── types.py                   # Type definitions for UAV parameters
    └── uav_models.py              # Pre-defined UAV models database
```

### Key Technical Functions

- **_build_state_space**: Constructs the A and B matrices for 4DOF simulation
- **_build_state_space_6dof**: Prepares parameters for 6DOF simulation
- **_simulate_response**: Performs integration for 4DOF dynamics
- **_simulate_response_6dof**: Performs integration for 6DOF dynamics
- **_analyze_modes**: Computes and categorizes eigenvalues
- **_create_time_domain_plot**: Generates visualization for 4DOF results
- **_create_time_domain_plot_6dof**: Generates visualization for 6DOF results

## 📚 References

* Marotta, Y. (2022). _Geometric modelling, stability and control analysis of the UAV Bayraktar TB-2 with OpenVSP._
* Marques, P., & Da Ronch, A. (2017). _Advanced UAV Aerodynamics, Flight Stability and Control._
* TRADOC (2021). _Design and Analysis of the Impact of Turkish UAVs._
* Stevens, B.L., Lewis, F.L., & Johnson, E.N. (2015). _Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems._
* Cook, M.V. (2012). _Flight Dynamics Principles: A Linear Systems Approach to Aircraft Stability and Control._

## 👨‍💻 Author & Contributing

* Original version and idea by Mattia Di Mauro
* UAV extension and documentation by Dağlar Duman
* UI development by Dağlar Duman
* 6DOF implementation by Dağlar Duman

Contributions to improve the Flight Dynamics Simulator are welcome! Please feel free to submit a pull request or open an issue to discuss potential improvements.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

> **Feel free to fork, use, or extend! For questions, improvements, or issues—open an issue or PR on GitHub.**

- **If you want to reach out**: [Email](mailto:daglarduman510@gmail.com)
- **GitHub Repository**: [UAV_flight_dynamics_simulator](https://github.com/daglar510/UAV_flight_dynamics_simulator)
