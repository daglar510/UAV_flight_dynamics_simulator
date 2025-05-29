Looking at both READMEs, I can create an improved version that combines the best aspects of both. The 0.1.0 branch README has excellent technical details about the simulator's physics and operation, while the 0.2.0 branch README has a more comprehensive structure and details about the new UI features. Here's a combined version:

# UAV Flight Dynamics Simulator

A comprehensive web-based application for simulating and analyzing the longitudinal flight dynamics of fixed-wing UAVs. This tool allows aerospace engineers, researchers, and students to study how elevator deflection affects UAV dynamic behavior through an intuitive interface.

![Flight Dynamics Simulator](assets/favicon.ico)

## ✈️ Supported UAV Models

| UAV Name | Manufacturer                | Country |
| -------- | --------------------------- | ------- |
| TB2      | Baykar                      | Turkey  |
| Anka     | TUSAŞ                       | Turkey  |
| Aksungur | TUSAŞ                       | Turkey  |
| Karayel  | Vestel                      | Turkey  |
| Predator | General Atomics             | USA     |
| Heron    | Israel Aerospace Industries | Israel  |
| Heron 2  | Israel Aerospace Industries | Israel  |

> ⚠️ **Some aerodynamic and inertial values are estimated or assumed based on academic literature, public data, or similar vehicles.** See the Bayraktar TB2 Model Documentation for detailed assumptions.

## 📐 How It Works

This simulator models **longitudinal motion only** (no roll/yaw) using a **linearized state-space system**:

* **State Vector:** `[u, α, q, θ]`  
   * `u` — Forward speed deviation \[m/s\]  
   * `α` — Angle of attack \[radians\]  
   * `q` — Pitch rate \[radians/sec\]  
   * `θ` — Pitch angle \[radians\]
* **Input Vector:** `[thrust, elevator deflection]`  
   * Typically, only elevator is deflected (thrust held at trim).
* **System Equation:**  
   * ẋ = **A**·x + **B**·u

The `A` and `B` matrices are computed from each UAV's unique aerodynamic and inertial properties.

**Simulation is performed using** scipy.integrate.solve_ivp (**Runge-Kutta 4(5)**), ensuring accurate time response to user-defined elevator pulses.

## 🎯 Features

### UAV Model Selection and Customization
* Library of pre-defined UAV models with realistic parameters
* Full customization of all aircraft parameters:
  * Physical properties (mass, wing area, chord length, etc.)
  * Aerodynamic coefficients (lift, drag, and moment derivatives)
* Save custom configurations for future use

### Elevator Control Inputs
* Define multiple elevator deflection pulses
* Configure start time, duration, and deflection angle for each pulse
* Add or remove pulses as needed

### Simulation Capabilities
* Time-domain simulation of aircraft response to control inputs
* Configurable simulation duration
* State-space model construction and analysis
* Automatic calculation of trim airspeed

### Stability Analysis
* Eigenvalue analysis of the state matrix
* Identification of natural modes (Short Period, Phugoid, Subsidence)
* Calculation of natural frequency and damping ratio for each mode
* Stability assessment and interpretation

### Visualization
* Interactive time-domain response plots showing:
  * Forward speed perturbation
  * Angle of attack
  * Pitch rate
  * Pitch angle
  * Elevator deflection
* 3D trajectory visualization in state space

## 📊 Output: What Do the Plots Show?

After simulation, you'll see:

### 1\. **Time-Domain Response (5 subplots)**

* **u**: Forward speed deviation \[m/s\] - Shows phugoid oscillation and the immediate response to elevator pulse.
* **α**: Angle of attack (degrees) - Displays both short-period (fast, damped) and phugoid (slow, lightly damped) dynamics.
* **q**: Pitch rate (degrees/sec) - Captures rapid changes in pitch due to elevator input and short-period mode.
* **θ**: Pitch angle (degrees) - Shows aircraft's nose-up/nose-down movement over time, combining all dynamic effects.
* **Elevator Input**: Commanded elevator angle (degrees) - The input signal—here, a 2-degree pulse between t=5s and t=15s.

**Interpretation:**
* The **short-period mode** is seen as a quick, heavily damped oscillation, especially in α and q.
* The **phugoid mode** is a slow, lightly damped oscillation, especially evident in u and θ, representing the exchange between kinetic and potential energy.
* If the states return to zero (equilibrium) after input, the UAV is stable.

### 2\. **3D Trajectory in State-Space**

* **Axes:** α \[deg\] (angle of attack), q \[deg/s\] (pitch rate), θ \[deg\] (pitch angle)
* **Trajectory:** The path shows how these three states evolve together after an elevator pulse. The **spiraling inward** reflects damped oscillations—initially, the system moves away from trim, then oscillates back as damping dominates.

**Why this is useful:**
* **Dynamic Coupling:** You see how pitch rate and angle of attack interact dynamically.
* **Stability Visualization:** If the spiral closes in toward the center, the system is dynamically stable for that trim condition.
* **Mode Structure:** You can visually separate the rapid (short-period) and slow (phugoid) motions.

## ⚙️ Technology Stack

* **Frontend**: Reflex UI framework with Tailwind CSS
* **Backend**: Python with NumPy, SciPy, and Plotly
* **Scientific Computing**:
  * NumPy for matrix operations
  * SciPy for ODE integration
  * Plotly for interactive data visualization

## 🚀 Installation & Usage

### Prerequisites
* Python 3.8 or higher
* Required Python packages:
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

### Command-line Version (v0.1.0)
If you prefer the original command-line interface:

```bash
git checkout 0.1.0
python UAV_flight_dynamics_simulator.py
```

Then follow the prompts to select a UAV model, set simulation duration, and define elevator pulses.

## 🧮 Theory and Background

### Natural Modes

The eigenvalues of the state matrix A determine the natural modes of the aircraft:

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

Stability is assessed through the damping ratio (ζ) of each mode:
* ζ < 0: Unstable (positive real part)
* 0 < ζ < 0.02: Very poorly damped
* 0.02 < ζ < 0.2: Poorly damped
* 0.2 < ζ < 0.7: Good aircraft response
* ζ > 0.7: Heavily damped (potentially non-oscillatory)

## ⚠️ Limitations

* No lateral/directional motion (roll/yaw)—longitudinal axis only.
* Linear model: Nonlinear effects, large maneuvers, and actuator limits not included.
* No atmospheric/altitude variation.
* Not validated for flight-critical use—**for research, learning, and pre-design only.**

## 📚 References

* Marotta, Y. (2022). _Geometric modelling, stability and control analysis of the UAV Bayraktar TB-2 with OpenVSP._
* Marques, P., & Da Ronch, A. (2017). _Advanced UAV Aerodynamics, Flight Stability and Control._
* TRADOC (2021). _Design and Analysis of the Impact of Turkish UAVs._

## 👨‍💻 Author & Contributing

* Original version and idea by Mattia Di Mauro
* UAV extension and documentation by Dağlar Duman
* UI development by Dağlar Duman

Contributions to improve the Flight Dynamics Simulator are welcome! Please feel free to submit a pull request or open an issue to discuss potential improvements.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

> **Feel free to fork, use, or extend! For questions, improvements, or issues—open an issue or PR on GitHub.**

- **If you wan to reach out**: [Email](mailto:daglarduman510@gmail.com)
