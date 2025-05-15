
  # UAV Longitudinal Flight Dynamics Simulator

  This Python-based simulator models the **longitudinal flight dynamics** of fixed-wing UAVs using a **linear state-space approach**. It allows aerospace engineers, researchers, and students to study how elevator deflection affects the UAV’s dynamic behavior in terms of:

  - Forward speed (`u`)
  - Angle of attack (`α`)
  - Pitch rate (`q`)
  - Pitch angle (`θ`)

  The simulator supports six UAV models from Turkey, the USA, and Israel. All are defined by their aerodynamic, geometric, and inertial properties.

  ---

  ## ✈️ Supported UAV Models

  | UAV Name   | Manufacturer        | Country    |
  |------------|---------------------|------------|
  | TB2        | Baykar              | Turkey     |
  | Anka       | TUSAŞ               | Turkey     |
  | Aksungur   | TUSAŞ               | Turkey     |
  | Karayel    | Vestel              | Turkey     |
  | Predator   | General Atomics     | USA        |
  | Heron      | Israel Aerospace Industries | Israel |

  > 🔍 **Note**: Some parameters (e.g., moment of inertia, aerodynamic coefficients) are **estimated** based on publicly available data or similar platforms.

  ---

  ## 📐 Technical Overview

  The simulator uses the following principles:

  - **Linearized Longitudinal Equations of Motion** about a trim condition
  - **State-Space Representation**:
    - **State Vector**: `[u, α, q, θ]`
    - **Input Vector**: `[thrust, elevator deflection]`
    - **Output**: System response over time
  - **Numerical Integration** using `scipy.solve_ivp` (Runge-Kutta 4(5))

  The UAV’s behavior is described by:
  ẋ = A·x + B·u

Where **A** and **B** matrices are built from aerodynamic and inertial coefficients.

---

## 🎯 Purpose and Applications

This tool can be used for:

- **Aerospace Education**: Teaching stability & control, response modes (phugoid, short-period).
- **Control System Design**: Baseline for designing pitch controllers or autopilots.
- **Flight Dynamics Research**: Comparing various UAV dynamics under identical input.
- **Simulation & Testing**: Testing elevator responses before actual flight tests.
- **Benchmarking**: Assessing how design choices affect pitch dynamics.

> ❌ Not suitable for certification, full non-linear simulation, or transonic/supersonic regimes.

---

## 💻 How to Use

### 1. Install Requirements

```markdown
### 1. Install Requirements

Install the required Python packages using pip:

```bash
pip install numpy matplotlib scipy

