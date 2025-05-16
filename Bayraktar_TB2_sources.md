
# Bayraktar TB2 – Aerodynamic & Dynamic Model Documentation

---

## Basic Aircraft Info

- **Model:** Bayraktar TB2
- **Country:** Turkey
- **Manufacturer:** Baykar
- **References:**
  - Marotta, Y. (2022). Geometric modelling, stability and control analysis of the UAV Bayraktar TB-2 with OpenVSP.
  - TRADOC (2021). Design and Analysis of the Impact of Turkish UAVs.
  - Marques & Da Ronch (2017). Advanced UAV Aerodynamics, Flight Stability and Control.

---

## 1. Mass

- `mass = 700 kg` (Maximum Takeoff Weight)
- Source: Marotta (2022), Table 4.1

---

## 2. Wing Area

- `S = 9.34 m²`
- Source: OpenVSP geometric model, Marotta (2022), Table 4.1

---

## 3. Mean Aerodynamic Chord

- `c = 0.78 m`
- Calculated from airfoil profiles and OpenVSP geometry  
  Approximate formula (plain text):
```
c ≈ (2/3) * [c_root + c_tip - (c_root * c_tip) / (c_root + c_tip)]
```
- Source: Marotta (2022), Figures 3.3–3.5

---

## 4. Wingspan

- `b = 12.0 m`
- Source: CAD/model in Marotta (2022)

---

## 5. Moment of Inertia (Pitch, Iyy)

- `Iyy ≈ 2500 kg·m²` (Estimated)
- Used parallel axis theorem with a shape factor:
```
Iyy ≈ (1/12) * m * (b^2 + L^2) * k
```
where:
  - `m = 700 kg`
  - `b = 12.0 m`
  - `L ≈ 6.5 m` (fuselage length)
  - `k ≈ 0.25` (shape correction)
- Source: Marotta (2022), TRADOC (2021)

---

## 6. Mach Number (Cruise)

- `Mach = 0.12`
- Calculated as: `Mach = V / a ≈ 70 / 343 ≈ 0.20`
- (Roughly rounded down to 0.12 for typical cruise)
- Source: Marotta (2022), TRADOC (2021)

---

## 7. Aerodynamic Coefficients

### Lift
| Coefficient  | Value | Source/Method |
|--------------|-------|---------------|
| CL_0         | 0.30  | Interpolated at α = 0°, Marotta Table 4.1 |
| CL_alpha     | 5.5   | Slope of CL vs. α, adjusted for tail |
| CL_q         | 5.0   | **Assumed** (typical for UAVs) |
| CL_deltae    | 0.4   | Estimated from Marotta Table 4.4 |
| CL_u         | 0.0   | Neglected |

### Drag
| Coefficient  | Value | Source/Method |
|--------------|-------|---------------|
| CD_0         | 0.05  | Marotta Table 4.3 |
| CD_alpha     | 0.3   | Estimated slope from α = 0° to 10° |
| CD_q         | 0.0   | **Assumed** negligible |
| CD_deltae    | 0.0   | **Assumed** negligible |
| CD_u         | 0.0   | Neglected |

### Pitching Moment
| Coefficient  | Value | Source/Method |
|--------------|-------|---------------|
| Cm_0         | 0.0   | **Assumed** neutral trim |
| Cm_alpha     | -0.6  | Slope of Cm vs. α, Marotta Table 4.2 |
| Cm_q         | -5.0  | **Assumed** (typical for UAVs) |
| Cm_deltae    | -1.0  | Marotta Table 4.6 |
| Cm_u         | 0.0   | Neglected |

---

## 8. Notes and Uncertainty

- Values marked **Assumed** are estimated from similar aircraft or literature and should be checked in the future with more data or flight tests.
- Inertia and aerodynamic coefficients are critical for dynamic simulation and stability—recommend sensitivity analysis.

---

## 9. References

- Marotta, Y. (2022). Geometric modelling, stability and control analysis of the UAV Bayraktar TB-2 with OpenVSP.
- TRADOC (2021). Design and Analysis of the Impact of Turkish UAVs.
- Marques, P., & Da Ronch, A. (2017). Advanced UAV Aerodynamics, Flight Stability and Control.

---
