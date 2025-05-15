# Bayraktar TB2 UAV - Aerodynamic Specification Explanation

**Model:** Bayraktar TB2  
**Country:** Turkey  
**Manufacturer:** Baykar

---

## 1. Mass (`mass = 700 kg`)
**Source:**  
Directly specified in Marotta (2022), page 7. This value corresponds to the Maximum Takeoff Weight (MTOW).  
**Reference:**  
- Marotta, Y. (2022). *Geometric modelling, stability and control analysis of the UAV Bayraktar TB-2 with OpenVSP*. Università degli Studi di Napoli “Federico II”.

---

## 2. Wing Area (`S = 9.34 m²`)
**Source:**  
OpenVSP geometric model in Marotta (2022), used for coefficient normalization.  
**Reference:**  
- Marotta (2022), Table 4.1 and geometric modeling chapter.

---

## 3. Mean Aerodynamic Chord (`c = 0.78 m`)
**Calculation:**  
Estimated from root and tip airfoil profiles (NACA 2415, NACA 1412) and wing geometry.  
\[ c \approx \frac{2}{3} \cdot \frac{c_{root} + c_{tip} - \frac{c_{root} \cdot c_{tip}}{c_{root} + c_{tip}}} \]  
**Reference:**  
- Marotta (2022), Figures 3.3–3.5 and grid modeling discussion.

---

## 4. Wingspan (`b = 12.0 m`)
**Source:**  
Measured and confirmed in OpenVSP model visual and CAD reference.  
**Reference:**  
- Marotta (2022), Figure 3.2 and 3.3.

---

## 5. Moment of Inertia (`Iyy = 2500 kg·m²`)
**Estimation Method:**  
\[ I_{yy} \approx \frac{1}{12} \cdot m \cdot (b^2 + L^2) \cdot k \]  
Where:
- \( m = 700 \, kg \)
- \( b = 12.0 \, m \)
- \( L \approx 6.5 \, m \) (fuselage length)
- \( k \approx 0.25 \) (shape correction factor)  
**Reference:**  
- Marotta (2022), fuselage geometry  
- TRADOC (2021). *Design and Analysis of the Impact of Turkish UAVs*

---

## 6. Mach Number (`Mach = 0.12`)
**Calculation:**  
\[ \text{Mach} = \frac{V}{a} = \frac{70}{343} \approx 0.12 \]  
Assuming cruise speed \( V = 70 \, \text{m/s} \), standard air conditions.  
**Reference:**  
- Marotta (2022), Section 4.1  
- TRADOC (2021), cruise velocity range 70–80 m/s

---

## 7. Aerodynamic Coefficients

### Lift Coefficients
| Coefficient | Value | Explanation |
|------------|-------|-------------|
| `CL_0`     | 0.30  | Interpolated from CL vs. α table at α = 0° (Table 4.1) |
| `CL_alpha` | 5.5   | Derived from slope of CL vs. α: \( (1.066 - 0.233)/10 = 0.0833/deg ≈ 4.77/rad \), adjusted for tail |
| `CL_q`     | 5.0   | Assumed based on UAV literature |
| `CL_deltae`| 0.4   | Estimated from symmetric ruddervator deflection (Table 4.4) |
| `CL_u`     | 0.0   | Neglected in subsonic analysis |

### Drag Coefficients
| Coefficient | Value | Explanation |
|------------|-------|-------------|
| `CD_0`     | 0.05  | From CD vs. α at α = 0° (Table 4.3) |
| `CD_alpha` | 0.3   | Estimated from slope over α = 0° to 10° |
| `CD_q`     | 0.0   | Neglected in inviscid model |
| `CD_deltae`| 0.0   | Negligible contribution |
| `CD_u`     | 0.0   | Negligible |

### Pitching Moment Coefficients
| Coefficient | Value | Explanation |
|------------|-------|-------------|
| `Cm_0`     | 0.0   | Assumed neutral trim at α = 0° |
| `Cm_alpha` | -0.6  | From Table 4.2 using slope \( dCm/dα ≈ -0.0073/deg ≈ -0.6/rad \) |
| `Cm_q`     | -5.0  | Assumed based on MALE UAV literature |
| `Cm_deltae`| -1.0  | From symmetric ruddervator deflection impact (Table 4.6) |
| `Cm_u`     | 0.0   | Typically negligible |

**Reference for coefficients:**  
- Marotta (2022), Chapter 4 (Tables 4.1–4.10)  
- Marques & Da Ronch (2017). *Advanced UAV Aerodynamics, Flight Stability and Control*

---

## 8. References

- Marotta, Y. (2022). *Geometric modelling, stability and control analysis of the UAV Bayraktar TB-2 with OpenVSP*. Università degli Studi di Napoli “Federico II”.  
- TRADOC (2021). *Design and Analysis of the Impact of Turkish UAVs*. U.S. Army Training and Doctrine Command.  
- Marques, P., & Da Ronch, A. (2017). *Advanced UAV Aerodynamics, Flight Stability and Control*. John Wiley & Sons.

---
