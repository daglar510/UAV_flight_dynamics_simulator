# UAV Flight-Dynamics Simulator – Technical References

The following publications, textbooks and standards were consulted (or directly implemented) while developing the *UAV Flight-Dynamics Simulator*.

## 1  Equations of Motion and Stability Theory

| Ref | Scope in Simulator | Bibliographic Reference |
|---|---|---|
| EOM-6DOF | Non-linear rigid-body six-degree-of-freedom equations, body-axis gravity transformation, inertia coupling. | Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation* (2nd ed.). John Wiley & Sons. §§1.5–1.7 |
| EOM-4DOF | Linearised longitudinal (u, α, q, θ) state-space matrices and derivative definitions. | Etkin, B., & Reid, L. D. (1996). *Dynamics of Flight: Stability and Control* (3rd ed.). Wiley. ch. 6 |
| Trim | Steady-flight lift & moment equilibrium; linear solution for (α, δe). | Cook, M. V. (2012). *Flight Dynamics Principles* (3rd ed.). Butterworth-Heinemann. §3.4 |
| Lat/Dir Derivatives | CY, Cl, Cn derivative signs and scaling. | Nelson, R. C. (1998). *Flight Stability and Automatic Control* (2nd ed.). McGraw–Hill. Appx A |

## 2  Aerodynamic Models

| Ref | Use | Bibliographic Reference |
|---|---|---|
| Low-speed fixed-wing UAV coefficients | Baseline CL₀, CLα, etc., for TB2/Anka family (public-domain estimates). | Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory & Practice*. Princeton UP. ch. 3 |
| Drag polar linearisation | CD ≈ CD₀ + CDα·α | Roskam, J. (2000). *Airplane Flight Dynamics and Automatic Flight Controls*, Part I. DARcorporation. |

## 3  Numerical Integration

| Ref | Use | Bibliographic Reference |
|---|---|---|
| RK45 + error control | `scipy.integrate.solve_ivp` (Dormand–Prince 5(4)) with tight tolerances for flight dynamics. | Hairer, E., Nørsett, S. P., & Wanner, G. (2008). *Solving Ordinary Differential Equations I*. Springer. |
| Step-size safety | Velocity cap and `max_step` safeguard to prevent stiffness blow-up. | Zipfel, P. H. (2007). *Modeling and Simulation of Aerospace Vehicle Dynamics* (2nd ed.). AIAA. §11.3 |

## 4  Software Architecture & Tooling

| Ref | Context | Bibliographic Reference |
|---|---|---|
| Static typing & CI | Use of `mypy` for gradual typing in Python. | *mypy Documentation* (2025). <https://mypy.readthedocs.io/> |
| Front-end framework | Reactive web UI built with *Reflex* (formerly Pynecone). | Reflex Docs v0.7.8 alpha. |
| Plotting | Time-series and 3-D visuals implemented with *Plotly*. | Plotly Python Graphing Library Docs. |

## 5  Validation & Best Practices

| Ref | Topic | Bibliographic Reference |
|---|---|---|
| Unit & Regression testing | Checklist for scientific code reviews. | Swimm Team. (2025). *Ultimate 10-Step Code Review Checklist*. <https://swimm.io/learn/code-reviews/ultimate-10-step-code-review-checklist> |

---
*Last updated*: 29 Jun 2025 