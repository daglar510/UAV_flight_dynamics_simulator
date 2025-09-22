import reflex as rx
import numpy as np
import math
from scipy.integrate import solve_ivp
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import Dict, List, Any, Tuple
import copy
from flight_dynamics_simulator_ui_design.utils.types import (
    UAVParameter,
    PulseData,
    UAVParameterValue,
)
from flight_dynamics_simulator_ui_design.utils.uav_models import UAV_DB_INITIAL
from flight_dynamics_simulator_ui_design.utils import constants as C
from flight_dynamics_simulator_ui_design.visualization.native import (
    create_time_domain_plot,
    create_3d_trajectory_plot,
    create_time_domain_plot_6dof,
    create_3d_trajectory_plot_6dof,
    NativeVisualizer,
)
from flight_dynamics_simulator_ui_design.visualization.flightgear import FlightGearVisualizer

DEFAULT_UAV_NAME = "TB2"


class FlightSimState(rx.State):
    simulation_mode: str = "4DOF"  # or "6DOF"
    uav_database: Dict[str, UAVParameter] = copy.deepcopy(
        UAV_DB_INITIAL
    )
    selected_uav_name: str = DEFAULT_UAV_NAME
    editable_params: UAVParameter = copy.deepcopy(
        UAV_DB_INITIAL[DEFAULT_UAV_NAME]
    )
    pulses: List[PulseData] = [
        {
            "start_time": 1.0,
            "duration": 0.5,
            "angle_deg": 2.0,
            "roll_deg": 0.0,
            "yaw_deg": 0.0,
            "throttle": 1.0,
        }
    ]
    simulation_duration: float = 10.0
    trim_speed_output: str = ""
    eigen_analysis_output: List[str] = []
    time_domain_plot_figure: go.Figure | None = None
    trajectory_plot_figure: go.Figure | None = None
    is_simulating: bool = False
    trim_speed_val_holder: float = 0.0

    visualization_mode: str = "native"

    def set_visualization_mode(self, mode: str):
        self.visualization_mode = mode

    def _get_uav_param_keys(self) -> list[str]:
        return [
            k
            for k in UAVParameter.__annotations__.keys()
            if k not in ["company", "country"]
        ]

    @rx.var
    def uav_names(self) -> List[str]:
        return list(self.uav_database.keys())

    @rx.var
    def current_editable_params_list(
        self,
    ) -> list[tuple[str, str]]:
        if self.editable_params is None:
            return []
        return [
            (k, str(self.editable_params[k]))
            for k in self._get_uav_param_keys()
            if k in self.editable_params
        ]

    def handle_uav_selection(self, name: str):
        self.selected_uav_name = name
        self.editable_params = copy.deepcopy(
            self.uav_database[name]
        )
        # Force UI refresh for the editable params
        self._clear_results()

    def update_editable_param(
        self, key: str, value_str: str
    ):
        try:
            value = float(value_str)
            if (
                self.editable_params
                and key in self.editable_params
            ):
                self.editable_params[key] = value
        except (ValueError, TypeError):
            yield rx.toast.error(
                f"Invalid input for {key}: '{value_str}' must be a number."
            )
            print(
                f"Error converting {value_str} to float for key {key}"
            )

    def add_pulse(self):
        self.pulses.append(
            {
                "start_time": 0.0,
                "duration": 1.0,
                "angle_deg": 0.0,
            }
        )

    def update_pulse_param(
        self, index: int, key: str, value_str: str
    ):
        try:
            value = float(value_str)
            if 0 <= index < len(self.pulses):
                self.pulses[index][key] = value
        except ValueError:
            yield rx.toast.error(
                f"Invalid input for pulse {key}: '{value_str}' must be a number."
            )
            print(
                f"Error converting {value_str} to float for pulse param {key}"
            )

    def remove_pulse(self, index: int):
        if 0 <= index < len(self.pulses):
            self.pulses.pop(index)

    def _clear_results(self):
        self.trim_speed_output = ""
        self.eigen_analysis_output = []
        self.time_domain_plot_figure = None
        self.trajectory_plot_figure = None
        self.trim_speed_val_holder = 0.0

    def save_uav_config(self):
        self.uav_database[self.selected_uav_name] = (
            copy.deepcopy(self.editable_params)
        )
        yield rx.toast.info(
            f"Configuration for {self.selected_uav_name} saved."
        )

    def reset_current_uav_to_saved(self):
        self.editable_params = copy.deepcopy(
            self.uav_database[self.selected_uav_name]
        )
        self.pulses = [
            {
                "start_time": 1.0,
                "duration": 0.5,
                "angle_deg": 2.0,
            }
        ]
        self.simulation_duration = 10.0
        self._clear_results()
        yield rx.toast.info(
            f"Inputs reset for {self.selected_uav_name}."
        )

    def _build_state_space(
        self, uav_p: UAVParameter
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        m, S, c_bar, Iyy, mach_val = (
            uav_p[k]
            for k in ("mass", "S", "c", "Iyy", "Mach")
        )
        U0 = mach_val * C.A_SOUND_SEA_LEVEL
        rho = C.RHO0
        m1 = m / (0.5 * rho * U0 * S)
        c1 = c_bar / (2 * U0)
        Jy1 = Iyy / (0.5 * rho * U0**2 * S * c_bar)
        CL0, CLa, CLq, CLde, CLu = (
            uav_p[k]
            for k in (
                "CL_0",
                "CL_alpha",
                "CL_q",
                "CL_deltae",
                "CL_u",
            )
        )
        CD0, CDa, CDq, CDde, CDu = (
            uav_p[k]
            for k in (
                "CD_0",
                "CD_alpha",
                "CD_q",
                "CD_deltae",
                "CD_u",
            )
        )
        Cm0, Cma, Cmq, Cmde, Cmu = (
            uav_p[k]
            for k in (
                "Cm_0",
                "Cm_alpha",
                "Cm_q",
                "Cm_deltae",
                "Cm_u",
            )
        )
        CXu = -2 * CD0 - CDu
        CXa = -CDa + CL0
        CXq = -CDq
        CZu = -2 * CL0 - CLu
        CZa = -CLa - CD0
        CZq = -CLq
        Cmu_val = 2 * Cm0 + Cmu
        Cma_val = Cma + Cm0
        Cmq_val = Cmq
        CXde = -CDde
        CZde = -CLde
        Cmde_val = Cmde
        M_matrix = np.array(
            [
                [m1, 0, 0, 0],
                [0, m1, 0, 0],
                [0, 0, Jy1, 0],
                [0, 0, 0, 1],
            ]
        )
        K_matrix = np.array(
            [
                [-CXu, -CXa, -CXq, m1 * (C.G / U0)],
                [-CZu, -CZa, -c1 * CZq - m1, 0],
                [-Cmu_val, -Cma_val, -c1 * Cmq_val, 0],
                [0, 0, -1, 0],
            ]
        )
        B_matrix = np.array(
            [[0, CXde], [0, CZde], [0, Cmde_val], [0, 0]]
        )
        A_matrix = np.linalg.solve(-M_matrix, K_matrix)
        Bt_matrix = np.linalg.solve(M_matrix, B_matrix)
        return (A_matrix, Bt_matrix, U0)

    def _analyze_modes(self, A_matrix: np.ndarray) -> List[str]:
        eigvals, _ = np.linalg.eig(A_matrix)
        output = ["Flight Dynamics Modes (Eigenvalues of A):"]
        
        # Sort eigenvalues by natural frequency (highest to lowest)
        mode_data = []
        for l_val in eigvals:
            wn = np.abs(l_val)
            sigma = l_val.real
            zeta = -sigma / wn if wn > 0 else 0
            mode_data.append((l_val, wn, zeta))
        
        # Sort by natural frequency (highest to lowest)
        mode_data.sort(key=lambda x: x[1], reverse=True)
        
        # Group complex conjugate pairs
        processed_indices = set()
        modes = []
        
        for i, (l_val, wn, zeta) in enumerate(mode_data):
            if i in processed_indices:
                continue
            
            # Check if this is part of a complex conjugate pair
            is_complex = abs(l_val.imag) > 1e-5
            
            if is_complex:
                # Find the conjugate pair
                for j, (other_val, _, _) in enumerate(mode_data):
                    if i != j and abs(l_val.real - other_val.real) < 1e-5 and abs(l_val.imag + other_val.imag) < 1e-5:
                        processed_indices.add(i)
                        processed_indices.add(j)
                        
                        # Identify the mode
                        if wn > 0.5:  # Higher frequency mode
                            mode_name = "Short Period Mode"
                        else:  # Lower frequency mode
                            mode_name = "Phugoid Mode"
                        
                        modes.append((mode_name, l_val, wn, zeta))
                        break
            else:
                processed_indices.add(i)
                # Real eigenvalue (subsidence mode)
                mode_name = "Subsidence Mode"
                modes.append((mode_name, l_val, wn, zeta))
        
        # Output the modes
        for i, (mode_name, l_val, wn, zeta) in enumerate(modes):
            mode_str = f"{mode_name}: lambda = {l_val:.4f} | wn = {wn:.4f} rad/s | zeta = {zeta:.3f}"
            output.append(mode_str)
            
            if zeta < 0:
                output.append("  --> UNSTABLE (Danger: positive real part!)")
            elif zeta < 0.02:
                output.append("  --> Very poorly damped (will oscillate significantly)")
            elif zeta < 0.2:
                output.append("  --> Poorly damped (will oscillate a lot)")
            elif zeta < 0.7:
                output.append("  --> Good aircraft mode")
            else:
                output.append("  --> Highly damped (probably not oscillatory)")
        
        return output

    def _simulate_response(
        self,
        A_matrix: np.ndarray,
        B_matrix: np.ndarray,
        pulses_input: List[PulseData],
        duration: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        t_eval = np.linspace(0, duration, 500)

        def delta_e_func(t_val):
            for p_item in pulses_input:
                if (
                    t_val >= p_item["start_time"]
                    and t_val
                    <= p_item["start_time"]
                    + p_item["duration"]
                ):
                    return p_item["angle_deg"] * np.pi / 180
            return 0.0

        def delta_T_func(t_val):
            return 0.0

        def f_ode(t_val, y_val):
            delta_input = np.array(
                [delta_T_func(t_val), delta_e_func(t_val)]
            )
            return A_matrix @ y_val + B_matrix @ delta_input

        y0 = np.zeros(4)
        sol = solve_ivp(
            f_ode,
            [0, duration],
            y0,
            t_eval=t_eval,
            rtol=1e-06,
            atol=1e-06,
        )
        de_array = np.array(
            [delta_e_func(tt) for tt in sol.t]
        )
        return (sol.t, sol.y, de_array)

    def set_simulation_mode(self, mode: str):
        if mode in ["4DOF", "6DOF"]:
            self.simulation_mode = mode
            self._clear_results()
            # Optionally reset pulses to match mode

    @rx.var
    def current_simulation_mode(self) -> str:
        return self.simulation_mode

    def _build_state_space_6dof(self, uav_p: UAVParameter) -> Tuple[Any, Any, float]:
        # Extract parameters
        m = uav_p["mass"]
        S = uav_p["S"]
        c = uav_p["c"]
        b = uav_p["b"]
        Ixx = uav_p["ixx"]
        Iyy = uav_p["Iyy"]
        Izz = uav_p["izz"]
        Ixz = uav_p["ixz"]
        Mach = uav_p["Mach"]
        U0 = Mach * C.A_SOUND_SEA_LEVEL
        rho = C.RHO0
        # Non-dimensionalization factors
        q_bar = 0.5 * rho * U0 * U0
        # Return all needed parameters for ODE
        return {
            "m": m, "S": S, "c": c, "b": b, "Ixx": Ixx, "Iyy": Iyy, "Izz": Izz, "Ixz": Ixz, "U0": U0, "rho": rho, "q_bar": q_bar,
            # Aerodynamic derivatives
            "CL_0": uav_p["CL_0"], "CL_alpha": uav_p["CL_alpha"], "CL_q": uav_p["CL_q"], "CL_deltae": uav_p["CL_deltae"], "CL_u": uav_p["CL_u"],
            "CD_0": uav_p["CD_0"], "CD_alpha": uav_p["CD_alpha"], "CD_q": uav_p["CD_q"], "CD_deltae": uav_p["CD_deltae"], "CD_u": uav_p["CD_u"],
            "Cm_0": uav_p["Cm_0"], "Cm_alpha": uav_p["Cm_alpha"], "Cm_q": uav_p["Cm_q"], "Cm_deltae": uav_p["Cm_deltae"], "Cm_u": uav_p["Cm_u"],
            # Lateral-directional derivatives
            "CYb": uav_p["CYb"], "CYp": uav_p["CYp"], "CYr": uav_p["CYr"], "CYda": uav_p["CYda"], "CYdr": uav_p["CYdr"],
            "Clb": uav_p["Clb"], "Clp": uav_p["Clp"], "Clr": uav_p["Clr"], "Clda": uav_p["Clda"], "Cldr": uav_p["Cldr"],
            "Cnb": uav_p["Cnb"], "Cnp": uav_p["Cnp"], "Cnr": uav_p["Cnr"], "Cnda": uav_p["Cnda"], "Cndr": uav_p["Cndr"],
        }, U0

    # ---------------------------------------------------------------------
    # Trim solver for 6-DOF longitudinal equilibrium (level, wings-level)
    # Solves for angle-of-attack (alpha) and elevator deflection (de) such
    # that vertical force and pitching moment are zero.
    # ---------------------------------------------------------------------
    def _compute_trim_6dof(self, params: dict):
        """Return alpha_trim (rad) and elevator deflection trim (rad)."""
        q_bar = params["q_bar"]
        S = params["S"]
        c = params["c"]
        m = params["m"]
        g = C.G

        # Aerodynamic derivatives
        CL0 = params["CL_0"]
        CL_alpha = params["CL_alpha"]
        CL_de = params["CL_deltae"]
        Cm0 = params["Cm_0"]
        Cm_alpha = params["Cm_alpha"]
        Cm_de = params["Cm_deltae"]

        # Required lift coefficient for level flight
        CL_trim_required = m * g / (q_bar * S)

        # Linear system:  [CL_alpha  CL_de ] [alpha] = CL_trim_required - CL0
        #                 [Cm_alpha  Cm_de] [de   ] = -Cm0
        A = np.array([[CL_alpha, CL_de], [Cm_alpha, Cm_de]])
        b = np.array([CL_trim_required - CL0, -Cm0])
        try:
            alpha_trim, de_trim = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            # Fallback: zero elevator, compute alpha only
            alpha_trim = (CL_trim_required - CL0) / (CL_alpha if CL_alpha != 0 else 1.0)
            de_trim = 0.0
        return alpha_trim, de_trim

    def _simulate_response_6dof(self, params: dict, pulses_input: List[PulseData], duration: float):
        # State: [u, v, w, p, q, r, phi, theta, psi]

        # Compute trimmed initial conditions
        alpha_trim, de_trim = self._compute_trim_6dof(params)

        t_eval = np.linspace(0, duration, 500)
        m, S, c, b = params["m"], params["S"], params["c"], params["b"]
        Ixx, Iyy, Izz, Ixz = params["Ixx"], params["Iyy"], params["Izz"], params["Ixz"]
        U0 = params["U0"]
        q_bar = params["q_bar"]
        g = C.G
        # Helper: get control input at time t
        def get_controls(t):
            roll_cmd = pitch_cmd = yaw_cmd = 0.0  # command increments (rad)
            throttle_sum = 0.0
            count = 0
            for p in pulses_input:
                if p["start_time"] <= t <= p["start_time"] + p["duration"]:
                    roll_cmd += np.deg2rad(p.get("roll_deg", 0.0))
                    # Positive "angle_deg" means pilot pulls BACK -> elevator UP (negative deflection)
                    pitch_cmd += -np.deg2rad(p.get("angle_deg", 0.0))
                    yaw_cmd += np.deg2rad(p.get("yaw_deg", 0.0))
                    throttle_sum += p.get("throttle", 1.0)
                    count += 1
            throttle = throttle_sum / count if count > 0 else 1.0
            return (roll_cmd, pitch_cmd, yaw_cmd, throttle)
        # ODE function
        def f_ode(t, y):
            u, v, w, p, q, r, phi, theta, psi, x, y_pos, z = y
            da_cmd, de_cmd, dr_cmd, throttle = get_controls(t)
            # Total surface deflections = trim + command increments
            de = de_trim + de_cmd
            da = da_cmd
            dr = dr_cmd

            # Instantaneous angle-of-attack (rad)
            alpha_inst = np.arctan2(w, u)

            # Updated aerodynamic coefficients (depend on alpha, not theta)
            CL = (
                params["CL_0"]
                + params["CL_alpha"] * alpha_inst
                + params["CL_q"] * q * c / (2 * U0)
                + params["CL_deltae"] * de
            )
            CD = (
                params["CD_0"]
                + params["CD_alpha"] * alpha_inst
                + params["CD_q"] * q * c / (2 * U0)
                + params["CD_deltae"] * de
            )
            Cm = (
                params["Cm_0"]
                + params["Cm_alpha"] * alpha_inst
                + params["Cm_q"] * q * c / (2 * U0)
                + params["Cm_deltae"] * de
            )
            CY = params["CYb"] * v / U0 + params["CYp"] * p * b / (2 * U0) + params["CYr"] * r * b / (2 * U0) + params["CYda"] * da + params["CYdr"] * dr
            Cl = params["Clb"] * v / U0 + params["Clp"] * p * b / (2 * U0) + params["Clr"] * r * b / (2 * U0) + params["Clda"] * da + params["Cldr"] * dr
            Cn = params["Cnb"] * v / U0 + params["Cnp"] * p * b / (2 * U0) + params["Cnr"] * r * b / (2 * U0) + params["Cnda"] * da + params["Cndr"] * dr
            # Dynamic pressure with speed cap to avoid numerical blow-up
            V_true = np.sqrt(u * u + v * v + w * w)
            V_capped = np.clip(V_true, 1.0, 150.0)  # 150 m/s ≈ 540 km/h
            q_bar_dyn = 0.5 * params["rho"] * V_capped * V_capped

            # Aerodynamic and propulsion forces in body axes (use capped q_bar)
            X = -q_bar_dyn * S * CD + throttle * 100.0
            Y = q_bar_dyn * S * CY
            Z = -q_bar_dyn * S * CL

            # Pre-compute trigonometric terms for gravity transformation
            sphi = np.sin(phi)
            cphi = np.cos(phi)
            sthe = np.sin(theta)
            cthe = np.cos(theta)

            # Gravity components expressed in body frame (positive z points down)
            # Reference: Stevens & Lewis, Eq. 1.5-13
            Xg = -m * g * sthe
            Yg = m * g * sphi * cthe
            Zg = m * g * cphi * cthe

            # Total forces (aero + thrust + gravity)
            X_tot = X + Xg
            Y_tot = Y + Yg
            Z_tot = Z + Zg

            # Moments remain unchanged (about body axes)
            L = q_bar_dyn * S * b * Cl
            M = q_bar_dyn * S * c * Cm
            N = q_bar_dyn * S * b * Cn

            # Translational equations of motion (body axes)
            # Positive directions: x-forward, y-right, z-down
            u_dot = r * v - q * w + X_tot / m
            v_dot = p * w - r * u + Y_tot / m
            w_dot = q * u - p * v + Z_tot / m
            denom = Ixx * Izz - Ixz ** 2
            p_dot = (Izz * L + Ixz * N - (Ixz * (Iyy - Ixx - Izz) * p * r + (Ixz ** 2 + Izz * (Izz - Iyy)) * q * r)) / denom
            q_dot = (M / Iyy)
            r_dot = (Ixx * N + Ixz * L + (Ixz * (Iyy - Ixx - Izz) * p * q + (Ixz ** 2 + Ixx * (Ixx - Iyy)) * q * r)) / denom
            phi_dot = p + np.sin(phi) * np.tan(theta) * q + np.cos(phi) * np.tan(theta) * r
            theta_dot = np.cos(phi) * q - np.sin(phi) * r
            psi_dot = np.sin(phi) / np.cos(theta) * q + np.cos(phi) / np.cos(theta) * r
            # Position integration (NED/inertial)
            # Rotation matrix from body to inertial
            cth = np.cos(theta); sth = np.sin(theta)
            cph = np.cos(phi); sph = np.sin(phi)
            cps = np.cos(psi); sps = np.sin(psi)
            R = np.array([
                [cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
                [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
                [-sth,      sph * cth,                  cph * cth],
            ])
            vel_body = np.array([u, v, w])
            vel_inertial = R @ vel_body
            x_dot, y_dot, z_dot = vel_inertial
            return [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot, x_dot, y_dot, z_dot]
        # Initial state vector at trim
        u0_body = U0 * np.cos(alpha_trim)
        w0_body = U0 * np.sin(alpha_trim)
        y0 = np.array(
            [
                u0_body,  # u
                0.0,      # v
                w0_body,  # w
                0.0, 0.0, 0.0,  # p q r
                0.0,           # phi
                alpha_trim,    # theta ~ alpha for small angles
                0.0,           # psi
                0.0, 0.0, 0.0,  # x y z initial pos
            ]
        )
        sol = solve_ivp(
            f_ode,
            [0, duration],
            y0,
            t_eval=t_eval,
            rtol=1e-6,
            atol=1e-6,
            max_step=0.1,
        )
        da_hist, de_hist, dr_hist, throttle_hist = [], [], [], []
        for tt in sol.t:
            da_cmd, de_cmd, dr_cmd, th = get_controls(tt)
            da_hist.append(np.rad2deg(da_cmd))
            de_hist.append(np.rad2deg(de_cmd))
            dr_hist.append(np.rad2deg(dr_cmd))
            throttle_hist.append(th)
        # Calculate derived quantities
        u, v, w = sol.y[0], sol.y[1], sol.y[2]
        x, y_pos, z = sol.y[9], sol.y[10], sol.y[11]
        alpha = np.degrees(np.arctan2(w, u))
        airspeed = np.sqrt(u**2 + v**2 + w**2)
        airspeed_safe = np.maximum(airspeed, 1e-3)
        beta = np.degrees(np.arcsin(np.clip(v / airspeed_safe, -1.0, 1.0)))
        kinetic = 0.5 * m * airspeed**2
        potential = m * g * (-z)  # NED: z down
        return sol.t, sol.y, np.array(da_hist), np.array(de_hist), np.array(dr_hist), np.array(throttle_hist), x, y_pos, z, alpha, beta, airspeed, kinetic, potential

    @rx.event(background=True)
    async def run_simulation(self):
        async with self:
            self.is_simulating = True
            self._clear_results()
            current_params_local = copy.deepcopy(self.editable_params)
            current_pulses_local = copy.deepcopy(self.pulses)
            current_sim_duration_local = self.simulation_duration
            current_uav_name_local = self.selected_uav_name
            current_mode = self.simulation_mode

        visualizer = None
        if self.visualization_mode == "flightgear":
            visualizer = FlightGearVisualizer()
        else:
            visualizer = NativeVisualizer()
        
        visualizer.initialize({})

        if not current_params_local:
            async with self:
                self.is_simulating = False
            yield rx.toast.error(
                "UAV parameters are not loaded. Please select a UAV."
            )
            return

        try:
            if current_mode == "4DOF":
                A_matrix, B_matrix, u0_calculated = self._build_state_space(current_params_local)
                eigen_analysis_results = self._analyze_modes(A_matrix)
                t_results, y_results, de_results = self._simulate_response(
                    A_matrix, B_matrix, current_pulses_local, current_sim_duration_local
                )
                time_domain_fig_obj = create_time_domain_plot(
                    t_results, y_results, de_results, current_uav_name_local, u0_calculated
                )
                trajectory_fig_obj = create_3d_trajectory_plot(
                    y_results, current_uav_name_local
                )
                async with self:
                    self.trim_speed_val_holder = u0_calculated
                    self.trim_speed_output = f"Trim speed U0 = {u0_calculated:.2f} m/s ({u0_calculated * 1.94384:.2f} knots)"
                    self.eigen_analysis_output = eigen_analysis_results
                    self.time_domain_plot_figure = time_domain_fig_obj
                    self.trajectory_plot_figure = trajectory_fig_obj
                    self.is_simulating = False
                yield rx.toast.success("Simulation Complete!")
            else:  # 6DOF
                params, u0_calculated = self._build_state_space_6dof(current_params_local)
                t_results, y_results, da, de, dr, throttle, x, y_pos, z, alpha, beta, airspeed, kinetic, potential = self._simulate_response_6dof(
                    params, current_pulses_local, current_sim_duration_local
                )
                time_domain_fig_obj = create_time_domain_plot_6dof(
                    t_results, y_results, da, de, dr, throttle, x, y_pos, z, alpha, beta, airspeed, kinetic, potential, current_uav_name_local
                )
                trajectory_fig_obj = create_3d_trajectory_plot_6dof(
                    y_vals=y_results, x=x, y_pos=y_pos, z=z, uav_name=current_uav_name_local
                )

                if self.visualization_mode == "flightgear":
                    for i in range(len(t_results)):
                        state_vector = {
                            "longitude": y_pos[i],
                            "latitude": x[i],
                            "altitude": -z[i],
                            "roll": np.degrees(y_results[6][i]),
                            "pitch": np.degrees(y_results[7][i]),
                            "heading": np.degrees(y_results[8][i]),
                            "speed": airspeed[i],
                            "rate_roll": np.degrees(y_results[3][i]),
                            "rate_pitch": np.degrees(y_results[4][i]),
                            "rate_yaw": np.degrees(y_results[5][i]),
                        }
                        visualizer.update(state_vector, t_results[i])

                async with self:
                    self.trim_speed_val_holder = u0_calculated
                    self.trim_speed_output = f"Trim speed U0 = {u0_calculated:.2f} m/s ({u0_calculated * 1.94384:.2f} knots)"
                    self.eigen_analysis_output = ["6DOF simulation: Eigen analysis not implemented."]
                    self.time_domain_plot_figure = time_domain_fig_obj
                    self.trajectory_plot_figure = trajectory_fig_obj
                    self.is_simulating = False
                yield rx.toast.success("6DOF Simulation Complete!")
        except Exception as e:
            yield rx.toast.error(f"Simulation Error: {e}")
            print(f"Simulation Error: {type(e).__name__} - {e}")
        finally:
            if visualizer:
                visualizer.cleanup()
            async with self:
                self.is_simulating = False
