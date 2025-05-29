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

DEFAULT_UAV_NAME = "TB2"


class FlightSimState(rx.State):
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
        }
    ]
    simulation_duration: float = 10.0
    trim_speed_output: str = ""
    eigen_analysis_output: List[str] = []
    time_domain_plot_figure: go.Figure | None = None
    trajectory_plot_figure: go.Figure | None = None
    is_simulating: bool = False
    trim_speed_val_holder: float = 0.0

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

    def _create_time_domain_plot(
        self,
        t_vals: np.ndarray,
        y_vals: np.ndarray,
        de_vals: np.ndarray,
        uav_name: str,
        u0_val: float,
    ) -> go.Figure:
        fig = make_subplots(
            rows=3,
            cols=2,
            subplot_titles=(
                f"Forward Speed Perturbation u<sub>p</sub> (m/s)",
                "AoA &alpha; (deg)",
                "Pitch Rate q (deg/s)",
                "Pitch &theta; (deg)",
                "Elevator &delta;<sub>e</sub> (deg)",
                "",
            ),
            vertical_spacing=0.12,
        )
        fig.add_trace(
            go.Scatter(
                x=t_vals, y=y_vals[0] * u0_val, name="u_p"
            ),
            row=1,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t_vals, y=np.degrees(y_vals[1]), name="α"
            ),
            row=1,
            col=2,
        )
        fig.add_trace(
            go.Scatter(
                x=t_vals, y=np.degrees(y_vals[2]), name="q"
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t_vals, y=np.degrees(y_vals[3]), name="θ"
            ),
            row=2,
            col=2,
        )
        fig.add_trace(
            go.Scatter(
                x=t_vals, y=np.degrees(de_vals), name="δe"
            ),
            row=3,
            col=1,
        )
        fig.update_layout(
            title_text=f"{uav_name} Time Domain Response",
            height=700,
            showlegend=False,
            margin=dict(t=80, b=30, l=50, r=30),
        )
        fig.update_xaxes(title_text="Time (s)")
        fig.update_yaxes(title_font=dict(size=10))
        return fig

    def _create_3d_trajectory_plot(
        self, y_vals: np.ndarray, uav_name: str
    ) -> go.Figure:
        fig = go.Figure(
            data=[
                go.Scatter3d(
                    x=np.degrees(y_vals[1]),
                    y=np.degrees(y_vals[2]),
                    z=np.degrees(y_vals[3]),
                    mode="lines",
                    name="Trajectory",
                    line=dict(color="blue", width=2),
                )
            ]
        )
        fig.update_layout(
            height=600,
            title=f"{uav_name} Trajectory in (&alpha;, q, &theta;) space",
            scene=dict(
                xaxis_title="AoA &alpha; (deg)",
                yaxis_title="Pitch rate q (deg/s)",
                zaxis_title="Pitch &theta; (deg)",
            ),
            margin=dict(l=0, r=0, b=0, t=50),
        )
        return fig

    @rx.event(background=True)
    async def run_simulation(self):
        async with self:
            self.is_simulating = True
            self._clear_results()
            current_params_local = copy.deepcopy(
                self.editable_params
            )
            current_pulses_local = copy.deepcopy(
                self.pulses
            )
            current_sim_duration_local = (
                self.simulation_duration
            )
            current_uav_name_local = self.selected_uav_name
        if not current_params_local:
            async with self:
                self.is_simulating = False
            yield rx.toast.error(
                "UAV parameters are not loaded. Please select a UAV."
            )
            return
        u0_calculated = 0.0
        eigen_analysis_results = []
        t_results, y_results, de_results = (
            None,
            None,
            None,
        )
        time_domain_fig_obj = None
        trajectory_fig_obj = None
        try:
            A_matrix, B_matrix, u0_calculated = (
                self._build_state_space(
                    current_params_local
                )
            )
            eigen_analysis_results = self._analyze_modes(
                A_matrix
            )
            t_results, y_results, de_results = (
                self._simulate_response(
                    A_matrix,
                    B_matrix,
                    current_pulses_local,
                    current_sim_duration_local,
                )
            )
            time_domain_fig_obj = (
                self._create_time_domain_plot(
                    t_results,
                    y_results,
                    de_results,
                    current_uav_name_local,
                    u0_calculated,
                )
            )
            trajectory_fig_obj = (
                self._create_3d_trajectory_plot(
                    y_results, current_uav_name_local
                )
            )
            async with self:
                self.trim_speed_val_holder = u0_calculated
                self.trim_speed_output = f"Trim speed U0 = {u0_calculated:.2f} m/s ({u0_calculated * 1.94384:.2f} knots)"
                self.eigen_analysis_output = (
                    eigen_analysis_results
                )
                self.time_domain_plot_figure = (
                    time_domain_fig_obj
                )
                self.trajectory_plot_figure = (
                    trajectory_fig_obj
                )
                self.is_simulating = False
            yield rx.toast.success("Simulation Complete!")
        except Exception as e:
            async with self:
                self.is_simulating = False
            yield rx.toast.error(
                f"Simulation Error: {str(e)}"
            )
            print(
                f"Simulation Error: {type(e).__name__} - {e}"
            )