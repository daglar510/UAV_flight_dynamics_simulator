import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from typing import Dict, Any
from .base import VisualizationBase

class NativeVisualizer(VisualizationBase):
    def initialize(self, aircraft_params: Dict[str, Any]) -> None:
        pass

    def update(self, state_vector: Dict[str, float], time: float) -> None:
        pass

    def cleanup(self) -> None:
        pass

def create_time_domain_plot(
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
            f"u_p (m/s): Forward speed perturbation",
            "α (deg): Angle of attack",
            "q (deg/s): Pitch rate",
            "θ (deg): Pitch angle",
            "Elevator (deg): Input",
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
            x=t_vals, y=np.degrees(de_vals), name="Elevator input"
        ),
        row=3,
        col=1,
    )
    fig.update_layout(
        title_text=f"{uav_name} 4DOF Time Domain Response",
        height=700,
        showlegend=True,
        margin=dict(t=80, b=30, l=60, r=60),
        annotations=[dict(
            text="Each plot shows a key state or control variable for the UAV.",
            xref="paper", yref="paper", x=0, y=1.13, showarrow=False, font=dict(size=14)
        )],
    )
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_font=dict(size=11))
    return fig

def create_3d_trajectory_plot(
    y_vals: np.ndarray, uav_name: str
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

def create_time_domain_plot_6dof(t_vals, y_vals, da, de, dr, throttle, x, y_pos, z, alpha, beta, airspeed, kinetic, potential, uav_name):
    plot_titles = [
        "u (m/s): Forward speed",
        "v (m/s): Side speed",
        "w (m/s): Down speed",
        "p (deg/s): Roll rate",
        "q (deg/s): Pitch rate",
        "r (deg/s): Yaw rate",
        "phi (deg): Roll angle",
        "theta (deg): Pitch angle",
        "psi (deg): Yaw angle",
        "Roll input (deg)",
        "Pitch input (deg)",
        "Yaw input (deg)",
        "Throttle (0-1)",
        "Alpha (deg): AoA",
        "Beta (deg): Sideslip",
        "Airspeed (m/s)",
        "Kinetic E (J)",
        "Potential E (J)",
        "Altitude (m)",
        "x (m): East",
        "y (m): North",
    ]
    plot_explanations = [
        "Forward speed of the UAV.",
        "Sideways (lateral) speed.",
        "Downward (vertical) speed.",
        "How fast the UAV is rolling.",
        "How fast the UAV is pitching.",
        "How fast the UAV is yawing.",
        "Current roll angle (tilt left/right).",
        "Current pitch angle (nose up/down).",
        "Current yaw angle (heading direction).",
        "Command sent to roll (left/right).",
        "Command sent to pitch (up/down).",
        "Command sent to yaw (turn left/right).",
        "Throttle setting (engine power).",
        "Angle between airflow and wing (AoA).",
        "Side slip angle (wind from the side).",
        "Total speed through the air.",
        "Energy from motion.",
        "Energy from altitude.",
        "Height above ground (upwards).",
        "Eastward position.",
        "Northward position.",
    ]
    fig = make_subplots(
        rows=21, cols=1,
        subplot_titles=plot_titles,
        vertical_spacing=0.02,
    )
    fig.add_trace(go.Scatter(x=t_vals, y=y_vals[0], name="u"), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=y_vals[1], name="v"), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=y_vals[2], name="w"), row=3, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[3]), name="p"), row=4, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[4]), name="q"), row=5, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[5]), name="r"), row=6, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[6]), name="phi"), row=7, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[7]), name="theta"), row=8, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[8]), name="psi"), row=9, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=da, name="Roll input"), row=10, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=de, name="Pitch input"), row=11, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=dr, name="Yaw input"), row=12, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=throttle, name="Throttle"), row=13, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=alpha, name="Alpha (AoA)"), row=14, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=beta, name="Beta (sideslip)"), row=15, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=airspeed, name="Airspeed"), row=16, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=kinetic, name="Kinetic E"), row=17, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=potential, name="Potential E"), row=18, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=-z, name="Altitude (up)"), row=19, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=x, name="x (East)"), row=20, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=y_pos, name="y (North)"), row=21, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[6]), name="Roll response", line=dict(dash='dot')), row=10, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[7]), name="Pitch response", line=dict(dash='dot')), row=11, col=1)
    fig.add_trace(go.Scatter(x=t_vals, y=np.degrees(y_vals[8]), name="Yaw response", line=dict(dash='dot')), row=12, col=1)
    for idx, explanation in enumerate(plot_explanations, start=1):
        fig.add_annotation(
            text=explanation,
            xref="paper",
            yref="paper",
            x=1.02,
            y=1 - (idx - 0.5) / 21.0,
            showarrow=False,
            font=dict(size=11, color="#444"),
            align="left",
        )
    fig.update_layout(
        title_text=f"{uav_name} 6DOF Time Domain Response",
        height=6000,
        showlegend=True,
        margin=dict(t=120, b=40, l=80, r=160),
    )
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_font=dict(size=14))
    return fig

def create_3d_trajectory_plot_6dof(y_vals, x, y_pos, z, uav_name):
    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=x,
                y=y_pos,
                z=-z,
                mode="lines",
                name="Position Trajectory",
                line=dict(color="blue", width=3),
            ),
            go.Scatter3d(
                x=[x[0]],
                y=[y_pos[0]],
                z=[-z[0]],
                mode="markers",
                name="Start",
                marker=dict(color="red", size=6, symbol="circle"),
            ),
        ]
    )
    fig.update_layout(
        height=700,
        title=f"{uav_name} 6DOF Position Trajectory",
        scene=dict(
            xaxis_title="x (East, m)",
            yaxis_title="y (North, m)",
            zaxis_title="z (Up, m)",
        ),
        margin=dict(l=0, r=0, b=0, t=50),
        legend=dict(x=0.01, y=0.99),
    )
    return fig