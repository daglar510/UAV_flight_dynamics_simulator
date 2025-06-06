import reflex as rx
import plotly.graph_objects as go
from flight_dynamics_simulator_ui_design.states.flight_sim_state import FlightSimState


def results_display_section() -> rx.Component:
    def pulse_summary_4dof(pulse, i):
        return rx.el.div(
            f"Start: {pulse['start_time']}s, Duration: {pulse['duration']}s, Angle: {pulse['angle_deg']}°",
            class_name="text-sm text-gray-600 ml-2"
        )
    def pulse_summary_6dof(pulse, i):
        return rx.el.div(
            f"Start: {pulse['start_time']}s, Duration: {pulse['duration']}s, "
            f"Roll: {pulse['roll_deg']}°, Pitch: {pulse['angle_deg']}°, Yaw: {pulse['yaw_deg']}°, Throttle: {pulse['throttle']}",
            class_name="text-sm text-gray-600 ml-2"
        )
    return rx.el.div(
        rx.el.h3(
            "Simulation Results",
            class_name="text-lg font-semibold text-gray-800 mb-3",
        ),
        rx.cond(
            FlightSimState.is_simulating,
            rx.el.div(
                rx.el.div(
                    class_name="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-500 mx-auto my-4"
                ),
                rx.el.p(
                    "Simulation in progress...",
                    class_name="text-center text-gray-600",
                ),
            ),
            rx.el.div(
                rx.el.div(
                    rx.el.h4(
                        "Simulation Inputs:",
                        class_name="text-md font-semibold text-gray-700",
                    ),
                    rx.el.div(
                        rx.el.p(
                            f"Duration: {FlightSimState.simulation_duration} seconds", 
                            class_name="text-sm text-gray-600"
                        ),
                        rx.el.div(
                            rx.el.p(
                                rx.cond(
                                    FlightSimState.current_simulation_mode == "6DOF",
                                    "Control Pulses:",
                                    "Elevator Pulses:",
                                ),
                                class_name="text-sm font-medium text-gray-600 mt-1"
                            ),
                            rx.cond(
                                FlightSimState.current_simulation_mode == "6DOF",
                                rx.foreach(
                                    FlightSimState.pulses,
                                    pulse_summary_6dof
                                ),
                                rx.foreach(
                                    FlightSimState.pulses,
                                    pulse_summary_4dof
                                ),
                            ),
                            class_name="mt-1"
                        ),
                        class_name="bg-gray-50 p-2 rounded-md"
                    ),
                    class_name="mb-3",
                ),
                rx.el.div(
                    rx.el.h4(
                        "Trim Speed:",
                        class_name="text-md font-semibold text-gray-700",
                    ),
                    rx.el.p(
                        FlightSimState.trim_speed_output,
                        class_name="text-sm text-gray-600 bg-gray-50 p-2 rounded-md",
                    ),
                    class_name="mb-3",
                ),
                rx.el.div(
                    rx.el.h4(
                        "Eigenvalue Analysis:",
                        class_name="text-md font-semibold text-gray-700",
                    ),
                    rx.el.ul(
                        rx.foreach(
                            FlightSimState.eigen_analysis_output,
                            lambda line: rx.el.li(
                                line,
                                class_name="text-sm text-gray-600",
                            ),
                        ),
                        class_name="list-disc list-inside bg-gray-50 p-2 rounded-md max-h-40 overflow-y-auto",
                    ),
                    class_name="mb-3",
                ),
                rx.cond(
                    FlightSimState.time_domain_plot_figure,
                    rx.el.div(
                        rx.el.h4(
                            "Time Domain Response:",
                            class_name="text-md font-semibold text-gray-700 mb-1",
                        ),
                        rx.plotly(
                            data=FlightSimState.time_domain_plot_figure if FlightSimState.time_domain_plot_figure is not None else go.Figure()
                        ),
                        class_name="mb-3 border rounded-md p-2",
                    ),
                    rx.el.div(),
                ),
                rx.cond(
                    FlightSimState.trajectory_plot_figure,
                    rx.el.div(
                        rx.el.h4(
                            "3D Trajectory:",
                            class_name="text-md font-semibold text-gray-700 mb-1",
                        ),
                        rx.plotly(
                            data=FlightSimState.trajectory_plot_figure if FlightSimState.trajectory_plot_figure is not None else go.Figure()
                        ),
                        class_name="border rounded-md p-2",
                    ),
                    rx.el.div(),
                ),
                rx.cond(
                    (FlightSimState.trim_speed_output == "")
                    & FlightSimState.time_domain_plot_figure.is_none(),
                    rx.el.p(
                        "Run simulation to see results.",
                        class_name="text-center text-gray-500 mt-4",
                    ),
                    rx.el.div(),
                ),
            ),
        ),
        class_name="p-4 bg-white rounded-lg shadow min-h-[200px]",
    )