import reflex as rx
from flight_dynamics_simulator_ui_design.states.flight_sim_state import FlightSimState
from flight_dynamics_simulator_ui_design.components.uav_editor import uav_editor_section
from flight_dynamics_simulator_ui_design.components.pulse_editor import pulse_editor_section
from flight_dynamics_simulator_ui_design.components.simulation_controls import (
    simulation_controls_section,
)
from flight_dynamics_simulator_ui_design.components.results_display import (
    results_display_section,
)


def index() -> rx.Component:
    return rx.el.div(
        rx.el.header(
            rx.el.h1(
                "Flight Dynamics Simulator",
                class_name="text-3xl font-bold text-center text-white py-4 bg-indigo-600 shadow-md",
            )
        ),
        rx.el.main(
            rx.el.div(
                rx.el.div(
                    uav_editor_section(),
                    rx.el.div(class_name="my-4"),
                    pulse_editor_section(),
                    rx.el.div(class_name="my-4"),
                    simulation_controls_section(),
                    class_name="w-full lg:w-1/3 p-4 space-y-6",
                ),
                rx.el.div(
                    results_display_section(),
                    class_name="w-full lg:w-2/3 p-4",
                ),
                class_name="flex flex-col lg:flex-row max-w-screen-2xl mx-auto",
            ),
            class_name="p-4 font-['Inter'] bg-gray-100 min-h-screen",
        ),
        rx.el.footer(
            rx.el.div(
                "Developed by Daglar Duman | ",
                rx.el.a(
                    "Contact",
                    href="mailto:daglarduman510@gmail.com",
                    class_name="text-blue-600 hover:text-blue-800",
                ),
                " | ",
                rx.el.a(
                    "GitHub Repository",
                    href="https://github.com/daglar510/UAV_flight_dynamics_simulator",
                    class_name="text-blue-600 hover:text-blue-800",
                ),
                class_name="text-center text-sm text-gray-600 py-4 bg-white border-t",
            ),
        ),
        rx.toast.provider(),
    )


app = rx.App(
    theme=rx.theme(appearance="light"),
    head_components=[
        rx.el.link(
            rel="preconnect",
            href="https://fonts.googleapis.com",
        ),
        rx.el.link(
            rel="preconnect",
            href="https://fonts.gstatic.com",
            crossorigin="",
        ),
        rx.el.link(
            href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap",
            rel="stylesheet",
        ),
    ],
)
app.add_page(index, title="Flight Dynamics Simulator")