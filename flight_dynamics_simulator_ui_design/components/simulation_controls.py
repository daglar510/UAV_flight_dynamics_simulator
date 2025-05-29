import reflex as rx
from flight_dynamics_simulator_ui_design.states.flight_sim_state import FlightSimState


def simulation_controls_section() -> rx.Component:
    return rx.el.div(
        rx.el.h3(
            "Simulation Setup",
            class_name="text-lg font-semibold text-gray-800 mb-3",
        ),
        rx.el.div(
            rx.el.label(
                "Simulation Duration (s):",
                class_name="text-sm font-medium text-gray-700",
            ),
            rx.el.input(
                type="number",
                step="any",
                default_value=FlightSimState.simulation_duration.to_string(),
                on_change=FlightSimState.set_simulation_duration,
                debounce_timeout=500,
                class_name="mt-1 block w-full px-3 py-2 bg-white border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 sm:text-sm",
            ),
            class_name="mb-4",
        ),
        rx.el.div(
            rx.el.button(
                "Run Simulation",
                rx.icon(
                    tag="play", class_name="ml-2 h-5 w-5"
                ),
                on_click=FlightSimState.run_simulation,
                is_loading=FlightSimState.is_simulating,
                loading_text="Simulating...",
                class_name="w-full px-4 py-2 bg-blue-600 text-white font-semibold rounded-lg shadow-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-opacity-75 flex items-center justify-center transition-colors duration-150 ease-in-out",
                margin_bottom="0.5rem",
            ),
            rx.el.button(
                "Save UAV Config",
                rx.icon(
                    tag="save", class_name="ml-2 h-5 w-5"
                ),
                on_click=FlightSimState.save_uav_config,
                class_name="w-full px-4 py-2 bg-green-600 text-white font-semibold rounded-lg shadow-md hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-green-500 focus:ring-opacity-75 flex items-center justify-center transition-colors duration-150 ease-in-out",
                margin_bottom="0.5rem",
            ),
            rx.el.button(
                "Reset Inputs",
                rx.icon(
                    tag="rotate_ccw",
                    class_name="ml-2 h-5 w-5",
                ),
                on_click=FlightSimState.reset_current_uav_to_saved,
                class_name="w-full px-4 py-2 bg-gray-500 text-white font-semibold rounded-lg shadow-md hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-gray-400 focus:ring-opacity-75 flex items-center justify-center transition-colors duration-150 ease-in-out",
            ),
            class_name="space-y-2",
        ),
        class_name="p-4 bg-white rounded-lg shadow",
    )