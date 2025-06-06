import reflex as rx
from flight_dynamics_simulator_ui_design.states.flight_sim_state import FlightSimState
from flight_dynamics_simulator_ui_design.utils.types import PulseData


def pulse_input_row(
    pulse: PulseData, index: int
) -> rx.Component:
    return rx.cond(
        FlightSimState.current_simulation_mode == "4DOF",
        # 4DOF fields
        rx.el.div(
            rx.el.input(
                type="number",
                step="any",
                placeholder="Start (s)",
                default_value=str(pulse["start_time"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "start_time", val),
                debounce_timeout=500,
                class_name="w-1/4 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Duration (s)",
                default_value=str(pulse["duration"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "duration", val),
                debounce_timeout=500,
                class_name="w-1/4 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Elevator (deg)",
                default_value=str(pulse["angle_deg"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "angle_deg", val),
                debounce_timeout=500,
                class_name="w-1/4 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.button(
                rx.icon(tag="circle_minus", class_name="h-4 w-4"),
                on_click=lambda: FlightSimState.remove_pulse(index),
                class_name="ml-2 p-1 text-red-500 hover:text-red-700 rounded-md",
            ),
            class_name="flex items-center space-x-2 mb-2",
            key=f"pulse-row-{index}",
        ),
        # 6DOF fields
        rx.el.div(
            rx.el.input(
                type="number",
                step="any",
                placeholder="Start (s)",
                default_value=str(pulse["start_time"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "start_time", val),
                debounce_timeout=500,
                class_name="w-1/5 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Duration (s)",
                default_value=str(pulse["duration"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "duration", val),
                debounce_timeout=500,
                class_name="w-1/5 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Roll, deg",
                default_value=str(pulse["roll_deg"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "roll_deg", val),
                debounce_timeout=500,
                class_name="w-1/5 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Pitch, deg",
                default_value=str(pulse["angle_deg"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "angle_deg", val),
                debounce_timeout=500,
                class_name="w-1/5 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Yaw, deg",
                default_value=str(pulse["yaw_deg"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "yaw_deg", val),
                debounce_timeout=500,
                class_name="w-1/5 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.input(
                type="number",
                step="any",
                placeholder="Throttle (0-1)",
                default_value=str(pulse["throttle"]),
                on_change=lambda val: FlightSimState.update_pulse_param(index, "throttle", val),
                debounce_timeout=500,
                class_name="w-1/5 px-2 py-1 border rounded-md text-sm",
            ),
            rx.el.button(
                rx.icon(tag="circle_minus", class_name="h-4 w-4"),
                on_click=lambda: FlightSimState.remove_pulse(index),
                class_name="ml-2 p-1 text-red-500 hover:text-red-700 rounded-md",
            ),
            class_name="flex items-center space-x-2 mb-2",
            key=f"pulse-row-{index}",
        )
    )


def pulse_editor_section() -> rx.Component:
    return rx.el.div(
        rx.el.h3(
            rx.cond(
                FlightSimState.current_simulation_mode == "6DOF",
                "Control Pulses",
                "Elevator Pulses",
            ),
            class_name="text-lg font-semibold text-gray-800 mb-3",
        ),
        rx.el.div(
            rx.foreach(
                FlightSimState.pulses, pulse_input_row
            ),
            class_name="max-h-60 overflow-y-auto p-1 border rounded-md mb-2",
        ),
        rx.el.button(
            "Add Pulse",
            rx.icon(
                tag="circle_plus", class_name="ml-1 h-4 w-4"
            ),
            on_click=FlightSimState.add_pulse,
            class_name="w-full px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-600 text-sm flex items-center justify-center",
        ),
        class_name="p-4 bg-white rounded-lg shadow",
    )