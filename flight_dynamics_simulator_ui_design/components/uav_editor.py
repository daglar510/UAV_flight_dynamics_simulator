import reflex as rx
from flight_dynamics_simulator_ui_design.states.flight_sim_state import FlightSimState
from flight_dynamics_simulator_ui_design.utils.types import UAVParameterValue
from typing import Tuple


def render_uav_parameter_input(
    param_tuple: Tuple[str, UAVParameterValue],
    item_index: int,
) -> rx.Component:
    param_key: str = param_tuple[0]
    param_value: UAVParameterValue = param_tuple[1]
    
    return rx.el.div(
        rx.el.p(
            f"{param_key.replace('_', ' ').title()}: {param_value}",
            class_name="text-sm font-medium text-gray-700 mb-1",
        ),
        rx.el.input(
            type="number",
            step="any",
            placeholder=f"Change {param_key}",
            on_blur=lambda val: FlightSimState.update_editable_param(param_key, val),
            debounce_timeout=500,
            class_name="mt-1 block w-full px-3 py-2 bg-white border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 sm:text-sm",
        ),
        class_name="mb-3",
        key=f"uav-param-{param_key}-{item_index}",
    )


def uav_editor_section() -> rx.Component:
    return rx.el.div(
        rx.el.h3(
            "UAV Parameters",
            class_name="text-lg font-semibold text-gray-800 mb-3",
        ),
        rx.el.div(
            rx.el.label(
                "Select UAV Model:",
                class_name="text-sm font-medium text-gray-700",
            ),
            rx.el.select(
                rx.foreach(
                    FlightSimState.uav_names,
                    lambda name: rx.el.option(
                        name, value=name
                    ),
                ),
                value=FlightSimState.selected_uav_name,
                on_change=FlightSimState.handle_uav_selection,
                class_name="mt-1 block w-full pl-3 pr-10 py-2 text-base border-gray-300 focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 sm:text-sm rounded-md",
            ),
            class_name="mb-4",
        ),
        rx.el.div(
            rx.el.p(
                f"Company: {FlightSimState.editable_params['company']}",
                class_name="text-sm text-gray-600",
            ),
            rx.el.p(
                f"Country: {FlightSimState.editable_params['country']}",
                class_name="text-sm text-gray-600 mb-2",
            ),
            class_name="p-2 bg-gray-50 rounded-md mb-3",
        ),
        rx.el.div(
            rx.foreach(
                FlightSimState.current_editable_params_list,
                render_uav_parameter_input,
            ),
            class_name="grid grid-cols-1 md:grid-cols-2 gap-x-4 max-h-96 overflow-y-auto p-1 border rounded-md",
        ),
        class_name="p-4 bg-white rounded-lg shadow",
    )