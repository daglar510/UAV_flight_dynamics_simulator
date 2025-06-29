import reflex as rx

# Tailwind is now enabled by passing a `tailwind` settings dictionary directly
# to the Config object (see https://reflex.dev/docs/styling/tailwind/).
# An empty dict enables Tailwind with its default configuration.

config = rx.Config(
    app_name="flight_dynamics_simulator_ui_design",
    tailwind={},  # add Tailwind options here if needed
)