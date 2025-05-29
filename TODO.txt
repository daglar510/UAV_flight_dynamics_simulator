# TODO: Flight Dynamics Simulator Improvements

## Animation Plan
1. **Create a 3D UAV Model**:
   - Use a simplified wireframe or import a basic 3D model (e.g., `.obj` or `.stl`).
   - Ensure the model can be scaled and rotated dynamically.

2. **Integrate Plotly Animation**:
   - Extend `_create_time_domain_plot` to include a new method `_create_uav_animation`.
   - Use `go.Frame` and Plotly's animation features to update the UAV's orientation based on simulation results.

3. **Key Parameters to Animate**:
   - Pitch angle (θ) for rotation.
   - Angle of attack (α) for orientation relative to airflow.
   - Forward speed (u) for horizontal motion.

4. **UI Integration**:
   - Add a new section in `results_display.py` to display the animation.
   - Include play/pause controls for user interaction.

## Additional Improvements Plan
1. **Stability Metrics**:
   - Add static margin and neutral point calculations.
   - Display these in a new "Stability Metrics" section.

2. **Performance Metrics**:
   - Calculate and display lift-to-drag ratio (L/D).
   - Estimate range and endurance if fuel consumption is modeled.

3. **Control Sensitivity**:
   - Add elevator control power and pitch rate sensitivity metrics.

4. **Dynamic Response Metrics**:
   - Include time to double/half amplitude for unstable/stable modes.

5. **Visualization Enhancements**:
   - Add phase-plane plots (e.g., angle of attack vs. pitch rate).
   - Include frequency response plots (Bode/Nyquist).

6. **Robustness Metrics**:
   - Calculate gain and phase margins for control robustness analysis.

## Next Steps
- Prioritize animation implementation first.
- Gradually add other metrics based on user feedback.
- Test each feature thoroughly before integration. 