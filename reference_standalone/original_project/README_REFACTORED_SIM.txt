This package is the refactored version of the WMM2025 magnetorquer-only ADCS simulation.

What changed in the refactor
- The truth dynamics were made pure. The derivative function now only computes state derivatives.
- Truth field update, sensor sampling, navigation, controller update, and logging now happen explicitly in main.cpp.
- The temporary controller is clearly separated from the physics so the magnetorquer team can replace it later.
- The disturbance model now includes atmosphere-relative drag, gravity-gradient torque, residual magnetic torque, and a first-pass solar radiation pressure force with eclipse gating.
- The sensor model now uses fixed per-axis bias plus fresh per-axis white noise each sample without a fake direct angle sensor.
- Navigation now uses a simple gyro-propagated quaternion estimate with magnetic-field correction instead of smoothed truth.

What this simulation is for right now
- Closed-loop detumble development with magnetorquers only.
- WMM2025 field validation in a coherent orbit / attitude loop.
- Debugging controller interfaces before the real team control code is dropped in.

What this simulation is not yet
- A flight-quality environment model.
- A flight estimator.
- A full actuator electronics model.
- A fully geometry-calibrated disturbance model. Aerodynamic and SRP torques still depend on center-of-pressure offsets that are currently left at zero until HuskySat-2-specific values are known.
- A final verification environment for hardware acceptance.
