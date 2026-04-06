# How the simulation works

This simulation is one closed loop with five stages.

1. **Truth state**  
   The spacecraft truth state is the 13-element vector
   `[position, velocity, quaternion, body rates]`.
   Orbit is propagated in an inertial frame. Attitude is propagated with a quaternion.

2. **Truth environment**  
   At the current truth state and time, the code computes the Earth magnetic field.
   The path is:
   ECI position -> ECEF position -> geodetic lat/lon/alt -> WMM2025 -> NED field -> ECEF -> ECI -> body frame.
   The final body-frame field is stored in Tesla.

3. **Sensors**  
   The truth magnetic field, truth body rates, and truth Euler angles are sampled at the sensor rate.
   The sensor model adds a fixed per-axis bias and fresh per-axis white noise.
   These become the measured signals.

4. **Navigation**  
   The measured signals go through a simple first-order smoothing filter.
   This is not a real flight estimator. It only provides a cleaner signal for controller bring-up.

5. **Controller and actuator**  
   The controller reads the navigation estimates and outputs magnetorquer coil currents in amps.
   Those currents define a magnetic dipole in the body frame.
   The dipole crossed with the body-frame magnetic field gives the control torque.

6. **Dynamics**  
   The truth state is advanced using RK4.
   Translational dynamics include central gravity and a simple drag force.
   Rotational dynamics include magnetorquer torque and rigid-body Euler equations.

## Why the files are separated this way
- `main.cpp` runs the simulation schedule and logging.
- `satellite.cpp` owns truth physics and WMM field conversion.
- `sensor.cpp` turns truth into measured values.
- `navigation.cpp` turns measured values into smoothed estimates.
- `control.cpp` turns estimates into coil current commands.
- `disturbance.cpp` adds non-control environment forces and torques.
- `wmm.cpp` is only the magnetic field backend.

## Current limitations
- The controller is still a placeholder detumble law.
- The navigation block is still just smoothing, not estimation.
- Disturbance torques are intentionally disabled until they are modeled correctly.
