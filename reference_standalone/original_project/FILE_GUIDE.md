# File-by-file guide to the refactored simulation

## `CMakeLists.txt`
Builds one executable called `adcs` from the simulation source files.

## `include/math.hpp`
Small vector and 3x3 matrix helpers.
Everything else depends on this file for basic math.

## `include/quaternion.hpp`
Quaternion representation and conversions.
Used by both the truth attitude dynamics and the plotting / logging helpers.

## `include/frames.hpp`
All coordinate-frame utilities live here.
That includes ECI/ECEF rotation, ECEF->geodetic conversion, and NED basis vectors.

## `include/wmm.hpp`
Public interface for the WMM2025 magnetic model backend.

## `include/sim_context.hpp`
Holds long-lived configuration and latched simulation signals.
Think of it as the shared notebook for the whole simulation.

## `include/modules.hpp`
Declares the main simulation modules and the 13-state layout.
This is the file to read first if you want to see the architecture at a glance.

## `include/params.hpp`
Contains the integrator time step.

## `src/main.cpp`
This is the top-level simulation schedule.
It does the work in the same order each step:
1. compute truth field
2. sample sensors
3. update navigation
4. update controller
5. log everything
6. integrate truth state one RK4 step

This is the best file to read if you want to understand the whole loop.

## `src/satellite.cpp`
This file owns the truth physics.
It contains:
- spacecraft and Earth parameter initialization
- state extraction helpers
- WMM field conversion into the body frame
- pure orbit and attitude derivatives

Nothing in this file should look like flight software scheduling.
It is just the truth model.

## `src/sensor.cpp`
This file turns truth into measured values.
Bias is set once. Noise is re-drawn each sample.
Right now it provides only magnetometer and gyro style measurements.

## `src/navigation.cpp`
This file holds the first estimator scaffold.
It propagates attitude with measured body rates and uses the expected magnetic field direction
for a lightweight correction.
It is still not a flight-ready estimator, but it no longer feeds the controller smoothed truth.

## `src/control.cpp`
This file is a placeholder controller interface.
Right now it computes coil currents from a simple rate-cross-field damping law.
This is the file the magnetorquer team should eventually replace.

## `src/disturbance.cpp`
This file adds non-control environmental effects.
It now models:
- atmosphere-relative drag force for a rotating atmosphere
- gravity-gradient torque
- residual magnetic dipole torque
- first-pass solar radiation pressure force with eclipse gating
- aerodynamic and SRP torque hooks through configurable center-of-pressure offsets

The aerodynamic and SRP torque offsets are still set to zero by default until HuskySat-2-specific geometry data is available.

## `src/wmm.cpp`
This is the embedded WMM2025 backend.
Its only job is to return local magnetic field components from geodetic position and decimal year.

## `plot_all.py`
Quick-look plots for body rates, Euler angles, magnetic field, actuator commands, coil power,
and the disturbance force/torque breakdown.
It is not part of the physics loop. It only reads the CSV output.

## `reference/`
Reference coefficient and test-value files for WMM2025.
These are included for traceability and cross-checking.
They are not the main runtime path because the current code embeds the coefficients in `wmm.cpp`.
