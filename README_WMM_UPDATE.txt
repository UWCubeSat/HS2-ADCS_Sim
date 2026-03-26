This package updates the earlier magnetorquer-only ADCS simulation to use WMM2025.

What changed
- Replaced the old placeholder magnetic model with a real WMM2025 spherical-harmonic model.
- Added ECI <-> ECEF rotation so the field is evaluated in an Earth-fixed frame.
- Added ECEF -> geodetic latitude / longitude / altitude conversion on WGS84.
- Kept the rest of the closed-loop flow the same: sensors -> nav filter -> controller -> magnetorquer torque.
- Kept comments plain and direct so the code is easier to follow.

Field pipeline
1. Orbit state is propagated in an inertial frame (ECI-like).
2. Position is rotated into ECEF using Earth rotation.
3. ECEF position is converted to geodetic lat/lon/alt.
4. WMM2025 returns local North/East/Down magnetic field in nanoTesla.
5. That local field is turned into an ECEF vector, then rotated back to ECI, then into the body frame.
6. The body-frame field is stored in Tesla for torque calculations.

Build
- Open a terminal in the folder that contains CMakeLists.txt.
- Run: cmake -S . -B build
- Run: cmake --build build
- Run: .\build\adcs.exe   (Windows)
- Then run: python .\plot_all.py

Notes
- This is a much better magnetic environment model than the old placeholder, but it is still not a full flight-quality simulator.
- The disturbance and sensor models are still simple.
- The navigation block is still just a smoothing filter, not a real estimator.