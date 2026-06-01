# Source inspection report

## Uploaded standalone simulation

Location inspected in the uploaded archive:

```text
adcs_wmm2025_updated_cohesive/
```

The standalone simulation is already organized as a small CMake C++17 executable:

```text
CMakeLists.txt
include/
src/
reference/
plot_all.py
adcs_output.csv
output_plots/
```

### Build structure

`CMakeLists.txt` builds one executable named `adcs` from:

```text
src/main.cpp
src/satellite.cpp
src/control.cpp
src/sensor.cpp
src/navigation.cpp
src/disturbance.cpp
src/wmm.cpp
```

It uses local headers in `include/` and no visible external library dependency.

### Standalone entry point

Entry point:

```text
src/main.cpp
```

Main loop order:

1. Compute truth magnetic field.
2. Sample sensors.
3. Update navigation.
4. Update controller.
5. Log CSV row.
6. Advance truth state with RK4.
7. Normalize quaternion.

### State definition

The truth state is a 13-element vector:

```text
[x, y, z, xdot, ydot, zdot, q0, q1, q2, q3, p, q, r]
```

Units:

- position: m
- velocity: m/s
- quaternion: scalar-first inertial/body convention used internally by the standalone code
- body rates: rad/s

### Dynamics files

| File | Responsibility |
|---|---|
| `src/satellite.cpp` | Truth orbit/attitude derivatives, Earth constants, spacecraft parameters, magnetorquer dipole and coil power conversions. |
| `src/disturbance.cpp` | Atmosphere-relative drag, solar radiation pressure force, gravity-gradient torque, residual magnetic torque, optional aero/SRP torque offsets. |
| `src/wmm.cpp` | Embedded WMM2025 coefficient backend. |
| `src/sensor.cpp` | Magnetometer and gyro-like sampled measurements with fixed bias and white noise. |
| `src/navigation.cpp` | Lightweight estimator scaffold using gyro propagation and magnetic-field correction. |
| `src/control.cpp` | Placeholder detumble/rate-cross-field magnetorquer controller. |
| `src/main.cpp` | Simulation schedule, RK4 integration, CSV logging. |

### WMM and magnetic field path

The standalone magnetic field path is:

```text
ECI position -> ECEF position -> geodetic lat/lon/alt -> WMM2025 NED -> ECEF -> ECI -> body
```

`reference/WMM2025.COF` and related files are included for traceability. The current runtime uses coefficients embedded in `src/wmm.cpp`.

### Controller logic

`src/control.cpp` is intentionally described by the standalone project as a placeholder controller. It computes a commanded current using a rate-cross-field damping law:

```text
m_desired = K * (omega_body x B_body)
current = clamp(m_desired / dipole_gain)
dipole = current * dipole_gain
```

The custom logic worth preserving is the controller API, actuator saturation, current-to-dipole conversion, and coil power calculation. The full truth dynamics should not be ported blindly because Basilisk already provides the simulation plant.

### Disturbance logic

`src/disturbance.cpp` currently models:

- atmosphere-relative drag force
- solar radiation pressure force with eclipse gating
- gravity-gradient torque
- residual magnetic dipole torque
- aero and SRP torque hooks through configurable center-of-pressure offsets

The aero/SRP torque offsets are zero in the standalone run, so these are scaffolds rather than validated HuskySat-specific geometry values.

### CSV output

The known-good `adcs_output.csv` contains 70 columns:

```text
t, x, y, z, xdot, ydot, zdot, q0, q1, q2, q3, p, q, r,
BBx, BBy, BBz, Bmx, Bmy, Bmz, BNx, BNy, BNz,
pqrm_x, pqrm_y, pqrm_z, pqrN_x, pqrN_y, pqrN_z,
ptpN_phi, ptpN_theta, ptpN_psi, qN0, qN1, qN2, qN3,
ix, iy, iz, mcmd_x, mcmd_y, mcmd_z,
pcoil_x, pcoil_y, pcoil_z, pcoil_total,
rho_kg_m3, vrel_m_s, sunlit,
Fdrag_x, Fdrag_y, Fdrag_z, Fsrp_x, Fsrp_y, Fsrp_z,
Taero_x, Taero_y, Taero_z, Tgg_x, Tgg_y, Tgg_z,
Tmag_x, Tmag_y, Tmag_z, Tsrp_x, Tsrp_y, Tsrp_z,
Tdist_x, Tdist_y, Tdist_z
```

The migration does not need to match this exact CSV schema. It should match equivalent physical metrics.

### Plot behavior

`plot_all.py` reads `adcs_output.csv` and creates:

```text
body_rates.png
nav_euler.png
mag_field.png
currents.png
dipole.png
coil_power.png
disturbances.png
summary_plots.png
```

It also prints the reference metrics that were provided in the migration request.

## Uploaded first Basilisk attempt

Location inspected:

```text
huskysat2_bsk/
```

Important files:

```text
scenario_huskysat2_detumble.py
plot_results.py
ExternalModules/mtqDetumble/mtqDetumble.cpp
ExternalModules/mtqDetumble/mtqDetumble.h
ExternalModules/mtqDetumble/mtqDetumble.i
fix.pypython
fix2.pypython
README.md
```

### What the old attempt did correctly

It correctly identified useful Basilisk pieces:

- `spacecraft.Spacecraft`
- `gravityEffector.GravBodyData`
- `magneticFieldWMM.MagneticFieldWMM`
- `simpleNav.SimpleNav`
- `magnetometer.Magnetometer`
- `MtbEffector.MtbEffector`
- Basilisk recorder messages

### Why the old attempt caused slow workflow

The old attempt imports:

```python
from Basilisk.ExternalModules import mtqDetumble
```

The old README instructs copying `ExternalModules/` into the Basilisk source tree or building Basilisk using `--externalModules`. That makes ordinary controller edits depend on a Basilisk source rebuild/SWIG workflow. This is the path to avoid for normal development.

### Hard-coded path issue

The old attempt uses a machine-specific WMM path:

```text
C:\Users\chine\Documents\basilisk\dist3\Basilisk\supportData\MagneticField\WMM2025.COF
```

The new runner avoids this by searching the installed `Basilisk` package support-data tree and falling back to the local standalone reference coefficient file if present.

### What from the old attempt is retained

The old attempt is archived under:

```text
archive/old_attempts/huskysat2_bsk/
```

It is retained as reference only. The new workflow does not require `Basilisk.ExternalModules.mtqDetumble`.
