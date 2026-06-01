# Basilisk capability audit

This audit classifies each standalone subsystem before migration. The recommendation prioritizes Basilisk-native modules where practical and keeps C++ only for project-specific ADCS logic.

## Classification key

1. **Replace with Basilisk native module**
2. **Keep as custom C++ ADCS logic and expose through Python**
3. **Temporarily reimplement in Python for fast testing**
4. **Keep only as reference/validation tool**
5. **Unknown; needs investigation**

## Subsystem table

| Subsystem | Current standalone implementation | Basilisk capability | Recommendation | Notes/TODOs |
|---|---|---|---|---|
| Orbit dynamics | `src/satellite.cpp` manually integrates position/velocity with RK4 and central gravity plus disturbances. | `spacecraft.Spacecraft` provides 6-DOF rigid-body translational and rotational dynamics; gravity bodies can be attached. | **1** | Use Basilisk as the truth plant. Keep standalone only for validation. |
| Attitude dynamics | Quaternion propagation and Euler rigid-body equations in `src/satellite.cpp`. | `spacecraft.Spacecraft` propagates attitude with MRP state and body rates, including dynamic effectors. | **1** | Interface must convert standalone quaternion expectations to Basilisk MRPs where needed. |
| Attitude state representation | Scalar-first quaternion in standalone CSV. | Basilisk uses MRP `sigma_BN` and `omega_BN_B` in standard state messages. | **1 / 3** | Use MRP natively in Basilisk; convert to Euler/quaternion only for plotting or comparison. |
| WMM / magnetic field | Embedded WMM2025 coefficients in `src/wmm.cpp`; reference COF files also present. | `magneticFieldWMM.MagneticFieldWMM` computes Earth WMM field at spacecraft location. | **1**, with validation | Compare magnitude/history and frame conventions against standalone WMM. TODO: verify N-frame sign and epoch conventions. |
| Magnetic field messages | Standalone stores body, measured, and nav field vectors in `SimContext` and CSV. | Basilisk WMM publishes `MagneticFieldMsgPayload`; modules can subscribe and record. | **1** | Log both inertial magnetic field and magnetometer body-frame field. |
| Magnetometer | `src/sensor.cpp` adds fixed bias plus white noise. | `magnetometer.Magnetometer` models TAM measurements in sensor frame. | **1** | Use zero noise for first equivalence pass; add bias/noise after frame validation. |
| Magnetorquer / MTB effector | Current -> dipole in `src/satellite.cpp`; torque `m x B` applied manually. | `MtbEffector.MtbEffector` converts commanded MTB dipoles and WMM magnetic field into body torque. | **1** | Use Basilisk effector for torque application; keep current/dipole/power conversion as custom diagnostics. |
| Detumble controller | Placeholder rate-cross-field controller in `src/control.cpp`. | Basilisk has general attitude-control modules, but no project-specific HuskySat detumble law is guaranteed. Python modules are supported for prototyping. | **2 / 3** | First use Python Basilisk module for fast testing. Optionally call small pybind C++ core. Do not use Basilisk ExternalModules as first path. |
| Pointing control | Not clearly implemented as final controller in uploaded standalone; scaffold only. | `mrpFeedback` provides general MRP feedback control for reference tracking and can work with reaction wheels. | **1 / 3 / 5** | For magnetorquer-only pointing, custom logic may be needed because magnetic actuation is underactuated. Start after detumble is stable. |
| Reaction wheels | Not used in standalone. | Basilisk has reaction-wheel state effectors and MRP feedback support. | **5 later** | Useful later if HuskySat architecture includes RWs or for comparison. Not first migration path. |
| SRP | First-pass SRP force and optional zero-offset torque in `src/disturbance.cpp`. | Basilisk has radiation-pressure dynamic effectors, including cannonball/faceted style models depending on version. | **1 / 5** | Add only after minimal + magnetic detumble work. Need actual geometry/area/reflectivity. |
| Drag | Atmosphere-relative drag in `src/disturbance.cpp`; simple density table/model. | Basilisk has drag-related dynamic effectors and atmosphere/environment modules depending on selected fidelity. | **1 / 5** | Add after core ADCS validation. Need chosen density model and geometry. |
| Disturbance torque | Gravity-gradient, residual magnetic dipole, aero/SRP offset torques. | Basilisk provides native effectors for many disturbances; `extForceTorque` can apply custom disturbance/control torque if needed. | **1 / 3 / 5** | Gravity-gradient/residual magnetic torque need explicit Basilisk mapping or temporary adapter. Aero/SRP offsets are TODO until geometry known. |
| Coil power | `I^2 R` in `src/satellite.cpp`. | Basilisk has power modules, but coil electronics/power model is project-specific. | **2 / 3** | Keep in ADCS core or Python diagnostics; not part of truth plant. |
| Actuator saturation | Current and dipole saturation in standalone logic. | MTB config has dipole limits; custom current limits remain controller-side. | **2 / 3** | Keep current/dipole conversions and saturation in ADCS core/adapter. |
| Sensor/nav estimator | `src/navigation.cpp` lightweight gyro propagation + magnetic correction. | Basilisk `simpleNav` can produce truth-like navigation output; dedicated filters exist for some cases. | **1 / 3 / 5** | Use `simpleNav` for early plant/controller debug. Revisit estimator fidelity later. |
| Output/logging | Manual CSV in `src/main.cpp`. | Basilisk messages support `.recorder()` logging. | **1** | Save canonical CSV from message logs for comparison. |
| Plotting | `plot_all.py` reads standalone CSV. | Python plotting remains appropriate. | **3 / 4** | New plotter reads canonical Basilisk CSV. Old plotter remains reference. |
| Validation | Known-good standalone metrics and plots. | Basilisk does not replace validation. | **4** | Preserve standalone as baseline and compare physical metrics. |
| Monte Carlo / tests | Not visible in standalone. | Basilisk supports repeatable Monte Carlo and pytest-based testing. | **1 later** | Add after nominal scenario is stable. |

## Main recommendation

Use Basilisk natively for the simulation environment:

- orbit and attitude propagation
- gravity
- WMM magnetic field
- magnetic-field messages
- magnetometer sensor
- magnetorquer torque application
- message logging
- later: SRP, drag, Monte Carlo, reaction wheels if needed

Keep project-specific ADCS logic outside Basilisk:

- detumble law
- pointing/mode logic
- current/dipole conversion
- saturation
- coil power
- diagnostic flags

The optional C++ core should be a small pybind library, not a Basilisk external module, until there is a strong reason to use Basilisk's C++ plugin path.

## External references used for the audit

- Basilisk install notes: PyPI wheels are available via `pip install bsk`, and `bsk[examples]` adds optional example dependencies.
- Basilisk spacecraft module: native 6-DOF translational and rotational spacecraft dynamics.
- Basilisk WMM example/module: WMM magnetic field can be evaluated at spacecraft locations.
- Basilisk magnetometer module: TAM measurements in sensor frame.
- Basilisk MtbEffector module: commanded magnetic torque-bar dipoles become body torque.
- Basilisk Python modules: suitable for quick prototyping without C++/SWIG module rebuilds.
