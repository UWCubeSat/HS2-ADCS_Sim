# Scenario status

Audit-derived record, 2026-09-05, anchored to 833b015 before documentation changes. No simulation was run or changed to create this document. Historical pass/fail observations are retained as audit findings, not newly reproduced results.

All scenarios below are **NOT FLIGHT VALIDATED**. WORKING DEVELOPMENT BASELINE denotes useful software bring-up. PROOF OF CONCEPT denotes limited feasibility/control demonstration. FAILED EXPERIMENT records an unsuccessful development attempt without proving physical impossibility. LEGACY REFERENCE is historical/comparison code, not ground truth.

Engineering values below are ASSUMED development inputs unless linked to a separately qualified source in [PHYSICAL_PARAMETERS.md](PHYSICAL_PARAMETERS.md).

## Current Basilisk minimal scenario

| FIELD | FINDING |
|---|---|
| Path | [basilisk_runner/scenario_huskysat2_minimal.py](../basilisk_runner/scenario_huskysat2_minimal.py) |
| Purpose | Verify installed Basilisk spacecraft/orbit/attitude propagation and output workflow. |
| Inputs | Legacy 2.6 kg, 0.1 by 0.1 by 0.2 m uniform box; diagonal inertia; COM offset zero. Circular 600 km/56 deg orbit, initial position on inertial +X; zero MRPs; omega = [0.8, -0.2, 0.3] rad/s. |
| Timing | 600 s duration; 0.1 s task; 1 s recording. |
| Actuator | None. |
| Sensors | None; truth state recorded directly. |
| Controller | None. |
| Environment | Earth central point-mass gravity. No magnetic field, drag, SRP, gravity-gradient torque, or residual magnetic torque. |
| Validation | Successful propagation/output is development evidence. Initial/final angular speed is printed. No comprehensive automated conservation/convergence acceptance suite. |
| Limitations | Generic legacy box, no actual HS-2 geometry or flight orbit; no ADCS performance claim. Constant rate magnitude is not a universal invariant for arbitrary asymmetric torque-free bodies; energy and angular momentum are the stronger checks. |
| Outputs | minimal_output.csv; shared body_rates.png and nav_euler.png in runner output folders. |
| Status | WORKING DEVELOPMENT BASELINE; NOT FLIGHT VALIDATED. |

## Current Basilisk magnetic detumble

Phase 5 operational addendum (2026-09-06): the recovered native-MtbEffector scenario offers explicit `--profile regression_baseline` (default, unchanged) and `--profile hs2_candidate` physical inputs. The latter uses the ASSUMED 3.72911 kg budget estimate, TBC 3U envelope, and ASSUMED uniform-prism inertia/zero COM described in [PHYSICAL_PARAMETERS.md](PHYSICAL_PARAMETERS.md#phase-5-explicit-physical-profiles). Orbit, controller, actuator limits, ideal sensors, environment and scheduling are shared. Candidate outputs use separate filenames and provenance manifests. [compare_physical_profiles.py](../basilisk_runner/compare_physical_profiles.py) compares saved runs using each tensor, independent volume-integration sanity checks and existing native torque/state validation. Status: WORKING DEVELOPMENT BASELINE / sensitivity study; NOT FLIGHT VALIDATED. One orbit does not verify the 24-hour detumble requirement; CAD properties, physical axes, actual hardware and test correlation remain unresolved. The following table retains the original audit findings at 833b015.

| FIELD | FINDING |
|---|---|
| Path | [basilisk_runner/scenario_huskysat2_detumble.py](../basilisk_runner/scenario_huskysat2_detumble.py), [adapter](../basilisk_runner/basilisk_adcs_adapter.py) |
| Purpose | Exercise magnetic damping through the Basilisk plant using a simple project controller. |
| Inputs | Same legacy box, COM/inertia, orbit, initial attitude/rates as minimal. Dipole limits [0.2, 0.2, 0.85] A m^2 on assumed body axes; WMM year assignment 2026.0. |
| Timing | One nominal 600 km orbit, approximately 5792 s; 0.1 s task/control; 1 s main recording. Environment/nav/TAM/controller run before spacecraft. |
| Actuator | ExtForceTorque receives controller-computed m cross measured B. Native MtbEffector is constructed/subscribed but unattached and unscheduled. Ideal instantaneous current with algebraic saturation and I^2 R power. |
| Sensors | Ideal SimpleNav body rate/attitude and one ideal body-aligned TAM: zero configured noise/bias, unit scale. No flight gyro, sun sensor, tracker, latency, quantization, or interference model. |
| Controller | K(omega cross B), K=67200, componentwise current/dipole clamps. Python implementation with optional pybind C++ dispatch. Despite class naming, it does not estimate dB/dt. |
| Environment | Central Earth gravity plus WMM2025 coefficient-file search. No planet ephemeris/Earth-orientation message is explicitly wired; assigning a field epoch does not define Earth rotation. No active disturbance-torque suite. |
| Validation | Existing range/invariant comparison checks finiteness, rate/energy reduction, magnetic magnitude/norm, cross-product consistency, power, and time to 0.05 rad/s in 3000-3500 s. Earlier development passes are limited to those checks. |
| Limitations | Logged applied torque is overwritten by command history; tau=m cross B check is self-referential. Field-norm preservation cannot establish direction. Input age/recorder phase and nearest-time joins need repair. No hardware current/PWM/thermal model or independent plant torque residual. One orbit and 0.05 rad/s do not verify the 24 h / 0.5 deg/s mission requirement. |
| Outputs | detumble_output.csv, detumble plots, comparison_metrics.json when comparison is separately invoked. |
| Status | WORKING DEVELOPMENT BASELINE; NOT FLIGHT VALIDATED. |

The unsaturated damping identity predicts nonpositive mechanical power when the same contemporaneous physical body field/rate enter the law. Per-axis clipping, sampled/biased measurements, and applied-versus-command timing require their own verification; they cannot be dismissed by that algebraic identity.

## Current direct-torque pointing

| FIELD | FINDING |
|---|---|
| Path | [basilisk_runner/scenario_huskysat2_pointing.py](../basilisk_runner/scenario_huskysat2_pointing.py) |
| Purpose | Bring up MRP attitude feedback and convergence metrics with an ideal torque source. |
| Inputs | Reuses legacy box/orbit. Initial sigma_BN=[0.22,-0.12,0.16], omega=[0.03,-0.02,0.015] rad/s; inertial identity reference. |
| Timing | 1800 s; 0.1 s control; 1 s recording. Navigation/controller/effector precede spacecraft. |
| Actuator | Ideal arbitrary direct body torque through ExtForceTorque, norm limited to 2e-4 N m. No magnetic allocation or field-dependent authority. |
| Sensors | Ideal SimpleNav; no physical camera/star-tracker/gyro model. |
| Controller | tau=-8e-5 sigma_BR - 1.8e-3 omega_BR_B, with MRP shadow handling and norm saturation. Gain units are N m and N m per rad/s respectively. |
| Environment | Central gravity inherited from spacecraft setup. No attached WMM, magnetic actuator, disturbance, Sun, eclipse, or payload visibility model. |
| Validation | Finiteness, reduced error/rate/energy, final error <1 deg, final rate <1e-4 rad/s, torque limit, saturation fraction <=5%. Controller-history error and command-torque fields supply several metrics. |
| Limitations | Full three-axis torque authority is not available instantaneously from magnetorquers. Inertial identity tracking is not Earth-limb/star-field acquisition, solar pointing, COMMS coverage, or a keep-out-constrained slew. Benchmarks are development thresholds, not HS-2 requirements. |
| Outputs | pointing_output.csv, pointing_metrics.json, pointing_* plots. |
| Status | PROOF OF CONCEPT; NOT FLIGHT VALIDATED. |

## Archived magnetic pointing, first attempt

| FIELD | FINDING |
|---|---|
| Path | archive/experiments/scenario_huskysat2_pointing_magnetic_failed_poc.py (ignored local archive) |
| Purpose | Constrain a pointing PD command to magnetorquer authority. |
| Inputs | Legacy box/circular-orbit setup and nonzero attitude/rate error; uses ADCSConfig current/dipole limits. Archived constants and saved-run provenance must be preserved together before any future reproduction. |
| Actuator | Compute dipole from B cross desired torque / B^2, clip current/dipole, then apply m cross B through a direct torque bridge. |
| Sensors | Development SimpleNav and TAM inputs, not flight-calibrated sensing. |
| Controller | Naive MRP feedback followed by instantaneous magnetic allocation. |
| Environment | Inherited WMM/central-gravity development setup and its frame/time limitations. |
| Validation | Historical failed pointing experiment; recomputed command cross-products do not independently verify applied torque. |
| Limitations | Instantaneous torque projection alone does not ensure orbit-wide attitude convergence. No released spacecraft geometry, disturbance/sensor realism, mission targets, or full verification package. Archive location/import assumptions may prevent direct standalone execution. |
| Status | FAILED EXPERIMENT; NOT FLIGHT VALIDATED. |

## Archived magnetic pointing, version 2

| FIELD | FINDING |
|---|---|
| Path | archive/experiments/scenario_huskysat2_pointing_magnetic_v2_failed.py (ignored local archive) |
| Purpose | Improve the first magnetic-pointing attempt with rate damping, acquisition, hold, and an energy-injection guard. |
| Inputs | Legacy box/orbit; initial sigma_BN=[0.08,-0.05,0.06], omega=[0.010,-0.007,0.005] rad/s; ADCSConfig magnetic limits. 0.1 s task, 1 s record interval. |
| Actuator | B cross desired torque / B^2 allocation, current/dipole clipping, direct m cross B bridge. Still not attached native MtbEffector or hardware driver dynamics. |
| Sensors | Ideal development navigation/magnetometer. |
| Controller | RATE_DAMP/ACQUIRE/HOLD logic; acquire gain 1.5e-5 N m, hold gain 5e-6 N m; ideal torque cap 8e-5 N m. Guard falls back to damping or zero torque when the allocated command would inject energy. |
| Environment | WMM and central-gravity setup; no validated spacecraft disturbance/environment model. |
| Validation | Historical failed experiment with pointing/rate/power/saturation/command-cross-product metrics. Final performance is not an accepted HS-2 result. |
| Limitations | Smaller initial errors and protective switching did not establish general magnetic pointing. Saved-run duration/backend/environment provenance must be reconciled before comparing numerical outcomes. Three-axis acquisition/hold over changing field direction remains open. |
| Status | FAILED EXPERIMENT; NOT FLIGHT VALIDATED. |

The branch name mtq-limited-pointing-v2 points to ecf3cab, not proof that either ignored experiment was merged or solved.

## Archived first Basilisk detumble integration

| FIELD | FINDING |
|---|---|
| Path | archive/old_attempts/huskysat2_bsk/scenario_huskysat2_detumble.py and ExternalModules/mtqDetumble/ |
| Purpose | Initial C++/SWIG Basilisk detumble integration. |
| Inputs | Legacy 2.6 kg / 0.1 by 0.1 by 0.2 m box; 600 km/56 deg orbit; [0.8,-0.2,0.3] rad/s; same nominal hybrid torquer constants. |
| Timing | 0.1 s task; 1 s recording; orbital simulation. |
| Actuator | Intended attached native MtbEffector. Archived layout uses MAX_EFF_CNT-strided GtMatrix_B rather than contiguous 3 by numMTB layout. |
| Sensors | SimpleNav and TAM bring-up configuration. |
| Controller | Basilisk.ExternalModules.mtqDetumble; source-build/SWIG integration required by that attempt. |
| Environment | WMM path with machine-specific dependency assumptions; central gravity. |
| Validation | Historical integration failure/debug evidence, not a working portable baseline or proof that native MtbEffector is unavailable. |
| Limitations | Hard-coded local paths, external module availability, incorrect layout, unresolved integration/timing; absent from tracked normal runtime. |
| Status | LEGACY REFERENCE; NOT FLIGHT VALIDATED. |

## Preserved standalone C++ simulation

| FIELD | FINDING |
|---|---|
| Path | [reference_standalone/original_project/src/main.cpp](../reference_standalone/original_project/src/main.cpp) and sibling modules |
| Purpose | Preserve an independently executed predecessor for development comparison. |
| Inputs | Legacy 2.6 kg box; 600 km/56 deg circular orbit; scalar-first identity quaternion; [0.8,-0.2,0.3] rad/s. Seed 1, decimal year 2026.0, initial Greenwich angle 0. |
| Timing | RK4 truth step 0.1 s over one orbit; sensor/nav 1 s; control 0.1 s with held measurements/current. CSV each truth step. |
| Actuator | Assumed X/Y CR0002 and Z MT01 current/dipole model; held current; plant recomputes m cross truth B during derivatives. No RL/PWM/thermal/quiet-period physics. |
| Sensors | Generic fixed biases and uniform white noise. Magnetic bias scale 4e-7 T, noise scale 1e-5 T; gyro bias scale 0.01 rad/s, noise scale 0.001 rad/s. These are not VN-100 calibrated noise-density values. |
| Controller | Placeholder rate-cross-field law, K=67200, same nominal actuator limits as migration. |
| Estimator | Gyro propagation plus one magnetic-vector correction and bias scaffold; blend 0.3, correction gains 0.8 and 0.02. Truth magnetic reference is supplied. A single vector leaves rotation about that vector instantaneously unobservable. |
| Environment | Central gravity using G times Earth mass rather than the active Basilisk mu; embedded WMM2025; ECI/ECEF/NED conversion; approximate atmosphere-relative drag, gravity gradient, residual magnetic dipole, analytic Sun/cylindrical eclipse, SRP. Aero/SRP COM-to-CP offsets are zero. |
| Validation | Finite RK4/state guards, normalized quaternion, plots and saved scalar comparisons. More modeled effects do not establish their correctness. |
| Known issues | ECI-to-ECEF helper uses positive Rz for positive Earth angle; orientation/sign consistency needs closure. SRP force follows the modeled Sun-direction vector, a sign concern for radiation arriving from the Sun. Generic noise, truth-aided magnetic correction, simplified geometry/environment, and sampled estimator behavior are unvalidated. Embedded WMM needs authoritative-vector regression; matching another shared model is insufficient. |
| Outputs | adcs_output.csv and output_plots/; copied reference CSV can be stale. BNx/BNy/BNz denote filtered body field. |
| Status | LEGACY REFERENCE; NOT FLIGHT VALIDATED. |

## Older root C++ orbit/field program

| FIELD | FINDING |
|---|---|
| Path | [src/main.cpp](../src/main.cpp), [src/satellite.cpp](../src/satellite.cpp), [include/params.hpp](../include/params.hpp), root CMakeLists.txt |
| Purpose | Propagate a six-component translational state and evaluate/log a GeographicLib magnetic field. |
| Inputs | 3 kg, 500 km circular equatorial orbit, 0.5 s RK4 step, one orbit. All are legacy ASSUMED values. |
| Actuator | None. |
| Sensors | No flight sensor model; computed magnetic field logged. |
| Controller | None; not an attitude simulation. |
| Environment | Legacy gravity/orbit and GeographicLib magnetic-model dependency. Not the active Basilisk environment. |
| Validation | Basic file/output workflow; no current ADCS requirement verification. |
| Limitations | No rotational plant or closed-loop ADCS. Machine/model data dependencies; advanced state is written with the loop's pre-increment time label. |
| Outputs | Working-directory-relative ../output/test.txt and ../output/bfield.txt, plotted by scripts/. |
| Status | LEGACY REFERENCE; NOT FLIGHT VALIDATED. |

## Validation boundaries and next decisions

| CHECK | WHAT IT CAN SUPPORT | WHAT IT DOES NOT ESTABLISH |
|---|---|---|
| Propagated state remains finite; energy/rate trend | Plant/control behavior for the stated numerical case, independently of merely printing a command. | Hardware realism, full conservation/work balance, mission envelope, or convergence proof. |
| Native effector capability/layout inspection from follow-up audit | Available implementation and integration requirements. | Successful use by the active detumble scenario, which bypasses it. |
| C++ core smoke test | One valid result and finite total coil power. | Python/C++ parity across boundary cases or actuator physics. |
| tau=m cross B using overwritten CSV torque | Command-side arithmetic consistency. | Independent applied torque or moment-to-dynamics transfer. |
| Magnetic norm preservation | A necessary rotational invariant. | Correct rotation direction, Earth orientation, body-axis mapping, or acquisition epoch. |
| Coil power computed as I^2 R | Consistency with the assumed algebraic model. | Bus consumption, PWM/RL response, driver losses, temperature, duty limits, or measured power. |
| Standalone comparison script pass | Existing scalar/range checks, mostly for Basilisk plus reference finiteness. | A quantified standalone-to-Basilisk equivalence test or independent truth validation. |
| First crossing of 0.05 rad/s | Development threshold timing (about 2.865 deg/s). | Sustained 0.5 deg/s mission compliance or detumble energy requirement. |
| Direct-torque pointing pass | Ideal-actuator MRP feedback bring-up. | Magnetorquer-only acquisition/hold, payload pointing accuracy, or Sun keep-out compliance. |

The Drive audit found ADCS unit tests marked not done, incomplete subsystem/system records, and blank performance-procedure results. Budget prose describing Monte Carlo/HIL verification is not a completed result. Required independent validation and archived-case reproduction belong to [RECOVERY_ROADMAP.md](RECOVERY_ROADMAP.md).
