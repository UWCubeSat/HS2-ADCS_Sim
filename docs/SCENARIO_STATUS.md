# Scenario status

Audit-derived record, 2026-09-05, anchored to 833b015 before documentation changes. No simulation was run or changed to create this document. Historical pass/fail observations are retained as audit findings, not newly reproduced results.

All scenarios below are **NOT FLIGHT VALIDATED**. WORKING DEVELOPMENT BASELINE denotes useful software bring-up. PROOF OF CONCEPT denotes limited feasibility/control demonstration. FAILED EXPERIMENT records an unsuccessful development attempt without proving physical impossibility. LEGACY REFERENCE is historical/comparison code, not ground truth.

Engineering values below are ASSUMED development inputs unless linked to a separately qualified source in [PHYSICAL_PARAMETERS.md](PHYSICAL_PARAMETERS.md).

## Phase 7F-2C live disconnected command-chain observer - 2026-09-30

**LIVE DISCONNECTED COMMAND-CHAIN OBSERVER / NO ACTUATOR AUTHORITY /
NOT CLOSED LOOP / NOT FLIGHT VALIDATED.** Evidence anchor: Phase 7F-2B `00d6909`.

| FIELD | FINDING |
|---|---|
| Paths / purpose | `disconnected_command_observer.py`, focused validator/tests, opt-in scenario wiring and a small health-gate application-window extension. Reuses coherent snapshot, unchanged controller calculation and health/restart components. |
| Inputs | Actual stored TAM/header and existing bridge quiet-window/reference evidence; post-publication MEKF NavAtt/quality and current consumer decision. Explicit test-only ideal Sun for validation acquisition. No observer truth substitution. |
| Schedule | Existing production priorities unchanged. Early witness 595, input tap 587, health/capture 554, calculation 552, diagnostic application 550. Actual callback/epoch traces confirm order; at 1.4 s early read sees 1.3 s MEKF while capture sees 1.4 s. |
| Epochs / generation | Nominal sample/capture 0.4 s, calculation 0.5 s, application checks 0.6-0.9 s at each transport tick. Immutable envelope binds cycle/command IDs and reset/acquisition/revocation generation; original timestamps retained. |
| Actuation / environment | No command output or effector connection. Production SimpleNav/cycle driver remains the sole actual command owner. Plant, environment, physical profiles, gains, cycle and estimator math unchanged. |
| Fault / restart | Fault after calculation inhibits before application; mid-burst fault inhibits subsequent intervals. Reset is delivered to actual shadow adapter and gate; reacquisition cannot revive an old envelope. Strictly fresh post-reacquisition sample/command restores diagnostic usability. |
| Validation | 183/183 tests including 8 new integration tests; standalone nine-case live validator passes. Live/component snapshots, calculations and health decisions agree exactly. Invalid pairing, TAM window, stale quality and old-generation replay fail explicitly. |
| Preservation | Three committed-versus-working 4 s production cases byte-identical when disabled. All nine enabled/fault-injected hosts also exactly match disabled production telemetry. Actuator/sensor wiring AST unchanged. No full orbit needed. |
| Outputs | Dedicated `output_data/disconnected_command_chain_validation.json`, including source hashes, full command provenance, telemetry, execution/injection events and check results. Optional scenario artifacts have `_disconnected` suffix; default schemas/paths unchanged. |
| Status / next | PASS for LIVE DISCONNECTED integration; DEVELOPMENT CANDIDATE / NOT FLIGHT VALIDATED. A separately authorized short closed-loop MEKF-vs-SimpleNav A/B is now justified, after implementing explicit single-owner routing and zero-on-inhibition at this boundary. No such routing exists in this phase. |
| Limits | Ideal sensor bridges and 1 s validation Sun cadence are ASSUMED / TEST-ONLY. Delivered in-run reset/status contract only, no full-process restart continuity. No closed-loop, pointing, requirements or flight-performance claim. |

See the [live scheduling and application-boundary evidence](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7f-2c-live-disconnected-command-chain-observer---2026-09-30).

## Phase 7F-2B evaluation-time command health gate - 2026-09-30

**EVALUATION-TIME COMMAND HEALTH GATE / NO ACTUATOR AUTHORITY /
NOT CLOSED LOOP / NOT FLIGHT VALIDATED.** Evidence anchor: Phase 7F-2A `de04df4`.

| FIELD | FINDING |
|---|---|
| Paths / purpose | New `command_health_gate.py` and `test_command_health_gate.py`: establish current command usability separately from capture coherence and mathematical validity. Isolated development policy; no scenario integration. |
| Inputs / provenance | Accepted immutable MEKF/TAM snapshot, existing disconnected calculation, current NavAtt/quality and Phase 7E consumer decision, exact use epoch and explicit in-run reset notification. No truth or new sensor model. |
| Controller / output | No math recomputation. Frozen decision retains original command/provenance and exposes usability, inhibition reason, source/lifecycle state and sample/capture/computation/use/quality epochs. Gate stores lifecycle scalars only, never a last command. |
| Timing | Current quality and navigation must describe the use epoch exactly. Stored vectors retain the sample epoch. This phase keeps use at the original compute tick; no hold or ACTUATE-burst permission is added. |
| Fault / restart | Health loss latches inhibition; clearing validity bits is insufficient. Explicit reset plus fresh increased-count acquisition restores source health. Usability additionally requires a newly coherent sample strictly after reacquisition and a new command. Old pre-fault commands remain unusable, including at their original deadline. |
| Actuator / environment | No native command output, effector, spacecraft dynamics or production import. No environment, configuration, cycle or controller change. |
| Validation | 16/16 focused tests pass. Includes actual existing MEKF adapter and persistent navigation latch through gyro-epoch fault, explicit reset, reacquisition and fresh-command restoration. Also tests nonfinite/malformed/stale/inconsistent metadata, same-timestamp restart ambiguity, immutability, provenance and no fallback. Compileall and diff checks pass. |
| Preservation | Existing tracked runtime, subscriptions, task priorities and saved outputs unchanged; no expensive simulations or prior regression reruns. |
| Gate / next | PASS for observed evaluation-time health gating only. Next: opt-in disconnected live-task observer proving end-to-end publication/reset-event delivery. Closed-loop A/B connection remains premature until that integration and actual application-boundary inhibition are defined/verified. |
| Limitations | Host must supply all health/reset events. Policy requires a witnessed acquisition event; full process Reset/counter restart is outside the in-run reset contract. Equal-epoch reacquisition samples are conservatively rejected after reset because no ordering token exists. No flight age threshold or physical performance established. |

See the [validity, epoch, lifecycle and evidence contract](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7f-2b-evaluation-time-command-health-gate---2026-09-30).

## Phase 7F-2A disconnected controller command mathematics - 2026-09-30

**DISCONNECTED CONTROLLER COMMAND MATHEMATICS / NO ACTUATOR AUTHORITY /
NOT CLOSED LOOP / NOT FLIGHT VALIDATED.** Evidence anchor: Phase 7F-1 `4d3cc0b`.
This isolated math gate narrows the earlier 7F-2 proposal; fault handling is deferred.

| FIELD | FINDING |
|---|---|
| Paths / purpose | New `disconnected_detumble_math.py` and `test_disconnected_detumble_math.py`: one claim, accepted coherent snapshot to correct disconnected command mathematics. No scenario integration. |
| Inputs / provenance | Immutable accepted Phase 7F-1 snapshot: sampled body rate rad/s and S-to-B transformed TAM in T, original epochs/quality/provenance, attitude carried but unused. Exact intended evaluation epoch plus unchanged `hs2_sim_config` with full source/status JSON and fingerprint. |
| Controller | Reuses actual `basilisk_adcs_adapter.controller_step`: K(omega x B), existing componentwise current/dipole clipping, then m x B. Active backend in these tests is Python; optional `adcs_core` unavailable. Only missing requested-dipole diagnostic is reconstructed with the existing cross helper. |
| Output / actuation | Frozen Python record of requested/clipped dipole, current, predicted torque, flags, computation validity and epochs/provenance. No native output message, subscriber, effector or schedule. Predicted torque is never presented as applied torque; computation validity is not authorization. |
| Timing / frames | Inputs share sample s; math evaluates at selected e, preserving both. Test-only diagnostic case s=0.4 s, e=0.5 s. B-field uses C_SB transpose, with explicit nonidentity-mount case. No production timing, axis mapping or hardware value changed. |
| Environment / sensors | Synthetic controlled accepted-snapshot fixtures, no orbit/WMM, live estimator, sensor-performance model or spacecraft propagation in this test. |
| Independent validation | 9/9 tests, 13 cases: zero/parallel/perpendicular/arbitrary, mixed/all saturation, positive/negative clipping each axis, and S-to-B direction. Actual production SysModel matches dipole/torque exactly. Independent 50-digit Decimal scalar equations agree within roundoff; maximum torque error 2.964615315390051e-21 N m. Sign/damping, current and epoch checks pass. |
| Preservation | Existing runtime/configuration and saved outputs unchanged. Compileall passes; focused tests only. No full orbit or existing regression rerun needed for this isolated addition. No static tooling installed. |
| Gate / limits / next | PASS for controlled command mathematics only. Capture-time coherence does not imply current health. Next candidate 7F-2B is evaluation-time inhibition/fault/reset handling, still disconnected. No source handover, actuator authority, closed-loop safety, pointing or flight claim. |

See the [math, timing, evidence and numerical results](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7f-2a-disconnected-controller-command-mathematics---2026-09-30).

## Phase 7F-1 coherent control snapshot — 2026-09-30

**CONTROL SNAPSHOT / SCHEDULING DEVELOPMENT CONTRACT / NO CONTROLLER AUTHORITY /
NO ACTUATOR AUTHORITY / NOT FLIGHT VALIDATED.** Phase 7E was committed at `d285cb4`
and the working tree was clean. This narrower phase precedes command computation.

| FIELD | FINDING |
|---|---|
| Paths / purpose | New `control_input_snapshot.py` and `test_control_input_snapshot.py`: define and test which published navigation estimate and stored magnetic acquisition belong together. No scenario integration. |
| Inputs | Native NavAtt/quality evidence plus matching accepted/unlatched Phase 7E consumer decision; stored TAM in S, explicit C_SB, acquisition/quiet validity, optional acquisition-epoch WMM N reference; selected cycle timing/provenance. No operational truth input. |
| Snapshot | Frozen owned tuples, source/frame/epoch/quality/provenance, capture epoch and exact intended evaluation epoch. Output is a coherent snapshot or explicit rejection, never a command. |
| Epoch rule | nav state = TAM acquisition = selected sample s; optional reference epoch=s; publication may follow state but must precede capture c, with s <= c <= compute e. Actual use must be at e. No time tolerance. |
| Ordering finding | Driver 600 samples before MEKF 580. At 1.4 s it can see only the 1.3 s estimator publication; capture after MEKF gets 1.4 s for evaluation at 1.5 s. Existing SimpleNav 800 has no such lag. A fully rewired driver would reject the stale header through its current-epoch guard. |
| Development candidate | Extend the Phase 7E post-publication frozen probe with stored TAM and this gate, then use the snapshot in a future disconnected calculation at the unchanged compute event. No production reordering performed. |
| Validation | 15/15 focused tests and compileall pass. Isolated native-message/actual-adapter fixture reproduces lag, post-publication freeze and delayed-vector replay without any controller or effector. Tests cover invalid/stale/mismatched/future/nonfinite data, source/quality conflicts, frame transform and immutable ownership. |
| Preservation | Existing runtime files, subscriptions, schedules and production outputs unchanged; new module has no active import. No full-orbit or command runs. The existing 135-test baseline was not rerun. |
| Limits / next | Validity is at capture, not a guarantee of future estimator health or command authorization. Next separately authorized 7F-2: disconnected unchanged-controller evaluation, current-health/revocation checks, explicit inhibition and independent math checks. No pointing or flight claim. |

See the [complete scheduling/epoch contract and options](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7f-1-coherent-snapshot-and-scheduling-contract--2026-09-30).

## Phase 7E dummy navigation consumer addendum — 2026-09-14

**DUMMY NAVIGATION CONSUMER / NO CONTROL AUTHORITY / NOT FLIGHT VALIDATED.**
Source: focused code trace, native-message tests and local diagnostic runs from
Phase 7D commit `dd4b97d`; CONFIRMED software behavior only. See the
[consumer contract](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7e-dummy-navigation-consumer--2026-09-14).

| FIELD | FINDING |
|---|---|
| Paths / purpose | `attitude_navigation_consumer.py`, `validate_navigation_consumer.py`, consumer tests and explicit detumble API opt-in: verify navigation acceptance and logical source switching before controller connection. |
| Inputs | Native NavAtt and mandatory quality companions only; no spacecraft truth or raw sensor input to consumer. SimpleNav quality is explicitly ASSUMED ideal-simulation availability. |
| Control / actuation | None. Existing SimpleNav consumers and native actuation unchanged. No consumer output message can command an effector. Default production and diagnostic requested source remain SIMPLE_NAV. |
| States / selection | UNINITIALIZED, VALID, STALE, DEGRADED, FAULTED, REACQUIRING. Explicit requests SIMPLE_NAV/MEKF/NONE; rejected MEKF selects NONE, never automatic fallback. Fault latches require explicit fresh reacquisition evidence. |
| Frames / timing | Principal/shadow MRP and quaternion equivalence for C_BN mapping N to B; omega_BN_B in rad/s. State age uses consumer minus state epoch. Point consumer requires age 0; frozen consumer requires exact 0.4 s sample at 0.5 s, preserving both source headers. These allowances are TEST-ONLY. |
| Schedule | Existing cycle/controller 600 and MEKF 580 unchanged; new SimpleNav quality 565, point probe 560 and frozen probe 555. The real cycle driver still samples before MEKF publication; direct rewiring would be incorrect. |
| Validation | 135 regression tests pass, including 21 new tests. Seven live cases, four isolated adapter fault/recovery cases, analytic bidirectional handover and degraded-measurement/history cases pass. Startup valid at 0.4 s; fault fixture at 0.8 s, reset 1.2 s, reacquisition 1.4 s. |
| Handover | Analytic attitude differences 4.44089e-16 / 1.11022e-16 rad, rate jumps zero, epoch jumps zero. Live detumble differences are reported separately in the architecture document; no estimator tuning or flight acceptance threshold. |
| Preservation | Three committed-versus-working 4 s production cases byte-identical; seven live diagnostic hosts exactly match disabled host. Existing full production CSV hashes match committed evidence; full orbits were not rerun. |
| Outputs | Dedicated `navigation_consumer_validation.json` with case telemetry, source transitions, acceptance/rejections, faults, epochs, ages, representations and provenance. Optional host artifacts have `_consumer` suffix; production schemas unchanged. |
| Limitations / next | No controller handover, command-inhibition/retention policy, closed-loop safety or pointing claim. Next separately authorized step is a disconnected unchanged-controller command dry run behind the quality/exact-sample gate. Calibrated sensor/frame/timing evidence and accepted performance budgets remain unresolved. |

Run `python basilisk_runner/validate_navigation_consumer.py --report basilisk_runner/output_data/navigation_consumer_validation.json` for dedicated cases. No default scenario command enables the consumer.

## Phase 7D live shadow navigation addendum — 2026-09-14

**SHADOW DEVELOPMENT INTEGRATION / NOT FLIGHT VALIDATED.** This addendum records
the current opt-in integration; historical audit tables below retain their stated
commit scope. See the [message, timing and validity contract](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7d-live-shadow-integration--2026-09-14).

| FIELD | FINDING |
|---|---|
| Path / purpose | `attitude_mekf_adapter.py`, opt-in detumble scenario, `validate_mekf_shadow.py`: test live message translation, epoch transfer, faults and offline-equivalent MEKF execution. |
| Inputs | Unchanged production profile; actual SCStates rate as ideal gyro; stored acquisition TAM and WMM reference with validity/epochs. Explicit `--shadow-ideal-sun` supplies a TEST-ONLY synthetic Sun vector. No installed sensor performance is modeled. |
| Actuator / controller | Existing native MtbEffector and SimpleNav-fed controller unchanged. Shadow output has no control or actuation consumer. Direct-reference actuation/command replay are excluded from this opt-in mode. |
| Estimator | Verified Phase 7C MEKF/replay core; new interface models and nonmutating current-point rate accessor. Fresh two-vector TRIAD acquisition; no absolute initialization from a single vector or gyro alone. Explicit prior supports gyro-only development cases. |
| Sensors / frames | Actual TAMSensorMsg in S, explicit C_SB, matched acquisition WMM B_N. No truth B substitution. Current ideal gyro in B and optional ideal Sun use declared TEST-ONLY bridges. Physical body/mount registration remains unresolved. |
| Environment / timing | Existing Earth/WMM and 0.1 s task unchanged. Bridge 590, MEKF 580 and observer 570 run after existing controller/cycle driver 600. Cycle SAMPLE occurs at 0.4 s; shadow NavAtt is current-point data, while production control at 0.5 s still uses its frozen 0.4 s SimpleNav/TAM snapshot. |
| Output | Native NavAtt sigma_BN, omega_BN_B, timeTag plus mandatory Python quality/epoch companion. vehSunPntB unsupported. Separate `_shadow_host*` CSV/JSON artifacts; default output paths unchanged. |
| Validation | 114 regression tests pass, including 17 new interface tests. Eight dedicated cases pass; final live/offline q, bias and P differences are exactly zero with matching counts/epochs. All eight shadow host DataFrames exactly equal shadow-disabled hosts. |
| Production preservation | Full continuous baseline, hs2_candidate and diagnostic-cycle CSV bytes match saved Phase 7C baselines/hashes exactly; applicable checks 20/20, 20/20 and 44/44. |
| Degraded / fault cases | Single/no-vector startup stays uninitialized; loss of Sun after acquisition preserves propagation/magnetic updates. Invalid/stale samples rejected; delayed updates replay with original epochs. Gyro/coverage faults latch invalid status and stop fresh NavAtt publication; explicit reset/new pair reacquires. |
| Limitations | Noise-free TEST-ONLY P0/Q/R and gyro interpolation; no real Sun sensor, calibrated timing/noise or flight validity gate. Fresh common-epoch startup pair required. Continuous magnetic updates lack a quiet-window contract. Companion transport and diagnostic storage are development interfaces. No controller handover, flight attitude knowledge, pointing accuracy or HS-2 compliance claim. |
| Next | Test a quality-gated, sample-epoch dummy navigation consumer before any opt-in controller connection; characterize mounted sensor/coil-cycle timing and errors against independent truth before performance modeling. |

Default CLI navigation remains `simple-nav`. An explicit development example is
`python basilisk_runner/scenario_huskysat2_detumble.py --navigation shadow-mekf --shadow-ideal-sun --magnetic-cycle diagnostic --duration 4 --no-plots`.
The ideal Sun option is not enabled automatically.

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

### Phase 6A opt-in magnetic acquisition/actuation cycle

The continuous mode above remains the recovered regression default. `scenario_huskysat2_detumble.py --magnetic-cycle diagnostic` selects a separate ASSUMED / TEST-ONLY timing configuration; `--cycle-config` accepts an explicit provenance-bearing cycle JSON. Both physical profiles remain unchanged. No HS-2 burst, settling, PWM or sample-phase value is released by this option.

| FIELD | PHASE 6A BEHAVIOR |
|---|---|
| Purpose/status | Validate actual coil-off acquisition and held-burst architecture. WORKING DEVELOPMENT BASELINE; NOT FLIGHT VALIDATED. |
| Phase sequence | In each diagnostic 1 s cycle: COIL_OFF at 0; QUIET; SETTLING at 0.2; SAMPLE at 0.4; COMPUTE at 0.5; ACTUATE over [0.6,1.0); repeat. Sample and compute are single events; their phase spans include the explicit subsequent waiting interval. |
| Actual sensor/controller execution | The driver calls the existing TAM only at SAMPLE and the unchanged controller only at COMPUTE. Field and nav are frozen at the same acquisition epoch. A sample from an earlier cycle cannot authorize a new burst. |
| Actuator/environment | One attached/scheduled native MtbEffector. Gated body dipole is republished each 0.1 s tick for the following interval. Current WMM inertial field continues supplying the plant; Earth orientation and magnetic physics are unchanged. |
| Validity | Native input dipole/effective dipole and completed-step torque must be zero at acquisition, with sufficient observed quiet history and current field/state/nav epochs. Invalid acquisition or control usage faults in strict mode; rejection counters remain visible. |
| Telemetry | Schema 5 records every plant tick, phase boundaries, actual TAM sample/message epochs, held sample state/field, compute/burst epochs, requested/clipped/electrical/native-input dipoles, and independent native torque. Current truth B_B is explicitly distinguished from held measured B. At a coil-off boundary the logged native torque can still describe the preceding burst; checks use the held input of the completed interval. |
| Validation/accounting | Independent event-grid reconstruction, corrupted-record tests, runtime invalid-input tests, native torque/state checks, and full-step work versus propagated energy. Electrical energy is only an estimate under the existing provisional I^2 R model. |
| Output | Separate `*_cycled.csv`, physical config, cycle config/hash, and run manifest; `validate_magnetic_cycle.py` produces timing/duty/energy results. |
| Limitations | Ideal instantaneous sampling and electrical actuation; no carrier, RL decay, magnetic contamination magnitude, remanence, measured settling criterion, thermal limit or hardware correlation. The nominal 40% diagnostic burst duty is not an HS-2 design selection. No 24-hour requirement verification. |

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
