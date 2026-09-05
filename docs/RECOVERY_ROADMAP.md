# Ordered engineering recovery roadmap

Audit-derived plan, 2026-09-05. Goal: a COMPLETE, validated HS-2 ADCS Basilisk simulation with traceable spacecraft configuration, realistic sensing/actuation, mission modes, and independent requirement verification.

This document authorizes no implementation. Current scope is the six documentation files only: no simulation changes, builds, runs, dependency changes, commits, or Google Drive writes were performed. Future phases require the user's expanded implementation scope. The sequence below preserves the completed audits and does not call for repeating broad exploration.

Read [REPO_MAP.md](REPO_MAP.md), [PHYSICAL_PARAMETERS.md](PHYSICAL_PARAMETERS.md), [REQUIREMENTS_BASELINE.md](REQUIREMENTS_BASELINE.md), and [SCENARIO_STATUS.md](SCENARIO_STATUS.md) together.

## Common completion rules

Each phase needs a defined input configuration/revision, status for every engineering value, reviewed frame/unit/sign/time conventions, meaningful tests, and explicit exit evidence. Preserve unresolved alternatives and failure cases. ASSUMED cases may support development or sensitivity work when labeled; they cannot become flight claims through a passing plot.

Before any future commit, run the applicable tests and review the scoped diff. Record code/configuration revisions, environment/dependency version, effective controller backend, coefficient hash/epoch, random seed, initial conditions, scheduling, and acceptance thresholds for every verification run. Keep generated artifacts distinct from source and published acceptance records.

## 1. Authoritative configuration

**Work:** Establish one approved set of requirements, ICDs, BOM, CAD assembly/configuration, mass properties, and calibration records. Resolve the 3U versus legacy box, mass-budget versus measured mass, GranSystems versus analyzed ISISpace, torquer variant/driver, camera/lens selection, sun-sensor model/count, and Sagitta truth/estimator role. Reconcile document IDs, unsigned/future-dated histories, and proposed versus approved budget changes.

**Inputs:** Existing Drive source register and contradiction tables. Request only the missing authoritative decision or artifact needed to close a specific item; no broad audit restart.

**Exit evidence:** Configuration authority and revisions named; actual mass, COM, full inertia tensor, reference origin/frame, deployed/stowed states, and uncertainty recorded or explicitly unresolved with a constrained development envelope. Approved requirement candidates distinguish obligations, predictions, and measurements. No implicit promotion of 3.72911 kg to released flight mass.

## 2. Frame/time repair

**Work:** Define inertial, Earth-fixed, body, sensor, actuator, camera, and truth frames. Establish physical body axes, origin/COM relation, DCM direction, quaternion/MRP convention, Euler diagnostic interpretation, polarity, and SI units. Set effective WMM epoch and Earth orientation through supported module interfaces. Resolve task/source age, acquisition timestamps, recorder phase, hold behavior, and 5/10 Hz flight-loop ambiguity.

**Checks:** Known-axis rotations and inverse/composition checks; directional field transformation against independent vectors; Earth rotation sign/phase at known times; explicit sensor/nav timestamps; startup and sample/command/plant ordering; different task/record-rate cases. Norm preservation alone is insufficient.

**Exit evidence:** Frame and timing contracts are testable and each telemetry sample has a defined physical epoch. Any one-step age or interpolation is explicit. Nearest-time joins cannot conceal misalignment.

## 3. Independent telemetry/validation

**Work:** Separate truth, measurement, estimate, command, and applied quantities. Remove the behavior that replaces applied-torque logs with command diagnostics when code changes are authorized. Record effector contributions, actual controller backend and fallback events, data validity, clipping, and source timestamps. Audit saved-result provenance before using historical metrics.

**Checks:** Independent effector torque versus expected moment crossed with plant truth field; rigid-body residual I*omega_dot + omega cross(I*omega) versus summed applied torque; rotational work/energy balance; torque-free energy/angular-momentum behavior; timestep convergence. Test missing/stale samples without silently replacing them with zero.

**Exit evidence:** A deliberately wrong torque sign, disconnected effector, stale sensor, backend failure, or mismatched epoch is caught by tests. Cross-product checks are not supplied entirely by the same controller history. Baseline metrics have explicit numerical tolerances and limitations.

## 4. Validated magnetic actuation

**Work:** Verify native MtbEffector in an isolated, understood case: contiguous 3 by numMTB layout, directions, message linkage, clipping, scheduler participation, and body/inertial field conversion. Attach only one control-torque path. Retain an independently tested bridge only if its role and limitations are explicit.

Resolve actual CR0002/MT01A hardware, winding gain/resistance/inductance, driver supply/current limits, polarity, PWM-plus-direction wiring, saturation, residual moment, current decay, duty limits, and temperature dependence. Distinguish driver bus power from winding I^2 R and coil-current command from measured current.

**Checks:** Orthogonal/parallel moment-field cases, sign/polarity tests, off-axis mounting, clipping boundaries, zero/invalid field, disconnected channels, reset/off behavior, and no double torque. Correlate current-to-dipole and moment-to-torque with calibrated hardware evidence when available.

**Exit evidence:** Independently logged applied torque agrees with the accepted actuator model across the declared envelope. Native availability is not mistaken for successful integration. The disputed 0.85 A m^2 air-coil value and 12 V/5 V operating points have an explicit disposition.

## 5. Sensor/estimator realism

**Work:** Model the selected VN-100 raw gyro/magnetometer and confirmed sun-sensor/ADC configuration. Include frame alignment, sample rates, noise density versus sampled variance, bias/drift, quantization, scale error, latency, dropouts, saturation, temperature, eclipse/albedo, and sensor validity. Implement measured MTQ quiet periods and magnetic contamination/calibration behavior. Freeze truth-sensor feedback policy.

**Checks:** Deterministic sensor cases; sample/hold/timestamp tests; stochastic distributions/seed reproducibility; calibrated illumination and magnetic sweeps; gyro/bias propagation; observability and estimator covariance/consistency across field/Sun geometries. Validate against truth that is not improperly fed into the estimate under evaluation.

**Exit evidence:** Sensor parameters have applicable hardware/calibration provenance. Attitude knowledge is measured over the operating envelope; budget predictions of 10 or 5 arcsec are not treated as demonstrated inputs. Truth-only SimpleNav remains labeled for debug scenarios.

## 6. Disturbances/environment

**Work:** Establish final orbit/epoch and add only justified environment fidelity: consistent gravity and gravity-gradient torque, geomagnetic field, residual dipole, atmosphere-relative drag, SRP, Sun/eclipse geometry, and deployed panel/antenna properties. Use measured or bounded COM-to-CP offsets and residual moments. Review standalone Earth-rotation and SRP signs before using its code/results as comparison evidence.

**Checks:** Authoritative WMM component test vectors, known Sun/eclipse cases, correct SRP force direction, drag relative-wind sign, zero/nonzero offset torque, gravity-gradient limiting cases, invariants, and integrator convergence. Compare independent implementations only after aligning assumptions.

**Exit evidence:** Each disturbance has an enabled configuration, frame, units, source/uncertainty, and isolated validation. Model omissions have bounded consequences. Shared legacy equations do not count as independent confirmation.

## 7. Detumble requirement verification

**Work:** Replace development acceptance criteria with the approved release-rate ensemble and ADCS requirement definition. Include deployment delays, initial attitude and rate directions, field/orbit variation, realistic sensing/quiet duty, actuator saturation, thermal constraints, and power availability.

**Checks:** From the accepted initial envelope, reach and sustain the accepted 0.5 deg/s condition within 24 h. Define norm versus component limits, dwell duration, failures/rebounds, and exposure constraints. Integrate energy over the agreed subsystem/bus boundary against the resolved 11/60 Wh conflict. Check rate and energy worst cases separately.

**Exit evidence:** Requirement-linked results with uncertainty and failures retained. A first crossing of 0.05 rad/s or a one-orbit development run cannot close ADCS-1/2/3. State exactly which initial conditions and environmental/hardware limits were verified.

## 8. Pointing/acquisition/hold

**Work:** Resolve the 10/15/20/42-43 deg candidates, Sun keep-outs, optical FOV, boresight transforms, attitude knowledge, slew ceiling, and any actual 180 deg slew obligation. Implement reachable Earth-limb/star-field acquisition and hold using the accepted magnetic actuators and moving references. Define roll choice, forbidden regions, settling time, recovery, and simultaneous target feasibility.

**Checks:** Field-dependent instantaneous underactuation, orbit-wide authority, initial attitude/rate ensembles, saturation, disturbances, eclipse/sensor loss, keep-out-respecting maneuvers, hold error and duration, and reacquisition. Preserve both failed magnetic experiments as negative regression cases after their source/run provenance is established.

**Exit evidence:** Accepted physical actuator achieves defined acquisition/hold requirements over a stated envelope. Direct-torque MRP convergence remains a separate proof of concept. Neither instantaneous torque projection nor smaller initial errors prove general magnetic pointing feasibility.

## 9. Mission modes

**Work:** Integrate deployment, detumble, safe, Sun pointing, payload acquisition/experiment, attitude hold, COMMS, and recovery behavior according to the approved state machine. Couple battery/power, thermal state, panel deployment, sensor availability, and command/telemetry timing. Fill the currently missing operational constraints through project decisions.

**Checks:** Day-in-the-life scenarios, mode entry/exit guards, hysteresis, stale-data handling, actuator zero-current reset/off, fault escalation, safe-mode recovery, exposure synchronization, data collection, Sun keep-outs, and link opportunities. Include interruptions and low-power/thermal states.

**Exit evidence:** Requirement-traceable mode transitions and fault cases work under the modeled mission timeline, with no use of prohibited truth inputs or ideal direct torque in flight-performance modes.

## 10. Monte Carlo / requirement verification / HIL correlation

**Work:** Define provenance-backed uncertainty distributions and correlations, reproducible ensembles, coverage/confidence goals, and requirement-specific statistical acceptance rules. Correlate the simulation with signed calibrated unit/subsystem/FlatSat/HIL records for the actual hardware and flight-software revision. Resolve every discrepancy rather than tuning only to favorable traces.

**Checks:** Repeatable seeds; boundary/worst cases; sensitivity to inertia, alignment, residual field, gyro bias, timing, duty/power/thermal limits, release conditions, and environment; independent truth comparisons; complete failure reporting. Reproduce selected cases from recorded manifests and raw data.

**Exit evidence:** Completed RVM verification package with source/configuration/software revisions, calibration records, raw telemetry, independent acceptance calculations, quantified uncertainty, and reviewed residual risks. Tests alone do not create a flight qualification claim; the applicable project authority must accept the evidence.

## Immediate blockers and documentation disposition

| BLOCKER | PRESENT EVIDENCE | PHASE / CLOSURE |
|---|---|---|
| Actual spacecraft mass/COM/full tensor | Legacy box; estimated budget; no released mass properties | 1, then2: released assembly mass properties and frames. |
| Physical body/boresight/mounting axes | Empty Axis Definition; STR ICD references not implemented | 1-2: completed ICD/CAD/alignment survey. |
| Effective field orientation and timing | WMM assignment plus missing explicit Earth-orientation input; sample/record phases | 2: directional/time tests and explicit contracts. |
| Applied torque not independent | CSV overwrites effector values with commands | 3: separate telemetry and plant residual/work checks. |
| Torquer variant/driver/sun-sensor conflicts | Model/count/voltage/PWM contradictions across ICD/MDD/budgets | 1,4,5: approved BOM/interfaces and calibrated hardware. |
| Flight sensor/estimator realism missing | Ideal active sensors; ambitious budget predictions without completed evidence | 5: realistic inputs and independent estimation validation. |
| Pointing and power requirements disagree | Multiple degree limits, keep-outs, energy and thermal interpretations | 1,7,8: approved obligations and measurement boundaries. |
| No located completed ADCS validation package | Tracker not-done entries, blank procedures, missing result records | 10: calibrated signed records and reproducible correlation. |

These questions did not prevent accurate audit documentation; they remain explicitly unresolved instead of being answered with arbitrary constants. No implementation phase was started by creating this roadmap.
