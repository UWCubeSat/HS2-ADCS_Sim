# Scenario status

Audit-derived record, 2026-09-05, anchored to 833b015 before documentation changes. No simulation was run or changed to create this document. Historical pass/fail observations are retained as audit findings, not newly reproduced results.

All scenarios below are **NOT FLIGHT VALIDATED**. WORKING DEVELOPMENT BASELINE denotes useful software bring-up. PROOF OF CONCEPT denotes limited feasibility/control demonstration. FAILED EXPERIMENT records an unsuccessful development attempt without proving physical impossibility. LEGACY REFERENCE is historical/comparison code, not ground truth.

Engineering values below are ASSUMED development inputs unless linked to a separately qualified source in [PHYSICAL_PARAMETERS.md](PHYSICAL_PARAMETERS.md).

## Phase 8B-2A post-acquisition synthetic TAM bias - 2026-10-02

**SYNTHETIC TAM-BIAS ESTIMATOR RESPONSE / POST-ACQUISITION SHADOW DEVELOPMENT TEST /
COLD-START ACQUISITION LIMIT IDENTIFIED / NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.**
Checkpoint `60c428f`, initially clean. PASS for the running-estimator response only.

| FIELD | FINDING |
|---|---|
| Profile | Unchanged TEST_BIAS_ONLY: [1,-2,3] microtesla in S, canonical Tesla, identity mounting, no noise/scale/cross-axis changes. ASSUMED / TEST-ONLY, tam_sensor_model.py revision 2026-10-02; runtime_usable_for_flight=false. |
| Cold start | Three finite, quiet-eligible biased TAM samples; pair cosine disagreement 0.0869286711 exceeds unchanged 1e-8 TEST-ONLY tolerance. inconsistent_acquisition_pair; estimator UNINITIALIZED. Cold-start capability remains BLOCKED, independently of sensor validity. |
| Post-acquisition boundary | Normal ideal TAM/Sun acquisition and initialization at 0.4 s; explicit bias enable at 1.4 s, biased acquisitions 1.4/2.4 s, processing 1.6/2.6 s. No forced state or acquisition-gate change. |
| Response | Bias delivered exactly once. Both magnetic updates reduce measured residual and pull truth attitude toward the biased reference; subsequent ideal Sun updates reduce their own disagreement. Independent vector geometry confirms innovation/correction signs. |
| Attitude / bias state | Final / peak published / peak witnessed attitude error 0.118854613 / 0.166220517 / 0.177154963 rad. Estimated gyro-bias peak norm 0.144095675 rad/s despite ideal gyro; movement occurs at vector corrections, not injected gyro bias. |
| Health / timing | Covariance finite, symmetric, positive definite; minimum eigenvalue 7.934908e-6 in numerical SI-state coordinates. Magnetic/Sun counts 3/4, two replays, no post-acquisition rejects. Q/R/P0 unchanged. |
| Isolation | Native TAM retains controller ownership; both bias selections rejected for MEKF actuation. Committed/current ideal and both biased host CSVs are byte-identical. Rejected shadow samples retain finite values. |
| Verification | 13 focused tests + 17 adapter regressions pass; 24 validator checks pass. Original BLOCKED cold-start artifact is retained inside the final evidence. Overall success explicitly retains cold_start_bias_acquisition_supported=false. |
| Next / blocked | Characterize acquisition-consistency robustness without gate tuning. Installed calibration, physical recovery, realistic vector errors and flight/requirements performance remain unresolved. |

Numerical observations are CONFIRMED only for this TEST-ONLY fixture, source
validate_tam_bias_response.py / phase8b2a_tam_bias.json, revision 2026-10-02.
See the [two-case contract and response evidence](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8b-2a-post-acquisition-synthetic-tam-bias---2026-10-02).

## Phase 8B-1 parametric TAM measurement and validity - 2026-10-02

**TAM MEASUREMENT / VALIDITY MODEL FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL /
NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** PASS for ideal equivalence
and explicit contracts, not realistic magnetic performance. HEAD `7b93165`; prior
uncommitted 8A-2C work preserved.

| FIELD | FINDING |
|---|---|
| Purpose / value model | Canonical Tesla; C_SB maps B to S; sensor-frame bias/gain/cross-axis/noise, optional clipping, explicit S and reconstructed B outputs. Caller-supplied local B is the future contamination insertion point; live input remains WMM-only. |
| Inputs / evidence | All profile errors are ASSUMED / TEST-ONLY, tam_sensor_model.py revision 2026-10-02; runtime_usable_for_flight=false. Native/actuator configuration and Phase 6A timing remain unchanged. |
| Validity / timing | Finite vector separate from eligibility; coil context, quiet evidence, saturation, source/fingerprint and age checks. Live current evidence is absent, not measured zero. Truth/acquisition/publication at 0.4/1.4/2.4 s; delayed processing at 0.4/1.6/2.6 s. |
| Live scope | Only exact IDEAL_REGRESSION can opt into the shadow bridge. Perturbations/delay are isolated-only; all TAM model selections rejected for MEKF actuator ownership. Native TAM still drives detumble. |
| Equivalence | 31-epoch native/Python value difference <=1.016440e-20 T; accepted live sample difference <=6.776264e-21 T. MEKF q/bias/P differences <=3.330669e-16 / 5.070441e-16 rad/s / 6.505213e-19. Event order, epochs and validity exact. |
| Rejection / preservation | One shadow-only invalid cycle-metadata sample is rejected identically while preserving its finite field. HEAD/current/model host bytes identical; default input/estimator traces exact. |
| Validation | 10 isolated + 5 live TAM + 17 adapter + 9 nominal-control tests pass. All 13 top-level validator checks pass. No shared cycle changes or perturbation-performance campaign. |
| Blocked / next | Installed calibration/mounting and coil-to-sensor recovery/filter history remain TBD. Next: one separately authorized deterministic TAM-bias shadow response, maintaining fixed Q/R and zero modeled-TAM actuator authority. |

Numerical observations are CONFIRMED only for the software fixture, source
validate_tam_sensor_model.py / phase8b1_tam_model.json, revision 2026-10-02.
See the [measurement, validity and equivalence contract](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8b-1-parametric-tam-measurement-and-validity---2026-10-02).

## Phase 8A-2C live combined synthetic gyro bias and noise - 2026-10-02

**COMBINED SYNTHETIC GYRO BIAS + NOISE RESPONSE / SHADOW DEVELOPMENT TEST /
NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Checkpoint `7b93165`.
PASS for composition in one short live fixture; no shared runtime changes.

| FIELD | FINDING |
|---|---|
| Profile / inputs | Harness-only TEST_BIAS_PLUS_NOISE, runtime_usable_for_flight=false. Existing bias [0.003,-0.002,0.001] rad/s, per-sample sigma [0.001,0.002,0.003] rad/s, seed 8101; identity C_SB, unit scale, zero cross-axis error/latency, same 3 s schedule/Q/R/P0. ASSUMED / TEST-ONLY Parameters reused from gyro_sensor_model.py revision 2026-10-01. |
| Composition | All 31 delivered measurements equal ideal + bias + independently seeded noise exactly. Same raw draws as noise-only, unchanged epochs, no extra/double effects. Critical gate precedes estimator interpretation. |
| Reference / repeat | All three references reused from source-bound 8A-2A/2B evidence; combined case run twice. Inputs, estimator histories, innovations, covariance, validity and event/replay counts match exactly. |
| Bias | Final estimate [3.599426e-3,-1.109702e-3,9.793997e-4] rad/s; error [5.994256e-4,8.902977e-4,-2.060026e-5] rad/s. Correct injected signs; maximum witnessed norm 3.891854e-3 rad/s; changes explained by vector updates, no convergence target or observed short-window runaway. |
| Attitude / innovations | Final error 6.231268e-4 rad; published/witnessed maxima 1.852583e-3 / 2.315632e-3 rad. Independent combined endpoint-interpolation check passes; all actual vector updates reduce measured-vector residuals. Nonlinear results need not superpose. |
| Covariance | Finite, symmetric, positive definite; minimum eigenvalue 7.951123e-6 in numerical SI-state coordinates. No unexplained growth/collapse; Q/R/P0 unchanged and uncalibrated. |
| Isolation | SimpleNav controls the host, no MEKF command owner, combined modeled-gyro closed-loop selection rejected. Host bytes match all three references and repeat. |
| Verification | Eight focused tests / 25 live checks pass. No prior statistical campaign, shared-code changes, Monte Carlo or long run. Five distinct post-acquisition updates, six callbacks, two replays, zero rejections. |
| Next / blocked | TAM measurement/validity branch first because calibration and actuation contamination affect both detumble and estimation. CSS geometry/channel reconstruction remains unresolved. Priority is architectural, not a measured error-budget ranking; installed calibration and justified covariance remain required. |

Numerical observations are CONFIRMED only for this TEST-ONLY experiment, source
validate_gyro_bias_noise_response.py / phase8a2c_gyro_bias_noise.json, revision 2026-10-02.
See the [four-case comparison and sensor-branch decision](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8a-2c-live-combined-synthetic-gyro-bias-and-noise---2026-10-02).

## Phase 8A-2B live synthetic gyro-noise response - 2026-10-02

**SYNTHETIC GYRO-NOISE ESTIMATOR RESPONSE / SHADOW DEVELOPMENT TEST /
NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Checkpoint `1366930`.
PASS for the deterministic seeded stochastic-response claim; no shared runtime changes.

| FIELD | FINDING |
|---|---|
| Purpose / inputs | Existing 3 s live ideal versus TEST_NOISE_ONLY case, repeated with seed 8101. Identity C_SB, zero deterministic bias/scale error, per-sample sigma [0.001,0.002,0.003] rad/s, unchanged vector schedule/Q/R/P0. All inputs ASSUMED / TEST-ONLY, gyro_sensor_model.py revision 2026-10-01. |
| Sequence / repeatability | All 31 samples match an independent PCG64 sequence: raw draws and ideal-plus-noise measurements exactly; subtraction within roundoff. Same-seed inputs, estimator states, innovations, covariance, validity and counts reproduce exactly. No extra draws or double addition. |
| Attitude | Final ideal/noise errors 3.720384e-6 / 7.411605e-4 rad; witnessed maxima 1.594657e-5 / 1.438195e-3 rad. Independent endpoint-interpolated local growth/sign check passes. No monotonicity or accuracy requirement. |
| Bias state | Zero true deterministic bias; final estimate [7.843059e-4,7.599627e-4,6.150821e-5] rad/s; maximum witnessed norm 1.280840e-3 rad/s. Finite, held during gyro-only intervals, updated from vectors; no sign expectation or observed short-window runaway. |
| Innovations / covariance | Both vector residual histories respond; all actual updates reduce their measured-vector residual with correct geometric direction. Covariance finite, symmetric and positive definite; no collapse/explosive growth. Not statistically calibrated to this injected noise. |
| Isolation / preservation | SimpleNav remains actual control source; no MEKF command owner, modeled-gyro closed-loop selection rejected. Ideal/noise/repeat host CSVs byte-identical to the saved ideal reference. |
| Verification | Nine focused tests / 26 live checks pass. Five distinct post-acquisition vector events, six callbacks, two replays, zero rejections. No broad suites, alternate seed, Monte Carlo or long run. |
| Next / limits | Combined BIAS + NOISE shadow response is justified as the next separate experiment; not run here. Installed sensor calibration and evidence-based covariance remain prerequisites for realistic HS-2 prediction. |

Numerical observations are CONFIRMED only for this TEST-ONLY experiment, source
validate_gyro_noise_response.py / phase8a2b_gyro_noise.json, revision 2026-10-02.
See the [full sequence, response and isolation evidence](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8a-2b-live-synthetic-gyro-noise-response---2026-10-02).

## Phase 8A-2A live synthetic gyro-bias response - 2026-10-01

**SYNTHETIC GYRO-BIAS ESTIMATOR RESPONSE / SHADOW DEVELOPMENT TEST /
NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Checkpoint `d8eb4d2`.
PASS for one deterministic bias case; no shared runtime changes.

| FIELD | FINDING |
|---|---|
| Purpose / inputs | Compare committed 8A-1 IDEAL_REGRESSION and TEST_BIAS_ONLY over the unchanged 3 s horizon/vector schedule. Identity C_SB, bias [0.003,-0.002,0.001] rad/s in S=B, zero stochastic noise/gyro latency. |
| Measurement / sign | All 31 delivered samples contain bias exactly once. Native NavAtt equals delivered measurement minus posterior estimated bias, exactly. First correction at 1.1 s has +X/-Y/+Z signs. |
| Bias state | Initial estimate zero at 0.4 s; final [0.002797670,-0.001858275,0.000918266] rad/s. Residual [-2.023299e-4,1.417249e-4,-8.173425e-5] rad/s: partial convergence, no speed requirement or extended run. |
| Attitude | Final ideal/bias errors 3.720384e-6 / 2.781083e-4 rad. Maximum witnessed including pre-update peaks 1.594657e-5 / 2.602469e-3 rad. Development truth comparison, not HS-2 knowledge. |
| Innovations | Magnetic/Sun residuals expose bias drift; every actual update reduces its vector residual with independently verified correction sign. Five distinct post-acquisition vector events, six callbacks including replay; no duplicate-event counting as new data. |
| Covariance / tuning | Finite symmetric positive definite; bias-case minimum eigenvalue 7.951371e-6 in numerical SI-state coordinates. No observed collapse/divergence. Existing TEST-ONLY Q/R/P0 unchanged and uncalibrated. |
| Isolation | SimpleNav controls the host; ideal/bias host CSV bytes match the prior ideal baseline. No MEKF command owner; existing modeled-gyro closed-loop guard still rejects selection. |
| Validation | Seven focused tests / 18 live checks pass, including b*dt short-interval sign/growth and tampered-evidence failures. No earlier audits, sensor-development suite, long campaign or shared-code changes. |
| Next / limit | NOISE-ONLY shadow response with existing seed and fixed Q/R/P0. Actual installed gyro/sensor calibration, timing and realistic estimator performance remain blocked by missing evidence. |

See the [full bias, innovation, covariance and isolation evidence](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8a-2a-live-synthetic-gyro-bias-response---2026-10-01).

## Phase 8A-1 configurable gyro sensor model - 2026-10-01

**GYRO SENSOR MODEL FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL /
NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Checkpoint `3fdffd2`.
PASS for the model framework and exact ideal-regression gate.

| FIELD | FINDING |
|---|---|
| Purpose / source | Optional gyro_model in ShadowOptions; existing truth source remains default. Only attitude_mekf_adapter.py changes shared runtime. |
| Measurement | Proper C_SB maps B to S; scale/off-diagonal gains, additive bias and discrete Gaussian noise act in S; optional S clipping precedes C_SB.T return to B. MEKF alone subtracts estimated bias. |
| Profiles / provenance | IDEAL_REGRESSION, TEST_BIAS_ONLY, TEST_SCALE_ONLY, TEST_NOISE_ONLY, TEST_DELAYED_SAMPLE. All ASSUMED / TEST-ONLY with units/source/revision/frame/treatment and runtime_usable_for_flight:false. No vendor value adopted as installed truth. |
| Timing | Acquisition and publication availability remain distinct integer ns. Isolated latency/cadence supported; live integration requires existing cadence, zero offset/latency. Delayed gyro is rejected, not relabeled. Existing delayed-vector replay is unchanged. |
| Sensor limits | Optional hard clipping marks saturation invalid. No quantization, aperture/filter, temperature, drift, random walk, CSS or full magnetic error model. Installed gyro parameters remain TBD/TBC. |
| Actuation isolation | Modeled gyro is shadow-only; unchanged MEKF_DEVELOPMENT guard rejects modeled profiles. Controller, plant, limits, scheduling and estimator tuning unchanged. |
| Validation | 18 new tests, 41 MEKF/adapter regressions, 9 nominal closed-loop regressions pass; 12 live checks pass. 31 ideal samples and all estimator/input/status traces exactly match committed adapter. |
| Preservation | Committed/default/ideal/bias-shadow host CSV bytes identical. Existing unmodeled closed-loop behavior unchanged. Three short production checks byte-identical. |
| Next / limit | One shadow-only synthetic bias-plus-noise consistency experiment with fixed Q/R; no convergence/accuracy or requirements-grade performance claim. Installed calibration/timing evidence remains necessary. |

See the [measurement, timing and verification contract](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8a-1-configurable-gyro-sensor-model---2026-10-01) and [gyro evidence](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#phase-8a-1-gyro-model-evidence-disposition---2026-10-01).

## Phase 7G-2C late-fault command-validity boundary - 2026-10-01

**LATE-FAULT COMMAND-VALIDITY BOUNDARY / DEVELOPMENT TIMING TEST / NOT FLIGHT VALIDATED.**
Starting checkpoint: `c4bd9a7` (7G-2B). PASS for one deterministic scheduling test.

| FIELD | FINDING |
|---|---|
| Purpose / scope | Establish the lifetime between health approval and native command publication. New harness/tests only; shared runtime and simulation configuration unchanged. |
| Exact insertion | Actual DynamicsTask table: gate 550, TEST-ONLY late fault 549, owner 548, native readback witness 547. All share the same 0.8 s timestamp; ordered witnesses establish causality. |
| Fault | Reuse existing late_quality mechanism: publish real 0.7 s quality with its original header after approval. Keep stale metadata exposed at priority 568 on later ticks. Source-interface freshness fault; internal MEKF remains healthy. No new sensor/fault model or fabricated health decision. |
| Decision lifetime | Owner relies on same-tick approval without re-reading health. Published at 0.8 s means approved before the fault, not approved after it. Authorization expires for later ticks; it covers one publication and following 0.1 s integration interval. |
| Native response | Approved ID 1 / generation (0,1,0) survives publication at 0.8 s. Exactly one post-fault nonzero interval [0.8,0.9 s); first native zero command 0.9 s, first zero-torque interval [0.9,1.0 s), completed record 1.0 s. |
| Persistent fault | 22 later native publications through 3.0 s are zero; stale_quality inhibition, no new approval, no extra retained command, no second owner or SimpleNav fallback. |
| Validation | Nine tests / 17 live checks pass. Native subscriber, held-input epochs, independent torque/state/work evidence agree. Maximum torque discrepancy 9.4701e-22 N m. Three short production traces byte-identical to HEAD. |
| Architecture decision | OPTION 1: deterministic bounded development decision lifetime; no runtime correction or extra owner-side health recheck required for this tested architecture. Assumes scheduled callbacks continue executing. |
| Limits / next | ASSUMED 0.8 s fault / 3 s horizon, not flight latency. Next highest-value phase: realistic sensor/estimator modeling, because ideal truth-derived inputs dominate knowledge/performance uncertainty. No campaign or pointing implementation in this phase. |

See the [scheduler, decision lifetime and actuator evidence](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7g-2c-late-fault-command-validity-boundary---2026-10-01).

## Phase 7G-2B closed-loop reset / reacquisition - 2026-10-01

**MEKF CLOSED-LOOP RESET / REACQUISITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED.**
Starting checkpoint: `a1d29d1` (7G-2A). PASS for one controlled lifecycle.

| FIELD | FINDING |
|---|---|
| Purpose / scope | Verify that reset/reacquisition cannot revive pre-reset actuation. New validation harness/tests only; all shared runtime, controller, estimator, owner, cycle, physical and environment code remain unchanged. |
| Fixture | Existing InputBatch validity/reset interface and ObserverOptions hook; one fault at 0.8 s, explicit reset at 1.0 s, unchanged old-envelope replay at 7.7 s. Nine-second TEST-ONLY horizon. No new reset architecture or fault model. |
| Reacquisition | Existing 0.7 s ideal-Sun cadence and 1 s TAM cycle next coincide at 7.4 s. Gate lifecycle REACQUIRING / estimator UNINITIALIZED from reset until then; actuator stays zero. No sensor cadence was shortened. |
| Generation | Tuple is (observer reset serial, estimator acquisition count, observer health-revocation serial). Initial (0,1,0), fault (0,1,1), reset (1,1,1), reacquired/fresh (1,2,1). The adapter retains its acquisition counter across the in-run reset event. |
| Reacquisition alone | Coherent candidate 7.4 s / calculation 7.5 s remains unusable at 7.6 s: fresh_post_reacquisition_snapshot_required. Existing policy requires a sample strictly later than reacquisition. |
| Old-command replay | At 7.7 s, replay immutable command ID 1 with sample 0.4 s / computation 0.5 s / generation (0,1,0). Healthy source cannot authorize it: snapshot_predates_current_acquisition, generation mismatch, native zero. Epochs/deadlines were not relabeled. |
| Fresh restart | TAM, native MEKF state/publication and capture 8.4 s; new calculation 8.5 s; command ID 3, generation (1,2,1), reaches native MtbEffector at 8.6 s. First completed resumed torque record 8.7 s. |
| Native safety / physics | 78 consecutive zero publications over [0.8,8.6 s), including 64 REACQUIRING boundaries. Quiet/sample phases remain zero. Resumed dipole [-0.2,0.2,0.8220612696753196] A m^2 matches the gated result; independent maximum torque error 3.4145e-21 N m. |
| Validation / preservation | Eight focused tests and live validator pass. Fresh snapshot matches independent TAM/NavAtt records. Finite continuous plant and energy/work checks pass. Three short production traces remain byte-identical to HEAD; no shared runtime changes required broader regression reruns. |
| Limits / next | In-run lifecycle only; no process restart/counter reset, arbitrary event timing, sensor realism, flight latency, detumble requirement or pointing claim. Next smallest test: one late fault at the gate-to-owner publication boundary. |

See the [generation, timing and actuator evidence](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7g-2b-closed-loop-reset--reacquisition---2026-10-01).

## Phase 7G-2A closed-loop fault inhibition - 2026-10-01

**MEKF CLOSED-LOOP FAULT INHIBITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED.**
Committed starting checkpoint: `0569ad8` (7G-1). PASS for the single tested fault.

| FIELD | FINDING |
|---|---|
| Purpose / inputs | Prove revocation of real nonzero native actuation. Existing MEKF, cycle, physical configuration, controller, ideal sensor bridge and actuator limits unchanged; 3 s TEST-ONLY horizon. |
| Fault | Reuse 7F-2C `LiveFixtures('mid_burst_fault')`: one `InputBatch.gyro_valid=False` event at 0.8 s, priority 588. Existing estimator fault `7F2C_TEST_ONLY_fault` stays latched despite subsequent valid input. No reset/reacquisition injection or new failure model. |
| Connection | Python-only `mekf_fault_test=ObserverOptions(...)` requires explicit MEKF_DEVELOPMENT ownership. No CLI fault flag. Gate/owner/effector logic and task priorities are unchanged; no second publisher or automatic SimpleNav fallback. |
| Pre-fault proof | Sample/capture 0.4 s, computation 0.5 s; nonzero commands at 0.6/0.7 s produce real native torque over two completed intervals. The stored calculation remains nonzero when revoked at 0.8 s. |
| Inhibition / latency | Event 0.8 s -> current gate health and native zero input 0.8 s -> zero-torque interval [0.8,0.9 s), recorded at 0.9 s. Event-to-zero-interval-start latency is 0 simulation ns in this schedule; completion/readback is 0.1 s later. This is not a flight latency requirement. |
| Persistence / cycle | All 23 publications from 0.8 through 3.0 s are fresh zero, including 10 ACTUATE boundaries. TAM acquisitions at 1.4/2.4 s remain valid with 0.6/1.6 s quiet age; no rejected acquisitions. No old command returns. |
| Independent validation | Gate/owner/native subscriber records, independent held-input recorder/epoch guard, native torque and propagated state agree. Native torque is exactly zero throughout completed inhibited intervals; rigid-body/energy checks confirm continuous torque-free rotation. |
| Telemetry | In fault fixtures, acquisition-only navigation diagnostics after a failed capture are explicitly labeled NO_CURRENT_COHERENT_CONTROL_SNAPSHOT. Historical numeric calculations remain diagnostic; they are not command authority. Nominal/default telemetry is preserved. |
| Tests / preservation | Nine focused actuator-boundary tests and 52 relevant regressions pass, including rejection of altered native-torque and retained-command evidence. Short committed-versus-working continuous baseline, candidate and cycled baseline remain byte-identical. See architecture record for commands and numerical results. |
| Boundary / next | PASS only for this delivered fault and deterministic schedule. Next: separately authorized CLOSED-LOOP RESET / REACQUISITION. No restart test, hardware latency, realistic sensor performance, mission detumble, pointing or flight claim. |

See the [fault-inhibition evidence and commands](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7g-2a-closed-loop-fault-inhibition---2026-10-01).

## Phase 7G-1 first nominal MEKF closed-loop A/B - 2026-09-30

**FIRST MEKF CLOSED-LOOP DEVELOPMENT INTEGRATION / NOMINAL SHORT-RUN ONLY /
NOT FLIGHT VALIDATED.** Committed starting checkpoint: `29e4149` (7F-2C).
This section supersedes earlier disconnected-only statements for the explicit
`MEKF_DEVELOPMENT` option. `SIMPLE_NAV_REFERENCE` remains the production default.

| FIELD | FINDING |
|---|---|
| Purpose / selection | First native-actuator connection of the verified MEKF, snapshot, unchanged controller calculation and application health gate. Explicit `--control-source MEKF_DEVELOPMENT`; nominal diagnostic cycle, explicit ideal Sun, explicit duration <=10 s required. No fixture callback, delayed/dropout sensor option or implicit full run. |
| Inputs | Existing regression-baseline profile, initial tumble, 0.1 s plant step, WMM/Earth orientation, actuator limits, controller gain/law and test-only sensor/covariance configuration unchanged. A/B both use the same existing ideal bridge: 0.7 s Sun cadence, first vector at 0.4 s. |
| Ownership | One native MTBCmd subscriber source: `MEKFCommandOwner` at priority 548. In opt-in mode the cycle driver does not publish MTBCmd and does not calculate a SimpleNav command. It retains acquisition at 600; native MtbEffector remains the sole magnetic dynamics effector. |
| Timing / provenance | Actual TAM/MEKF sample and capture 0.4 s, unchanged calculation 0.5 s, health decision and publication 0.6 s, first completed nonzero-torque interval at 0.7 s. Repeats each existing cycle. Sample/cycle/command/generation IDs and publication/subscriber epochs are retained. |
| Zero enforcement | Fresh zero payload each invocation unless that tick's existing chain decision is usable. Pre-acquisition, SAMPLE, COMPUTE and quiet phases command zero. Quiet history reflects actual final publication. A COIL_OFF row can report torque from the preceding burst; checks use held inputs. |
| Independent evidence | Native subscriber readback/module ID, independent MEKF NavAtt recorder, native final-RK-stage output, accepted plant state, held inputs and RK4/work reconstruction. Applied torque is never overwritten. |
| Nominal result | PASS for two 6 s runs: six cycles, 24 energized and 36 zero intervals; all bounds, freshness, finite-state/covariance, torque, state-step and work checks pass. Nine focused tests and 121 relevant existing regressions pass. |
| A/B differences | Final rate: SimpleNav 0.8742458216779315 rad/s; MEKF 0.8742458126971391 rad/s. Maximum rate-vector difference 1.8620311143422534e-8 rad/s; attitude difference 2.4724589342520592e-8 rad. Small estimated-bias corrections change commands and subsequent plant feedback. This is not a performance comparison or improvement claim. |
| Preservation | Committed-versus-working six-second continuous baseline, continuous candidate and cycled baseline CSV serializations are byte-identical. Both changed runtime modules are loaded from HEAD for the committed comparison. No physical parameters, MEKF math, cycle durations or controller tuning changed. |
| Limits / next | Nominal native control only; faults/resets were not newly exercised with actuator authority. Next justified work is a separately authorized short CLOSED-LOOP fault/inhibition test. Stability envelopes, detumble requirements, hardware realism, pointing and flight performance remain unverified. |

Details, tolerances, commands, and artifact provenance are in the
[Phase 7G-1 architecture record](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-7g-1-first-nominal-closed-loop-integration---2026-09-30).

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
