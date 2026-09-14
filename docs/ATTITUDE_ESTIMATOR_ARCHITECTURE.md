# HS-2 attitude estimator architecture — Recovery Phase 7B

Revision: 2026-09-07; document ID: HS2-SIM-EST-ARCH-7B, revision 1.
**Proposed simulation architecture: ASSUMED; NOT FLIGHT VALIDATED.**
The [machine-readable companion](../basilisk_runner/config/attitude_estimator_architecture.json) is an evidence/design artifact with `runtime_usable: false`. It is not a runtime configuration or a released flight algorithm.

Recommend a gyro-propagated multiplicative extended Kalman filter (MEKF), with a nominal unit quaternion and residual gyro bias, deterministic two-vector acquisition, and explicit validity/observability outputs. This phase specifies and checks the mathematics; it does not implement a running estimator, add sensors, connect an estimate to control, or establish pointing accuracy.

## Evidence and authorization boundary

The evidence is the [Phase 7A register](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md) and its [JSON](../basilisk_runner/config/attitude_sensor_estimator_evidence.json), together with the already inspected current interfaces. No Drive or web sources were fetched in Phase 7B. References such as E01 and T01 below are decision IDs in that register; I1/B3/SW1 identify its source catalog entries, including original document revisions and evidence qualifications.

Verified local HEAD is `b1a21e75cdc92f5f8793648123a55adfd9999830` (Phase 7A evidence documentation). Both Phase 7A files are tracked at this commit and were preserved unchanged. Their internal evidence anchor remains the preceding code baseline, `8341ae8cb71bafc5723876329e49e7aa2b2d3f1e`. Reproducibility uses the verified HEAD plus these exact evidence-file SHA-256 hashes:

| Evidence artifact | SHA-256 |
|---|---|
| Phase 7A Markdown | `0d1e386972aa709f2ec795e310431618d264fcffac37c0b3bb686d04fba6befd` |
| Phase 7A JSON | `c8ad069d40e67225f90708d790ebc54057bcd078284e652cb6024a80a52e8c8c` |

The companion JSON also records the hashes of current interface files. Its source references resolve through the unchanged Phase 7A catalog; no historical source is claimed to have been newly verified.

Engineering statuses remain CONFIRMED, TBR, TBC, TBD and ASSUMED, with the meanings in [AGENTS.md](../AGENTS.md). Proposed design choices are ASSUMED until reviewed. A mathematically checked equation is CONFIRMED only within its stated convention/model; it is not evidence of installed sensor performance. Unknown numerical quantities remain TBD/null. Test fixtures are synthetic algebra inputs, never spacecraft parameters or flight tuning.

I1 (ADCS ICD revision 5, 2026-09-01) is unsigned and describes cached data/interfaces without a complete estimator. B3 provides own-estimator/raw-channel intent and independent truth-tracker intent, not equations or measured performance. SW1 is explicitly outdated. Their unresolved ownership and interface conflicts remain TBC. New choices below define a candidate simulation architecture; they do not silently settle flight documents.

## Architecture decisions and alternatives

| ID | Decision | Status / Phase 7A evidence | Reason and boundary |
|---|---|---|---|
| A01 | Nominal q_BN and residual gyro bias b_g_B; error state [delta_theta_B, delta_b_g_B] | ASSUMED; E01–E05, P01–P03 | Seven stored nominal numbers, six local error coordinates. Bias is needed for gyro drift and sparse aiding; adding a state does not make it observable. |
| A02 | MEKF for propagation/local correction; TRIAD for valid two-vector acquisition and independent deterministic checks | ASSUMED; E03–E05 | Unit-quaternion propagation and a three-component attitude error avoid a redundant quaternion covariance. Async aiding and uncertainty bookkeeping are explicit. This is a recommendation, not an existing approved selection. |
| A03 | Raw gyro + valid magnetic vector + reconstructed Sun direction are baseline estimator inputs | ASSUMED; S01, S05, S06, E01, E02 | The candidate hardware roles support this chain; installed configuration and preprocessing remain TBC/TBD. |
| A04 | Reserve the truth tracker for independent validation; exclude LOST/FOUND and vendor AHRS attitude from baseline corrections | ASSUMED; S07, S08, E02, E03, E05 | Resolves simulation ownership explicitly while retaining the contradictory tracker-feedback ICD as TBC. An aided variant would need its own independent validation reference. |
| A05 | Timestamped event processing with gyro history and delayed-update replay | ASSUMED; T01–T04 | Physical acquisition epoch controls the model. Message arrival/publication time does not replace it. Buffer limits and flight latency policy remain TBD. |
| A06 | Health, initialization, directional observability and covariance credibility are separate outputs | ASSUMED; E01, K01, V01 | A normalized quaternion or small covariance is insufficient to authorize a mode. |
| A07 | Precalibrate mounting/scale/offset outside the filter; defer extra calibration states | ASSUMED; F01–F04, P07, P09, T03 | Unknown mounts, offsets and magnetic contamination cannot be repaired credibly by adding weakly observable states. |
| A08 | Advance to a separately authorized, noise-free Phase 7C prototype with synthetic mathematical cases | ASSUMED; E04, K01, V01 | The architecture is sufficient for convention/interface testing. Installed-performance modeling and pointing remain blocked. |

| State option | Benefit | Main limitation | Disposition |
|---|---|---|---|
| Attitude only | Small deterministic propagation/correction implementation | Unmodeled gyro bias accumulates between vector updates; tuning correction gain does not identify drift | Useful acquisition/unit-test comparator; not the recommended continuous candidate |
| Attitude + residual gyro bias | Represents the main drift mechanism and its coupling to attitude | Bias components need temporal excitation/aiding; initial bias and noise characterization remain missing | A01 recommended |
| Add magnetic bias, gyro scale, mounting misalignment, CSS gains, thermal states or time offsets | Could estimate selected calibration errors with sufficient independent excitation | Confounds attitude, reference error and sensor error; enlarged unobservable subspaces; no justified installed priors/process laws | Defer pending calibration evidence and state-specific observability study |

| Algorithm | Suitability | Decision |
|---|---|---|
| TRIAD | Deterministic attitude from two simultaneous or correctly transported noncollinear vector pairs; no gyro propagation or bias estimation; order/weighting matters under error | Acquisition and independent noise-free reference |
| QUEST / Davenport q-method | Weighted Wahba solution; accommodates multiple vector observations at a common attitude epoch; weights require defensible error models | Future acquisition alternative after measurement weights exist; not a continuous bias estimator by itself |
| Complementary observer | Gyro propagation and vector feedback with potentially simple bias adaptation | Valid comparator, but gain choices and bias observability still need evidence; less direct covariance/async measurement bookkeeping for this task |
| MEKF | Gyro integration, local vector updates, residual bias and covariance with explicit dropouts/delays | Recommended candidate; local validity and correct noise/correlation models are essential |
| Older “Kalman” proposal / vendor AHRS / legacy standalone filter | SW1 lacks a complete state/equations and is outdated; vendor AHRS has different internal aiding; standalone scaffolding is historical | Evidence only, not a validated implementation to inherit |

No algorithm remedies missing reference vectors, unknown alignment, wrong epochs or unobservable geometry.

## State, ownership and future interface

The nominal state is qhat_BN (scalar-first unit quaternion, dimensionless) and bhat_g_B (rad/s). The error state is delta_x = [delta_theta_B (rad), delta_b_g_B (rad/s)]. P is 6 by 6, in that order; it is not a four-component quaternion covariance. Bias means the residual after declared static calibration, avoiding double subtraction of the same offset.

| Owner | Responsibility | Prohibited implicit substitution |
|---|---|---|
| Basilisk spacecraft/environment | Propagate plant truth and generate synthetic observations; retain existing WMM/Earth model | No estimated state drives plant truth; no truth attitude/body field bypass into an operational estimator |
| Sensor managers | Convert units; apply documented calibration/mounts; report acquisition support, quality and clock provenance | No default identity flight mount, fabricated Sun vector, or publication-time acquisition |
| Reference provider | Magnetic B_N from declared orbit/position estimate and field model at the measurement epoch; inertial Sun direction from declared ephemeris/position inputs at that epoch | No truth attitude in reference construction; orbit/reference uncertainty is retained |
| Estimator | Gyro propagation, vector validation/correction, bias/error covariance, event history and quality | No controller law, actuator torque, point-target selection or plant integration |
| Control/mode supervisor | Consume a deliberately selected estimate epoch and quality; decide fallback/mode permission | No silent conversion of “has quaternion” into pointing readiness |
| Independent evaluator | Compare against plant truth in synthetic tests, or separately calibrated tracker/test truth later | Truth observations used for validation cannot also be an undisclosed correction input |

Future input contract (architecture only; concrete message definitions/packet configuration are TBD):

| Input | Contents / units | Time and validity contract |
|---|---|---|
| Gyro | Calibrated z_g_B in rad/s; source frame/mount/calibration revision; raw values retained outside estimate | Rate sample versus interval delta-angle and filter support must be known; native clock, mapped acquisition support and sequence ID; coverage/saturation flags |
| Magnetic observation | Calibrated v_m_B in T and normalized direction; raw sensor vector/source ID; clean-window and interference flags | Physical acquisition/filter support and quiet validity; use reference B_N at that epoch, not latest WMM message by convenience |
| Sun observation | Reconstructed body unit vector, or invalid status; participating CSS channels, geometry/calibration and uncertainty model | Per-channel support/clock agreement, reconstruction epoch, illumination/FOV/shadow/saturation flags; scalar intensity array is not a vector |
| Inertial references | B_N in T and unit Sun direction in N; reference position/orbit/ephemeris/model revision and uncertainty | Evaluate at each observation's actual acquisition epoch; record validity interval and reference lineage |
| Calibration/initialization policy | Sensor-to-body rotations, scales/offsets, P0, symbolic stochastic model, validity policy | Versioned, applicable configuration; missing flight values block performance use |

Future output bundle: qhat_BN, sigmahat_BN for compatibility, omega_hat_BN_B = z_g_B - bhat_g_B at its declared rate epoch, bhat_g_B, P, estimate state epoch, publication/availability epoch, last accepted observation epochs/ages, gyro coverage, initialized flag, per-source validity/rejection reason, local geometry/observability classification, covariance model/evidence status, revision/replay lineage and source/calibration IDs. Quaternion sign changes are representation changes, not motion.

The existing adapter consumes `NavAttMsg.sigma_BN`, `NavAttMsg.omega_BN_B` and `TAMSensorMsg.tam_S`; the current detumble path assumes S=B. A later facade may publish the required NavAtt fields with separate metadata for quality, bias, covariance and physical epochs. A header timestamp alone does not express this contract. Generic `tam_S` remains sensor-frame data until a documented transformation makes it a body vector; do not merely relabel it.

The existing ICD's requested angular acceleration is **not** a nominal state or automatically available from this filter. If required later, define a separate time-supported, uncertainty-aware derived product. Its “previous B” is sensor history, not an attitude state. FOUND may consume the estimate in a later interface; LOST results and Sagitta validation remain outside baseline correction ownership.

## Frame and attitude convention

| Symbol | Definition / direction | Evidence / status |
|---|---|---|
| N | Existing Earth-centered inertial ICRF/J2000 simulation frame | Current scenario contract, CONFIRMED as software convention |
| P | Existing low-order Earth-fixed/planet-fixed frame | Existing environment implementation, CONFIRMED as software convention; environmental fidelity unchanged |
| B | Mathematical spacecraft body frame; right-handed | F01: physical HS-2 axes/origin/mount registration remain TBD |
| S_g, S_m, S_css_i, S_tracker, S_LOST, S_FOUND | Individual sensor/optical frames | F02–F04: physical transformations/calibration remain TBD/TBC |
| C_AB | Maps components from B to A: v_A = C_AB v_B; inverse C_BA = C_AB^T | Architecture convention, ASSUMED adoption; SO(3) mathematics checked |
| sigma_BN / q_BN | Attitude of B relative to N, representing C_BN, **N to B** | Matches existing Basilisk convention |
| C_PN | Maps r_N into P for WMM | Existing environment contract; B_N = C_PN^T B_P |
| C_SmB | Body-to-magnetometer sensor rotation | Raw ideal measurement v_Sm = C_SmB C_BN B_N; recover body measurement using C_SmB^T |
| C_SgB / C_Scss_iB | Body-to-gyro / body-to-CSS-element frame | Proper mounting rotation; scale/nonorthogonality calibration is separate |

Do not infer physical spacecraft +X/+Y/+Z from a simulation axis or a vendor label. No new physical mount or Earth model is selected here.

Use [a]x b = a cross b. For scalar-first Hamilton quaternions,

```text
q (*) p = [q0 p0 - qv dot pv,
           q0 pv + p0 qv + qv cross pv]
C(q) = (q0^2 - qv dot qv) I + 2 qv qv^T - 2 q0 [qv]x
C(q (*) p) = C(p) C(q)                         (passive composition)
Exp_q(a) = [cos(|a|/2), sin(|a|/2) a/|a|]       (continuous limit at zero)
q_true = qhat (*) Exp_q(delta_theta_B)
C_true = exp(-[delta_theta_B]x) Chat
```

This Hamilton product with passive DCM reverses matrix composition order. A physical positive 90-degree body rotation about +Z maps inertial +X to body -Y. Copying an active-rotation quaternion rule with the same symbol names would reverse a sign/order.

Nominal propagation:

```text
z_g_B = C_SgB^T A_g (raw_g_Sg - offset_g_Sg(T))    [rad/s]
z_g_B = omega_true_BN_B + b_true_g_B + n_g_B
omega_hat = z_g_B - bhat_g_B
qhat_dot = (1/2) qhat (*) [0, omega_hat]
bhat_dot = 0                                     (candidate mean model)
Chat_dot = -[omega_hat]x Chat
qhat(t+h) = qhat(t) (*) Exp_q(omega_hat h)         (constant body-rate subinterval only)
```

A_g and the static offset are calibration inputs, not assumed identity/zero installed values. Variable-rate integration, sample support, coning correction and bias change require their own validated discrete treatment later.

Normalize qhat numerically after propagation/injection; reject nonfinite/zero norm. Normalization is not a measurement and does not justify reducing P. q and -q represent the same attitude; preserve internal sign continuity. For external principal MRPs, choose q0 >= 0 and sigma = qv/(1+q0); the shadow is -sigma/(sigma dot sigma) away from zero. The branch at 180 degrees changes coordinates, not attitude or rate.

## Symbolic error dynamics and covariance

With delta_b = b_true - bhat and candidate residual-bias random walk b_true_dot = n_b:

```text
delta_theta_dot = -[omega_hat]x delta_theta - delta_b - n_g
delta_b_dot     = n_b
F = [ -[omega_hat]x  -I ]       G = [ -I  0 ]
    [       0         0 ]           [  0  I ]
P_dot = F P + P F^T + G Qc G^T
Phi(t1,t0) solves dPhi/dt = F Phi, Phi(t0,t0)=I
Qd = integral(t0..t1) Phi(t1,s) G(s) Qc(s) G(s)^T Phi(t1,s)^T ds
P_minus = Phi P_plus Phi^T + Qd
```

Qc is a positive-semidefinite continuous-time spectral intensity under a declared white-noise convention. Block diagonal Qc = diag(S_g,S_b) is an **ASSUMED independence model**, not an installed fact. Cross-correlations must be retained when evidence requires them. Gyro spectral intensity S_g has units rad^2/s; residual-bias-driving S_b has units rad^2/s^3. P's attitude, cross and bias blocks have units rad^2, rad^2/s and rad^2/s^2. A quoted amplitude noise density, bias stability, range, resolution or sample rate is not automatically one of these matrices.

For zero nominal rate, constant independent spectral matrices and interval h:

```text
Phi = [ I  -h I ]     Qd = [ h S_g + h^3 S_b/3   -h^2 S_b/2 ]
      [ 0    I ]          [   -h^2 S_b/2            h S_b    ]
```

This special case independently anchors the bias coupling sign and covariance units. General discretization must preserve symmetry/PSD and the coupling terms. An unexplained Qc*h substitution or forward-Euler P step is insufficient. Symmetrization may remove roundoff; it cannot repair a bad model or justify silently clipping negative eigenvalues. Tests use synthetic SPD matrices solely to check algebra.

## Magnetic and Sun measurement models

Calibrate a magnetic sensor before rotation/normalization:

```text
v_m_B = C_SmB^T A_m (raw_m_Sm - offset_m_Sm(T, configuration))
v_m_N = B_model_N(r_est_N, t_acq)
```

A_m accounts for documented scale/nonorthogonality/soft-iron treatment; static/thermal offsets and actuator-related contamination need separate evidence. A time-varying coil artifact is not justified as white noise or a new estimated bias merely because a filter can accept it. T03's clean-window and filter-memory questions remain open.

The CSS manager must reconstruct a Sun direction from calibrated intensities, surveyed normals, response models and valid illuminated geometry. It must identify unresolved angular ambiguity/insufficient channels rather than output an invented vector. Current CSS count/model/reconstruction conflicts S05/S06 remain open. An accelerometer in orbital free fall is not a gravity-direction attitude reference.

For either valid vector, let y_B = v_B/|v_B| and u_N = v_ref_N/|v_ref_N|; uhat_B = Chat_BN u_N. With measured-minus-predicted residual:

```text
y_B - uhat_B = [uhat_B]x delta_theta + noise + higher-order terms
E^T E = I2, E^T uhat_B = 0                      (E is 3 by 2)
r = E^T (y_B - uhat_B)
H = [ E^T [uhat_B]x   0_(2x3) ]                (positive sign)
J_norm(v) = (I - u u^T)/|v|
R_tangent = E^T J_norm R_raw_B J_norm^T E       (sensor-only term)
```

R_raw_B includes the applicable calibration/rotation covariance transformation. Reference-vector errors, common orbit/model errors, alignment uncertainty, CSS reconstruction errors and correlations between channels/updates must also be treated explicitly. They are not all independent noise terms to add repeatedly. Systematic unknowns may require a separate sensitivity study/model discrepancy bound instead of a Gaussian R. Do not claim estimator confidence until these choices are evidenced.

Normalization leaves two independent measurement directions. A three-component normalized-vector residual has singular radial covariance; the tangent representation avoids pretending it supplies three independent constraints. Measure full angular discrepancy as atan2(|y cross uhat|, y dot uhat) before a local update. At y=-uhat the tangent residual is exactly zero although the error is 180 degrees. This is a mandatory acquisition/local-validity distinction; no numerical flight gate is supplied.

Symbolic local correction:

```text
S = H P_minus H^T + R
K = P_minus H^T S^(-1)                         (implementation should solve, not invert)
delta_x_hat = K r
Pe = (I-KH) P_minus (I-KH)^T + K R K^T          (Joseph form)
a = delta_theta_hat
qhat_plus = qhat_minus (*) Exp_q(a)
bhat_plus = bhat_minus + delta_b_hat
Jr(a) = I - (1-cos(|a|))/|a|^2 [a]x
          + (|a|-sin(|a|))/|a|^3 [a]x^2
Gamma = diag(Jr(a), I)
P_plus = Gamma Pe Gamma^T; local error mean reset to zero
```

The reset follows Log(Exp_q(a)^(-1) (*) Exp_q(a+e)) = Jr(a)e + higher-order terms. Its small-angle sign is **I - [a]x/2**. Bias remains expressed in the fixed B coordinate basis; an attitude-estimate injection does not rotate those axes. Cross-covariance must still undergo Gamma transport. Numerically guard small-angle series, finite values and positive-definite innovation solves. A singular innovation model calls for a justified constrained treatment, not hidden jitter.

## Timing, acquisition and delayed measurements

A05 is a future estimator contract; it does not alter the active simulation schedule.

1. Preserve sensor-clock timestamp and sequence, clock mapping/revision/uncertainty, physical acquisition interval (including integration/filter support), chosen effective epoch, receipt epoch and publication epoch. Define the run's mapping between monotonic nanoseconds and the environment calendar epoch. A midpoint approximation is an explicitly qualified model, not a measured latency correction.
2. Determine whether gyro packets contain point rate, averaged rate or delta angle, and which interval they support. Build propagation from valid gyro coverage in physical time. Missing data cannot be replaced with spacecraft truth or an indefinitely fresh held rate.
3. Process accepted events by acquisition epoch with deterministic same-epoch ordering. Evaluate each inertial reference at that observation's epoch. Log both accepted and rejected events, including duplicate IDs, invalid vectors, saturation, interference, clock uncertainty and insufficient coverage.
4. For a delayed event within retained history, restore the checkpoint immediately before its epoch, propagate to the epoch, update, then replay **all** subsequent gyro segments and accepted measurement events in order. Recompute their estimates/innovations under the revised history. Replay must neither omit later corrections nor apply an event twice.
5. Rejection/local-validity gates may change under revised estimates; define a deterministic replay admission policy and preserve original/revised dispositions. External validity facts remain recorded. History length, computation budget, lateness/freshness limits and gate values are TBD. Reject outside-history/future-clock events explicitly instead of treating them as current.
6. Publish state at a declared output epoch with lineage and observation ages. Replaying history does not retroactively change previously issued commands. A rate output must identify its gyro support and bias epoch; it cannot silently reuse the last gyro sample as a current rate across an uncovered interval.

A delayed-update example: acquire a magnetic vector at t0, receive it after a Sun update at t1 and gyro propagation to t2, with t0<t1<t2. Rewind before t0, insert the magnetic update using B_N(t0), replay the Sun update using Sun_N(t1), and propagate to t2 using the same gyro events. Reusing the t0 magnetic vector with C_BN(t2) is invalid even if its norm is unchanged.

For initial TRIAD acquisition, prefer two valid noncollinear pairs at a common attitude epoch. If an earlier observation must be transported, the candidate relation is y_B(t0;ti) = C_B(t0)B(ti) y_B(ti), paired with its **original** inertial reference u_N(ti). It is not generally paired with u_N(t0): the magnetic reference can change with position and Earth rotation. Relative gyro rotation, residual-bias uncertainty, reference changes, transport uncertainty and cross-correlations must be accounted for. Without that justified history, wait for a valid common-epoch pair. No nearest-time join or header renaming supplies the missing rotation.

The current continuous path has a common 0.1 s state/environment/sensor/controller epoch and commands apply over the following interval. The existing diagnostic magnetic cycle freezes nav and magnetic data at 0.4 s, computes at 0.5 s and actuates at 0.6 s within its 1 s test cycle (T04; software facts, not flight timing requirements). A later estimator integration must supply a **sample-epoch posterior rate/attitude snapshot paired with that frozen magnetic sample** if this controller contract is retained. Publishing a current propagated rate alongside the old B vector would change that contract. A separate current-state packet can serve other future consumers. No integration or timing change occurs in Phase 7B.

## Initialization, dropout and observability by mode

Initialization has two independent conditions: a justified attitude solution/prior and a declared residual-bias prior/P0. TRIAD cannot measure gyro bias instantaneously. Flight P0 and bias prior remain TBD; zero bias is permitted only as an explicitly synthetic prototype assumption. No arbitrary identity quaternion is marked initialized. A credible prior can start propagation, but its source and uncertainty must be retained.

| Mode / ID | Available information | Observable information and required response |
|---|---|---|
| Deployment detumble / M01 | Gyro and quiet-window magnetic vector; Sun availability conditional | Rate-based detumble need not require full absolute attitude. One vector does not initialize all attitude axes. Expose estimator status without changing existing detumble control. |
| Magnetic without Sun / M02 | Gyro + one magnetic direction | Instantaneously two attitude directions constrained; rotation about B is unobserved. Changing field/dynamics can improve temporal observability, not guaranteed. From a prior, propagate/correct locally; without a prior, full attitude remains uninitialized. |
| Sun only / M03 | Gyro + reconstructed valid Sun direction | Rotation about Sun remains unobserved instantaneously. Reconstruction may itself be ambiguous with insufficient CSS geometry. No full-attitude initialization from this vector alone. |
| Sun + magnetic / M04 | Two valid noncollinear pairs | Local attitude rank three; poor separation is ill-conditioned. Bias needs multiple epochs and suitable dynamics. Use deterministic acquisition or local corrections as appropriate. |
| Eclipse / M05 | No valid Sun vector; quiet magnetic and gyro may remain | Fall back to M02, or gyro-only during invalid magnetic intervals. Do not replace the Sun observation with zero or enable tracker aiding implicitly. |
| Gyro only / M06 | Valid gyro coverage and initialized prior | Attitude propagation only; residual bias cannot be identified from gyro alone. Report aiding age/uncertainty/model limitations; no bounded accuracy promise. |
| Lost attitude / M07 | No credible prior or local filter outside its valid region | Mark full attitude invalid; wait for valid two-vector acquisition or an explicitly approved independent prior. Reinitialize with explicit P0/bias disposition and record discontinuity; do not force a large local MEKF correction. |
| Experiment pointing / M08 | Same baseline sensor ownership; availability mode dependent | Estimator output alone does not demonstrate pointing accuracy/control authority. LOST remains experiment output, FOUND a downstream consumer, tracker independent validation. |
| Fault/degraded / M09 | Sensor dropout, saturation, clock error, stale history, contaminated mag or poor geometry | Exclude invalid observations with reasons. Missing gyro coverage blocks fresh propagation unless a separately validated fallback is selected. Supervisor thresholds and mode permissions remain TBD. |

For a single unit vector H has rank two with null attitude direction along the vector and zero direct bias sensitivity. At zero rate with an unchanging reference, two-epoch linear observability has rank four: attitude and bias along the vector remain unobservable. Two noncollinear directions give instantaneous attitude rank three and, over time with the gyro-bias coupling, rank six in the stated ideal constant-geometry case. Those algebraic examples do not establish observability on an actual orbit with eclipse, magnetic cycling, missing CSS coverage, uncertain timing or calibration errors.

Reacquisition must distinguish invalid measurements, poor geometry and an attitude prior outside the local basin. Use full angular checks before tangent updates. If a prior remains credible, resume aiding and monitor consistency; if lost, use a new valid acquisition, a justified bias prior/reset policy and explicit covariance reinitialization. Never silently collapse uncertainty or retain falsely precise unobservable components.

## Symbolic inputs that remain unresolved

All rows below are TBD for installed performance; no numerical Q, R, gates, random distributions or flight initial errors are selected.

| ID | Input | Units / required evidence |
|---|---|---|
| U01 | Gyro residual initial bias and P0, including attitude/bias cross terms | rad/s; P blocks rad^2, rad^2/s, rad^2/s^2; calibration/initialization method and temperature/configuration |
| U02 | Gyro white spectral intensity S_g and its convention | rad^2/s; installed stationary/rate-table data, bandwidth/filter/sample settings, PSD definition |
| U03 | Bias-driving S_b and validity of random-walk model | rad^2/s^3; duration/temperature-dependent characterization; other drift models if evidence demands |
| U04 | Magnetic calibration/covariance/reference discrepancy and clean-window validity | T, T^2, rad and s as applicable; installed powered/quiet tests, mounting, orbit/model error, filter memory |
| U05 | CSS suite, calibrated responses, reconstruction and direction covariance | channel-specific intensity units, rad^2 for tangent error; count/placement/normal survey and illumination tests |
| U06 | All sensor/optical mounts and uncertainty/correlation | dimensionless SO(3) transforms, rad/rad^2; physical B registration and survey revision |
| U07 | Clock mapping, acquisition/filter support, gyro packet semantics and replay horizon | s/ns, rad or rad/s as appropriate; readback, timestamp loopback and measured latency records |
| U08 | Reference orbit/Sun provider and uncertainty | m, s, T, rad as applicable; estimated-position source, ephemeris/model revision and applicability |
| U09 | Geometry/local-residual/freshness/consistency thresholds and supervisor permissions | rad, s or dimensionless statistics; requirements and validated error/availability models |
| U10 | Cross-channel, temporal and reference-error correlations | mixed covariance units; shared sensor electronics, common references, filtering and calibration inclusion |
| U11 | Tracker-independent truth calibration and pointing/error-budget requirements | rad, s, confidence convention and operating mode; independent calibrated test chain and resolved K01–K07 |

Vendor capability is not installed noise evidence. The design-budget own-estimator 10 arcsec and FOUND-input 5 arcsec refer to different unresolved chains (K01), not adopted R/P0/accuracy targets. Alignment/pointing conflicts in K02–K07 are retained in Phase 7A rather than arbitrarily combined here.

## Pointing-error-budget boundary

Separate (1) estimator attitude knowledge error, (2) sensor-to-body and payload-boresight knowledge error, (3) control tracking/acquisition error, (4) orbit/target/reference direction error, (5) clock/exposure/latency error, and (6) thermal/flexible alignment drift and image-motion effects. Keep the independent truth reference's own uncertainty in the validation comparison. Do not count alignment, timing or common reference errors twice merely because they appear in both an estimator covariance and a payload budget.

Map each small error through its applicable frame/boresight/time Jacobian. For a line-of-sight unit vector l, directional sensitivity is a cross-product projection, so roll about that line is not the same metric as cross-axis error. Timing sensitivity depends on motion and reference dynamics over actual acquisition/exposure support. Random terms may combine as J Sigma J^T only with an explicit covariance/correlation model; systematic biases, bounds and different confidence levels require separate treatment. No scalar RSS or total pointing number is justified by this phase. Estimator P is not achieved spacecraft pointing performance.

## Mathematical verification and its limits

The focused [test file](../basilisk_runner/test_attitude_estimator_architecture.py) imports NumPy and installed Basilisk rotation utilities only. It imports no scenario/controller, writes no simulation output and contains no reusable runtime estimator. All fixture angles, intervals, matrices and numerical tolerances are synthetic algebra choices. Basilisk utility comparison is a convention check; independent literal matrices, Rodrigues rotations, finite differences, rank calculations and positive-weight integration provide separate mathematical evidence.

| ID | Check | Independent anchor / failure exposed |
|---|---|---|
| V01 | Literal +Z rotation | Known +X to -Y vector; rejects DCM transpose |
| V02 | Noncommuting quaternion composition and gyro propagation | Rodrigues composition and finite-difference C_dot; catches product/order/sign error |
| V03 | Quaternion/MRP/sign/shadow/normalization | Same proper rotation through 180 degrees; zero quaternion rejected |
| V04 | Sensor/Earth/body chains | Nonidentity mounts and Earth rotation; equal norms cannot hide wrong direction |
| V05 | Vector H sign | Finite-difference true attitude perturbation; rejects negative measurement Jacobian |
| V06 | Normalization covariance | Numerical derivative, radial null direction and positive tangent covariance |
| V07 | F attitude/bias coupling | Perturb actual quaternion/rate propagation; checks negative bias term |
| V08 | TRIAD acquisition | Recover a known arbitrary rotation; exact collinearity rejected |
| V09 | Delayed vector transport | Original inertial reference retained; stale attitude and changed reference epoch fail despite equal norms |
| V10 | Observability | Single-vector rank two; constant one-vector temporal rank four versus two-vector rank six |
| V11 | Qd units/coupling/PSD | Closed zero-rate expression versus independent positive-weight quadrature |
| V12 | Error reset Jacobian | Finite-difference quaternion retraction; rejects transposed reset Jacobian |
| V13 | Joseph/reset covariance | Symmetry, PSD, independent Schur expression and nontrivial cross-covariance transport |
| V14 | Antipodal guard | Tangent residual zero at 180-degree actual discrepancy |

Run: `.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_attitude_estimator_architecture.py -v`.
Result: **14 tests passed**. These are mathematical checks, not a running MEKF convergence, replay implementation, CSS reconstruction, sensor realism, detumble regression or flight accuracy test. No active code was changed, so full scenario runs and the prior broad regression suite were not repeated.

Document checks also require valid JSON, unique/resolving IDs, source hashes, working local links, matching frame/time statements, unchanged pre-existing Phase 7A files and an allowlist diff review. Their execution result belongs in the completion report; this document does not imply a test ran merely by listing it.

## Phase 7C gate and ordered work

**Recommendation: proceed to a separately authorized noise-free estimator prototype.** Phase 7B removes the architecture-definition blocker only for a synthetic prototype. It does not remove Phase 7A's blockers to realistic installed-performance implementation.

1. Review A01–A08 and fix the versioned prototype interface/conventions from this document. Define an isolated synthetic profile; retain all current plant/controller configurations and a reproducible baseline.
2. Implement deterministic quaternion propagation, TRIAD acquisition and local attitude/bias MEKF kernels with the tests above as independent anchors. Synthetic nonzero initial errors/biases are test cases, not new HS-2 assumptions. Start with valid noncollinear vectors and exact gyro support.
3. Add event/history semantics and explicit sensor/reference/output epochs before any control connection. Test delayed/out-of-order/duplicate events, replay of multiple later corrections, missing gyro coverage and unchanged chronological results.
4. Verify noise-free attitude recovery, unit norms, bias convergence only in demonstrably observable cases, single-vector/gyro-only null modes, large-angle acquisition, antipodal/near-collinear rejection, eclipse/dropout/reacquisition and covariance numerical integrity. Test wrong mounts, units, signs and epochs intentionally. Define numerical tolerances from arithmetic/integration error, not an invented flight threshold.
5. Keep truth exclusively in synthetic observation generation and the independent evaluator. Do not initialize the estimator from truth unless the case explicitly tests propagation from a supplied perfect prior; acquisition tests must recover attitude from observations.
6. A noise-free measurement stream does not justify singular all-zero Q/R/P0 in an ordinary Kalman solve. Use declared synthetic positive-definite algebra weights for local-filter tests, or an explicitly constrained deterministic solver; neither supplies calibrated confidence. Keep numerical fixture values separate from the authoritative runtime configuration.
7. Only after isolated tests pass and later authorization is granted, design the Basilisk facade and controller-consumption compatibility checks. This phase authorizes no insertion into detumble or pointing.
8. Before realistic sensor performance or pointing claims, close U01–U11 using mounted characterization and resolved subsystem interfaces; identify an approved attitude-knowledge requirement; test covariance consistency with justified stochastic models, independent truth and repeated cases.

Remaining blockers include physical B/mount registration, CSS selection/reconstruction, raw magnetic producer/clean-window contract, actual packet clocks/filter support, bias/noise/correlation calibration, reference-provider uncertainty, truth independence acceptance, mode thresholds and unresolved pointing requirements. A functioning prototype would still be **NOT FLIGHT VALIDATED**.

## Phase 7C implementation addendum — 2026-09-07

Status: **NOISE-FREE DEVELOPMENT PROTOTYPE / NOT FLIGHT VALIDATED**. Source baseline:
`6d725b1` (committed Phase 7B architecture). The preceding sections remain the
Phase 7B design record; their future-implementation wording is superseded only
by the bounded prototype described here. The Phase 7B architecture JSON remains
an evidence artifact with `runtime_usable: false`.

The isolated [MEKF core](../basilisk_runner/attitude_mekf.py),
[shadow harness](../basilisk_runner/attitude_mekf_prototype.py),
[test configuration](../basilisk_runner/config/attitude_mekf_test_only.json) and
[regression tests](../basilisk_runner/test_attitude_mekf.py) implement A01–A07
for mathematical/software testing. No production scenario imports this estimator.
SimpleNav, controller behavior, physical profiles, Earth orientation and magnetic
cycle scheduling are unchanged. The harness consumes an in-memory run of the
existing cycled scenario and does not feed estimates back to its plant/controller.

Closeout review: 2026-09-13. The successful numerical verification below was
completed before the interruption and remains valid. This closeout corrects and
clarifies documentation; it does not reinterpret those results as flight evidence.

| EVIDENCE CLASS | WHAT IS ESTABLISHED |
|---|---|
| KNOWN / VERIFIED SOFTWARE RESULTS | Analytic/unit behavior, bounded replay, tested observability and unchanged production results reported below. |
| CANDIDATE / DEVELOPMENT ARCHITECTURE | MEKF is the HS-2 simulation estimator candidate; live opt-in consumer integration remains a separate gate. |
| ASSUMED / TEST-ONLY | Conditioning covariances, synthetic biases, ideal measurements, S=B in the prototype magnetic path, zero injected alignment error and numerical test thresholds. |
| BLOCKED / NOT YET CLAIMABLE | Installed sensor performance or mounting calibration, flight Q/R and gyro/magnetometer/CSS stochastic models, flight estimator or attitude-knowledge accuracy, and total pointing accuracy. |

### Implemented contract

- Nominal scalar-first unit q_BN, residual bias in B (rad/s), local error
  [delta_theta_B, delta_b_B] and 6 by 6 P. The Phase 7B Hamilton/passive convention,
  negative bias coupling, positive vector Jacobian, Joseph update and right-Jacobian
  covariance reset are implemented. No eigenvalue clipping or hidden jitter.
- Constant-rate quaternion propagation and exact constant-rate F transition;
  positive-weight quadrature for nonzero test Qc. The standard deterministic
  fixture has Qc=0. P0 and positive tangent weights exist for numerical conditioning
  and initial uncertainty in tests, **not installed sensor statistics**.
- Valid magnetic/Sun vectors can be normalized from arbitrary nonzero magnitude.
  Declared sensor-frame data require a proper supplied C_SB; already-body data are
  explicitly labeled B. Epoch, finite-value, dimension, zero-vector, mount,
  local-angle/antipodal and acquisition-pair consistency checks reject bad inputs.
  A coherent wrong rotation or a corruption within the local gate is not generally
  detectable without additional independent information.
- TRIAD requires two valid noncollinear pairs at the same physical epoch. It
  resets to the explicitly configured bias prior and P0 on acquisition/reacquisition.
  It rejects inconsistent inter-vector angles. Asynchronous **initial acquisition**
  by gyro transport is deferred; asynchronous local updates are supported.
- Gyro intervals have explicit, contiguous start/end coverage, interpreted as
  piecewise constant rates. Vector events retain acquisition/reference epoch,
  source and unique ID; receipt time is separate. Future/uncovered, outside-history,
  duplicate and capacity-exceeding events are explicitly rejected. Gyro gaps raise
  an error; the estimator does not substitute truth or propagate stale rate silently.
- Delayed events restore the checkpoint before the affected epoch and replay
  all later gyro segments **and vector/loss events**. Same-epoch order is loss,
  magnetic, Sun, then event ID. Numerical gates are reevaluated; revised dispositions
  and replay lineage are recorded. Counts represent the resulting history, not
  the number of replays. The TEST-ONLY history is bounded by 3 s and 512 events;
  the capacity bound may shorten the usable time window.
- Outputs include q/MRP, bias, P, initialization flag, instantaneous attitude rank,
  physical state epoch and last-update source/event/epoch. A rate is explicitly
  an interval-supported estimate, not a point sample. Following a correction or
  acquisition it is invalidated until new gyro coverage exists; a live NavAtt
  consumer adapter therefore remains required.
- Magnetic-cycle admission uses the actual acquisition row: SAMPLE event,
  sample/TAM/reference epochs, valid flag, native coil-off witnesses and quiet age.
  The measurement comes from `tam_sample_B_B_*`, checked against the frozen cycle
  sample. Current truth `B_B_*` is never used as a magnetic observation.
  Receipt during actuation can replay an earlier valid acquisition; an ACTUATE
  row containing held data cannot masquerade as an acquisition.

### Noise-free truth harness and numerical finding

All numerical assumptions are **ASSUMED / TEST-ONLY**, revision 2026-09-07,
sourced to the isolated test configuration. It specifies zero injected measurement
noise/alignment error, a zero-bias base case and separately imposed constant biases.
The inertial Sun direction is synthetic; CSS reconstruction, installed mounts and
ephemeris accuracy are not modeled. The existing WMM/cycle produces magnetic truth
and TAM samples. No hardware timing or sensor rate is inferred from these fixtures.

The initial endpoint-trapezoidal gyro treatment at the 0.1 s truth record interval
passed eight of nine harness checks but missed the three-axis bias bound. In the
zero-bias case it produced a false estimated bias norm of 2.40962e-4 rad/s. The
failed diagnostic is preserved in
[the coarse-gyro report](../basilisk_runner/output_data/attitude_mekf_coarse_gyro_diagnostic.json).
The final sensor synthesizer uses four-point cubic interpolation of recorded
Basilisk **angular rate**, sampled at 0.01 s subinterval midpoints. It does not
derive gyro from attitude differences. Offline lookahead belongs to synthetic
sensor generation, not a proposed real-time sensor packet. Production clocks stay
unchanged. Polynomial-rate tests independently verify the interpolation and
integration refinement. Covariance weights and pass bounds were not loosened.

The refined zero-bias residual bias norm is 3.19503e-7 rad/s, supporting the
discretization diagnosis. Finite interpolation, integration and filter-linearization
error remains. These residuals are algorithm-test results, not knowledge accuracy.

### Verified results

**97 regression tests pass**, including 24 new Phase 7C tests and all 73 existing
tests. Propagation covers zero, +X/+Y/+Z and arbitrary constant body rates against
analytic rotations (principal-angle discrepancy below 2e-15 rad in those fixtures).
Zero/one-axis/three-axis imposed-bias cases converge with two-vector aiding
(final bias norm error below 2e-6 rad/s in the controlled stationary tests).
Removing aiding leaves the imposed bias unestimated and causes the expected drift.

Single-vector tests preserve the ambiguous attitude and bias direction. A rotating
body test explicitly follows the unobservable inertial Sun rotation as its
components change in B; its covariance projection does not fall below its prior.
The filter never labels a single update as full instantaneous observability.
These tests do not prove calibrated covariance consistency for arbitrary nonlinear
trajectories or uncertain sensor models.

Interior-interval/asynchronous updates, same-epoch ordering, delayed updates with
intervening measurements, checkpoint pruning, duplicate/future/outside-history
rejection and explicit loss/reacquisition pass. Delayed versus chronological
q/bias/P agree within 2e-16 absolute numerical tolerance in the controlled replay
fixture. This is a numerical test tolerance, not a flight timing specification.

The five 60 s Basilisk truth cases pass all nine harness checks. Angles below are
principal-angle errors in **rad**, measured independently with Basilisk conversion
utilities and a sign-invariant quaternion metric; bias errors are norms in **rad/s**.
The report also retains an axis-resolved sin(angle)-times-axis vector; it approximates
a rotation vector only for small errors.

| CASE | INITIAL ANGLE | FINAL ANGLE | MAX ANGLE | FINAL BIAS ERROR | MAG / SUN UPDATES | INVALID REJECTIONS | ACQUISITIONS / REACQUISITIONS |
|---|---:|---:|---:|---:|---|---|---|
| 1: gyro + magnetic + Sun | 2.32634e-16 | 4.87204e-6 | 4.87204e-6 | 3.19503e-7 | 60 / 86 | 0 | 1 / 0 |
| 2: gyro + intermittent magnetic, no Sun | 0.269258 | 0.0135648 | 0.269258 | 2.92009e-4 | 20 / 0 | 0 | 0 / 0 |
| 3: gyro + Sun; actuation magnetic rejected | 0.269258 | 0.111541 | 0.269258 | 1.97463e-5 | 0 / 86 | 60 | 0 / 0 |
| 4: gyro-only gap, explicit loss/reacquisition | 2.32634e-16 | 4.06116e-6 | 4.06116e-6 | 3.51802e-7 | 54 / 77 | 0 | 2 / 1 |
| 5: imposed three-axis bias with both vectors | 0.269258 | 1.11113e-4 | 0.270366 | 1.42915e-5 | 60 / 86 | 0 | 0 / 0 |

Cases 1/4 start uninitialized and first acquire at 0.4 s; their initial reported
error is the first valid estimate. Cases 2/3/5 use a declared synthetic imperfect
prior. Case 4 suppresses vector updates during 8–14.4 s, explicitly marks attitude
lost at 14.4 s, then reacquires. Cases 1/5 each replay 59 delayed magnetic samples.
No complete-convergence assertion is made for single-vector cases 2/3.

All reported P matrices remain finite, symmetric and PSD; the smallest eigenvalue
observed across cases is 9.56483e-9. Eigenvalues mix attitude/bias units and are
numerical health diagnostics, **not attitude variances or an accuracy certificate**.
The final six eigenvalues, ascending, are:

| CASE | FINAL P EIGENVALUES |
|---|---|
| 1 | 9.79320e-9, 4.71132e-8, 9.16714e-8, 8.85504e-7, 2.71930e-6, 4.41537e-6 |
| 2 | 7.26339e-8, 3.32176e-7, 6.08423e-7, 5.90154e-6, 2.59121e-5, 2.46407e-3 |
| 3 | 1.55204e-8, 7.59660e-8, 1.59603e-7, 1.16322e-6, 6.28636e-6, 6.54248e-3 |
| 4 | 1.93493e-8, 5.79742e-8, 1.09880e-7, 1.13993e-6, 3.56936e-6, 7.25126e-6 |
| 5 | 9.56483e-9, 4.57297e-8, 9.03869e-8, 8.74677e-7, 2.68309e-6, 4.36866e-6 |

Full precision, counts, configuration fingerprints and health metrics are in the
[separate prototype report](../basilisk_runner/output_data/attitude_mekf_prototype_report.json).
These generated local reports are not tracked evidence releases.

### Production preservation and commands

The full continuous baseline (5,793 rows), candidate profile (5,793 rows) and
diagnostic cycle (57,924 rows) reproduced their existing CSV bytes exactly using
`write_outputs=False` and comparing serialized in-memory results by SHA-256.
Independent detumble checks passed 20/20 for each continuous profile; combined
detumble/cycle checks passed 44/44. The standalone-reference check was explicitly
excluded from these in-memory validations because no reference CSV was supplied.

| SAVED PRODUCTION CSV | UNCHANGED SHA-256 |
|---|---|
| detumble_output.csv | `c9e8aff00b93ac1a70829a6b38ffb106d2a2dd2763f770bf59776dd6ac62a960` |
| detumble_output_hs2_candidate.csv | `8fd59baf3c79bf9d01c5611b1d4030f8fbbfb6f1c7e272bc1cb750bbd4ffbeb1` |
| detumble_output_cycled.csv | `791eb062bfa0ec89456fe1132d7798c6a6875cffe8faabd7e9e8cc46e7d4f57e` |

Commands from the repository root (existing interpreter, no environment changes):

```powershell
.\.venv\Scripts\python.exe -m compileall basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_attitude_mekf.py -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_*.py' -q
.\.venv\Scripts\python.exe -B basilisk_runner/attitude_mekf_prototype.py --report basilisk_runner/output_data/attitude_mekf_prototype_report.json
git diff --check
```

The in-memory full-run verifier was invoked through `python.exe -B -` with
`basilisk_runner` on sys.path. It called `run(config=get_profile_config(profile),
cycle=cycle, write_outputs=False, make_plots=False)` for
(regression_baseline, None), (hs2_candidate, None), and
(regression_baseline, diagnostic_cycle_config()). It compared
SHA256(frame.to_csv(index=False).encode('utf-8')) to the corresponding saved CSV,
then ran `build_validation({}, summarize_basilisk(frame))` and, for the cycled case,
`cycle_checks(frame, config)`. No production CSV was rewritten.

### Readiness and next minimum step

1. The implemented mathematics/software is trustworthy **within the exercised
   ideal-data, local-error and bounded replay contracts**. It is a testable
   prototype, not a general fault-tolerant or statistically calibrated estimator.
2. It is ready for a separately authorized integration trial, but **not yet a
   drop-in SimpleNav replacement**. Add a live NavAtt/quality facade and validate
   actual gyro packet support, rate validity, sample-epoch controller consumption,
   acquisition startup and failure handling before connecting estimates to control.
3. It is **not ready to replace SimpleNav as the default production path**.
   Production continues to use SimpleNav; no prototype imports/configuration
   changes have been introduced into that path.
4. It is **not ready for realistic HS-2 performance prediction**. U01–U11 remain
   unresolved: physical body/mount registration, installed calibration/noise/bias,
   CSS selection/reconstruction, gyro/filter/clock support, magnetic clean-window
   behavior, correlations/reference errors, independent truth and accepted requirements.
5. The next minimum software step toward estimator-derived pointing analysis is
   that opt-in facade with independent epoch/rate/truth checks. Credible numerical
   knowledge or pointing accuracy additionally requires measured sensor/timing
   evidence, justified stochastic covariance models, observability/consistency
   studies and a resolved boresight/control/error-budget chain. No pointing
   controller or flight-performance claim is supplied here.
