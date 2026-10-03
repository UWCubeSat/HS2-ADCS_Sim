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

## Phase 7D live shadow integration — 2026-09-14

**SHADOW DEVELOPMENT INTEGRATION / NOT FLIGHT VALIDATED.** This addendum
supersedes the Phase 7C *integration readiness* statements above, not its
mathematical contract or unresolved engineering evidence. Starting commit:
`1b79a50`. Software findings below are CONFIRMED only for the tested development
configuration. Sensor bridges and numerical fixtures remain ASSUMED / TEST-ONLY.

### Actual message and task contract

The detumble scenario defaults to `--navigation simple-nav`. Explicit
`--navigation shadow-mekf` adds three Python SysModels with no connection to a
production consumer. The existing controller still reads SimpleNav, including
the frozen acquisition-epoch navigation message in cycled operation. Shadow mode
currently requires native actuation and excludes command replay. No pointing
path, controller law, actuator, configuration value or existing priority changed.

| DATA / MESSAGE | PRODUCER → CONSUMER | FRAME / UNITS | EPOCH AND ORDER |
|---|---|---|---|
| SCStatesMsg: sigma_BN, omega_BN_B, r_BN_N | Spacecraft → SimpleNav, WMM, TAM, ideal bridge, observer | sigma_BN dimensionless; omega_BN_B rad/s in B; position m in N | Spacecraft priority 1000 publishes state at current integer simulation ns after propagating the preceding interval. |
| Planet-state orientation and MagneticFieldMsg.magField_N | Existing Earth publisher/WMM → TAM, native actuator; bridge verifies field header | C_PN maps N to Earth-fixed P; WMM returns Tesla in N | Earth 925, epoch guard 910, WMM 900; current position, orientation and field epoch. Implementation unchanged. |
| NavAttMsg: sigma_BN, omega_BN_B, timeTag | SimpleNav → existing controller/cycle driver; shadow observer | N-to-B MRPs, rad/s in B, seconds | SimpleNav 800; current header and payload timeTag. Cycle driver retains the acquisition message for its later control event. |
| TAMSensorMsg.tam_S | Existing ideal TAM → controller/cycle snapshot and shadow bridge | Tesla in sensor S; C_SB maps B to S | Continuous TAM 700; in cycled mode driver 600 invokes TAM at SAMPLE and stores that acquisition. No active Sun measurement exists. |
| Cycle history/validity plus stored WMM reference | MagneticCycleDriver → ideal bridge | Actual TAM in S with explicit C_SB; independently stored reference in N, Tesla | Driver 600 completes before bridge. Bridge requires current history, SAMPLE event, valid flag, matching TAM/WMM/acquisition ns and existing zero-command/zero-torque quiet checks. |
| InputBatch, Python DevelopmentChannel | IdealLiveBridge 590 → MEKFNavigationAdapter 580 | Point gyro in B rad/s; covered gyro intervals; vector samples with frames, transforms, acquisition/reference epochs and validity | Actual SCStates header retained as gyro epoch; channel publication and sim ns checked separately. Delays retain acquisition ns. |
| NavAttMsg plus mandatory statusOut companion | Adapter 580 → ShadowNavigationObserver 570 only | Estimated sigma_BN, point omega_BN_B rad/s, timeTag seconds; status epochs in ns | Written at current ns only after initialization, contiguous gyro coverage and all updates/replays delivered this tick. Existing recorders follow. |

The full descending order is: preceding-interval recorders 1100, native-input
guard 1050, spacecraft 1000, native effector 980, applied-torque recorders 975,
sensor-state recorder 950, Earth 925, WMM guard 910, WMM 900, SimpleNav 800,
continuous TAM 700 when applicable, controller/cycle driver 600, **bridge 590,
MEKF 580, shadow observer 570**, then ordinary recorders. The existing replay
priority 550 and direct-torque latch 500 are outside this shadow mode.

In the existing ASSUMED diagnostic cycle (source: `diagnostic_cycle_config`,
Phase 6A), the period is 1 s, acquisition offset 0.4 s, control offset 0.5 s and
actuation starts at 0.6 s. The task step is 0.1 s. Control at 0.5 s consumes the
frozen 0.4 s SimpleNav/TAM snapshot. A new magnetic command applies over the
following integration interval; the native plant uses its existing WMM path.
The shadow filter acquires at 0.4 s and publishes current-epoch estimates on
subsequent ticks without affecting that control path.

N is the existing Earth-centered inertial ICRF/J2000 frame; B is the mathematical
body frame whose physical HS-2 registration remains unresolved; P is Earth-fixed.
`sigma_BN` describes B relative to N; `C_BN` maps N components to B. The native
TAM relation is `tam_S = C_SB C_BN B_N`; the core receives S plus C_SB and converts
with `C_SB.T`. Historical cycle CSV names containing `*_B_B` hold raw `tam_S`;
this adapter carries the explicit mounting transform instead of assuming those
labels establish a physical frame. Present production mounting remains unchanged.

### Output validity, acquisition and fault containment

`attitude_mekf_adapter.py` supplies a native NavAttMsg for **sigma_BN,
omega_BN_B and timeTag only**. `vehSunPntB` is not supplied or validated, so this
is not a full semantic replacement for every possible SimpleNav consumer.
Output rate is the current point gyro minus the posterior bias through the new,
nonmutating core `point_rate` accessor; it is not the last propagation interval's
average rate. No estimator propagation, correction, reset or replay equation was
changed. Native header time, payload timeTag, status epoch and estimator epoch
must all agree before any future consumer uses this output.

The mandatory `statusOut` is a deep-copy Python DevelopmentChannel, **not a
flight/SWIG message ABI**. It records validity, initialization/fault state, source
availability, per-delivery validity/provenance/disposition, received and accepted
sensor epochs/ages separately, last update, replay/rejection counts and
acquisition/reacquisition events. Rejected data cannot refresh accepted-data age.
Accepted-epoch metadata is conservative when older evidence leaves replay history.

- Startup needs a fresh common-current-epoch noncollinear magnetic/Sun pair.
  TRIAD acquires at 0.4 s in the dedicated two-vector case. A delayed startup pair
  is explicitly rejected; arbitrary-attitude gyro propagation is not attempted.
- Magnetic-only, Sun-only and gyro-only startup remain UNINITIALIZED and publish
  no NavAtt. Gyro-only propagation works from an explicitly supplied TEST-ONLY
  prior. Loss of one vector after acquisition retains propagation and the
  remaining vector updates; covariance follows the unchanged core. Validity is
  interface/propagation validity, not a flight attitude-knowledge threshold.
- Invalid, missing or stale TAM is rejected without replacing it with truth B.
  Only actual SAMPLE acquisitions enter the magnetic channel. Continuous mode
  has no quiet-window contract and therefore admits no magnetic updates.
- Delayed local vector updates retain acquisition/reference epochs and use the
  Phase 7C replay contract. The existing fixture bounds history to 3 s / 512
  events; the delivery queue is also bounded. These are TEST-ONLY capacity choices.
- Missing/stale/nonfinite gyro, invalid epoch or incomplete interval coverage
  latches FAULT. No fresh NavAtt is written; a previous message retains its old
  timestamp. Explicit reset plus a new valid pair is required for reacquisition.
  There is no automatic truth or SimpleNav substitution inside the estimator.

### Ideal bridge and telemetry limitations

The bridge uses actual current SCStates body rate as ideal gyro data. Ten
subinterval rates per 0.1 s task interval are reconstructed from the previous and
current rate endpoints by causal linear interpolation. Both endpoints are known
at delivery; no future sample or attitude difference is used. This differs from
Phase 7C's offline cubic sensor synthesis. It is an ASSUMED aperture approximation,
not an installed IMU packet/filter model or a new production integration rate.

`--shadow-ideal-sun` must be explicitly selected. It generates a synthetic N vector
from the unchanged Phase 7C fixture through current truth C_BN. The 0.7 s cadence,
0.4 s first sample, zero measurement noise/bias, P0/Q/R conditioning and synthetic
Sun direction come from the existing TEST-ONLY fixture/ShadowOptions; no CSS,
ephemeris, installed calibration or flight tuning is claimed. Dedicated delay
cases use 0.2 s delivery delays after 1 s; these are interface tests only.

Shadow files use a separate `_shadow_host` prefix. The host CSV retains production
columns; `_nav.csv` adds truth/SimpleNav/MEKF attitude, truth/estimated rates, bias,
P diagonal and eigenvalue bounds, independent truth-angle diagnostic, all source,
state and publication epochs, ages, validity and update/rejection diagnostics.
`_status.json` retains options and detailed per-tick quality. Host config/run/cycle
manifests are also separate. Full input/core traces are returned in DataFrame
attributes for the equivalence harness; they are not serialized in the CLI files.
Diagnostic histories grow with run duration even though estimator replay storage
is bounded. This is not a flight memory or message-delivery implementation.

### Verification and preservation

Source: Phase 7D local tests and runs, 2026-09-14; CONFIRMED development results.
All **114 regression tests pass**, including 17 new adapter/integration tests.
Compileall and diff whitespace checks pass. Optional static analysis was not run:
the previously established environment lacks `typing_extensions` for that tool;
no dependency was installed or changed.

| DEDICATED 8 s CASE | VERIFIED RESULT |
|---|---|
| Two vectors | Acquired at 0.4 s; 77 valid rows; magnetic/Sun update counts 8/11. |
| Magnetic only; Sun only; gyro only without prior | All three stay UNINITIALIZED; no navigation publication. |
| Gyro only with explicit prior | 81 valid rows from epoch zero; no vector updates. |
| Sun loss after 2 s | 77 valid rows; counts 8/3; eight missing-Sun rejections; covariance continues to evolve. |
| Delayed magnetic | Seven replays; acquisition times remain unchanged. |
| Delayed Sun | Ten replays; acquisition times remain unchanged. |

For every initialized case, final live versus independent chronological event
application has **exactly zero** quaternion-component, bias and covariance
difference; update counts and epochs agree. The numerical comparison tolerance
is TEST-ONLY 1e-12 absolute. This verifies adapter delivery/replay equivalence
using the same verified core, not independent estimator mathematics or sensor
accuracy. All eight host DataFrames exactly match the shadow-disabled host.

Full shadow-disabled production runs reproduce the Phase 7C saved CSV bytes and
the SHA256 values recorded above: continuous regression baseline 5,793 rows
(20/20 applicable checks); hs2_candidate 5,793 rows (20/20); diagnostic cycle
57,924 rows (44/44 including cycle checks). The standalone-reference finiteness
check is explicitly inapplicable to these in-memory runs. No saved production
CSV was overwritten. A separate 4 s CLI shadow run also verifies output writing.

Commands (repository root, existing project Python):

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_attitude_mekf_adapter.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_*.py' -q
.\.venv\Scripts\python.exe -B basilisk_runner/validate_mekf_shadow.py --report basilisk_runner/output_data/mekf_shadow_validation.json
.\.venv\Scripts\python.exe -B basilisk_runner/scenario_huskysat2_detumble.py --navigation shadow-mekf --shadow-ideal-sun --magnetic-cycle diagnostic --duration 4 --no-plots
git diff --check
```

The full production checks used an inline `python -B -` harness calling `run`
with `write_outputs=False, make_plots=False` for the default, hs2_candidate and
diagnostic-cycle configurations, then comparing UTF-8 `to_csv(index=False)`
bytes and applying the same validators described in the Phase 7C closeout above.

### Next gate

The smallest next software experiment is a separately authorized, opt-in
**navigation-consumer boundary test**: retain shadow actuation isolation while a
dummy controller consumer exercises frozen sample-epoch delivery, uninitialized
startup, stale publication, fault latching and explicit reacquisition. It must
require quality metadata and demonstrate that invalid/stale attitude and rate
cannot become a command. Native NavAtt fields make a later controller experiment
technically feasible, but production SimpleNav should remain the default.

Realistic attitude knowledge and pointing prediction remain blocked by U01–U11.
The next physical evidence step is a synchronized, mounted sensor/coil-cycle
characterization against independent attitude truth: establish frame registration,
gyro aperture/latency, magnetic clean-window validity, Sun reconstruction and
noise/bias/correlation before deriving Q/R and testing covariance consistency.
Released boresight alignment, control authority and an accepted knowledge/pointing
error budget are additionally required for defensible pointing accuracy.

## Phase 7E dummy navigation consumer — 2026-09-14

**DUMMY NAVIGATION CONSUMER / NO CONTROL AUTHORITY / NOT FLIGHT VALIDATED.**
Starting commit `dd4b97d` contains Phase 7D. This addendum closes the tested dummy
consumer boundary; it does not authorize or implement controller handover. Source:
the scoped consumer trace, new native-message tests and local validation below.
Findings are CONFIRMED only as software behavior; all new policies/fixtures are
ASSUMED / TEST-ONLY and are outside HS-2 runtime physical/sensor configuration.

### Existing production consumers and future connection surface

| CONSUMER | INPUT / INTERPRETATION | TIME, VALIDITY AND HANDOVER LIMIT |
|---|---|---|
| Continuous PythonBdotMTQController | Native NavAttMsg from SimpleNav; consumes omega_BN_B in B [rad/s], not attitude. Optional project C++ receives the same numeric rate/field arrays through the wrapper. | Uses execution time for controller_step; does not check NavAtt header/timeTag or estimator quality. Numeric finiteness and magnetic-field floor checks cannot distinguish invalid navigation from valid zero rate. Stale numeric nav is possible if its publisher stops. |
| MagneticCycleDriver and its internal detumble controller | Reads current SimpleNav NavAttMsg; stores complete acquisition payload, MRPs sigma_BN and omega_BN_B; internal controller reads frozen sample_nav. | Acquire at 0.4 s checks current message headers, then compute at 0.5 s checks frozen acquisition epoch. The 0.1 s age is intentional. Validity is magnetic-cycle validity, not estimator initialization/quality. A fresh invalid zero NavAtt could pass timestamp checks. |
| Historical PythonMRPDirectTorqueController in pointing | SimpleNav sigma_BN is treated as N-to-B attitude error relative to inertial identity; uses principal/shadow MRPs and omega_BN_B [rad/s]. | Uses execution time, assumes navigation validity, no estimator quality or stale-navigation gate. Read for this connection trace only; pointing code/behavior remain unchanged. |
| Navigation recorders and Phase 7D ShadowNavigationObserver | Native SimpleNav attitude/rate/time fields; observer also compares estimator/truth for diagnostics. | Recording is not a control consumer. The shadow observer checks publication epochs; it does not authorize any actuator. Minimal scenario has no SimpleNav consumer. |

The actual cycle driver's acquisition priority 600 is **before** the MEKF's 580.
Directly substituting the MEKF message at that existing acquisition call would
therefore read the preceding estimator publication. A future controller interface
must capture the matching estimate *after* publication, retain the corresponding
TAM epoch and quality, and consume that snapshot at the existing compute event.
The new frozen probe demonstrates this boundary without changing the driver.

### Input, freshness and representation contract

`attitude_navigation_consumer.py` receives only bound native NavAtt readers and
published quality companions. It receives no spacecraft, raw sensor, estimator
core, controller or actuator handle. SIMPLE_NAV is an explicitly permitted
comparison/source option; operational decisions never read spacecraft truth.
The module creates no native output message and cannot issue actuator commands.

Each snapshot retains source identity, original NavAtt header, timeTag, sigma_BN,
omega_BN_B, original quality-channel header and quality payload. MEKF quality is
the unchanged Phase 7D companion. The explicit SimpleNavQuality facade declares
ASSUMED ideal-simulation availability from a current written header/timeTag;
SimpleNav itself has no validity bit. This is not installed sensor health evidence.
Source identity is a fixed subscription binding, not inferred from numerical data.

Acceptance requires explicit initialized/valid quality, no fault, consistent
quality headers, state/publication/gyro epochs, finite representable navigation,
and matching payload timeTag. `state_epoch <= publication_epoch <= quality_epoch
<= consumer_epoch`; age is **consumer epoch minus state epoch**. Current-point
consumption uses TEST-ONLY maximum age 0 ns. The frozen probe permits the existing
100,000,000 ns acquisition-to-compute delay but additionally requires the **exact
specified acquisition epoch**. This inclusive age boundary passes at 100,000,000 ns
and fails at 100,000,001 ns. A current but wrong-sample estimate is rejected too.
No nearest-time matching or flight stale threshold was introduced.

Repeated publication without state advancement is separately visible through
state/publication-advanced flags and cannot refresh state age. Future epochs,
inconsistent headers/timeTag, source mismatch and nonfinite navigation fail the
contract. Faults do not turn an old valid attitude into a fresh sample.

The internal diagnostic representation contains principal MRPs, scalar-first
quaternion and C_BN (N components to B), with unchanged omega_BN_B [rad/s in B].
Equivalent MRP shadow sets are canonicalized; quaternion sign-equivalent attitudes
are compared geometrically. A passive +X rotation test independently checks the
N-to-B sign. Unsupported NavAtt vehSunPntB remains outside the contract.

### Deterministic state and selection rules

| STATE | ENTRY / ACCEPTANCE / EXIT |
|---|---|
| UNINITIALIZED | Quality says no acquisition; selected source NONE, no navigation accepted. Fresh valid acquired quality permits VALID. |
| VALID | All quality, epoch and representation checks pass. Only this state exposes internal navigation for hypothetical consumption. |
| STALE | Quality/state age exceeds the explicit test allowance or the required sample epoch differs. Selected NONE; fresh matching input may recover without an invented estimator reset. |
| DEGRADED | Explicit NONE selection, unavailable source/quality, or initialized but invalid quality. Selected NONE; no automatic fallback. |
| FAULTED | Estimator fault or invalid protocol/navigation; per-source latch prevents subsequent merely fresh packets from restoring acceptance. |
| REACQUIRING | A latched source reports uninitialized after reset. Accept only a fresh post-fault acquisition event with increased acquisition count. |

Source requests are explicit ordered `(epoch_ns, source)` commands: SIMPLE_NAV,
MEKF or NONE. Default diagnostic selection is SIMPLE_NAV; default production
selection also remains SimpleNav. Requesting disabled/uninitialized/stale/faulted
MEKF selects NONE, even if SimpleNav is valid. There is no automatic flight
failover policy. Fault latches survive logical source switches. Recovery checks
are evaluated at consumer execution events; no claim is made about disabling a
held physical command between those events.

Sun loss, invalid magnetic windows, rejected out-of-history samples and delayed
updates do not directly determine consumer state. The consumer follows published
estimator quality. An initialized estimator with valid gyro coverage can still
publish usable interface data during gyro-only propagation. VALID does not imply
absolute observability or satisfaction of an attitude-knowledge bound.

### Live scheduling, telemetry and handovers

Explicit Python API `run(..., shadow=ShadowOptions(...),
navigation_consumer=ConsumerOptions(...))` enables the experiment. Default None
performs no consumer import, construction or execution. No CLI default changes.
The standalone validator is the dedicated runnable entry point.

Existing priorities remain unchanged through controller/cycle 600, bridge 590,
MEKF 580 and shadow observer 570. New SimpleNav quality runs at 565, point dummy
consumer at 560 and frozen probe at 555. The frozen probe copies both published
navigation/quality snapshots after estimator publication at SAMPLE, preserving
their original headers, and consumes them at COMPUTE. It owns no controller.

Telemetry records requested/selected source, state, acceptance, quality validity,
initialization, state/publication/quality/consumer epochs, required acquisition
epoch, state age, received and canonical attitude/rate, fault/latch/reset evidence,
rejection reason and source transitions. Unavailable internal navigation is null,
not fabricated zero attitude/rate. Handover diagnostics compare the two published
sources **at the same state epoch**; unequal epochs are flagged and no false angle
jump is computed. These diagnostics do not influence selection. Analytic truth
comparisons exist only in validation code.

Source changes after an unavailable interval are recorded without inventing a
continuity measurement across the gap. This is a diagnostic switch, not a verified
closed-loop bumpless transfer. Report JSON retains all case telemetry, baseline
commit and source-file hashes. Host outputs, if explicitly requested, receive an
additional `_consumer` suffix; consumer histories are returned in DataFrame attrs
and saved by the validator, not added to production CSV schemas.

### Verification results

Source: local Phase 7E validation, 2026-09-14. **135 regression tests pass**, including
21 new boundary/integration tests. The initial no-truth dependency test incorrectly
introspected Basilisk's wrapped constructor; it now checks the first-party AST and
message APIs directly. No third-party code or dependencies were changed.

- Seven live 4 s cases pass: startup, default source, bidirectional handover, Sun
  loss, delayed magnetic delivery, gyro-only with explicit prior, and single-vector
  startup. All host DataFrames exactly match the consumer-disabled host.
- Live startup is unavailable at 0–0.3 s and VALID at 0.4 s. First frozen
  consumption at 0.5 s retains state/publication 0.4 s and age 0.1 s.
- Four isolated actual-adapter/native-message fault cases pass: nonfinite gyro,
  missing interval coverage, explicit gyro-validity fault and corrupted nonfinite
  NavAtt. Sequence: acquired 0.4 s; FAULTED at 0.8 s and latched through 1.1 s;
  explicit reset at 1.2 s; REACQUIRING through 1.3 s; fresh reacquisition/VALID at
  1.4 s. Synthetic common-epoch pairs are TEST-ONLY, not a flight acquisition cadence.
- Invalid magnetic-window and replay-history-limit rejection leave initialized
  propagation valid, with no fabricated vector updates. Freshness tests include
  missing/stale quality, refreshed publications with old states, future epochs,
  payload/header mismatch, exact age boundary and exact frozen-sample matching.

| DIAGNOSTIC HANDOVER | EPOCH [s] | ATTITUDE DIFFERENCE [rad] | RATE DIFFERENCE [rad/s] | STATE / PUBLICATION EPOCH JUMP [ns] |
|---|---:|---:|---:|---|
| Analytic spin SIMPLE_NAV → MEKF | 1.0 | 4.44089e-16 | 0 | 0 / 0 |
| Analytic spin MEKF → SIMPLE_NAV | 2.0 | 1.11022e-16 | 0 | 0 / 0 |
| Live detumble SIMPLE_NAV → MEKF | 1.0 | 1.36917e-5 | 0 | 0 / 0 |
| Live detumble MEKF → SIMPLE_NAV | 3.0 | 3.72038e-6 | 2.09412e-5 | 0 / 0 |

The analytic fixture uses a 0.4 rad initial +X angle and constant 0.02 rad/s spin,
with deliberately shadow-set SimpleNav MRPs. Maximum analytic DCM difference is
4.44089e-16. Numerical acceptance 1e-12 is ASSUMED / TEST-ONLY. Live differences
are measured development effects of ideal gyro reconstruction/estimation; the
filter was not tuned to remove them. No flight handover tolerance was selected.

Production preservation uses committed Phase 7D versus working-code **4 s runs**
for continuous baseline, hs2_candidate and diagnostic cycle: byte-identical in
all three. Existing full-run CSV hashes still match committed evidence above.
Full orbits were not rerun because changes are conditional diagnostic additions
and the user authorized this regression/hash strategy. No production output was
overwritten. Compileall and whitespace checks pass; optional static tooling was
not installed or run.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_*.py' -q
.\.venv\Scripts\python.exe -B basilisk_runner/validate_navigation_consumer.py --report basilisk_runner/output_data/navigation_consumer_validation.json
git diff --check
```

### Next minimum experiment and remaining blockers

Next: separately authorize a **disconnected command dry run** of the unchanged
detumble controller behind this quality/exact-sample gate. Feed captured MEKF nav
and matching TAM to a diagnostic controller instance whose outputs have no
effector subscription; verify startup, handover, fault/reset and inhibition of
stale/invalid commands. This is the smallest step before closed-loop actuation.

The consumer boundary now supports a future opt-in experiment, but direct
rewiring is unsafe: acquisition scheduling, command retention/inhibition during
faults, restart behavior and closed-loop sensitivity have not been verified.
SimpleNav remains production default. Native quality transport, calibrated
measurement/time/frame models, meaningful validity/uncertainty thresholds,
boresight/error budgets and the U01–U11 evidence remain unresolved. This phase
improves integration credibility; it does not establish attitude knowledge or
pointing accuracy.

## Phase 7F-1 coherent snapshot and scheduling contract — 2026-09-30

**CONTROL SNAPSHOT / SCHEDULING DEVELOPMENT CONTRACT / NO CONTROLLER AUTHORITY /
NO ACTUATOR AUTHORITY / NOT FLIGHT VALIDATED.** Checkpoint: Phase 7E is committed
at `d285cb4`; working tree was clean before this phase. This phase narrows the
next step to one claim: the state and magnetic acquisition belonging together
can be identified and frozen unambiguously. No new controller instance, command
computation, actuator connection or production scheduling change was made.

Source: the current detumble scenario, MagneticCycleDriver, Phase 7D adapter,
Phase 7E consumer, and new isolated timing/contract tests. Scheduling and test
findings are CONFIRMED software facts. Cycle durations and synthetic inputs remain
ASSUMED / TEST-ONLY; physical body-axis registration and flight timing remain open.

### Current causal order, unchanged

Let `t` be the current 0.1 s task tick, `s` the selected cycle's acquisition epoch,
`e` its compute epoch and `a` its actuation-start epoch. In the unchanged Phase 6A
diagnostic config, period=1 s, offsets s=0.4 s, e=0.5 s and a=0.6 s. These are
ASSUMED diagnostic timings from `diagnostic_cycle_config`, revision 2026-09-06.

| ORDER / MODULE | MESSAGE / DATA EPOCH / PUBLICATION | FRAME AND VALIDITY SEMANTICS |
|---|---|---|
| 1100 previous-interval recorders; 1050 native-input guard | Preserve/check held inputs before propagation. | Existing native command/field timing checks; no new estimator connection. |
| 1000 spacecraft dynamics, including attached native MtbEffector derivative evaluation | SCStatesMsg is published at t after the preceding interval is integrated. Native dynamics uses held command/field messages during that interval. | r_BN_N in N [m], sigma_BN N-to-B MRPs, omega_BN_B in B [rad/s]. Native magnetic torque uses its existing field/attitude contract at derivative evaluations. |
| 980 native effector; 975 applied-torque logs; 950 sensor-state log | Native UpdateState publishes the preceding interval's most recent dynamics torque; logs retain its semantics. | This scheduled publication is distinct from native torque computation inside dynamics. |
| 925 Earth orientation; 910 WMM guard; 900 WMM | Planet-state orientation and MagneticFieldMsg.magField_N at t, using state/orientation at t. | C_PN maps N to Earth-fixed P; field in N [T]. Epoch guards remain unchanged. |
| 800 SimpleNav | NavAttMsg header/timeTag/state at t. | sigma_BN, omega_BN_B [rad/s]; ideal development navigation, no native estimator-quality bit. |
| 700 TAM only in continuous mode | Continuous TAM publishes at t. In cycled mode TAM is not independently scheduled here. | TAMSensorMsg.tam_S [T] in S, C_SB maps B to S. |
| 600 cycle driver, SAMPLE event s | First checks zero native command/effective dipole/applied torque, quiet age, and current state/nav/WMM headers. Then explicitly calls TAM.UpdateState(s), freezes sample_nav and stores TAM and WMM at s. | Current SimpleNav satisfies this ordering. Cycle validity concerns acquisition/quiet-state evidence, not MEKF quality. No valid sample is inferred merely from finite TAM values. |
| 600 cycle driver, COMPUTE event e | Existing controller consumes frozen NavAtt/TAM at s. Computation time is e; acquisition payload/header remains s. | Uses body rate and sampled TAM (existing S=B assumption), current/dipole conversion and clipping unchanged. No attitude term is used by the existing detumble law. |
| 600 cycle driver, every tick / ACTUATE event a | MTBCmdMsg header is current t; it carries zero outside permitted burst and the held computed dipole during ACTUATE. | Command computed at e is first gated on at a and applies over the following interval. Native dynamics receives the production command, never a shadow command. |
| 590 ideal bridge | Current point gyro and interval support; actual stored TAM/reference and original validity/acquisition metadata; optional TEST-ONLY Sun delivery. | Gyro in B [rad/s]. Magnetic measurements retain acquisition epochs even when delivery is delayed. |
| 580 MEKF adapter | First propagates covered gyro intervals to t, then submits vector updates/replay, then publishes valid NavAtt at t. | Header=timeTag/state epoch=t for the current adapter. A delayed vector does not make the final navigation state historical; vector age and state epoch are distinct. |
| 570 shadow observer; 565 SimpleNav quality; 560 dummy consumer; 555 frozen navigation probe | Existing optional Phase 7D/E diagnostics operate after MEKF publication. Frozen probe retains s and checks it at e. | These observe published navigation/quality; no production controller uses their outputs. |

N is the existing inertial frame; B is the mathematical body frame; S is the TAM
sensor frame. Physical HS-2 alignment is not established here. Native dynamics
continues using its live environment input independently of sampled control data.

### Why direct rewiring is wrong

At SAMPLE, the priority-600 driver runs before bridge 590 and MEKF 580. At 1.4 s,
the most recent MEKF publication is therefore the state at 1.3 s. At the first
0.4 s acquisition, MEKF has not acquired yet at that point in the task. SimpleNav
has already published the current state at priority 800, so the active path has
no such lag.

The isolated Basilisk trace uses native messages and the actual MEKF adapter:

| EXECUTION | MEKF STATE / PUBLICATION | TAM ACQUISITION | RESULT |
|---|---|---|---|
| Priority-600 read at 0.4 s | Not initialized/published | 0.4 s | Reject. |
| Post-publication capture at 0.4 s | 0.4 / 0.4 s | 0.4 s | Accept for 0.5 s evaluation. |
| Priority-600 read at 1.4 s | 1.3 / 1.3 s | 1.4 s | Reject preceding estimate. |
| Post-publication capture at 1.4 s | 1.4 / 1.4 s | 1.4 s | Accept for 1.5 s evaluation. |

This refines the 7E warning: fully rewiring the driver's nav source **and** its
current reader would trigger the existing current-epoch guard, rather than
silently accepting 1.3 s navigation. Bypassing that guard or copying stale payload
under a new header would conceal the mismatch. Neither was done. The root cause
is causal task order plus sampling-before-publication, not incorrect MEKF epoch
labels. Moving only compute after MEKF publication does not repair a stale frozen
sample or justify pairing a new e-state with old s-TAM.

### Immutable snapshot and exact epoch relationship

New, unscheduled `control_input_snapshot.py` composes the existing Phase 7E quality
assessment/consumer decision with magnetic acquisition evidence. Its only result
is an immutable ControlSnapshot or an explicit rejection with no snapshot.

| CONTENT | DEFINITION |
|---|---|
| Navigation | Bound source; original sigma_BN and omega_BN_B; state/publication/quality epochs; valid, initialized, estimator/fault/consumer state; provenance. An accepted, unlatched Phase 7E decision must describe exactly those values and epochs. |
| Magnetic | Owned copy of actual sampled tam_S [T], explicit C_SB, transformed tam_B=C_SB.T tam_S [T], acquisition/publication epochs, original SAMPLE event and quiet/valid flags, provenance. |
| Reference | Optional acquisition-epoch WMM vector in N [T] and its epoch. Retained for audit/estimator association; the existing rate-based controller law does not require it. A current WMM vector cannot replace an older TAM sample. |
| Control window | Selected cycle period/sample/compute offsets with provenance; exact intended evaluation epoch e; derived sample s; actual capture c; coherence status/rejection. No default timing is selected by this module. |

For cycle index k, derive `s=k*period+sample_offset` and
`e=k*period+compute_offset`. Evaluation must be an actual compute event. Require:

```text
navigation state epoch = TAM acquisition epoch = s
optional reference epoch = s
s <= navigation publication <= quality publication <= capture c <= e
s <= TAM publication <= capture c
actual evaluation epoch = e
```

Payload timeTag must represent the state epoch; original message/quality headers
must agree. Integer ns comparisons are exact. No flight age allowance, nearest
sample join or time tolerance is introduced. The Phase 7E age check receives the
derived `c-s` allowance together with an exact required state epoch s; this cannot
admit an old coherent pair from a previous cycle.

Navigation published later can legitimately describe s **only if its payload,
quality, gyro support and state epoch actually describe s**, with publication by
the capture/compute deadline. A native-message fixture at publication/capture
0.5 s and state 0.4 s passes; a current 0.5 s state paired with 0.4 s TAM fails.
Changing only headers/quality cannot relabel current data. The current MEKF
adapter does not publish historical-state views; that fixture proves the data
contract, not a new history-retrieval capability.

In the replay test, a Sun measurement acquired at 0.9 s is delivered at 1.4 s.
The actual adapter replays and returns navigation at 1.4 s, which pairs with TAM
at 1.4 s. It must not be paired with a 0.9 s magnetic sample just because the last
received vector was old. Previously frozen snapshots stay immutable through later
updates. Inputs from a future epoch, wrong cycle, invalid quiet window, ACTUATE
acquisition, stale/mismatched pair, uninitialized/faulted/latched navigation,
conflicting source/decision or nonfinite vectors are rejected.

No truth state or truth field is stored. Input validity/quiet flags are supplied
producer evidence; the snapshot layer neither models coil decay nor independently
proves sensor cleanliness. The optional reference is identified model data, not
measured TAM. C_SB is explicitly supplied, not assumed from legacy `*_B_B` labels.
The only new numerical tolerance is a TEST-ONLY 1e-12 orthogonal-rotation check;
it is floating-point conditioning, not a mounting/alignment specification.

### Scheduling choices

| OPTION | CAUSALITY / EPOCH CORRECTNESS | CYCLE / PRODUCTION EFFECT | COMPLEXITY, LAG AND DELAYED-DATA IMPLICATION |
|---|---|---|---|
| A. Move future controller evaluation after MEKF | Causal reading alone does not make nav(e) and TAM(s) coherent. Evaluate/capture at s instead would change event semantics. | Moving the existing driver wholesale could also move acquisition and break bridge inputs. No production move authorized. | Low superficial code change, high risk of retaining or hiding a stale sample. Does not solve delayed state/reference association alone. |
| B. Freeze after publication at s, evaluate from snapshot at e | Causal, exact pair; captures current MEKF state and actual stored TAM with retained headers. | Keeps Phase 6A SAMPLE/COMPUTE/ACTUATE timing and existing production controller unchanged. Add a separate development capture after quality gate. | Small extension; avoids one-estimate lag. Incorporates updates/replay already completed before capture; later updates do not mutate the frozen snapshot. |
| C. Retrieve/reconstruct historical estimator state s | Can be correct if historical posterior, point rate/gyro support, provenance and quality are explicitly exposed by deadline e. | Could preserve cycle timing, but requires an estimator output/history interface not present today. | More complexity: replay revisions, bounds, historical bias/rate semantics and missing coverage. Never merely relabel the current state. |
| D. Extend Phase 7E frozen probe with stored TAM and this gate | Implements B using the already demonstrated after-publication capture location. | Future diagnostic-only extension; zero production changes in 7F-1. | Smallest DEVELOPMENT CANDIDATE. Retain original acquisition/field evidence, no command output yet. |

Select **B through D**: for a separately authorized 7F-2 path, capture after MEKF
580 and consumer 560 (the existing diagnostic 555 location is suitable), then
evaluate the unchanged disconnected controller from that snapshot at e. Do not
change the production driver or rewire its SimpleNav input.

Capture-time coherence is **not continuing command authorization**. The snapshot's
`evaluation_rejection()` checks only exact scheduled use. A future command gate
must also check latest fault/revocation state at evaluation, inhibit on reset or
loss, and reject old-cycle snapshots. This phase does not implement command hold,
inhibition or restart. Such policy must not be inferred from a frozen valid bit.

### Verification and next gate

**15/15 focused snapshot/timing tests pass.** They cover matching and mismatched
pairs, stale/future/invalid/nonfinite inputs, uninitialized/faulted/latched quality,
source conflicts, reference association, explicit S-to-B direction, immutable
ownership, legitimate later publication, actual replay, before/after-publication
order and exact later evaluation. The isolated 1.5 s Basilisk fixture has no
spacecraft, controller or effector; synthetic native TAM/reference packets carry
TEST-ONLY provenance. It does not revalidate WMM or physical sensor performance.

Compileall and whitespace checks pass. Existing runtime Python/configuration,
production subscriptions, task priorities and outputs are unchanged. The new
module is not imported by the active scenario/controller/adapter/cycle paths;
structural tests enforce that isolation. The committed 135-test baseline was
not rerun, nor were full orbits or command cases: no existing runtime path changed.
No dependency was installed or optional static tooling run.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_control_input_snapshot.py' -v
git diff --check
```

The snapshot/timing gate is PASS within the tested development contract. The
smallest justified **Phase 7F-2**, subject to separate user authorization, is an
unchanged disconnected controller calculation from accepted snapshots with
evaluation-time health checks, explicit inhibition, no command retention through
fault/reset, and independent command-math verification. This phase establishes
neither command correctness nor closed-loop, pointing or flight performance.

## Phase 7F-2A disconnected controller command mathematics - 2026-09-30

**DISCONNECTED CONTROLLER COMMAND MATHEMATICS / NO ACTUATOR AUTHORITY /
NOT CLOSED LOOP / NOT FLIGHT VALIDATED.** Evidence anchor: committed Phase 7F-1
`4d3cc0b`, initially clean working tree. This narrower phase implements only the
command-math part of the preceding next-gate proposal. Evaluation-time health,
fault inhibition, reset/restart and source handover remain unimplemented.

**Engineering gate: PASS for the controlled command-mathematics claim.** Given
an accepted coherent snapshot, the isolated calculation reuses the unchanged
production dispatcher and agrees with both the actual production SysModel and
independent scalar arithmetic in the cases below. This is CONFIRMED software/math
behavior for the stated configuration, not confirmation of HS-2 hardware or
permission to apply a command.

### Reused production contract

Source: `basilisk_adcs_adapter.py` and `hs2_sim_config.py` at `4d3cc0b`; inspected
2026-09-30. The existing `PythonBdotMTQController` reads body rate from NavAtt and
`tam_S` from TAMSensorMsg, assuming S=B, then calls `controller_step`. Despite its
class name, the law is rate-cross-field; it does not numerically differentiate a
measured magnetic field. For each body-aligned actuator axis i:

```text
m_requested_B = K (omega_BN_B x B_B)
I_bound_i = min(I_limit_i, m_limit_i / g_i)
I_i = clip(m_requested_i / g_i, -I_bound_i, +I_bound_i)
m_clipped_i = g_i I_i
tau_predicted_B = m_clipped_B x B_B
```

`omega_BN_B` is rad/s; B_B is T; K is A m^2 s/(rad T); g is A m^2/A;
current is A; dipole is A m^2; predicted torque is N m. Saturation flags use the
existing current-clipping difference threshold of 1e-15 A. Finite-input and
minimum-field checks remain those of the existing controller. Configured axes
are identity/body aligned; this does not implement allocation for other axes.

K=67200, dipole limits=(0.2, 0.2, 0.85) A m^2, rod gains=(2.3, 2.3) A m^2/A,
resistances=(51, 51, 4.4) ohm, rod voltage limits=(5, 5) V, aircoil power
limit=1.75 W, and minimum field=1e-12 T are unchanged **ASSUMED** legacy/provisional
inputs from `hs2_sim_config.py` (recovery provenance 2026-09-06). Rod current
limits follow V/R, Z current follows sqrt(P/R), and Z gain follows m_limit/I_limit.
These are not released hardware values. The full status/source-bearing config is
retained in every calculation record as canonical JSON with fingerprint
`99de374af2f512e27d34e40fdc3327356662ef48804f7ba7945b7a703a68ad8f`.

`disconnected_detumble_math.evaluate_snapshot` calls that public dispatcher once.
The optional `adcs_core` is unavailable, so this run exercises the
Python backend. The dispatcher remains unchanged, including its optional-core
behavior; no C++ parity claim is added. The record distinguishes optional-core
availability/enabling, not a guaranteed backend when optional dispatch falls back.

The production return value has no unclipped dipole field. Only that diagnostic
is reconstructed using K and the existing cross-product helper, as in existing
cycle telemetry. Clipped dipole, current, predicted torque, saturation flags and
computation validity come directly from the production return value. There is no
second clipping/controller implementation in the development module.

### Frozen inputs and output boundary

The input is the Phase 7F-1 accepted immutable `ControlSnapshot`. N is the existing
inertial frame, B the modeled spacecraft body frame, and S the TAM sensor frame.
The snapshot has already transformed `B_B = C_SB^T B_S`, where C_SB maps B to S.
The calculation uses this B-frame vector, never the raw S components. sigma_BN
defines C_BN mapping N to B and is retained with provenance; this rate-based law
does not consume attitude or the optional inertial reference field.

Sample/state epoch s, original publication/quality headers, capture epoch c and
intended compute epoch e remain distinct in the nested snapshot. The function
requires actual evaluation at e and passes e in seconds to the production math;
both vectors remain the frozen sample at s. Controlled fixtures use the unchanged
ASSUMED / TEST-ONLY diagnostic sample at 0.4 s and evaluation at 0.5 s. This
stateless law currently does not depend numerically on elapsed time. No task is
scheduled and no hold/application interval is implemented by this module.

The immutable Python record contains requested and clipped body dipoles, current,
predicted torque, saturation flags, computation-valid flag, evaluation epoch,
input snapshot and full configuration provenance. It can be serialized with
`dataclasses.asdict`; the focused tests evaluate independent math from that record.
There is no output message, write/subscribe method, SysModel, effector or production
import of this development module. Its actuator-authority field is false.
`command_valid` means only the existing numerical calculation accepted its inputs.
It is **not** a current-health assessment, command authorization or applied-torque
measurement. The exact-epoch precondition does not replace a future safety gate.

### Focused independent verification

`test_disconnected_detumble_math.py` passes **9/9 tests across 13 controlled
accepted snapshots**: zero rate, rate parallel to B, perpendicular rate/field,
arbitrary unsaturated vectors, mixed clipping, all-axis clipping, positive and
negative clipping separately on X/Y/Z, and a nonidentity S-to-B fixture. Inputs
and tolerances are **ASSUMED / TEST-ONLY**, sourced to these fixtures, 2026-09-30.
They do not run a live MEKF or establish estimation accuracy.

The reference is the actual unchanged `PythonBdotMTQController`, invoked in
isolation with equivalent synthetic native input messages. The already transformed
B field is supplied at its S=B math boundary. Only a test reader observes its
command header; no actuator subscribes. Inputs, current, flags and computation
validity match; clipped dipole and predicted torque are exactly equal. Input
headers are s=0.4 s; native reference publication is e=0.5 s. The development
record itself has no publication/actuation event.

Independent checking uses 50-digit Decimal scalar component equations from the
logged input/config record, derives V/R and sqrt(P/R) limits independently,
clamps in dipole space before recovering current, and computes m x B without a
production helper or NumPy cross product. A hand case (+X rate, +Y field) gives
+Z requested dipole 0.1344 A m^2 and -X torque 2.688e-6 N m. Sign reversal and
nonpositive predicted torque dot sampled rate are also checked. These sampled
math checks do not assert energy decay for a held command in a changing field.

| Comparison | Maximum absolute error | Units |
|---|---:|---|
| Production SysModel versus development clipped dipole | 0 | A m^2 |
| Production SysModel versus development predicted torque | 0 | N m |
| Independent requested dipole | 1.7763568394002505e-15 | A m^2 |
| Independent clipped dipole | 6.938893903907228e-17 | A m^2 |
| Independent current | 3.122502256758253e-17 | A |
| Independent predicted torque | 2.964615315390051e-21 | N m |

Independent tolerance is relative 1e-12 plus absolute 1e-14 A m^2 for dipole,
1e-14 A for current, or 1e-18 N m for torque; these are floating-point test tolerances,
not physical accuracy requirements. Zero/parallel cases are exactly zero.
Structural tests check immutable output, no command endpoint, and no active
imports in scenario, controller, cycle driver or MEKF adapter.

The initial test harness incorrectly requested a timestamp from an output message
object. It was corrected to use a test-only native message reader; the final
compileall and all nine focused tests pass. No production correction was needed.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_disconnected_detumble_math.py' -v
git diff --check
```

Existing runtime/configuration files, estimator, subscriptions, scheduling and
saved outputs remain unchanged. Existing regression suites and full-orbit runs
were not repeated because this phase changes no shared runtime path. No static
dependency was installed and no static-check result is claimed.

**Next candidate, Phase 7F-2B:** separately authorize evaluation-time
health/revocation checks, explicit inhibition and no retention across fault/reset,
with disconnected restart tests. Do not infer current validity from capture-time
coherence. Source handover, actuator connection, closed-loop estimation/control,
pointing and flight performance remain blocked by subsequent engineering gates.

## Phase 7F-2B evaluation-time command health gate - 2026-09-30

**EVALUATION-TIME COMMAND HEALTH GATE / NO ACTUATOR AUTHORITY /
NOT CLOSED LOOP / NOT FLIGHT VALIDATED.** Evidence anchor: committed Phase 7F-2A
`de04df4`, initially clean working tree. Sources are the unchanged Phase 7D/E
adapter/consumer interfaces and Phase 7F-1/2A records at that commit. No audit,
external-source lookup, controller redesign or shared runtime change was performed.

**Engineering gate: PASS within the observed development lifecycle.** The isolated
`command_health_gate.py` rejects a mathematically valid stored command when current
source health is lost. Health loss latches; clearing a valid/fault bit is not
recovery. Explicit reset, fresh acquisition, a newly coherent post-reacquisition
snapshot and a new numerical command are all required. These are CONFIRMED test
results for this implementation; the policy remains a DEVELOPMENT CANDIDATE.

### Three separate validities and five epoch meanings

| Evidence / epoch | Meaning and check |
|---|---|
| Capture-time validity | Phase 7F-1 accepted coherent MEKF/TAM snapshot. Remains historical evidence when source health later fails. |
| Mathematical command validity | Unchanged Phase 7F-2A `command_valid`; records numerical computation validity, not source health or authority. |
| Evaluation-time usability | New `command_usable`, established on every gate call using current source evidence and retained lifecycle state. A previous true decision is not reusable authorization. |
| Sample epoch s | Original matching nav state/TAM acquisition; unchanged in the command record. Test fixture 0.4 s. |
| Snapshot capture epoch c | Original freeze epoch, with s <= c <= compute. Test fixture 0.4 s. |
| Command computation epoch k | Existing `CommandMathematics.evaluation_epoch_ns`, explicitly identified as computation time here. Test fixture 0.5 s. |
| Command use/evaluation epoch u | New gate-call epoch. This phase keeps u=k=the selected compute event; computation and health evaluation are ordered operations at that same timestamp. It does not add a hold or later-use permission. |
| Current quality epoch q | Development quality-channel publication stamp, status_epoch_ns and sim_epoch_ns must agree with u. Current native NavAtt state/publication/timeTag and gyro support also describe u. Stored command inputs still describe s. |

All timestamps are exact integer simulation ns. No nearest-time join or flight
age threshold is introduced. The zero-age check reuses Phase 7E `assess` with
required current state epoch u. A fault can occur after computation as a later
operation at the same 0.5 s timestamp; that case inhibits use. A later timestamp
also cannot reuse the command: its original selected evaluation deadline is over.
This gate does **not** establish permission over the Phase 6A later ACTUATE burst.
Any eventual actuator-bound interface must define and recheck health at the actual
application boundary; no such interface is implemented in this phase.

### Gate inputs, outputs and lifecycle

Inputs are an accepted immutable `ControlSnapshot`, the unchanged disconnected
`CommandMathematics`, current Phase 7E `Snapshot`/consumer decision and current
evaluation epoch. `observe` consumes source publications between command events;
`notify_reset` receives the existing explicit `InputBatch.reset_acquisition`
request. Both must be supplied by an eventual development host. No reset of the
estimator, native output subscription or task attachment occurs inside this gate.

Output is an immutable `CommandUseDecision`: numerical command/provenance retained
by reference, usability, explicit inhibition reason, sample/capture/computation/use
epochs, separate capture/math flags and frozen current-health evidence. Health
evidence includes source state, quality epoch, lifecycle state, acquisition epoch
and count, and reset epoch. Original command numbers, configuration fingerprint,
source strings and snapshot epochs remain unchanged. Configuration JSON/fingerprint,
snapshot binding, finite metadata and provenance fields are checked without
recomputing control mathematics. These checks detect inconsistent supplied records;
they are not cryptographic authentication or proof against a dishonest producer.

| Event / condition | Deterministic outcome |
|---|---|
| Startup | Await a witnessed current healthy acquisition event with positive count. A valid bit alone or prior-only initialization does not bootstrap this development lifecycle. |
| Healthy current MEKF + accepted consumer + current acquisition provenance | Command usable only at its selected evaluation event with valid bound snapshot/command records. No SimpleNav alternative is selected. |
| Estimator fault, navigation latch, uninitialized source after readiness, revoked validity, nonfinite/inconsistent/stale source metadata | Command unusable; lifecycle FAULTED. Inhibition persists even if later input merely clears the fault bit. |
| Missing/mismatched/invalid snapshot or command provenance | Command unusable with an explicit record reason. It does not fabricate a source fault or mutate the numerical command. |
| Explicit reset notification | Revoke old acquisition eligibility and enter REACQUIRING; uninitialized status remains inhibited. A further fault requires a new explicit reset. |
| Fresh acquisition after reset | Require a current healthy acquisition event with count greater than the previously accepted count, and a cleared current navigation latch. Source can become healthy, but an old command remains unusable. |
| Fresh post-reacquisition snapshot and command | Sample must be strictly later than reacquisition and the latest revocation epoch; exact selected computation/use epoch and current health must also pass. Only then usability resumes. |
| Repeated evaluations / backward epochs | No stored last-command fallback. Each call assesses current health; backward observation faults, and late command use rejects. |

The strict post-reacquisition sample relation is deliberate: an old sample and a
reset/reacquisition can share an integer timestamp, and the existing snapshot has
no within-timestamp sequence token. Such a sample is rejected even if captured
later in that timestamp. This may defer eligibility to a later existing sample;
it changes no cycle offsets or controller behavior. Initial acquisition can use
a same-epoch sample because there has been no preceding command revocation.

Acquisition counts come from the existing adapter's in-run reset protocol and
remain monotonic across `reset_acquisition`. Full adapter/process `Reset` clears
those counters and has no continuity contract here; it must not silently reuse
this gate or old commands. A host must observe all health/reset events. This
isolated gate does not discover unreported faults or promise thread atomicity.

### Focused evidence and remaining gate

**16/16 focused tests pass**, covering healthy acceptance, fault after capture,
fault after computation, persistent inhibition, uninitialized/revoked/nonfinite
states, navigation latch, stale/future/mismatched/missing/malformed status,
reset/reacquisition, fresh-command restoration, no fallback, record immutability,
provenance mismatch, same-timestamp old-command rejection and structural isolation.
Synthetic fixture times and source inputs are **ASSUMED / TEST-ONLY**, sourced to
`test_command_health_gate.py`, 2026-09-30. No new numerical tolerance is used.
Phase 7F-1 frame checks and Phase 7F-2A sign/clipping/m x B tests were not duplicated.

One focused test uses the actual existing MEKF adapter and persistent Phase 7E
consumer, with synthetic gyro/two-vector messages and no plant or effector:

| Time (s), TEST-ONLY | Observed outcome |
|---|---|
| 0.3 | Initial acquisition observed. |
| 0.4 | Actual NavAtt/quality paired with synthetic TAM; coherent snapshot captured. |
| 0.5 | Existing math computes a valid command; a gyro-epoch fault occurs before gate evaluation; command unusable. |
| 0.6 | Explicit in-run reset; uninitialized/reacquiring, command still unusable. |
| 0.7 | Actual fresh acquisition count 2 clears consumer latch; old command remains unusable. |
| 1.4 / 1.5 | New coherent snapshot / new computation with healthy current source; usability restored. |

An additional test reacquires before the old command's original deadline and
rejects it **at that deadline**, proving rejection is not merely expiration.
The gate retains only lifecycle scalars, never a last command. Earlier decision
records stay immutable historical evidence; their old true bits are not authority
for subsequent use. There is no MtbEffector/ExtForceTorque connection, native
command message, write call or active import of this gate in production.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_command_health_gate.py' -v
git diff --check
```

Compilation, focused tests and whitespace checks pass. Existing tracked runtime,
configuration, subscriptions, priorities and saved simulation outputs are unchanged.
No full-orbit run or existing regression rerun was needed. No dependency installed.

**Next smallest experiment:** an opt-in, disconnected end-to-end observer in the
existing live task order, capturing after MEKF publication and exercising the
snapshot -> unchanged math -> current-health gate through fault/reset/reacquisition.
Demonstrate reset-event delivery and absence of missed status events in that host.
The isolated gate PASS alone does **not** justify immediately connecting a first
closed-loop MEKF A/B run. Scheduled integration and an explicit application-boundary
inhibition contract remain open. No source handover, pointing or flight claim.

## Phase 7F-2C live disconnected command-chain observer - 2026-09-30

**LIVE DISCONNECTED COMMAND-CHAIN OBSERVER / NO ACTUATOR AUTHORITY /
NOT CLOSED LOOP / NOT FLIGHT VALIDATED.** Evidence anchor: Phase 7F-2B `00d6909`;
the tree was clean and no edits remained from the interrupted inspection. This
phase reuses the completed component contracts rather than repeating their audits.

**Engineering gate: PASS for the tested live integration.** Actual Basilisk task
execution now exercises stored TAM -> post-publication MEKF snapshot -> unchanged
controller calculation -> current health gate -> diagnostic application decision.
The implemented observer has no command message output or actuator connection.

### Live schedule and epochs

Source: unchanged production priorities in `scenario_huskysat2_detumble.py` at
`00d6909`, plus the new explicitly selected stages below; verified 2026-09-30.
Priority runs highest first on the existing 0.1 s task. Times/clock are existing
ASSUMED development configuration, not flight timing requirements.

| Priority | Operation | Data/publication/capture meaning |
|---:|---|---|
| 1100 / 1050 | Existing held-input recorders / native input guard | Previous command/field for the integration interval ending now. |
| 1000 | Spacecraft | Propagates current truth at t using previous held inputs. |
| 980 / 975 | Native effector publication / torque recorders | Existing completed-step final-stage torque semantics unchanged. |
| 925 / 910 / 900 | Earth orientation / WMM input guard / WMM | Current state, orientation and inertial field at t. |
| 800 | SimpleNav | Current production navigation at t; still owns production control input. |
| 600 | Magnetic cycle driver | At s acquires TAM; at k computes production control; at ACTUATE ticks publishes production commands for the following interval. Unchanged. |
| 595 | New early-read witness | Records current TAM header and previous MEKF publication before the bridge/MEKF execute. |
| 590 | Existing ideal MEKF input bridge | Current gyro and valid acquisition TAM/reference evidence, original sample epoch retained. |
| 588, validation only | Test input relay | Injects an explicit shadow fault/reset into the actual adapter input and the gate's observed reset stream. No production input changed. |
| 587 | New acquisition tap | Stores actual TAM payload/header and the bridge's validated quiet-acquisition evidence/reference. No truth-field substitution. |
| 580 / 570 | Existing MEKF / shadow monitor | Publishes NavAtt and quality describing t; monitor keeps its existing publication guard. |
| 568, validation only | Late-quality fixture | Delivers prior quality after the monitor but before the consumer; retains old publication epoch. |
| 565 / 560 | Existing SimpleNav quality / point consumer | Existing Phase 7E tasks, fixed MEKF diagnostic selection for this option. |
| 556, validation only | Owned-view fixture | Corrupts only observer-owned sample/navigation views or replays a diagnostic old envelope. |
| 555 | Existing frozen probe | Original Phase 7E diagnostic sample/evaluation probe, unchanged. |
| 554 | New health observation and capture | Forwards reset once, observes current health every tick, captures matching MEKF/TAM at s after publication. |
| 552 | New disconnected calculation | At k, calls the existing dispatcher once from the frozen snapshot; records computation-time health decision. |
| 550 | New diagnostic application boundary | Refreshes current NavAtt/quality and checks health, source/reset generation and selected cycle at every ACTUATE transport tick. No publication. |

Production priorities are unchanged. Callback traces verify new-stage order in every live
case. At 1.4 s the early witness actually reads MEKF publication 1.3 s alongside
TAM 1.4 s; capture at 554 reads state/publication 1.4 s. A deliberately used early
view is rejected as stale. This is observed execution evidence, not just a list
of intended priorities.

Nominal diagnostic timing is s=c=0.4 s, k=0.5 s, with application checks
u=0.6, 0.7, 0.8 and 0.9 s. Original sample/publication/capture/computation stamps
remain separate from current health and application stamps. The next cycle clears
pending eligibility; it cannot inherit an old usable decision.

### Explicit later-application contract

The necessary small `CommandHealthGate.evaluate(..., application_cycle=cycle)`
extension reuses all existing health, provenance and restart checks. Without this
argument, the Phase 7F-2B compute-only behavior remains unchanged. With it, the
snapshot's period/sample/compute contract must match the explicitly supplied
existing cycle, and u must be in that sample's ACTUATE window. No computation
timestamp is relabeled and no controller equation is duplicated.

The observer invokes this gate at **every existing task tick in the burst**, not
only its start. A true decision means diagnostic permission for the following
plant interval [u,u+0.1 s), contingent on this fixed single-threaded schedule; it
does not grant permission for the rest of the burst. Current quality/NavAtt/gyro
epochs must all equal u. Each boundary checks cycle ID, command ID, and source
generation `(reset serial, acquisition count, revocation serial)` bound at capture.
The extended gate also rejects a wrong cycle and use beyond that cycle's window.
No flight age threshold or new timing offset is introduced.

At present the real production driver has already published its **SimpleNav**
command at priority 600. The observer's permission is diagnostic only and cannot
revoke or replace that production command. A future authorized connection must
publish through one explicitly selected owner after the gate, before the next
plant propagation, and apply zero on inhibition/quiet phases. This phase adds
neither that owner nor any actuator subscription.

### Fault, reset and provenance evidence

Nine live cases run for 3.9 s each: nominal, previous MEKF/current TAM,
stale previous-cycle TAM/current MEKF, invalid TAM window, fault before first
application, fault/reset/reacquisition with old-envelope replay, reset after
computation, late quality, and fault during an already-started burst. Fixtures are
**ASSUMED / TEST-ONLY**, sourced to `validate_disconnected_command_chain.py`,
2026-09-30. The existing ideal-Sun option is explicitly set to a 1 s cadence
coincident with quiet samples for short reproducible reacquisition; no default
sensor setting or MEKF mathematics changed. All comparisons use exact equality.

| Case / event | Verified outcome |
|---|---|
| Startup to first sample | Uninitialized at 0-0.3 s; acquisition/capture 0.4 s; calculation 0.5 s; first usable diagnostic decision 0.6 s. |
| Previous estimate + current TAM at 1.4 s | `navigation:stale_quality`; no new command for that cycle. |
| Previous-cycle TAM at 1.4 s | `tam_does_not_match_selected_sample_epoch`; no new command. |
| Invalid quiet-window evidence at 1.4 s | `invalid_magnetic_acquisition`; no new command. |
| Valid command 0.5 s, fault delivered 0.6 s | Actual adapter fault reaches consumer/gate before application; explicit inhibition at 0.6 s. Numerical command remains diagnostic evidence. |
| Late quality at 0.6 s | Old status epoch remains visible and fails current-health evaluation. The existing shadow monitor was not weakened. |
| Mid-burst fault at 0.8 s | Decisions at 0.6/0.7 s usable; 0.8/0.9 s inhibited. Earlier permission is not retained. |
| Fault 1.6 s, reset 1.8 s | Current command inhibited; reset delivered to adapter and gate, REACQUIRING. |
| Reacquisition 2.4 s, replay of pre-reset command 2.6 s | Source healthy but old `(0,1,0)` generation differs from current `(1,2,1)`; old command remains unusable. Its original sample 1.4 s and computation 1.5 s are not changed. |
| Fresh sample 3.4 s, computation 3.5 s | Current generation matches; usability resumes 3.6 s. The preserved strict post-reacquisition rule rejects same-epoch acquisition samples. |
| Reset immediately after first computation | Reset 0.6 s inhibits; reacquisition 1.4 s alone insufficient; fresh sample 2.4 s permits use 2.6 s. |

Full records include simulation/cycle/selected-sample IDs, raw stored TAM validity,
acquisition/publication epochs, sample MEKF publication, capture, computation,
application window, current state/quality epochs, lifecycle/reset/generation,
mathematical validity, usability/reason, requested/clipped dipole and predicted
torque. Immutable command records retain complete configuration/snapshot provenance.
Predicted torque is never written into applied-torque columns. Truth remains in
the pre-existing explicit ideal sensor bridge/validation telemetry, not substituted
by the observer for TAM or navigation.

### Validation, preservation and next gate

**183/183 regression tests pass**, including **8 new integration tests**. The
standalone validator passes all nine cases. Replaying received live inputs through
the component APIs gives exact agreement for 360 health observations, 36 snapshot
results, 27 controller calculations, 36 compute decisions and 144 application
decisions. These are component-equivalence checks; independent command arithmetic
remains the already-established Phase 7F-2A evidence, also passing in the full suite.

Three 4 s committed-versus-working runs (continuous regression baseline,
continuous candidate profile, and cycled baseline) produce byte-identical CSV
serialization with observer disabled. All nine enabled/injected observer cases
also match disabled-host production telemetry exactly. The actuator/sensor
subscription and dynamic-effector attachment ASTs match the committed scenario.
The observer owns only readers, component objects and Python records: no native
command output, effector handle, production controller handle or write call.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_disconnected_command_observer.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_*.py' -v
.\.venv\Scripts\python.exe -B basilisk_runner/validate_disconnected_command_chain.py --report basilisk_runner/output_data/disconnected_command_chain_validation.json
git diff --check
```

The dedicated JSON records base commit, source SHA-256 hashes, telemetry,
commands, execution order, injections and checks. No full-orbit run was needed;
existing production result files were not regenerated. The scenario option is
`--disconnected-commands`, requiring explicit shadow navigation and magnetic cycle.
It does not enable an ideal Sun implicitly. Requested scenario artifacts receive
a separate `_disconnected` suffix; the validator itself uses `write_outputs=False`.

**Next smallest justified experiment:** a separately authorized short closed-loop
MEKF-versus-SimpleNav detumble A/B under identical existing plant/cycle inputs,
with one explicit actuator-command owner and the verified boundary gate enforcing
quiet/invalid zero commands. This integration result justifies that controlled
experiment; it does not implement the connection or establish closed-loop stability,
sensor realism, pointing, requirement compliance or flight performance. Full-process
counter restart and faults outside the delivered status/reset contract remain
outside this in-run development evidence.

## Phase 7G-1 first nominal closed-loop integration - 2026-09-30

**FIRST MEKF CLOSED-LOOP DEVELOPMENT INTEGRATION / NOMINAL SHORT-RUN ONLY /
NOT FLIGHT VALIDATED.** Engineering gate: **PASS for the nominal tested case**.
The starting committed checkpoint is `29e4149`; the working tree was clean.
Earlier sections retain their phase-specific disconnected authorization/results.
This opt-in connection does not change the default `SIMPLE_NAV_REFERENCE` mode.

### Ownership and actual scheduling

`MEKF_DEVELOPMENT` uses one new `mekf_command_owner.MEKFCommandOwner`, the sole
publisher connected to the existing native MtbEffector's command reader. The
7F snapshot, controller mathematics, health gate and observer are reused without
algorithm changes. The observer itself still has no native command endpoint;
the new owner consumes its current diagnostic decision and pending calculation.
There is no ExtForceTorque connection, competing publisher, source handover,
truth substitution for sampled TAM, or gain/limit change.

| Priority / event | Contract |
|---|---|
| 1100 / 1050 | Record held inputs/state; guard their preceding-interval epochs. |
| 1000 / 980 / 975 | Propagate spacecraft; publish/record native final-RK-stage torque. |
| 950 / 925 / 910 / 900 / 800 | Record acquisition state; Earth orientation, WMM guard, WMM and SimpleNav at current epoch. |
| 600 | Existing cycle acquisition/quiet checks and TAM execution. In MEKF mode the SimpleNav computation and driver MTBCmd writes are disabled. Publish provisional acquisition metadata for the existing bridge. |
| 595 / 590 / 587 | Existing early witness, ideal input bridge and actual stored TAM acquisition tap. |
| 580 / 570 | MEKF publication and independent navigation monitor. |
| 565 / 560 / 555 | Existing source quality, current consumer and frozen consumer. |
| 554 / 552 / 550 | Existing coherent capture/current health, selected compute event, and application-time health/generation/cycle decision. |
| 548 | New single owner publishes zero or the usable immutable clipped dipole for [t,t+0.1 s). Verify native input equality; finish actual-command quiet/electrical bookkeeping. |
| 0 | Record messages, including a separate native MEKF NavAtt recorder. |

Before acquisition, during quiet/SAMPLE/COMPUTE, or on an unusable/missing current
decision, each owner invocation creates a fresh zero payload. It has no retained
usable-command cache. The cycle driver finalizes quiet age from this actual
publication before the next tick. A missed finalization is an explicit error.
The driver message remains unwritten in this mode, verified through a separate
reader; publisher/subscriber module IDs and epochs agree on every tick.

`cycle_sample_sigma_BN` and `cycle_sample_omega_B` represent the coherent MEKF
snapshot in this mode, frozen starting on the SAMPLE tick, not changed at compute.
The source is explicitly labeled. Independent `mekf_nav_*` native message records
verify that sample; generic `nav_*` remain labeled SimpleNav diagnostic witnesses.
Pre-acquisition MEKF recorder payloads are invalid zero sentinels with
`mekf_nav_valid=false`. Basilisk emits four unwritten-NavAtt recorder warnings at
0-0.3 s, as expected; no estimate or command authority is invented to suppress them.
Control expectation, sampled-controller torque, native applied torque and plant
truth stay distinct. Native torque at a coil-off boundary belongs to the previous
integration interval, so zero-torque checks use the held dipole.

### Nominal fixture and measured evidence

Source: `validate_mekf_closed_loop.py`, revision 2026-09-30; 6 s duration is
**ASSUMED / TEST-ONLY**. Six complete existing diagnostic cycles exercise repeated
capture, compute, gated use and quiet reacquisition without an orbital campaign.
The 10 s CLI/API ceiling is a development scope guard, not an engineering limit.
Both runs use the unchanged regression baseline (legacy assumed mass/inertia),
orbit, initial states, 0.1 s plant step, diagnostic cycle, controller, provisional
actuator configuration and ideal bridge. The existing explicit ideal Sun has
0.7 s cadence and 0.4 s first acquisition. Covariance fixtures and ideal gyro/TAM
remain test-only. No hardware selection or sensor performance is newly confirmed.

First command: TAM and post-publication MEKF capture at 400000000 ns; computation
at 500000000 ns; current health evaluation/publication/native input at 600000000 ns;
command ID 1, generation `(0,1,0)`, cycle 0. Dipole is
`[-0.2,-0.2,0.42702955144649996] A m^2` in the existing body-aligned configuration.
It applies during [0.6,0.7 s); independent nonzero native torque first appears at
the 0.7 s completed-interval record. Before that interval, A/B states are identical.

The following are **CONFIRMED numerical software observations for this fixture**,
not flight parameters or requirement compliance:

| Quantity | SimpleNav reference | MEKF development |
|---|---:|---:|
| Initial rate magnitude, rad/s | 0.8774964387392122 | 0.8774964387392122 |
| Final rate magnitude, rad/s | 0.8742458216779315 | 0.8742458126971391 |
| Actuation time / observed duration, s | 2.4 / 6 | 2.4 / 6 |
| Energized / zero held intervals | 24 / 36 | 24 / 36 |
| Maximum independent native-stage torque error, N m | 7.100e-21 | 5.082e-21 |
| Maximum rigid-body rate-step error, rad/s | 1.110e-16 | 0 at recorded precision |
| Maximum coupled state-step error | 1.110e-16 | 1.110e-16 |
| Maximum per-interval energy/work residual, J | 1.558e-14 | 1.558e-14 |
| Integrated magnetic work, J | -2.903374745438213e-5 | -2.903384999517699e-5 |

The independent predictor uses held recorded dipole, WMM field and prior plant
state to reconstruct RK stages, then compares native torque AND accepted plant
state. Work integrates stage torque dot rate, while rotational energy comes from
independently propagated states. Checks also reject altered native torque, altered
gate-to-command evidence and altered independent MEKF sample telemetry. Numerical
tolerances (ASSUMED test acceptance) are 1e-15 N m stage torque, 1e-12 rad/s rate
step, 1e-12 MRP-component error and 1e-10 J per-interval work residual. They are not
HS-2 requirement thresholds. Unchanged component replay yields 103 exact comparisons.

Maximum A/B differences: requested dipole 3.4690953874294284e-5 A m^2; published
clipped dipole 1.6043811018495724e-5 A m^2; native torque 3.110152792687033e-10 N m;
rate vector 1.8620311143422534e-8 rad/s; attitude 2.4724589342520592e-8 rad.
The law uses sampled body rate and TAM, not attitude directly. On the MEKF plant,
the rate-input difference versus SimpleNav equals negative estimated gyro bias
to 6.109371697254099e-17 rad/s. Its maximum is 2.3349284838398337e-5 rad/s; dispatching
the same controller on that plant with its SimpleNav sample rate isolates a
1.6050731675654184e-5 A m^2 command difference. Ideal-bridge reconstruction and
estimator numerical residuals are the modeled source of the bias correction;
subsequent feedback changes the trajectory. No extra physical bias was injected.
Near equality is expected for ideal observations and does not prove general
closed-loop stability or physical estimator accuracy.

### Verification, artifacts and next gate

Nine focused connection tests and 121 relevant pre-existing regressions pass.
Compileall and `git diff --check` pass. The short validator runs both A/B cases,
native/rigid-body/work validation, exact component replay and three preservation
pairs; it exits nonzero on a failed check. The committed comparison loads BOTH
scenario and cycle driver from HEAD, avoiding an old-scenario/new-driver mixture.
Continuous baseline, continuous candidate and cycled baseline six-second CSV
serializations are byte-identical with the feature disabled.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_mekf_closed_loop.py -v
.\.venv\Scripts\python.exe -B -c "import sys,unittest;sys.path.insert(0,'basilisk_runner');names=['test_detumble_telemetry','test_magnetic_environment','test_native_magnetic_actuation','test_magnetic_control_cycle','test_attitude_mekf_adapter','test_attitude_navigation_consumer','test_control_input_snapshot','test_disconnected_detumble_math','test_command_health_gate','test_disconnected_command_observer'];result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(names));sys.exit(not result.wasSuccessful())"
.\.venv\Scripts\python.exe -B basilisk_runner\validate_mekf_closed_loop.py --report basilisk_runner\output_data\phase7g1_nominal.json
git diff --check
```

The ignored report and its `_A_simple_nav.csv` / `_B_mekf.csv` companions contain
configuration/cycle/sensor provenance, source/CSV hashes, base commit, checks,
first-command and complete owner/chain trace. Production artifacts are preserved.
For a separately requested scenario artifact the opt-in CLI is
`--duration 6 --no-plots --magnetic-cycle diagnostic --navigation shadow-mekf --shadow-ideal-sun --control-source MEKF_DEVELOPMENT`;
it writes a distinct `_mekf_closed_loop` suffix and ownership manifest.

**Next:** a separately authorized short CLOSED-LOOP fault/inhibition experiment
checking actual zero native input and subsequent torque after a late fault, then
reset/fresh-generation restoration. Existing disconnected fault regressions still
pass, but this phase adds no closed-loop fault campaign. Performance campaigns,
requirements verification, hardware realism, pointing and flight claims remain
blocked by their previously documented evidence gaps.

## Phase 7G-2A closed-loop fault inhibition - 2026-10-01

**MEKF CLOSED-LOOP FAULT INHIBITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED.**
Engineering gate: **PASS for the single tested persistent fault**. Starting HEAD
`0569ad8` contains 7G-1; the initial working tree was clean. No controller equation,
gain, magnetic-cycle duration, MEKF mathematics, physical configuration, orbit,
environment or actuator limit changed.

### One existing fault, real prior actuator authority

Source: `validate_closed_loop_inhibition.py`, revision 2026-10-01. The harness
reuses `validate_disconnected_command_chain.LiveFixtures('mid_burst_fault')` and
the existing input-validity interface. Its single event sets `gyro_valid=False`
with reason `7F2C_TEST_ONLY_fault` at 800000000 ns, priority 588, after the normal
bridge at 590 and before MEKF at 580. The existing adapter fault, navigation latch
and health gate stay faulted thereafter. Subsequent input batches are otherwise
normal; clearing input validity alone does not clear the latched source fault.
No reset, source handover, reacquisition or new sensor-failure model is injected.

The only runtime extension is an explicit Python-only `mekf_fault_test` argument
carrying existing `ObserverOptions` with a test callback. It is rejected outside
MEKF_DEVELOPMENT. Normal CLI/API operation remains unchanged, and the 10 s
development ceiling remains. The new harness runs 3 s: prior real actuation plus
two complete post-fault sample/burst cycles. Fault timing/duration and ideal
sensor/covariance fixtures are **ASSUMED / TEST-ONLY**, not flight requirements.

The existing task relationship is preserved: acquisition 600; early witness 595;
bridge 590; test relay 588; TAM tap 587; MEKF 580; monitor 570; navigation consumers
565/560/555; capture/current health 554; calculation 552; application gate 550;
sole MEKF owner 548. The reused fixture's 568/556 callbacks do not inject quality,
reset or owned-view changes in this case. Native propagation/torque recording
still precede these stages. No gate, observer or command-owner algorithm changed.

### Timing, zeroing and continued rotation

The following are **CONFIRMED software observations for this test configuration**:

| Event | Simulation epoch / evidence |
|---|---|
| Coherent sample / calculation | 0.4 / 0.5 s; command ID 1, initial generation (0,1,0). |
| Native nonzero publications | 0.6 and 0.7 s; dipole [-0.2,-0.2,0.42702955144649996] A m^2 in the unchanged body-aligned fixture. |
| Real pre-fault application | [0.6,0.7 s) and [0.7,0.8 s); native torque is nonzero and the plant responds normally. |
| Fault / first health observation | 0.8 s at priority 588 / 554; current adapter quality is invalid. |
| First application inhibition | 0.8 s at 550, reason estimator_fault:7F2C_TEST_ONLY_fault. The historical numeric command remains valid/nonzero, but unusable. |
| Zero owner publication / native input | 0.8 s at 548; payload, native subscriber values and message epoch agree exactly. |
| First zero commanded-torque interval | [0.8,0.9 s); held input/epoch guard confirm the zero command used for integration. |
| First completed zero native-torque record | 0.9 s. The nonzero torque recorded at 0.8 s correctly belongs to the preceding interval. |
| Persistent fault | 23 fresh zero publications from 0.8 through 3.0 s, including all 10 remaining ACTUATE boundaries. Completed post-fault integration covers 2.2 s; the last publication at 3.0 s has no subsequent simulated interval. |
| Continued acquisition | TAM samples at 1.4 and 2.4 s remain valid; actual quiet ages 0.6 and 1.6 s. Three total valid acquisitions, zero rejected. |
| Source selection | Consumer requests MEKF throughout; selected source becomes NONE. SimpleNav remains published/current but never gains actuator ownership. |

Event-to-health detection, event-to-zero publication and event-to-zero-interval
start are all **0 simulation ns** for this aligned event and single-threaded task
ordering. The completed native-torque record follows **0.1 s** later. This is not
an asynchronous-fault latency bound, flight timing allocation or coil-current
decay measurement. The native algebraic actuator still lacks electrical decay.

The actuator receives a new zero at every inhibited boundary. The old computed
command is not erased to make the test pass: its original sample, computation,
generation and nonzero clipped dipole remain visible at first inhibition. Current
health revokes its use. At later cycles failed MEKF captures do not restore it.
Acquisition diagnostics remaining in the cycle driver are explicitly labeled
`NO_CURRENT_COHERENT_CONTROL_SNAPSHOT`; they are not SimpleNav fallback control.

Validation compares separate health decisions, owner publications, native input
readback/module identity, held messages, native final-RK-stage torque and accepted
spacecraft state. Independent held-input RK4 predicts the state/torque, and stage
torque-dot-rate work is compared with rotational energy. The state is identical
to a matching nominal MEKF run through the 0.8 s propagated state, before the new
command acts. It remains finite and follows torque-free dynamics afterward.
Continued rotation is expected: inhibition removes new commanded magnetic torque,
not angular momentum. Numerical tolerances are TEST-ONLY: 1e-15 N m torque,
1e-12 coupled-state component error and 1e-10 J interval work residual.

### Verification and next boundary

Nine focused tests exercise prior real nonzero actuation, delivery of the fault,
immediate native zeroing, fresh persistent zeros, cycle/quiet compatibility,
finite continuous plant response, no fallback, explicit fixture selection and
default preservation. Two tests alter saved evidence to ensure nonzero native
torque or a retained command cannot produce a passing report. Existing estimator
and health policy tests are reused rather than reimplemented.

All nine focused tests and 52 relevant regression tests pass; the standalone live
validator and all three byte-preservation cases pass. Maximum native torque error
is 9.470116246213047e-22 N m; coupled-state step error is 1.1102230246251565e-16;
interval energy/work residual is 5.195858281860778e-15 J. Final rate remains
0.877273881915547 rad/s, consistent with continuing rotation after torque removal.
These are fixture observations, not flight-performance results. Compilation and
diff/allowlist checks pass. Expected unwritten-NavAtt warnings before the first
0.4 s acquisition retain the Phase 7G-1 invalid-startup semantics.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_closed_loop_inhibition.py -v
.\.venv\Scripts\python.exe -B -c "import sys,unittest;sys.path.insert(0,'basilisk_runner');names=['test_mekf_closed_loop','test_disconnected_command_observer','test_magnetic_control_cycle','test_attitude_mekf_adapter','test_native_magnetic_actuation'];result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(names));sys.exit(not result.wasSuccessful())"
.\.venv\Scripts\python.exe -B basilisk_runner\validate_closed_loop_inhibition.py --report basilisk_runner\output_data\phase7g2a_inhibition.json
git diff --check
```

The dedicated ignored JSON/CSV artifacts retain the base commit, source/CSV
hashes, configuration/cycle/sensor provenance, fault mechanism/event, independent
native/plant records, current quality/consumer/health records, latency and every
owner decision. Preservation reloads the committed runtime modules and compares
three short default cases byte-for-byte; no production output is replaced.

**Next smallest justified experiment:** separately authorized CLOSED-LOOP RESET /
REACQUISITION, proving that explicit restart and a completely fresh coherent
post-reacquisition command are necessary before native nonzero actuation resumes.
This phase does not perform that experiment. General fault coverage, arbitrary
event arrival times, process restart, hardware safeing/latency, realistic sensors,
requirements-grade detumble, attitude knowledge and pointing remain unverified.

## Phase 7G-2B closed-loop reset / reacquisition - 2026-10-01

**MEKF CLOSED-LOOP RESET / REACQUISITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED.**
Engineering gate: **PASS for this one controlled in-run lifecycle**. Starting HEAD
`a1d29d1` contains 7G-2A; the working tree was clean. No shared runtime or scenario
wiring was changed. The new harness uses existing ObserverOptions/InputBatch hooks
and unchanged snapshot, controller, health gate, command owner and native actuator.

### Existing generation semantics, inspected before implementation

The source/command generation is the observer tuple `(reset serial, acquisition
count, revocation serial)`, bound to the snapshot at capture and copied into its
immutable PendingCalculation. The middle element is the adapter's monotonically
increasing acquisition count. There is no new estimator reset-ID field: the
observer increments its reset serial on the existing delivered
`InputBatch.reset_acquisition` event, and increments its revocation serial on a
healthy-to-unhealthy transition. The adapter clears its engine/fault on this event
while retaining the acquisition count. The test does not call SysModel.Reset or
restart a process/counter, and adds no reset semantics.

| Event | Source generation | Command provenance / authority |
|---|---|---|
| Initial capture 0.4 s / calculation 0.5 s | (0,1,0) | Command ID 1 drives native actuation at 0.6/0.7 s. |
| Fault 0.8 s | (0,1,1) | Old generation revoked; owner/native input zero. |
| Explicit reset 1.0 s | (1,1,1) | Prior provenance obsolete; gate REACQUIRING, source UNINITIALIZED; zero persists. |
| Reacquisition 7.4 s | (1,2,1) | Source healthy, but acquisition alone grants no command authority. |
| Same-acquisition candidate 7.4 / 7.5 s | (1,2,1) | Command ID 2 is numerically valid but rejected at 7.6 s: fresh_post_reacquisition_snapshot_required. |
| Replay old envelope 7.7 s | Current (1,2,1), old command (0,1,0) | ID 1 retains sample 0.4 s, computation 0.5 s and original application window. Rejected; native input stays zero. |
| Fresh capture 8.4 s / calculation 8.5 s | (1,2,1) | ID 3 is eligible and reaches native actuator at 8.6 s. |

The key invariant remains: an old pre-reset command cannot become usable after
reset. Reacquisition restores source health, not authority for historical inputs.
The existing gate additionally requires a sample strictly after reacquisition;
a coherent same-epoch acquisition sample is insufficient. These policies were
already present in 7F-2B/2C and were not changed to achieve a passing result.

### One live sequence and explicit replay rejection

Source: `validate_closed_loop_reacquisition.py`, revision 2026-10-01. All event
times and the 9 s horizon are **ASSUMED / TEST-ONLY**. The input relay at priority
588 delivers the already-supported `gyro_valid=False` fault at 0.8 s and
`reset_acquisition=True` at 1.0 s. The owned-view fixture at 556 replays the saved
immutable envelope at 7.7 s. It does not forge quality, modify numeric command
values, relabel timestamps, renew deadlines or touch actuator messages.

The current ideal-Sun cadence is 0.7 s, first vector 0.4 s; TAM acquisition is
once per existing 1 s cycle at offset 0.4 s. The next coincident post-reset pair is
7.4 s. The initial-acquisition contract requires a current common-epoch pair, so
absolute navigation remains invalid during the intervening gyro/vector activity.
This waiting time is a fixture scheduling consequence, not demonstrated hardware
reacquisition performance. The sensor cadence and magnetic cycle were preserved.

At 7.6 s the source is healthy and the new same-acquisition candidate has matching
generation/cycle but remains inhibited by the strict fresh-sample rule. At 7.7 s
the old envelope is separately rejected with `snapshot_predates_current_acquisition`.
The captured inputs and numeric calculation remain valid finite historical data,
and the current source is healthy. The rejection therefore occurs on provenance
before the gate reaches application-window checks; it is not merely a failed old
deadline. The observer also records a generation mismatch. Old numerical
diagnostics remain visible without replacing independently recorded native input
or applied torque.

The next TAM/MEKF sample at 8.4 s is strictly later than both reset and reacquisition.
Its body rate, attitude, TAM transform and publication epochs match independent
native NavAtt/TAM records. Computation at 8.5 s creates ID 3 in generation (1,2,1).
The current health/application gate accepts it at 8.6 s; the single unchanged owner
publishes `[-0.2,0.2,0.8220612696753196] A m^2`, exactly matching native readback.
The first completed resumed-torque record at 8.7 s is
`[1.0465049729882324e-6,1.8364838631639162e-5,-4.213392431318644e-6] N m`.
These are **CONFIRMED numerical observations for the assumed development fixture**,
not released flight actuator values or a required resumed command.

### Independent evidence and verification boundary

There are 78 consecutive fresh zero publications over [0.8,8.6 s), including all
64 boundaries with gate lifecycle REACQUIRING. Native torque is exactly zero on
the completed corresponding intervals; the 8.6 s torque record still describes
the previous zero interval. Quiet/sample phases remain zero before and after
recovery, and TAM quiet-window checks remain valid. SimpleNav remains available
but never acquires control; publisher identity stays singular and unchanged.

Independent held-input RK4 reconstruction verifies every accepted spacecraft step
through fault, reset, reacquisition and resumption. Maximum native-stage torque
error is 3.4144989710325696e-21 N m, coupled-state component error
1.1102230246251565e-16, and interval energy/work residual 7.401933358369738e-15 J.
Test-only limits remain 1e-15 N m torque, 1e-12 coupled-state component error and
1e-10 J interval work residual. The plant remains finite and rotates naturally
while command torque is zero; neither reset nor replay rewrites plant state.

Eight focused tests and the standalone lifecycle validator pass. Negative
evidence checks reject premature command resumption and altered native torque.
The three established short continuous-baseline, candidate and cycled-baseline
preservation cases remain byte-identical to HEAD. Shared runtime/wiring did not
change, so the previously passed broad regression suites were not repeated.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_closed_loop_reacquisition.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_closed_loop_reacquisition.py --report basilisk_runner\output_data\phase7g2b_reacquisition.json
git diff --check
```

The dedicated ignored JSON/CSV artifacts carry the correct reset/reacquisition
scope, event/generation/envelope records, source/configuration/CSV hashes, current
health/quality, owner/native/plant evidence and preservation results. The harness
uses `write_outputs=False`; the existing scenario's fault-inhibition manifest is
not emitted or relabeled, and prior production artifacts are not replaced.
Compilation and diff/allowlist checks pass. Expected invalid startup NavAtt
recorder warnings retain their existing 0-0.3 s meaning.

**Next smallest experiment:** one late-fault test at the health-gate-to-owner
publication boundary, establishing which interval may still use an earlier
decision and verifying deterministic inhibition at the first eligible boundary.
This phase does not establish arbitrary arrival-time safety, process restart or
counter-reset semantics, broad fault robustness, hardware latency, realistic
sensor/estimator performance, requirements-grade detumble or pointing. A longer
performance campaign is not the next step.

## Phase 7G-2C late-fault command-validity boundary - 2026-10-01

**LATE-FAULT COMMAND-VALIDITY BOUNDARY / DEVELOPMENT TIMING TEST /
NOT FLIGHT VALIDATED.** Engineering gate: **PASS for one deterministic source
quality fault between health evaluation and command publication**. Starting
checkpoint `c4bd9a7` contains 7G-2B; the working tree was clean. The new validator
and tests reuse the existing ObserverOptions hook. No shared runtime, estimator,
controller, owner, actuator, magnetic cycle, physical or environment code changed.

### Actual scheduler and decision-to-publication window

The validator records the actual Basilisk task model table after setup and
checks its ordering. All listed models run in DynamicsTask, period 0.1 s.
Higher priorities run first. Priorities are CONFIRMED software configuration at
this checkpoint, not flight scheduling allocations.

| Priority | Role and epoch contract |
|---|---|
| 1100 / 1050 | Record held inputs; inspect native command/field subscribers from t - 0.1 s. |
| 1000 | Spacecraft propagates the interval ending at t. Native MtbEffector reads held command and field during RK dynamics calls. |
| 980 / 975 | MtbEffector publishes final RK-stage torque for that completed interval; native/logger records follow. UpdateState does not latch a new command. |
| 925 / 910 / 900 | Existing Earth orientation, WMM guard and current WMM field. |
| 800 / 600 | SimpleNav diagnostic; unchanged magnetic-cycle acquisition/quiet scheduling. Cycle publisher remains disabled for MEKF ownership. |
| 595 / 590 / 587 | Early navigation witness, ideal input bridge, stored TAM input tap. |
| 580 / 579 / 570 | MEKF NavAtt/quality publication, TEST-ONLY publication witness, normal shadow monitor. |
| 568 | TEST-ONLY persistence: after the initial late fault, re-expose stale quality before consumers. No injection here on the initial fault tick. |
| 565 / 560 / 555 | SimpleNav quality facade, point navigation consumer, frozen sample probe. |
| 554 | Coherent snapshot capture/current source-health observation. |
| 552 | Unchanged calculation at the existing compute offset only. |
| 550 | Current application health/generation/cycle decision. |
| 549 | TEST-ONLY witness of the completed gate decision, then the initial late quality fault. |
| 548 | Sole MEKF command owner publishes; no independent health read here. |
| 547 | TEST-ONLY independent native subscriber readback and exposed-quality witness. |
| -1 | Current command, state, field and native navigation recorders. |

The **decision-to-publication window** is causally between priorities 550 and 548.
Integer priority 549 permits exact insertion. Simulation time does not advance
between those callbacks, but this is not an atomic operation or a zero-width
causal window. Witness sequence 25 (completed approval), 26 (fault), 27 (native
readback after owner) all carry 0.8 s. Neither a wall-clock duration nor flight
latency can be inferred from identical message timestamps.

### Command-validity lifetime in this development schedule

1. A decision becomes usable when the priority-550 evaluation completes with
   current health, coherent command provenance, matching source generation and
   an eligible cycle/application window. At 0.8 s it certifies the 0.8 s
   NavAtt/quality/consumer evidence and generation (0,1,0), bound to command ID 1,
   sample/capture 0.4 s and computation 0.5 s.
2. The owner at 548 consumes that same-tick result. It checks the delivery epoch
   and pending calculation; it does not re-read current source quality. A source
   fault arriving after 550 therefore does not retroactively change that decision.
   The published command was approved BEFORE the fault, not newly approved after it.
3. That authorization is for the one scheduled publication at t, whose command
   applies over [t,t + 0.1 s). It is not authority for a later tick. Each owner
   call starts with a fresh zero payload, requires the current gate row, and each
   scheduled gate evaluation reassesses health. The unchanged pending envelope
   is a numeric record, not a retained usability authorization.
4. For this persistent post-gate fault, the first eligible inhibition is the
   next priority-550 evaluation followed by priority-548 publication, at 0.9 s.
   The corresponding zero-torque interval is [0.9,1.0 s).

This is OPTION 1: a bounded, explicit decision lifetime acceptable for this
development test. **No additional owner-side health recheck is required for the
current tested development architecture.** This conclusion assumes the recorded
task executes each boundary; it does not cover scheduler stalls, extra owner
calls, pending-envelope replacement, asynchronous hardware or process failure.
No flight safeing-latency requirement is established here. Future requirements
may impose a stricter maximum fault-to-safe delay.

### Fault mechanism and independently observed actuator response

Source: `validate_late_fault_boundary.py`, revision 2026-10-01. Fault time 0.8 s
and horizon 3 s are **ASSUMED / TEST-ONLY**. The supported 7F-2C `late_quality`
mechanism republishes the real 0.7 s quality payload with its original 0.7 s
header. The fault is stale mandatory source metadata, **not an internal MEKF
math fault**. No fault/valid bits, command values or epochs are fabricated.
Native NavAtt remains current. Normal adapter/monitor history and the quality
actually exposed to consumers are saved separately; the former remains healthy.

At 0.8 s, approval precedes the stale publication at priority 549. On subsequent
ticks the same stale payload is re-exposed at 568, after the normal 570 monitor
and before the 560 consumer and 550 gate. This matches the existing late-quality
fixture insertion and keeps the source-interface fault persistent at use events.
The health gate reports `stale_quality` at 0.9 s and remains FAULTED; no new command
is approved after the injected fault and no reset or fallback is requested.

**CONFIRMED numerical observations for this assumed fixture:**

| Evidence | Result |
|---|---|
| First post-fault owner/native input | 0.8 s, command ID 1, generation (0,1,0), exactly [-0.2,-0.2,0.42702955144649996] A m^2. |
| Surviving decision lifetime | Exactly one post-fault nonzero publication; held during [0.8,0.9 s). No new post-fault approval. |
| Final approved native torque | Record at 0.9 s: [-1.022381403694854e-5,3.265062364161126e-6,-3.25914290906363e-6] N m. Final RK-stage torque, not an interval average. |
| Final approved interval work | -9.656230729328371e-7 J, included in independent work/state reconstruction. |
| First zero native command | 0.9 s; fault-to-zero-input delay 0.1 simulation s. |
| First zero torque interval | [0.9,1.0 s), completed native torque record at 1.0 s; fault-to-completed-readback delay 0.2 simulation s. |
| Persistence | All 22 subsequent publications through 3.0 s are fresh zero, including later ACTUATE phases. No second publisher or SimpleNav fallback. |
| Independent physics | Maximum native torque error 9.470116246213047e-22 N m; MRP/rate component step discrepancy 5.551115123125783e-17 in their respective native units; interval energy/work residual 5.195858281860778e-15 J. |

The actual native subscriber is read separately after owner publication. Existing
pre-propagation native epoch guards, held-input records, independently published
native torque and integrated spacecraft state agree. The 0.9 s torque record
still describes the final nonzero interval even though the new command is zero.
All following completed intervals have exactly zero native torque. Finite,
continuous inertial rotation is expected; it is not treated as a fault.

Nine focused tests and all 17 live validation checks pass. Negative checks reject
an extra retained command, unaccounted native torque and reversed event-order
evidence. Three six-second committed-versus-working SimpleNav preservation cases
(continuous baseline, continuous candidate, cycled baseline) are byte-identical
and contain no development owner. No shared runtime change required broader
regression reruns. Four expected unwritten startup NavAtt recorder warnings retain
their existing pre-acquisition meaning.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_late_fault_boundary.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_late_fault_boundary.py --report basilisk_runner\output_data\phase7g2c_late_fault.json
git diff --check
```

The ignored phase7g2c_late_fault JSON/CSV artifacts preserve the base commit,
source/CSV hashes, assumed configuration, exact scheduler table, ordered fault
witnesses, old approved envelope, native/owner/gate evidence and preservation
results. Existing production artifacts are not overwritten.

**Highest-value next phase: realistic sensor/estimator modeling.** Nominal control,
inhibition, reset/reacquisition and this late-fault boundary now have development
evidence. The ideal truth-derived gyro and Sun inputs still dominate uncertainty
in estimator knowledge and resulting control performance. Build an explicitly
sourced, bounded sensor/estimator fidelity assessment before requirements-grade
detumble claims; preserve TBC/TBD/ASSUMED hardware parameters where unresolved.
Broader timing robustness remains open, but another command-path micro-test or a
longer run with the same ideal inputs would not address that dominant uncertainty.
No further experiment is executed or authorized by this recommendation. This
phase establishes neither flight safeing latency, arbitrary fault-arrival
coverage, process-restart behavior, realistic hardware latency, realistic
estimator performance, requirements-grade detumble nor pointing performance.

## Phase 8A-1 configurable gyro sensor model - 2026-10-01

**GYRO SENSOR MODEL FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL /
NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Gate: **PASS for
software/model behavior and ideal equivalence**, not installed-sensor fidelity.
Starting checkpoint `3fdffd2`; initially clean working tree. The only shared
runtime change is an optional gyro source in attitude_mekf_adapter.py. Scenario
wiring, task order, magnetic cycle, MEKF equations/P0/Q/R, controller, spacecraft,
actuator and environment are unchanged. SimpleNav remains the default.

### Measurement, frame and sign contract

Input is current point truth omega_BN_B in rad/s, expressed in the existing
simulation body frame B. Numeric B-to-installed-spacecraft alignment is still
unresolved. For a proper right-handed C_SB mapping B components to sensor S:

```text
omega_S = C_SB * omega_BN_B
u_S = (diag(scale) + cross_axis) * omega_S + bias_S + noise_S
y_S = clip(u_S, -range_S, +range_S)       # only when range is enabled
y_B = transpose(C_SB) * y_S
MEKF current body rate = y_B - estimated_bias_B
```

Scale stores gains (unity means no error). Cross-axis terms are dimensionless
off-diagonal additive gains with an enforced zero diagonal; they are not another
rotation matrix or a conversion of the vendor angular cross-axis specification.
Bias is additive in S and rotates into B before estimator consumption. The sensor
model never subtracts the MEKF estimate. Rotation validity, parameter units/frame,
finite values, positive scales/ranges, nonnegative noise and integer epochs are
checked. Independent axis projections, a signed quarter turn, arbitrary bases,
known bias/scale and off-diagonal leakage verify direction/sign and round trips.

Optional hard clipping occurs in S before inverse mounting. Saturated axes are
recorded; a saturated sample is explicitly invalid for this linear gyro consumer.
Clipped measurements remain available as diagnostics. Invalid/nonfinite truth
produces an invalid record rather than a fabricated valid rate. No ADC resolution,
aperture filter, temperature dependence, drift, random walk or g sensitivity is
claimed. These are future evidence-dependent model additions.

### Discrete noise, cadence and timing contract

Noise is independent zero-mean Gaussian per acquired sample/axis in S with the
configured discrete rad/s sigma; covariance in B is C_SB.T diag(sigma_S^2) C_SB.
An explicit PCG64 seed is required for nonzero noise. Zero noise constructs no
RNG and draws nothing. Off-grid observation calls draw nothing. No VN-100 noise
density, bandwidth, bias stability or RMS attitude value is converted into sigma,
random walk, covariance or estimator tuning. A 20,000-sample synthetic test checks
mean, variance and inter-axis correlations with stated six-standard-error bounds.
This is a software distribution check, not calibrated statistical confidence.

The isolated model accepts monotonically advancing integer simulation ns calls;
acquisition occurs only at offset + k*period, k >= 0. Off-grid calls return no
sample. Duplicate/out-of-order epochs are rejected. The caller supplies truth
for the requested epoch and owns delivery; skipped acquisitions are not invented.
Publication availability equals acquisition + configured latency. The immutable
sample retains both epochs. age(processing) is processing minus acquisition and
rejects processing before availability. A publication epoch is an availability
contract, not evidence that a message was actually transported at that instant.

**Live boundary:** the current adapter accepts only its existing 0.1 s task
cadence, zero offset and zero gyro latency. It receives the modeled current-point
B rate and runs the unchanged ten-subinterval linear endpoint reconstruction.
That reconstruction is a development aperture approximation: interpolated noise
is correlated between subintervals, not ten new independent gyro observations.
IDEAL_REGRESSION preserves endpoint values exactly, including signed zero.

The existing ReplayEstimator supports delayed vector updates with contiguous,
forward gyro coverage; it does not provide arbitrary delayed gyro ingestion.
Nonzero-latency/different-cadence gyro profiles are therefore isolated-only and
are explicitly rejected by live configuration validation. A delayed sample sent
directly with its original acquisition epoch triggers the existing
gyro_or_batch_epoch_mismatch guard; no epoch is relabeled. Delayed-vector replay
with the ideal gyro model remains exact. No asynchronous gyro reconstruction or
new stale-gyro hold/extrapolation policy was introduced to make tests pass.

### Profiles, provenance and opt-in ownership

All numerical defaults/profiles are **ASSUMED / TEST-ONLY**, revision 2026-10-01,
sourced in gyro_sensor_model.py; actual installed parameters remain TBD/TBC.

| PROFILE | NONIDEAL TERMS | TIMING / USE |
|---|---|---|
| IDEAL_REGRESSION | Identity C_SB, zero bias/cross/noise, unity gain, no range clipping | Existing 0.1 s acquisitions, zero latency; live shadow or isolated. |
| TEST_BIAS_ONLY | [0.003,-0.002,0.001] rad/s in S; Phase 7C synthetic fixture values | Same timing; no convergence/accuracy acceptance. |
| TEST_SCALE_ONLY | [1.01,0.98,1.03] dimensionless gains | Same timing; synthetic +1%, -2%, +3% errors. |
| TEST_NOISE_ONLY | [0.001,0.002,0.003] rad/s discrete sigma in S; seed 8101 | Same timing; no density-to-sigma conversion. |
| TEST_DELAYED_SAMPLE | 200000000 ns publication latency | Isolated-only; original acquisition timestamp retained. |

GyroConfig reuses the provenance-bearing Parameter type without editing
hs2_sim_config.py or importing any vendor prior into runtime configuration.
Each parameter carries units, frame, source/revision, status and treatment.
to_dict() reports runtime_usable_for_flight:false; a SHA-256 fingerprint binds
each sample to the configuration. Frozen configurations can be explicitly
replaced with new provenance-bearing parameters; there is no implicit fallback.
Profiles are defined once in code and exported in the validator report, avoiding
a second independently maintained JSON constant set.

Opt-in example (Python API only):

```python
from attitude_mekf_adapter import ShadowOptions
from gyro_sensor_model import profile_config
# Pass this to the existing run(..., shadow=...) with SIMPLE_NAV_REFERENCE.
shadow = ShadowOptions(ideal_sun=True, gyro_model=profile_config("IDEAL_REGRESSION"))
```

The default gyro_model=None keeps the original source. Additive InputBatch
gyro_sample metadata records modeled acquisition/publication and validity;
original input fields and current processing epochs keep their prior meanings.
The existing MEKF_DEVELOPMENT option guard rejects every modeled gyro profile,
including IDEAL_REGRESSION. Perturbed measurements have no actuator authority;
the one live bias case is shadow-only. There is no new CLI flag or controller
connection. Existing unmodeled MEKF development control remains unchanged.

### Verification and remaining gate

The validator loads the committed adapter directly from HEAD, then compares it
with the current default and current ideal-model adapters on identical 3 s runs
with delayed magnetic vector delivery. All 31 point measurements, gyro intervals,
vector events, estimator states, bias/covariance histories, update counts, epochs,
status and shadow telemetry match exactly. Chronological replay has zero
quaternion/bias/P difference. Committed/default/ideal-model/bias-shadow host CSV
bytes are identical; synthetic bias reaches the shadow MEKF once without changing
the host controller/actuator/plant. A separate committed-versus-current unmodeled
MEKF closed-loop run also preserves CSV and estimator trace exactly.

**Results:** 18 new tests, 41 existing MEKF/adapter regressions, 9 existing nominal
closed-loop regressions pass (68 total); all 12 live equivalence checks pass.
Three six-second SimpleNav preservation cases (continuous baseline, candidate,
cycled baseline) remain byte-identical to HEAD. These are CONFIRMED software
observations for ASSUMED fixtures. No full-orbit or performance campaign ran.
Expected pre-acquisition unwritten NavAtt warnings retain their previous meaning.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_gyro*.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_attitude_mekf*.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_mekf_closed_loop.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_gyro_sensor_model.py --report basilisk_runner\output_data\phase8a1_gyro_model.json
git diff --check
```

Ignored phase8a1_gyro_model JSON/CSV artifacts retain the checkpoint, source hashes,
profile provenance, sample/config fingerprints, estimator trace, CSV hashes and
preservation evidence. Installed model/serial/firmware, mounting/calibration,
bias, scale/coupling, noise spectrum/correlation, sample/filter configuration,
acquisition aperture and clock/latency remain unresolved. See the bounded
[gyro evidence disposition](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#phase-8a-1-gyro-model-evidence-disposition---2026-10-01).

**Smallest next experiment:** a shadow-only, single-axis controlled bias plus
seeded discrete-noise case, compared with the unchanged ideal run, recording
innovation, bias-state response, covariance behavior and validity/rejections.
Keep existing Q/R fixed and label all inputs synthetic; use the result to expose
model/estimator consistency limits before any evidence-based tuning or closed-loop
performance claim. Installed sensor calibration evidence is still required to
turn such a software experiment into realistic HS-2 prediction. Do not execute
that next experiment as part of this phase.

## Phase 8A-2A live synthetic gyro-bias response - 2026-10-01

**SYNTHETIC GYRO-BIAS ESTIMATOR RESPONSE / SHADOW DEVELOPMENT TEST /
NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Engineering gate: **PASS** for
the single deterministic estimator/model consistency claim. Starting checkpoint
`d8eb4d2` contains 8A-1; the working tree was clean. No shared runtime, sensor
model, MEKF equation, Q/R/P0, controller, cycle, actuator or physical parameter
was changed. Two validation files and this/scenario documentation are added.
The sensor-evidence register receives no new installed-hardware classification.

### Fixed experiment and read-only live witnesses

Reuse the committed 8A-1 three-second IDEAL_REGRESSION case: same 0.1 s gyro
cadence, identity C_SB, zero latency/noise, existing ideal Sun and TAM inputs,
and existing 0.2 s magnetic delivery delay after 1 s. The only perturbation is
the existing TEST_BIAS_ONLY profile, bias_S = [0.003,-0.002,0.001] rad/s; S=B.
All such inputs and numerical policies are **ASSUMED / TEST-ONLY**, sourced in
gyro_sensor_model.py and config/attitude_mekf_test_only.json, unchanged at this
checkpoint. The horizon was not extended to seek convergence.

Temporary validation-only wrappers around the existing adapter update and core
observe calls copy pre/post states and record the processing epoch. They call the
original methods exactly once with unchanged arguments/return values and restore
the methods afterward. The record includes every actual callback during replay;
it does not fabricate a fresh observation when an older event is replayed.
No actuator handle is acquired. The ideal estimator trace and host CSV match the
source-bound saved 8A-1 reference exactly, checking instrumentation neutrality.
If that ignored reference is unavailable, the validator uses a short
uninstrumented ideal run and still requires unchanged shared runtime versus HEAD.

### Measurement sign, bias response and independent expectation

All 31 delivered samples satisfy y_B = omega_true_B + [0.003,-0.002,0.001]
rad/s exactly. Acquisition/publication/processing epochs remain equal for this
zero-latency profile. Independent native NavAtt readback equals delivered y_B
minus the current posterior bias estimate, with zero numerical discrepancy.
Thus bias is added once in S=B and subtracted once by the estimator.

Bias estimates below are **CONFIRMED numerical observations of this TEST-ONLY
case**, source validate_gyro_bias_response.py, revision 2026-10-01, in rad/s:

| AXIS | INITIAL ESTIMATE AT 0.4 s | FIRST CORRECTION, SUN AT 1.1 s | FINAL ESTIMATE AT 3 s | FINAL ESTIMATE MINUS INJECTED BIAS |
|---|---|---|---|---|
| X | 0 | +1.5450121e-5 | +0.0027976701 | -0.0002023299 |
| Y | 0 | -8.6062267e-6 | -0.0018582751 | +0.0001417249 |
| Z | 0 | +2.0355536e-6 | +0.0009182657 | -0.0000817343 |

The first and subsequent estimates have the injected signs, remain finite, and
move toward the known bias as vector information becomes available. Full
convergence is not achieved at 3 s. No acceptance criterion specifies convergence
speed; no estimator tuning or additional horizon was used to reduce the residual.

An independent small-interval check uses only b*dt over [0.4,0.5 s), before any
post-acquisition vector correction. Define excess estimated rotation by the
principal passive rotation of C_est C_truth.T, approximately +b*dt; this is the
opposite sign of the MEKF's true-minus-estimate correction convention. Expected
vector is [0.0003,-0.0002,0.0001] rad; observed bias-minus-ideal excess is
[0.00029836156,-0.00020048051,0.00010383615] rad. Norms are 0.00037416574 and
0.00037415772 rad respectively. Vector discrepancy 4.19898e-6 rad is within the
1.64866e-5 rad first-order body-rotation transport allowance derived from the
observed rate and interval. This is a local sign/growth sanity check, not a second
MEKF or a flight performance model.

### Direct ideal-versus-bias comparison

These metrics are simulation-truth development diagnostics only. Published-state
maxima exclude pre-update peaks. Witnessed maxima also include pre/post vector
updates, including replay states; neither is a continuous-time maximum claim.
Final-history innovations count each event once using its last replay evaluation.

| METRIC | IDEAL_REGRESSION | TEST_BIAS_ONLY |
|---|---|---|
| Final principal attitude error, rad | 3.720384e-6 | 2.781083e-4 |
| Maximum published attitude error, rad | 1.369167e-5 | 2.231006e-3 |
| Maximum witnessed attitude error, rad | 1.594657e-5 | 2.602469e-3 |
| Final estimated bias, rad/s | [-1.790200e-5,1.085055e-5,5.648901e-7] | [0.002797670,-0.001858275,0.000918266] |
| Final bias error, estimate minus injected, rad/s | [-1.790200e-5,1.085055e-5,5.648901e-7] | [-2.023299e-4,1.417249e-4,-8.173425e-5] |
| Maximum magnetic angular residual, rad | 7.474803e-6 | 1.249729e-3 |
| Maximum Sun angular residual, rad | 1.566772e-5 | 2.560936e-3 |
| Maximum magnetic tangent-innovation norm, dimensionless | 7.474803e-6 | 1.249729e-3 |
| Maximum Sun tangent-innovation norm, dimensionless | 1.566772e-5 | 2.560933e-3 |
| Covariance minimum eigenvalue, numerical SI-state coordinates | 7.951391e-6 | 7.951371e-6 |
| Covariance maximum eigenvalue, same coordinates | 0.10113308 | 0.10113263 |
| Maximum covariance asymmetry | 0 | 0 |
| Estimator validity | Uninitialized until 0.4 s; then valid | Same |
| Rejected events | 0 | 0 |
| Magnetic / Sun update counts, including acquisition pair | 3 / 4 | 3 / 4 |
| Delayed-vector replays | 2 | 2 |
| Distinct post-acquisition vector updates / actual callbacks | 5 / 6 | 5 / 6 |

The bias-case final excess-rotation vector is
[2.122940e-4,-1.576353e-4,8.617781e-5] rad. Error grows during gyro-only intervals
and decreases on valid corrections. At the first Sun update, the full principal
error drops from 2.602469e-3 to 4.630367e-4 rad; that vector's angular residual
drops from 2.560936e-3 to 2.548739e-6 rad. A small single-vector residual does not
prove small error about that vector's unobservable axis. All five final-history
updates reduce full truth-angle error in this particular case; this is not a
general single-vector guarantee.

Every actual update, including replay, reduces its measured-vector residual.
Independent geometric descent direction body_vector cross predicted_vector has
nonnegative projection onto the observed attitude correction. Recorded tangent
innovation norm matches the independent cross-product norm (sin of residual
angle), and all innovations are finite. Both vector types expose the injected
propagation error without a new acceptance threshold. The small existing ideal
baseline residual and nonzero estimated bias are preserved, not attributed to
the new perturbation; this phase does not isolate their numerical contributions.

### Covariance, isolation and verification

The existing P0 attitude diagonal is [0.1,0.1,0.1] rad^2 and bias diagonal is
[0.001,0.001,0.001] (rad/s)^2; Q remains zero and the vector algebra weight remains
0.0001 in its existing TEST-ONLY convention. The biased final attitude diagonal
is [1.781607e-4,2.374448e-4,1.409711e-4] rad^2; bias diagonal is
[7.564262e-5,1.417780e-4,1.528130e-4] (rad/s)^2. All sampled/pre/post-update
covariances are finite, symmetric and positive definite, well above the numerical
5.68434e-14 health tolerance. Bias covariance does not grow above P0; attitude
trace remains below the zero-Q kinematic bound 0.654 rad^2. There is no observed
numerical collapse or divergence. None of these results calibrates covariance
statistically to the synthetic perturbation or to installed hardware.

SimpleNav remains the actual production controller input. Ideal and biased host
CSV bytes, including commands/applied torque/spacecraft state, are identical and
match the saved 8A-1 host hash. No development command owner is present. An actual
attempt to select MEKF_DEVELOPMENT with TEST_BIAS_ONLY is rejected by the unchanged
guard. Shared runtime and numerical-policy files match HEAD; Q/R/P0 were not tuned.

Seven focused tests and all 18 live checks pass. Negative evidence tests reject
double-addition-style measurement inconsistency and reversed correction evidence.
Compilation and diff checks pass. Shared code did not change, so earlier sensor
and core suites and redundant production campaigns were not rerun.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_gyro_bias_response.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_gyro_bias_response.py --report basilisk_runner\output_data\phase8a2a_gyro_bias.json
git diff --check
```

Ignored JSON and ideal/bias host CSV artifacts carry the source/checkpoint hashes,
complete model and unchanged numerical-policy provenance, timestamped gyro/bias/
attitude/covariance/validity histories, raw and unique replay/update witnesses,
independent expectations and preservation evidence. Actual HS-2 installed gyro
calibration, mounting, noise/filter/timing and realistic sensor/estimator accuracy
remain unresolved. No new mathematics defect or reason to change equations was
exposed by this deterministic case.

**Next smallest justified experiment: NOISE-ONLY shadow estimator response.** Use
the existing TEST_NOISE_ONLY profile with zero injected bias, its recorded seed,
unchanged vector schedule and Q/R/P0, to isolate stochastic response before any
combined bias/noise case. Do not claim statistically calibrated confidence or
flight accuracy. That experiment is not executed in Phase 8A-2A.

## Phase 8A-2B live synthetic gyro-noise response - 2026-10-02

**SYNTHETIC GYRO-NOISE ESTIMATOR RESPONSE / SHADOW DEVELOPMENT TEST /
NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Engineering gate: **PASS** for
one stochastic software/estimator consistency claim. Starting checkpoint
`1366930` contains 8A-2A; the working tree was clean before this phase. The two
new validation/test files reuse the committed 8A-2A live-case and read-only update
witnesses. No shared runtime, gyro model, MEKF math, Q/R/P0, controller, timing,
actuator, spacecraft or environment code was changed.

### Fixed inputs, independent sequence and exact repeatability

Use the same 3 s horizon, 0.1 s gyro cadence, initial state, ideal vector schedule
and delayed-vector replay fixture as 8A-2A. TEST_NOISE_ONLY has identity C_SB
(S=B), zero deterministic bias, unit scale, zero cross-axis error, no clipping,
zero latency, discrete Gaussian sigma [0.001,0.002,0.003] rad/s per acquired
sample, and PCG64 seed 8101. These are **ASSUMED / TEST-ONLY**, sourced from
gyro_sensor_model.py, Phase 8A-1 revision 2026-10-01 at the starting checkpoint.
They are not converted vendor noise density or confirmed HS-2 hardware values.
The 31-sample realization is not required to reproduce population statistics.
The earlier 20,000-sample distribution test was not repeated.

An independent NumPy PCG64 generator constructs the complete expected sequence
without calling GyroModel or using recorded noise to construct the expectation.
Every logged raw noise draw matches exactly, starting with the t=0 acquisition;
every delivered S/B measurement equals ideal measurement plus that draw exactly.
Subtraction of the two delivered floating-point rates recovers the expected noise
within arithmetic roundoff. All 31 acquisition/publication/processing epochs and
configuration fingerprints are checked. No extra RNG draws, hidden scale/bias
term or double noise addition appear.

Two independent TEST_NOISE_ONLY live runs reproduce exactly: full input batches
(including gyro metadata, interpolated intervals and vector events), attitude,
bias, covariance, native navigation telemetry, validity, innovations, raw replay
callbacks and counts. The JSON records the NumPy version, both case hashes,
source hashes and checkpoint. No alternate seed or Monte Carlo was used.

### Observed ideal-versus-noise response

Numbers below are **CONFIRMED numerical observations of this TEST-ONLY case**,
source validate_gyro_noise_response.py and output_data/phase8a2b_gyro_noise.json,
revision 2026-10-02. Attitude errors compare C_est with spacecraft truth C_BN;
bias components are in B. Published and witnessed maxima are sampled maxima,
not continuous-time bounds. Witnessed maxima include pre/post vector updates and
replay; innovation maxima use each event's final retained-history evaluation.

| METRIC | IDEAL_REGRESSION | TEST_NOISE_ONLY |
|---|---|---|
| Final principal attitude error, rad | 3.720384e-6 | 7.411605e-4 |
| Maximum published attitude error, rad | 1.369167e-5 | 1.391670e-3 |
| Maximum witnessed attitude error, rad | 1.594657e-5 | 1.438195e-3 |
| Maximum gyro-only sample-to-sample error-vector change, rad | 2.308185e-6 | 5.467263e-4 |
| Final estimated bias, rad/s | [-1.790200e-5,1.085055e-5,5.648901e-7] | [7.843059e-4,7.599627e-4,6.150821e-5] |
| Maximum witnessed bias-estimate norm, rad/s | 2.094123e-5 | 1.280840e-3 |
| Maximum magnetic tangent-innovation norm, dimensionless | 7.474803e-6 | 6.617142e-4 |
| Maximum Sun tangent-innovation norm, dimensionless | 1.566772e-5 | 1.214317e-3 |
| Maximum magnetic angular residual, rad | 7.474803e-6 | 6.617143e-4 |
| Maximum Sun angular residual, rad | 1.566772e-5 | 1.214318e-3 |
| Minimum covariance eigenvalue, numerical SI-state coordinates | 7.951391e-6 | 7.951144e-6 |
| Maximum covariance eigenvalue, same coordinates | 0.10113308 | 0.10113308 |
| Maximum covariance asymmetry | 0 | 0 |
| Validity | Uninitialized until 0.4 s, then valid | Same |
| Rejected events | 0 | 0 |
| Magnetic / Sun update counts, including acquisition pair | 3 / 4 | 3 / 4 |
| Delayed-vector replays | 2 | 2 |
| Distinct post-acquisition vector updates / actual callbacks | 5 / 6 | 5 / 6 |

Noise perturbs propagation and subsequent magnetic/Sun residuals. Error need not
grow or improve monotonically. Every actual vector update, including replay,
reduces its corresponding measured-vector angular residual, with consistent
geometric correction direction and finite innovations. Recorded tangent norm
matches the independently reconstructed cross-product norm. The five final-history
updates also reduce full truth-angle error in this realization; that is not a
general single-vector observability guarantee or an innovation acceptance limit.

Initial estimated bias is zero. Noise-case witnessed axis excursions, rad/s:
X [-1.620780e-6,7.925890e-4], Y [-2.668329e-5,1.144243e-3],
Z [-2.365657e-4,6.689737e-5]. With zero true deterministic bias, final bias error
equals the final estimate above. Bias is held exactly through gyro-only intervals
and changes on vector processing; individual random gyro samples are not assigned
as a fixed known bias. No deterministic sign, zero final estimate or convergence
speed is required. The recorded trajectory shows no runaway over this short
window; it establishes no long-term stability or bias-performance bound.

### Independent endpoint-interpolated propagation check

Use [0.4,0.5 s], immediately after identical vector acquisition and before any
post-acquisition vector correction or bias update. Existing midpoint substeps
linearly interpolate the received endpoint gyro rates. Thus the independent
first-order noise increment is 0.5*(n_0+n_1)*0.1 s, not either endpoint alone.
The actual endpoint noise vectors, rad/s, are:
[-0.0012080503,0.0034731817,-0.0013765342] and
[-0.0005579447,-0.0041174260,0.0029059894].

Expected excess rotation is [-8.829975e-5,-3.221222e-5,7.647276e-5] rad;
observed noisy-minus-ideal excess is [-8.685414e-5,-3.064080e-5,7.413704e-5] rad.
Norms are 1.211716e-4 and 1.182320e-4 rad, with positive directional projection.
The vector discrepancy 3.164609e-6 rad is within the 2.237507e-5 rad first-order
rotation-transport allowance derived from endpoint peak noise/rate and duration.
The error convention is the passive principal rotation of C_est C_truth.T,
excess estimated rotation, opposite to the MEKF true-minus-estimate correction.
This local check is not a long-term random-walk model. Interpolation correlates
subinterval errors and does not generate additional independent measurements.

### Covariance, control isolation and verification

All recorded covariances are finite, symmetric and positive definite; variances
are positive, and the minimum eigenvalue remains well above the 5.68434e-14
numerical tolerance. Noise-case final attitude covariance diagonal is
[1.780689e-4,2.374570e-4,1.410240e-4] rad^2; final bias covariance diagonal is
[7.558610e-5,1.415906e-4,1.528713e-4] (rad/s)^2. Maximum attitude trace is
0.301439664 rad^2, below the existing zero-Q kinematic bound 0.654 rad^2; bias
diagonals do not exceed P0 [0.001,0.001,0.001] (rad/s)^2. No observed numerical
collapse or unexplained explosive growth. Q remains zero, R's existing algebra
weight and P0 are unchanged. This covariance is not statistically calibrated to
the injected noise; numerical health does not imply realistic uncertainty.

SimpleNav remains wired to the actual controller. No MEKF command owner is
created; an actual attempt to select MEKF_DEVELOPMENT with TEST_NOISE_ONLY is
rejected by the unchanged guard. All three host CSVs, including spacecraft state,
commands and independently applied torque, are byte-identical and match the
source-bound saved 8A-2A ideal reference. The ideal estimator/update evidence
also matches that reference exactly. If the ignored artifact is absent, the
validator falls back to an uninstrumented ideal run with unchanged shared source.

**Nine focused tests / 26 live checks pass.** Tests also reject doubled delivered
noise, a shifted RNG draw, and altered covariance in repeatability evidence.
Compilation and whitespace checks pass. No shared runtime changes occurred, so
unrelated regression suites and previous audit/model campaigns were not rerun.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_gyro_noise_response.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_gyro_noise_response.py --report basilisk_runner\output_data\phase8a2b_gyro_noise.json
git diff --check
```

The JSON and ideal/noise/repeat host CSV artifacts are ignored generated evidence.
There is no new installed-sensor evidence classification. Realistic prediction
still requires applicable gyro calibration, noise/bandwidth, mounting/timing,
vector-sensor errors and an evidence-based estimator covariance configuration.

**Next smallest justified experiment: combined BIAS + NOISE shadow response**
using the already separate synthetic bias/noise magnitudes, the same recorded
seed, schedule and fixed Q/R/P0, under separate authorization. It is not executed
here and would still establish only software/model consistency.

## Phase 8A-2C live combined synthetic gyro bias and noise - 2026-10-02

**COMBINED SYNTHETIC GYRO BIAS + NOISE RESPONSE / SHADOW DEVELOPMENT TEST /
NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Engineering gate: **PASS** for
composability of the previously verified synthetic effects. Checkpoint `7b93165`
contains 8A-2B; the starting working tree was clean. No runtime, sensor-model
registry, flight configuration, estimator math, Q/R/P0, controller, timing,
actuator, spacecraft or environment changes were made.

### Combined configuration and critical measurement gate

The validation harness constructs TEST_BIAS_PLUS_NOISE with dataclasses.replace,
copying the existing sourced bias/noise Parameters. This is **ASSUMED / TEST-ONLY**,
runtime_usable_for_flight=false, not a new registered production gyro mode.
Bias_S=[0.003,-0.002,0.001] rad/s, discrete sigma_S=[0.001,0.002,0.003] rad/s per
sample, and PCG64 seed=8101 are unchanged Phase 8A-1 fixtures from
gyro_sensor_model.py, revision 2026-10-01. C_SB is identity (S=B), scale is unity,
cross-axis error is zero, clipping is disabled and latency is zero. The existing
3 s horizon, 0.1 s gyro cadence, vector schedule and numerical policy are retained.

Before characterizing the estimator, every delivered sample is checked against
ideal_measurement + deterministic_bias + independently generated seeded_noise.
All 31 measurements match exactly, with the same raw noise draws as 8A-2B and
unchanged acquisition/publication/processing epochs. Combined minus noise-only
recovers the deterministic bias within subtraction roundoff. S/B measurements,
configuration fingerprints and unchanged truth-rate history are checked. The
critical gate saves failed evidence and skips estimator interpretation if
composition fails; negative tests reject missing bias, doubled bias and doubled
noise. No Gaussian-distribution or prior isolated-estimator campaign is repeated.

Source-bound passing evidence in phase8a2a_gyro_bias.json and
phase8a2b_gyro_noise.json supplies IDEAL, BIAS_ONLY and NOISE_ONLY. Artifact/source
hashes are checked; overlapping ideal evidence must agree, or the validator
raises a contradiction. All references were reused in this phase. If ignored
artifacts are absent/unbound, only missing references are regenerated from the
unchanged short fixture. The combined case is run twice with seed 8101; inputs,
attitude/bias/covariance, innovations, native navigation telemetry, validity and
event/replay counts reproduce exactly. There is no alternate seed or Monte Carlo.

### Four-case development comparison

These are **CONFIRMED numerical observations of TEST-ONLY software fixtures**.
Source: validate_gyro_bias_noise_response.py / phase8a2c_gyro_bias_noise.json,
revision 2026-10-02; reference values reuse their source-bound 8A-2A/2B histories.
Bias components are B-frame rad/s. Bias error means estimate minus injected
deterministic bias; injected bias is zero for IDEAL and NOISE_ONLY. Maxima cover
recorded states, not continuous time. Witnessed maxima include pre/post vector
updates and replay; innovation maxima use each event's last retained evaluation.

| METRIC | IDEAL | BIAS_ONLY | NOISE_ONLY | BIAS_PLUS_NOISE |
|---|---|---|---|---|
| Final principal attitude error, rad | 3.720384e-6 | 2.781083e-4 | 7.411605e-4 | 6.231268e-4 |
| Maximum published attitude error, rad | 1.369167e-5 | 2.231006e-3 | 1.391670e-3 | 1.852583e-3 |
| Maximum witnessed attitude error, rad | 1.594657e-5 | 2.602469e-3 | 1.438195e-3 | 2.315632e-3 |
| Final estimated bias, rad/s | [-1.790200e-5,1.085055e-5,5.648901e-7] | [2.797670e-3,-1.858275e-3,9.182657e-4] | [7.843059e-4,7.599627e-4,6.150821e-5] | [3.599426e-3,-1.109702e-3,9.793997e-4] |
| Final bias error, rad/s | [-1.790200e-5,1.085055e-5,5.648901e-7] | [-2.023299e-4,1.417249e-4,-8.173425e-5] | [7.843059e-4,7.599627e-4,6.150821e-5] | [5.994256e-4,8.902977e-4,-2.060026e-5] |
| Maximum witnessed bias-estimate norm, rad/s | 2.094123e-5 | 3.481861e-3 | 1.280840e-3 | 3.891854e-3 |
| Maximum magnetic tangent-innovation norm, dimensionless | 7.474803e-6 | 1.249729e-3 | 6.617142e-4 | 1.739300e-3 |
| Maximum Sun tangent-innovation norm, dimensionless | 1.566772e-5 | 2.560933e-3 | 1.214317e-3 | 2.178413e-3 |
| Minimum covariance eigenvalue, numerical SI-state coordinates | 7.951391e-6 | 7.951371e-6 | 7.951144e-6 | 7.951123e-6 |
| Maximum covariance eigenvalue, same coordinates | 0.10113308 | 0.10113263 | 0.10113308 | 0.10113264 |
| Maximum covariance asymmetry | 0 | 0 | 0 | 0 |
| Validity | Valid from 0.4 s | Same | Same | Same |
| Rejected events | 0 | 0 | 0 | 0 |
| Magnetic / Sun updates including acquisition | 3 / 4 | 3 / 4 | 3 / 4 | 3 / 4 |
| Delayed-vector replays | 2 | 2 | 2 | 2 |
| Distinct post-acquisition updates / actual callbacks | 5 / 6 | 5 / 6 | 5 / 6 | 5 / 6 |

Combined initial bias estimate is zero; the first meaningful correction at the
1.1 s Sun update is [+1.392610e-5,-5.348455e-6,+2.655848e-6] rad/s. First and
final directions agree with the injected deterministic signs in this realization.
Final combined-minus-noise-only bias estimate also has the injected signs, without
requiring linear superposition. All estimates remain finite. Bias is exactly held
during gyro-only intervals; every published change matches the last actual vector
update at that processing epoch, including replay. No unexplained discontinuity
or short-window runaway is observed. No long-term stability, convergence speed or
flight bias limit is established.

The final combined bias error is larger than bias-only; final attitude error is
larger than bias-only but smaller than noise-only. Magnetic innovation maximum
exceeds both isolated cases while the Sun maximum lies between them. This is
consistent with the direction/time-dependent interaction of the fixed noise
realization and deterministic drift. Independent composition and geometric
checks expose no unexplained math/frame effect. Scalar error norms are not
required to add, and this nonlinear filter is not required to obey superposition.
No tuning or extended horizon was used to change these outcomes.

Every actual vector update reduces its measured-vector residual, including replay,
with finite innovations and independently checked correction direction. All five
final-history updates also reduce full truth-angle error in this case; this is not
a general single-vector observability guarantee. Combined maximum gyro-only
sample-to-sample error-vector change is 6.023065e-4 rad. Full histories retain
propagation variation and pre/post-update errors at their actual epochs.

### Independent local check and covariance health

For [0.4,0.5 s], after identical vector acquisition and before a vector/bias
correction, the independent increment is
(bias + 0.5*(noise_at_0.4s + noise_at_0.5s))*0.1 s. This accounts for both
endpoints of the existing linear gyro interpolation. Expected excess rotation is
[2.117003e-4,-2.322122e-4,1.764728e-4] rad; observed combined-minus-ideal excess
is [2.115064e-4,-2.311320e-4,1.779557e-4] rad. Norms are 3.603917e-4 and
3.603126e-4 rad. Directional projection is positive; discrepancy 1.844867e-6 rad
is within the 3.389218e-5 rad local first-order rotation-transport allowance.
Excess rotation uses the passive principal vector of C_est C_truth.T, opposite to
the MEKF true-minus-estimate correction convention. This is not a long-term model.

Combined covariance is finite, symmetric and positive definite; no negative
variances, NaNs/Infs or numerical collapse appear. Minimum eigenvalue exceeds the
5.68434e-14 numerical tolerance. Maximum attitude trace is 0.301439414 rad^2,
below the unchanged zero-Q kinematic bound 0.654 rad^2; bias diagonals never exceed
P0 [0.001,0.001,0.001] (rad/s)^2. There is no unexplained explosive growth.
Q/R/P0 remain unchanged TEST-ONLY fixtures, not calibrated uncertainty for the
combined injected errors or installed hardware.

### Isolation, verification and next sensor branch

SimpleNav remains the actual control source. No MEKF command owner is created,
and selecting MEKF_DEVELOPMENT with the combined modeled gyro is rejected by the
unchanged runtime guard. Combined/repeat host CSV bytes match all three reference
host hashes, covering spacecraft state, commands and independently applied torque.
The combined profile never gains actuator authority.

**Eight focused tests / 25 live checks pass.** Compilation and whitespace checks
pass. Shared runtime/helpers/configuration match HEAD; prior isolated suites and
long performance simulations were not rerun. The ignored JSON stores source and
reference-artifact hashes, complete four-case histories, original sourced profile
parameters, policy, independent expectations and repeat hashes. Two new ignored
CSV artifacts retain the combined/repeat host outputs.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_gyro_bias_noise_response.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_gyro_bias_noise_response.py --report basilisk_runner\output_data\phase8a2c_gyro_bias_noise.json
git diff --check
```

**Next highest-value branch: magnetic/TAM measurement and validity model.**
The existing evidence register identifies installed magnetic calibration and
actuation contamination as unresolved [INPUT-07](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#input-07),
with characterization in [MEAS-03](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#meas-03).
TAM is used by both detumble feedback and estimator correction, so its frame,
calibration, local-field contamination and clean-acquisition/filter-history
contract affect two active chains. A narrow next phase should define and test a
parametric shadow TAM measurement/validity interface with ideal regression and
explicit provenance, preserving cycle timing and keeping unsupported magnitudes
TBD/TEST-ONLY until installed data exist. No such model is implemented here.

CSS/Sun-vector reconstruction remains essential for absolute attitude availability,
but [INPUT-08](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#input-08) retains unresolved
array selection/count, populated normals, channel calibration and inversion.
That branch needs its geometry/channel contract before realistic reconstruction.
TAM-first is an engineering priority based on shared impact and available
interfaces, not proof that magnetic error numerically dominates an error budget.
Installed sensor characterization and evidence-based covariance remain blockers
to realistic HS-2 prediction. No new sensor-evidence classification was established.

## Phase 8B-1 parametric TAM measurement and validity - 2026-10-02

**TAM MEASUREMENT / VALIDITY MODEL FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL /
NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Engineering gate: **PASS**
for the explicit measurement/validity contract and ideal live equivalence only.
HEAD was `7b93165`; the completed 8A-2C validator/tests and documentation were
already uncommitted and are preserved. No gyro campaign was repeated.

### Existing path and opt-in boundary

The existing environment is unchanged: WMM evaluates current spacecraft position
and Earth orientation and publishes B_N in Tesla, in Earth-centered inertial N.
Current sigma_BN gives C_BN, mapping N to mathematical body B. Native TAM reports
tam_S = C_SB C_BN B_N in sensor S, with C_SB mapping B to S. The physical HS-2
body/mounting registration remains unresolved. Native TAM executes at the cycle's
SAMPLE event; the cycle retains that sample and acquisition-epoch SimpleNav for
detumble. Historical cycle fields named *_B_B still contain raw tam_S. The MEKF
bridge already carries S plus C_SB and uses C_SB.T for reconstruction.

Only explicit Python ShadowOptions(tam_model=TAMConfig()) selects the new path.
At unchanged bridge priority 590 it independently computes B_B from current
SCStates/WMM, then calls the model. Native TAM and the original
magnetic_cycle_sample predicate remain the acquisition/validity witnesses and
the unchanged controller input. MEKF runs at 580 afterward. All other task order,
control equations/gains, actuator, environment and cycle timings are unchanged.
The default tam_model=None path matches the read-only HEAD adapter exactly.

Live opt-in permits only the complete unchanged IDEAL_REGRESSION configuration;
renaming a perturbed configuration to IDEAL does not bypass this guard. Bias,
scale, noise and nonzero model latency remain isolated-only. Existing
MEKF_DEVELOPMENT selection rejects every non-None TAM model, including ideal.
The value module has no Basilisk dependency, actuator handle or command publisher.

### Value, frames, units and provenance contract

The standalone module implements:

```text
B_S,true = C_SB B_local,B
y_S,raw  = (diag(scale) + cross_axis) B_S,true + bias_S + noise_S
y_S      = optional per-axis clipping of y_S,raw
y_B      = C_SB.T y_S                 # reconstructed measurement, not truth
```

All field/bias/noise/range values are in **Tesla**. Gains/matrices are
dimensionless; epochs/latency are integer simulation ns. Explicit vector helpers
convert microtesla to Tesla by 1e-6 and back by 1e6. Configuration rejects uT/nT
where T is required; input names explicitly require Tesla, with no magnitude-based
unit guessing. C_SB must be a proper rotation, cross-axis diagonal must be zero,
and gains positive. Nonidentity signed axes, arbitrary sensor bases, inverse
reconstruction and cross-axis/bias ordering are independently tested.

The caller-supplied B_local,B is the insertion point for future
B_environment + B_spacecraft + B_coil_residual at one field epoch. The present
live bridge supplies environment only and records that limitation. No residual
dipole, contamination amplitude, current decay, ADC, aperture or filter is added.
Such future fields can enter before mounting/calibration without redesigning the
value model, but require evidence and separate implementation authorization.

TAMConfig parameters carry value, units, frame, status, source/date and treatment;
all are ASSUMED / TEST-ONLY, source tam_sensor_model.py, revision 2026-10-02.
runtime_usable_for_flight is required false. Profiles are IDEAL_REGRESSION,
TEST_BIAS_ONLY ([1,-2,3] microtesla), TEST_SCALE_ONLY ([1.01,0.98,1.03]), and
TEST_NOISE_ONLY (discrete sigma [0.1,0.2,0.3] microtesla, PCG64 seed 8201).
Perturbations are unit-test inputs, not installed specifications. No vendor
density-to-sigma conversion is made. Zero noise creates no RNG; seeded draws occur
once per valid numeric acquisition, even if cleanliness makes that value ineligible.
Duplicate acquisition epochs are rejected. A fingerprint binds the full config.

### Separate acquisition eligibility and later usability

TAMSample carries S measurement, reconstructed B measurement, truth/acquisition/
publication epochs, noise realization, saturation axes, validity/rejection reasons,
configuration fingerprint, field source and CleanlinessContext. Finite values are
retained when ineligible; clipping retains the clipped vector but flags invalid.
Nonfinite/unavailable model values are None, not fabricated zero fields.
The legacy bridge's missing/native-invalid exception handling remains an explicit
invalid input; it does not substitute accepted truth for a failed native witness.

CleanlinessContext carries phase/cycle identity, commanded actuator-axis dipoles,
effective B-frame dipole, optional coil-current evidence, quiet history and required
history, context epoch/source and cycle fingerprint. Any nonzero command/effective
dipole or supplied current rejects clean acquisition. Missing/stale metadata,
non-SAMPLE/ineligible phase, insufficient quiet history, or unestablished evidence
also reject it. The implementation supports **TEST_ONLY_SCHEDULER** eligibility;
it does not interpret measured zero current alone as physical magnetic settling.
Current is None in live records: no measured coil current is available.

The current 0.4 s requirement comes from unchanged Phase 6A diagnostic bookkeeping,
not invented flight settling. The existing predicate still requires matching
TAM/WMM/acquisition epochs, actual zero native command/effective dipole/torque,
valid sample event and matching frozen sensor readback. Unknown physical settling
remains TBD even when a sample is eligible under this simulated contract.

At acquisition, valid means eligible under that context. usable_at separately
checks publication availability, caller-specified maximum age and expected
fingerprint; no flight age threshold is chosen. Isolated model latency tests
preserve acquisition while shifting availability. Live model latency remains zero:
truth = acquisition = publication at 0.4, 1.4, 2.4 s. The existing bridge transport
delay yields actual MEKF processing at 0.4, 1.6, 2.6 s, without retimestamping
acquisition or adding a second delay. Scheduled delivery and actual processing
are cross-checked in the validator.

### Independent equivalence, rejection and preservation results

All numbers here are CONFIRMED software observations of this TEST-ONLY fixture,
source validate_tam_sensor_model.py / phase8b1_tam_model.json, revision 2026-10-02.
The native sensor independently executes against all 31 recorded WMM/state epochs;
the live comparison includes all three accepted cycle acquisitions. A second A/B
pair temporarily invalidates the 1.4 s cycle metadata only for the shadow bridge,
restoring it before subsequent control. This rejects a finite field at 1.6 s in
both cases, with the same reason and original acquisition time. No actuator or
sensor value is modified by that test hook.

| CHECK | RESULT |
|---|---|
| Isolated ideal axes/arbitrary fields | Exact equality; nonidentity mounting and units tests pass |
| 31-epoch WMM/native sensor value comparison | Max difference 1.016440e-20 T; numerical allowance 6.469456e-19 T |
| Three accepted live sensor vectors | Max difference 6.776264e-21 T; numerical allowance 6.467362e-19 T |
| Nominal maximum quaternion-component difference | 3.330669e-16 |
| Nominal maximum bias-component difference | 5.070441e-16 rad/s |
| Nominal maximum P-entry difference | 6.505213e-19 in numerical SI-state coordinates |
| Estimator numerical equivalence criterion | Existing TEST-ONLY 1e-12 absolute, not accuracy acceptance |
| Accepted/rejected sequence, epochs, event order, counts | Exact, both nominal and rejection A/B pairs |
| Nominal updates / replays / rejects | Magnetic 3, Sun 4; 2 replays; 0 rejects |
| Rejection fixture updates / replays / rejects | Magnetic 2, Sun 4; 1 replay; one magnetic_cycle_invalid |
| Disabled path versus read-only HEAD adapter | Input/estimator traces and host CSV exact |
| Host spacecraft/commands/applied torque | Byte-identical between HEAD, current default, ideal model and shadow rejection fixture |

Tiny ideal-mode numeric differences come from independent Python versus native
C++ field transformations; no time shifts, data substitution or fitted tolerance
were used. Finite rejected model values equal their nominal counterparts. Source
labels intentionally identify the new producer. The legacy VectorSample reason
string is meaningful only when valid=false; new TAMSample rejection_reasons is
empty for eligible samples.

Ten isolated TAM tests, five live TAM tests, 17 affected adapter regressions and
nine nominal-control regressions pass (41 tests). All 13 top-level validator
checks and both six-check A/B comparisons pass. Compilation and diff checks pass.
Nominal-control regressions emit the existing unwritten-NavAtt startup warnings;
validity-gated initialization tests pass. Magnetic-cycle code was not changed,
so its separate suite was not rerun. No gyro/long-orbit/perturbation-performance
campaign was run, and no MEKF Q/R or controller gains were changed.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_tam_*.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_attitude_mekf_adapter.py -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_mekf_closed_loop.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_tam_sensor_model.py --report basilisk_runner\output_data\phase8b1_tam_model.json
git diff --check
```

The JSON retains sourced profiles, cycle/configuration, field/context samples,
native/model estimator histories, rejection evidence, source hashes and host hashes;
native/model host CSVs are isolated ignored artifacts. Existing 8A-2C work is not
overwritten. The most limiting physical gap is installed coil-to-TAM field/sensor
recovery against an approved cleanliness criterion over the complete aperture/filter
history, together with mounting and hard/soft-iron calibration.

**Next smallest justified experiment:** one deterministic TEST_BIAS_ONLY TAM offset
through the shadow MEKF, with frame/sign and innovation/rejection checks, ideal
gyro/Sun and fixed Q/R. This needs a separately authorized narrow live opt-in while
retaining the actuator guard; it is not enabled or evaluated in 8B-1.

## Phase 8B-2A post-acquisition synthetic TAM bias - 2026-10-02

**SYNTHETIC TAM-BIAS ESTIMATOR RESPONSE / POST-ACQUISITION SHADOW DEVELOPMENT TEST /
COLD-START ACQUISITION LIMIT IDENTIFIED / NOT HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.**
Start checkpoint `60c428f`, initially clean. PASS applies only to the running
estimator after normal ideal acquisition. Cold-start acquisition with the same
offset remains blocked by the unchanged consistency gate. Sensor validity is not
estimator acquisition acceptance.

### Configuration and explicit transition

Reuse the unchanged 8B-1 TEST_BIAS_ONLY profile: sensor-frame S bias
[1,-2,3] microtesla = [1e-6,-2e-6,3e-6] T; identity C_SB (B to S), unit scale,
zero cross-axis error/noise, no clipping, zero model publication latency.
All are ASSUMED / TEST-ONLY, source tam_sensor_model.py revision 2026-10-02;
runtime_usable_for_flight=false. Configuration fingerprint:
`ea58a793c99beb1ad738a662748682e6e7a6c7355e38f53d3dde93f6d609acdb`.
No installed magnetic calibration or contamination value is inferred.

The Python-only ShadowOptions.tam_bias_test_only flag permits only that exact
profile with the existing ideal gyro/Sun and no initial attitude override.
Without the flag, the prior ideal-only guard remains. Noise/scale profiles,
renamed profiles, gyro-model combinations and off-grid boundaries are rejected.
The existing MEKF_DEVELOPMENT guard rejects both cold-start and delayed-bias
options; no modeled TAM owns an actuator or replaces native controller TAM.

ShadowOptions.tam_bias_enable_ns explicitly selects a TEST-ONLY acquisition
boundary. Zero retains the cold-start test. At 1,400,000,000 ns, the bridge uses
IDEAL_REGRESSION for the 0.4 s acquisition and TEST_BIAS_ONLY at 1.4/2.4 s.
Each sample retains its actual selected model fingerprint/source. The transition
does not initialize the estimator, alter a message timestamp, or change cycle
timing. The validator independently verifies successful ideal initialization
before the transition.

| EVENT | EPOCH / SOURCE |
|---|---|
| Initial ideal TAM/Sun acquisition and ordinary MEKF initialization | 0.4 s, existing diagnostic fixture |
| Explicit bias enable / first biased acquisition | 1.4 s, user-authorized TEST-ONLY experiment boundary |
| Subsequent biased acquisition | 2.4 s, unchanged cycle |
| Model publication | Same as each acquisition; unchanged zero availability latency |
| Biased measurement processing | 1.6 and 2.6 s, existing 0.2 s bridge delay/replay |

All estimator states before the first biased delivery match the ideal reference
exactly. Every delivered biased measurement equals its ideal S-frame vector plus
the declared offset exactly. Body reconstruction is independently checked by
scalar sensor-axis projections; canonical Tesla, epoch/reference, gyro and Sun
inputs are unchanged. No model-contract audit or noise campaign was repeated.

### Case A: cold-start acquisition boundary

The three cold-start TAM acquisitions all contain the finite, correctly biased
field and remain eligible under the diagnostic quiet contract. At 0.4 s,
independent pairwise dot products give body-pair cosine -0.582042714899338 versus
reference cosine -0.4951140437674714: disagreement 0.08692867113186659. The
unchanged acquisition_pair_tolerance is 1e-8 (dimensionless cosine difference,
ASSUMED / TEST-ONLY, config/attitude_mekf_test_only.json, Phase 7C 2026-09-07).
Rotation preserves this pairwise angle; acquisition therefore returns
inconsistent_acquisition_pair. Later delayed magnetic events cannot initialize
the existing current-pair acquisition path (two initial_pair_not_current rejects).
The estimator remains UNINITIALIZED, with no correction/covariance-response claim.
Undefined results are null, never zero errors or a healthy-filter assertion.

The original BLOCKED/nonzero discovery report is retained under
cold_start_discovery in the final JSON. Its original source hashes describe that
earlier observation. Current cold-start traces are also retained. The final report
explicitly declares cold_start_bias_acquisition_supported=false. Overall validator
success verifies this boundary and the separate post-acquisition response; it
does not validate biased cold-start capability.

### Case B: running-estimator response

Numbers below are CONFIRMED software observations of the three-second TEST-ONLY
fixture, source validate_tam_bias_response.py / phase8b2a_tam_bias.json, 2026-10-02.
No Q/R/P0, acquisition tolerance, controller gain, cycle/environment or plant
parameter was changed. Covariance values refer to numerical SI-state coordinates,
not calibrated confidence in installed hardware.

| METRIC | IDEAL_REGRESSION | POST_ACQUISITION_TAM_BIAS |
|---|---|---|
| Bias in S, microtesla | [0,0,0] | [1,-2,3], beginning at 1.4 s |
| Final principal attitude error, rad | 3.720384e-6 | 0.1188546133 |
| Maximum published error, rad | 1.369167e-5 | 0.1662205166 |
| Maximum witnessed error including pre-update states, rad | 1.594657e-5 | 0.1771549633 |
| Final estimated gyro bias in B, rad/s | [-1.790200e-5,1.085055e-5,5.648901e-7] | [-0.002107963,0.000109611,-0.004804438] |
| Maximum magnetic tangent innovation norm, dimensionless | 7.474803e-6 | 0.1644910032 |
| Maximum Sun tangent innovation norm, dimensionless | 1.566772e-5 | 0.1481721571 |
| Minimum covariance eigenvalue | 7.951391e-6 | 7.934908e-6 |
| Maximum covariance eigenvalue | 0.1011330799 | 0.1011330799 |
| Maximum covariance asymmetry | 0 | 0 |
| Estimator valid after acquisition / rejected events | true / 0 | true / 0 |
| Magnetic / Sun counts (including acquisition) | 3 / 4 | 3 / 4 |
| Replay count | 2 | 2 |

The final retained event history below distinguishes acquisition from processing.
All six actual update callbacks (five distinct events, including replay of a Sun
update) remain in the artifact; geometric/residual checks cover every callback.

| ACQUISITION / PROCESSING s | UPDATE | MEASURED-VECTOR RESIDUAL BEFORE -> AFTER, rad | TRUTH ATTITUDE ERROR BEFORE -> AFTER, rad |
|---|---|---|---|
| 1.4 / 1.6 | Biased magnetic | 0.165241963 -> 0.046711970 | 7.936841e-6 -> 0.135515785 |
| 1.8 / 1.8 | Ideal Sun | 0.148719771 -> 0.042033330 | 0.177154963 -> 0.105249147 |
| 2.4 / 2.6 | Biased magnetic | 0.071266091 -> 0.017266651 | 0.097960197 -> 0.136829080 |
| 2.5 / 2.6 | Ideal Sun, retained replay | 0.089537626 -> 0.047099471 | 0.138836687 -> 0.117945965 |

At the first biased update, direct normalization of B_ideal+bias changes direction
by 0.1652491694 rad. The recorded innovation, decoded from its tangent coordinates,
matches the independently projected vector difference. The known-bias shift
discrepancy is 3.47e-17; correction dot the introduced passive rotation axis is
positive (0.0203272549). No Kalman gain or state-update equations are duplicated.
Every magnetic/Sun correction follows its measured-vector descent direction and
reduces that residual. The first magnetic correction increases simultaneous ideal
Sun disagreement from 6.608617e-6 to 0.091394421 rad; later ideal Sun updates act
against it. Increased truth error under biased magnetic data is expected here.

The ideal gyro contains no injected rate bias, but the coupled estimator's bias
state moves at vector corrections. Maximum witnessed bias-estimate norm is
0.1440956746 rad/s, at the first biased magnetic update; the following Sun update
reverses much of that movement. Full bias histories and update increments are
retained. Bias holds exactly during propagation-only intervals, and published
posterior bias matches the last observed correction at each processing epoch.
These are inferred filter states under inconsistent absolute references, not
real gyro bias, a convergence claim, or acceptable flight performance.

Covariance remains finite, symmetric and positive definite, with nonnegative
diagonals. Bias covariance stays within unchanged P0 and attitude trace within
the existing zero-Q kinematic bound. This checks numerical health only; no
statistical calibration is claimed. Numerical geometry allowance is 256 machine
epsilons, not a newly selected innovation acceptance gate.

### Validity, preservation and verification

Bias changes value without changing quiet eligibility. A separate shadow-only
metadata rejection at 1.4 s retains the finite biased field and explicit
magnetic_cycle_invalid reason. Native TAM, the frozen controller acquisition,
SimpleNav, command and applied-torque paths remain untouched.

Host CSV SHA256 is identical for committed 8B-1 ideal, current ideal, cold-start
bias, post-acquisition bias and the rejected-sample fixture:
`1b9c7c38321df3ea9c650a03e2a52e7571212d995a28a2d89de409646c1f6f1c`.
The current ideal estimator/input traces also match the read-only HEAD adapter.
Only the shared adapter's explicit TEST-ONLY opt-in/source selection changed.

Thirteen focused 8B-2A tests and 17 affected adapter regressions pass. All 24
validator checks pass, compilation passes, and git diff --check passes. Negative
tests reject wrong units/sign/double bias, reversed correction evidence and a
negative covariance witness. No unrelated suite, long run, noise or CSS campaign.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_tam_bias_response.py -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_attitude_mekf_adapter.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_tam_bias_response.py --report basilisk_runner\output_data\phase8b2a_tam_bias.json
git diff --check
```

**Next:** narrowly characterize acquisition-consistency robustness before a TAM
noise-only response study. Acquisition currently assumes essentially ideal pair
geometry and can prevent an otherwise finite sensor stream from producing any
navigation state. Characterize that boundary without tuning the gate or claiming
an evidence-based tolerance. Installed calibration, contamination/recovery,
realistic vector errors and requirements-grade estimator performance remain blocked.

## Phase 8B-2B cold-start acquisition-consistency boundary - 2026-10-02

**COLD-START ACQUISITION-CONSISTENCY BOUNDARY / DEVELOPMENT CHARACTERIZATION /
NO TUNING PERFORMED / NOT FLIGHT VALIDATED.** PASS for deterministic geometric
characterization at the saved 0.4 s acquisition. Checkpoint `e78427c`, initially
clean. No runtime, model, tolerance, Q/R/P0, timing or controller code changed.
No post-acquisition state/performance analysis was performed in this phase.

### Exact current predicate

For valid, current, common-epoch magnetic/Sun samples, let unit(v)=v/norm(v):

```text
m_B = unit(C_SB.T measured_magnetic_S)  # S to B; omit transform for explicit B input
s_B = unit(measured_Sun_B)
m_N = unit(magnetic_reference_N)
s_N = unit(Sun_reference_N)
D   = abs(dot(m_B,s_B) - dot(m_N,s_N))

if D > acquisition_pair_tolerance: reject inconsistent_acquisition_pair
otherwise: attempt TRIAD acquisition
```

The unchanged tolerance is **1e-8 in dimensionless cosine difference**, not radians
or a magnetic-field tolerance. Equality passes this consistency test. Acceptance
also requires finite, nonzero vectors, explicit proper mounting transforms and
valid epoch/source metadata. Invalid sensor flags are rejected independently.
The adapter's cold-start path requires a current common-epoch pair; the core does
not propagate an arbitrary uninitialized attitude to repair missing acquisition.

After consistency passes, TRIAD independently checks both measured and reference
pair cross-product norms. A sine **<=1e-6** rejects acquisition_geometry; strictly
greater passes this conditioning check. A zero vector rejects with finite nonzero
vector required. A measured collinear pair inconsistent with a noncollinear
reference fails the cosine test first. A separate algebraic test with consistent
collinear measured/reference pairs reaches the later geometry rejection.

Both 1e-8 and 1e-6 are ASSUMED / TEST-ONLY, source
config/attitude_mekf_test_only.json, Phase 7C 2026-09-07. Exact implementation is
VectorSample.body_and_reference, MEKF.observe and triad in attitude_mekf.py.
Neither gate was changed, and this phase does not select a flight criterion.

### Exact saved geometry and experiment boundary

Reuse the passing, source-bound phase8b2a_tam_bias.json acquisition inputs; all 16
source hashes still match. The validator replays only that pair through fresh
MEKFNavigationAdapter/MEKF instances, never a plant or actuator. The Sun, gyro,
reference vectors, timing and quiet evidence are copied unchanged. Deterministic
offsets use the existing isolated TAM model with a harness-only ASSUMED Parameter
in S; no live profile is added and runtime_usable_for_flight=false.

Numerical observations in this section are CONFIRMED software results, source
validate_acquisition_boundary.py / phase8b2b_acquisition_boundary.json, 2026-10-02.
They are local to this TEST-ONLY acquisition geometry and are not flight limits.

| QUANTITY | VALUE / FRAME / UNITS |
|---|---|
| Acquisition / publication | 0.4 / 0.4 s; copied from the existing diagnostic cycle |
| Ideal magnetic vector in B | [-4.15595761375e-6,9.47344529923e-6,2.02401152896e-5] T |
| Local magnetic magnitude | 2.27306052815e-5 T = 22.7306052815 microtesla |
| Magnetic reference in N | [-6.78840641155e-6,2.11305859807e-6,2.15901120473e-5] T |
| Sun unit vector in B | [0.327791540324,0.568288377914,-0.754719170036] |
| Sun reference in N before normalization | [0.3,0.8,-0.5], existing synthetic direction |
| B/Sun included angle | 2.08876241514 rad |
| Included-angle cosine / sine | -0.495114043767 / 0.868827994291 |
| Mounting | Existing identity C_SB, B to S; no installed mapping inferred |
| Original [1,-2,3] microtesla offset in S | D = 0.0869286711318666; sensor valid; acquisition REJECT |

### Basis and angular interpretation

Construct e_parallel=unit(B), e_cross=unit(e_parallel cross unit(Sun)), and
e_in_plane=unit(e_cross cross e_parallel). This right-handed orthonormal basis is
well conditioned in the saved geometry; e_in_plane points toward the Sun's
projection perpendicular to B.

| BASIS DIRECTION IN B | COMPONENTS |
|---|---|
| e_parallel | [-0.182835325425,0.416770481116,0.890434506208] |
| e_in_plane | [0.273088809946,0.891588785390,-0.361236680930] |
| e_cross | [-0.944454205168,0.177120873522,-0.276829280431] |

Let g=norm(B), c=cos(theta), k=sin(theta), and bias/g=a*e_parallel+b*e_in_plane+d*e_cross.
Then, provided the perturbed vector is nonzero:

```text
c_measured = (c*(1+a) + k*b) / sqrt((1+a)^2+b^2+d^2)
D          = abs(c_measured - c_reference)
delta_B    = atan2(hypot(b,d),1+a)
```

These expressions reproduce the independently calculated metrics and field-angle
changes to roundoff. D is an included-angle cosine discrepancy. It is not delta_B.
Locally, a change in included angle gives D approximately k*abs(delta_theta), but
magnetic direction can change without the same included-angle change.

For pure in-plane perturbation beta=b, c_measured=(c+k*beta)/sqrt(1+beta^2)
and delta_B=atan(abs(beta)). The first-order sensitivity is k/g per Tesla, the
largest local sensitivity for a fixed small bias magnitude. For cross-plane beta=d,
c_measured=c/sqrt(1+beta^2): sensitivity begins at second order,
D approximately abs(c)*beta^2/2. Parallel magnitude perturbations preserve direction
while 1+a>0 and therefore cancel under normalization.

Actual parallel tests with a=[-0.9,-0.5,0.1,1,10] all accept: measured magnitude
spans 0.1 to 11 times the original, with cosine discrepancy at roundoff. At a=-1
the field vanishes and normalization rejects; at a=-2 polarity reverses and the
consistency test rejects. These finite synthetic values retain sensor validity.
The magnitude-only statement therefore excludes zero field and polarity reversal.

### Measured accept/reject brackets

Positive and negative perpendicular grids use fractions
[1e-9,1e-8,1e-7,1e-5,1e-4,1e-3] of the local field. Deterministic bisection finds
the first local departure from the zero-bias accepted region on each ray. Endpoints
below are the largest tested accepted and smallest tested rejected magnitudes,
not a claim about the last representable floating-point value.

| DIRECTION | ACCEPTED MAGNITUDE T | REJECTED MAGNITUDE T | B-FRACTION BRACKET | B-ANGLE BRACKET rad |
|---|---|---|---|---|
| +in-plane | 2.61623373109e-13 | 2.61624019152e-13 | [1.15097407161e-8,1.15097691378e-8] | [1.15097406787e-8,1.15097691208e-8] |
| -in-plane | 2.61623373109e-13 | 2.61624019152e-13 | [1.15097407161e-8,1.15097691378e-8] | [1.15097406787e-8,1.15097690735e-8] |
| +cross-plane | 4.56849481484e-9 | 4.56850010722e-9 | [2.00984301046e-4,2.00984533876e-4] | [2.00984298339e-4,2.00984531170e-4] |
| -cross-plane | 4.56849481484e-9 | 4.56850010722e-9 | [2.00984301046e-4,2.00984533876e-4] | [2.00984298339e-4,2.00984531170e-4] |
| Scaled original direction | 3.77402305205e-13 | 3.77403155959e-13 | [1.66032668524e-8,1.66033042801e-8] | [1.48909418796e-8,1.48909754512e-8] |

| DIRECTION | D AT ACCEPTED ENDPOINT | D AT REJECTED ENDPOINT |
|---|---|---|
| +in-plane | 9.99998497894e-9 | 1.00000096748e-8 |
| -in-plane | 9.99998484436e-9 | 1.00000095403e-8 |
| +cross-plane | 9.99998868591e-9 | 1.00000118921e-8 |
| -cross-plane | 9.99998872676e-9 | 1.00000118586e-8 |
| Scaled original direction | 9.99999110712e-9 | 1.00000136618e-8 |

For lambda*[1,-2,3] microtesla, lambda is accepted at 1.00865008790e-7 and rejected
at 1.00865236163e-7. Lambda=1 remains rejected. The most sensitive in-plane
threshold is about 0.261624 picotesla, versus about 4.56850 nanotesla cross-plane.
The nearly equal positive/negative brackets do not resolve higher-order in-plane
asymmetry; exact cross-plane geometry is even in beta.

### Numerical stability, validity and verification

Independent normalization/dot products use 70-digit Decimal arithmetic on the
exact input floats, without calling estimator predicate/math helpers. All 237
tested grid/bisection points agree with actual adapter decisions. Both endpoints
of all five boundaries repeat with identical decisions for fresh runs, reversed
same-epoch delivery order and the original pre-acquisition idle history. Repeated
measurement generation and high-precision calculations are exact.

Maximum observed binary64 versus independent metric difference is 2.220446e-16.
Bisection stops at a separately declared arithmetic band of 16 machine epsilons
(3.552714e-15); it does not alter the 1e-8 gate. The final midpoint is explicitly
geometry-only. Reported bracket widths are 6.460427e-19 T (in-plane), 5.292381e-15 T
(cross-plane) and 8.507544e-19 T (original direction), about 1.16 to 2.47 parts per
million relative. Finer last-bit transitions are not claimed; tested endpoints
remain outside the arithmetic band. No NaN or ill-conditioned original geometry
occurs. The zero-vector edge is deliberately separate.

Every finite synthetic measurement retains the original quiet eligibility and
its value after estimator rejection. Both existing TEST-ONLY quiet evidence and
its configuration fingerprint are preserved. This establishes sensor-validity /
acquisition-acceptance separation, not physical magnetic cleanliness.

Eleven focused tests and all 14 validator checks pass; compilation and whitespace
checks pass. One initial test assertion incorrectly demanded exact zero after
70-digit normalization (observed 1e-70); its numerical assertion was corrected,
with no predicate/model change. Shared adapter code is untouched, so its regression
suite was not rerun. No prior phase, long run or estimator-performance study repeated.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p test_acquisition_boundary.py -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_acquisition_boundary.py --report basilisk_runner\output_data\phase8b2b_acquisition_boundary.json
git diff --check
```

**Next:** an evidence-based acquisition-design phase. The current predicate is an
ideal-pair consistency fixture with strongly direction-dependent sensitivity; this
characterization alone cannot choose a flight tolerance. Define calibration/error,
timing, geometry/observability and acceptance requirements first, retaining these
tests as the unchanged baseline. Installed TAM/Sun errors, mounting, contamination,
recovery and approved acquisition success criteria remain unresolved. No tuning is
authorized or performed by this recommendation.

## Phase 8B-2C acquisition criterion design basis - 2026-10-02

**ACQUISITION CRITERION DESIGN BASIS / NO THRESHOLD CHANGE /
NO FLIGHT CRITERION SELECTED.** Design-only continuation at committed `cfcf724`;
no runtime, acquisition logic, Q/R/P0, sensor profile or controller edits.
The complete derivation, evidence matrix and comparison are in
[ACQUISITION_CRITERION_DESIGN.md](ACQUISITION_CRITERION_DESIGN.md).

The unchanged predicate is the absolute measured/reference normalized-pair
cosine difference D <=1e-8, followed by both pair cross norms >1e-6, with separate
finite/nonzero/frame/epoch/sensor-validity prerequisites. Git traces the tolerance
to `1b79a50` (2026-09-14), whose Phase 7C source revision is 2026-09-07:
ASSUMED / TEST-ONLY ideal-pair consistency. No installed-error allocation or
quantitative flight rationale supports this number.

For active vector-error rotations in B, `delta_m=delta_phi_m cross m` and
`delta_s=delta_phi_s cross s`, the signed cosine perturbation is
`delta_c=(delta_phi_m-delta_phi_s) dot (m cross s)`. Reference cosine error is
subtracted. The component of relative rotation normal to the B/Sun plane enters
at first order with coefficient sin(theta); out-of-plane vector errors enter
at second order locally. Using signed `r=c_measured-c_reference` and `D=abs(r)`,
a future uncertainty calculation can use `sigma_r^2=J*P_joint*J^T`, including
both sensor and reference errors and
correlations. No covariance, sigma, quantile or threshold is selected here.

**Common-rotation blind spot:** `(R*m) dot (R*s)=m dot s` exactly. A consistently
wrong frame, mounting rotation or suitable common attitude-age error can pass
pair consistency while attitude is wrong. Scalar normalization or an angle
residual cannot repair this. Sensor validity, pair consistency and initialized
attitude observability/accuracy need distinct evidence. Near parallel or
antiparallel geometry loses one attitude degree of freedom; the existing sine
floor is a numerical fixture, not a flight quality criterion.

At the reused 0.4 s geometry, sin(theta)=0.868827994292. Illustrative deterministic
1e-4 rad in-plane error in either sensor yields D=8.68852748546e-5; both toward
each other yield 1.73775499981e-4. A common rotation leaves D zero to roundoff.
These are ASSUMED / TEST-ONLY examples, not HS-2 accuracy or acceptance bounds.

The isolated [analysis helper](../basilisk_runner/analyze_acquisition_criterion.py)
passes 11 focused math/policy checks without runtime imports, simulation or output
files. Checks include sign derivatives for both measured and reference vectors,
cross-plane terms, common rotations, correlated uncertainty and information-matrix
rank loss. The design document records exact commands and precision limits.

**Next:** a CSS/Sun-vector measurement model framework. A credible numeric flight
criterion needs Sun-direction uncertainty/availability (or equivalent installed
end-to-end evidence), plus TAM calibration/cleanliness, reference/timing errors,
observability requirements and false-accept/reject policy. A framework alone is
not calibration. Existing sources cannot establish quantitative TAM-versus-Sun
dominance; magnetic contamination/recovery remains independently unresolved.

## Phase 8C-1 parametric CSS / Sun-vector measurement framework - 2026-10-02

**CSS / SUN-VECTOR MEASUREMENT FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL /
NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.** Starting HEAD is
`cfcf724`; the completed, uncommitted 8B-2C design changes are preserved. This
phase adds a measurement framework and ideal shadow integration, not acquisition
tuning, a selected flight array or demonstrated HS-2 attitude accuracy.

### Truth, channel and reconstruction contract

[sun_sensor_model.py](../basilisk_runner/sun_sensor_model.py) separates immutable
SunTruth, ChannelMeasurement, SunReconstruction and SunSample records. The model
has no Basilisk, spacecraft or actuator dependency. Configurations use the existing
provenance-bearing Parameter type; every supplied profile is ASSUMED / TEST-ONLY,
source sun_sensor_model.py, revision 2026-10-02, runtime_usable_for_flight=false.
Profiles are defined in Python, following the gyro/TAM pattern; no duplicate JSON
configuration or flight profile is introduced.

| LAYER / QUANTITY | FRAME, UNITS, SIGN AND CONTRACT |
|---|---|
| Inertial Sun reference | Existing Phase 7C synthetic [0.3,0.8,-0.5], dimensionless, directed from spacecraft toward Sun in N. Not an ephemeris, irradiance or distance. |
| Sun truth | C_BN maps N to B. Store original reference and normalized N/B directions at truth epoch. Requires a finite nonzero vector and proper rotation; invalid inputs remain explicitly invalid. |
| Channel geometry | Unit normal in each S_i and proper C_SB mapping B to S_i. Thus normal_B=C_SB.T*normal_S. No installed orientation or count is implied. |
| Incidence / eligibility | q_i=normal_B dot unit_sun_B; positive incidence is illuminated. Front-side q>0 and q>=cos(half_FOV) determine generic eligibility, with 16 machine epsilons of FOV arithmetic allowance. FOV boundary is inclusive; exactly perpendicular is not illuminated. This is not a physical signal/shadow threshold. |
| Channel measurement | Dimensionless normalized response: y_i=gain_i*max(0,q_i)+offset_i+noise_i for eligible, unoccluded channels. Geometrically excluded/occluded channels report dark offset/noise and remain invalid; unavailable Sun or bad truth/metadata gives no response. This is a generic cosine development law, not a TensorCSS transfer function or voltage calibration. |
| Calibration | Forward gain/offset and reconstruction-known gain/offset are separate parameters. Corrected response is (y_i-calibration_offset_i)/calibration_gain_i. Their mismatch can produce directional error. |
| Optional effects | Seeded PCG64 independent discrete channel noise; optional upper clipping. Defaults zero/disabled, no noise-density conversion. Saturated readings are retained but excluded from reconstruction. No lower clipping, ADC, temperature, albedo or self-occlusion model is inferred. |
| Reconstruction | Valid calibrated responses b and corresponding normal_B rows A only. Minimize the Euclidean norm of A*x-b by linear least squares, then normalize x. No truth vector or spacecraft attitude enters this method. Output direction is dimensionless in B. |
| Reconstruction validity | At least three independent usable rows, rank 3, singular-value ratio above configured numerical floor, finite nonzero solution. This establishes a structurally available estimate, not an accuracy/confidence bound or a flight consistency decision. |
| Invalid data | Channel validity and reconstruction validity are separate. Invalid channels are excluded; malformed metadata or nonfinite purportedly valid values reject. No illuminated/usable channels, rank loss, ill conditioning or zero solution yields None plus reasons, never a valid zero vector. |

Array eligibility currently uses truth incidence/FOV and an explicit mask as
development-model inputs. The illuminated flag means geometric q>0; physical
availability and occlusion are recorded separately. No flight channel-selection algorithm
or measured signal-validity threshold is claimed. The unconstrained linear fit
does not certify a noisy solution's physical consistency or supply covariance.
Future calibrated selection/error policy remains required.

### Profiles and synthetic geometry

| PROFILE / PARAMETER | VALUE / UNITS / STATUS / LIMIT |
|---|---|
| IDEAL_REGRESSION | DIRECT_VECTOR mode, no channels, zero error, zero latency. Explicit development truth bridge; NOT a physical CSS reconstruction. |
| Direct-vector representation | Preserve C_BN*[0.3,0.8,-0.5] exactly as the old MEKF measurement payload, including its non-unit magnitude. Store normalized direction separately; MEKF performs its unchanged normalization. |
| TEST_ARRAY_GEOMETRY | Eight unit cube-corner normals (+/-1,+/-1,+/-1)/sqrt(3), identity C_SB; synthetic algebra geometry only, NOT an HS-2 count/layout. |
| Generic channel defaults | Forward and calibration gains 1; offsets and discrete noise sigma 0; half FOV pi/2 rad; upper clipping None. ASSUMED / TEST-ONLY. This hemisphere is not the vendor TensorCSS FOV. |
| Reconstruction numerical guard | minimum_singular_ratio=1e-10, dimensionless ASSUMED / TEST-ONLY numerical floor. Not a flight observability/accuracy requirement. |
| Timing and RNG defaults | Latency 0 ns; seed None; no random draws when noise is zero. Tests use explicitly sourced hypothetical 1e-4 response sigma / seed 8301 solely for reproducibility checks. |

The array recovers the six signed axis directions and two arbitrary directions
with maximum angular discrepancy **2.702860796744808e-16 rad** in the isolated
validator. Arbitrarily rotated arrays/nonidentity attitude also pass focused
tests. This is numerical algebra accuracy, not sensor accuracy. The [1,1,0]
symmetry edge leaves two positive independent rows and rejects rank deficiency;
valid individual channels are not enough. Uniform positive calibrated-response scaling leaves
direction unchanged. A synthetic differential gain/offset changes direction;
matching reconstruction calibration removes that known error to roundoff.

### Availability, epochs and provenance

SunAvailability carries epoch, explicit available/unavailable state, source and
optional channel-occlusion mask/reason. Unavailable Sun (for example an externally
supplied TEST-ONLY eclipse state) remains distinct from a visible Sun with invalid
or obscured channels. Truth direction can remain defined even when no measurement
is available. There is no new eclipse, irradiance, albedo or body-shadow model.

| EPOCH / PROVENANCE | MEANING |
|---|---|
| Truth / acquisition | Integer nonnegative simulation ns. Must match; stale truth or availability metadata rejects. Repeated acquisition epochs raise rather than consume duplicate random draws. |
| Channel publication | acquisition_ns + configured latency_ns. Isolated latency is a declared availability model, not a measured transport delay. |
| Reconstruction | reconstruction_ns=publication_ns; no separate computational latency modeled. The functional model can calculate a future-available record immediately; usable_at enforces availability. |
| MEKF processing | Actual bridge delivery tick, logged separately. Existing bridge transport delay preserves original acquisition/reference epochs; it never relabels old data as newly acquired. |
| Fingerprint / source | Deterministic SHA-256 over full config/provenance; per-channel/sample fingerprint, truth/availability source, channel identities and original epochs retained. Reconstruction checks channel metadata against its own config. |

### Live ideal shadow integration and authority boundary

Opt in through `ShadowOptions(ideal_sun=True, sun_model=SunConfig())` with the
existing ideal gyro and native TAM. The exact unchanged IDEAL_REGRESSION config
is the only live Sun-model selection accepted; arrays, renamed arrays, error or
latency changes stay isolated. `sun_model=None` retains the existing bridge.
No CLI default, Sun cadence, acquisition gate, estimator math or Q/R/P0 changes.

Task order remains spacecraft/current environment/sensors/cycle acquisition,
then ideal input bridge priority 590, MEKF adapter 580, navigation observer 570.
The new producer runs inside the existing Sun acquisition branch. Current Sun
acquisitions remain offset 0.4 s, period 0.7 s, ASSUMED / TEST-ONLY. The short
validator uses existing 0.2 s bridge transport delay after 1 s to exercise replay:
Sun acquisitions 0.4/1.1/1.8/2.5 s process at 0.4/1.3/2.0/2.7 s. Model publication
and reconstruction remain at acquisition in this live ideal case.

The existing missing-Sun delivery fixture is kept separate from physical eclipse.
It retains a finite but invalid event as before, while explicit unavailable input
to the isolated model gives no inferred measurement. No false valid zero vector
is substituted. New model records appear only in `sun_model_samples` DataFrame
attributes; native spacecraft/actuator CSV values are unchanged.

The existing exact-options MEKF_DEVELOPMENT guard rejects **every** modeled Sun
configuration, including ideal direct mode. Native TAM/SimpleNav retain control
ownership. No Sun-model object receives an effector or production command channel.

### Verification and scope of the result

[validate_sun_sensor_model.py](../basilisk_runner/validate_sun_sensor_model.py)
uses three-second fixtures and loads committed scenario and adapter sources in
memory. No checkout, long simulation or production-output replacement. Separate
evidence: output_data/phase8c1_sun_model.json, including source hashes, configuration,
sample/estimator records and preservation hashes.

- 21 isolated model tests and six live/comparison tests pass (27 total).
- 41 existing MEKF/core-adapter regression tests pass.
- All 17 live validator checks pass. Nominal and rejected-delivery cases have
  exactly equal measured vectors, event ordering/epochs/validity, acquisition
  decisions, attitude, bias and covariance; all maximum numerical differences 0.
- The producer label intentionally changes. The comparator canonicalizes only
  those two exact known labels, including retained acquisition provenance; it
  detects unknown sources, altered vectors/epochs and missing events. An initial
  comparison failed on the retained source label only; no estimator/state change
  was needed to resolve it.
- Feature-disabled committed/current bridge inputs and source metadata match
  exactly. Short continuous-native and diagnostic-cycle production CSVs are
  byte-identical to HEAD. Cycled/ideal/rejected shadow-host SHA-256 remains
  1b9c7c38321df3ea9c650a03e2a52e7571212d995a28a2d89de409646c1f6f1c.

```powershell
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_sun*.py' -v
.\.venv\Scripts\python.exe -B -m unittest discover -s basilisk_runner -p 'test_attitude_mekf*.py' -v
.\.venv\Scripts\python.exe -B basilisk_runner\validate_sun_sensor_model.py --report basilisk_runner\output_data\phase8c1_sun_model.json
git diff --check
```

**Next:** isolated deterministic differential channel-calibration error versus
reconstructed Sun direction, across valid synthetic geometry, retaining failed
geometry cases. This is the smallest extension of the observed calibration
sensitivity; no live perturbed Sun/MEKF control, acquisition tuning or flight
error claim is justified. Installed CSS count/model/normals, per-channel response,
calibration, visibility and timing remain TBD/TBC.
