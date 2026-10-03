# Cold-start acquisition criterion design basis — Phase 8B-2C

**ACQUISITION CRITERION DESIGN BASIS / NO THRESHOLD CHANGE / NO FLIGHT CRITERION SELECTED**

Revision: 2026-10-02. Repository checkpoint: `cfcf724`, initially clean.
Scope: mathematical design and reuse of repository evidence only. No runtime,
gate, Q/R/P0, sensor-profile, controller, timing or environment change. No new
acquisition implementation, simulation campaign or flight-performance claim.

## Decision and evidence boundary

The future decision must distinguish **sensor validity**, **relative pair
consistency**, and **attitude observability/accuracy**. A scalar pair-angle check
cannot replace the other two. Both TAM and Sun directional uncertainties, their
correlations, reference errors and synchronization belong in the design basis.
A common rotation of both measured vectors is invisible to pair consistency,
even when it gives the wrong attitude.

**No numeric flight threshold can be justified from the reviewed evidence.**
The next software branch should establish a CSS/Sun-vector measurement model
framework, with unresolved inputs explicit. This is a missing measurement-path
contract, not evidence that Sun error numerically dominates TAM contamination.
Installed magnetic calibration/recovery remains independently required.

The source register is the existing [sensor/estimator evidence](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md)
and [magnetic evidence](MAGNETIC_CONTROL_EVIDENCE.md), not a new external audit.
Their source IDs/revisions below are retained; originals were not fetched again.
CONFIRMED identifies a documented fact, not installed performance. VENDOR PRIOR
identifies a documented component capability with installed applicability TBC.
TEST-ONLY means ASSUMED engineering status. TBD/TBC denotes missing evidence or
unresolved applicability/conflict. No source is promoted by modification date.

## Current predicate and provenance

In [attitude_mekf.py](../basilisk_runner/attitude_mekf.py),
`VectorSample.body_and_reference` reconstructs sensor-frame TAM through
`C_SB.T`, then normalizes it; `C_SB` maps B to S. `C_BN` maps inertial N to body B.
`MEKF.observe` evaluates:

```text
r = m_hat_B dot s_hat_B - m_hat_N dot s_hat_N
D = abs(r)
reject inconsistent_acquisition_pair if D > 1e-8
then TRIAD rejects acquisition_geometry if either pair cross norm <= 1e-6
```

Equality passes the cosine test; the sine test is strictly greater. Finite,
nonzero values, proper transforms, sensor validity, metadata and current
common-epoch acquisition remain separate prerequisites. The adapter does not
invent an attitude to propagate an uninitialized, delayed pair. Post-initialization
innovation handling is a separate design question.

Focused Git history (`git log -S acquisition_pair_tolerance` and `git log -G`
on the config/core) finds introduction in **`1b79a50`**, *Add noise-free MEKF
development prototype*, commit date **2026-09-14**. The introduced and current
[configuration](../basilisk_runner/config/attitude_mekf_test_only.json) both state:

- Source revision: Phase 7C mathematical/software fixtures, **2026-09-07**;
  engineering status ASSUMED, label TEST-ONLY, `flight_runtime_usable=false`.
- `1e-8`: dimensionless cosine difference; ideal-pair consistency, explicitly
  not an installed sensor threshold.
- `1e-6`: dimensionless minimum sine; test-only geometry rejection with no
  operational threshold selected.
- All numerical entries are fixture choices for conditioning/convergence/failure
  tests; the vector weight is not calibrated measurement covariance.

**Disposition: inherited numerical fixture / development tolerance, not
evidence-backed flight criterion.** Provenance is known; a quantitative rationale
for exactly `1e-8` is not present in the inspected introduction/configuration and
associated design records. No installed-sensor error budget supports it.
The source revision date and later Git commit date are distinct, not a conflict.

## Physical error inputs and what the residual sees

Mapping codes: **A** differential direction, **B** common rotation (invisible if
identical for both vectors), **C** positive magnitude-only scaling (removed by
normalization), **D** relative epoch/reference mismatch. These describe possible
mechanisms, not measured allocations. A zero vector, polarity reversal, saturation,
eclipse or invalid channel set cannot be treated as benign magnitude error.

### TAM

| ERROR INPUT / UNITS / FRAME | MAPPING INTO RESIDUAL | EVIDENCE CLASS AND EXISTING SOURCE / REVISION |
|---|---|---|
| Mounting/alignment residual, rad; S-to-B reconstruction | A for magnetic-specific misalignment; B for a shared erroneous body rotation | TBD/TBC installed transform. [F02](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#f02), ICD-ADCS-001 rev 5, 2026-09-01; lateral plate intent but no numeric surveyed C_SB. Identity is TEST-ONLY, TAM framework 2026-10-02. |
| Hard-iron offset, T in S or B with declared transform | A from transverse offset; radial component is C locally while direction/polarity remains unchanged | TBD installed residual. [P07](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p07), H4 hardware v7.0 (2023), compensation capability is VENDOR PRIOR. TEST-ONLY [1,-2,3] microtesla in S is an existing synthetic fixture, not calibration. |
| Soft-iron, relative scale and cross-axis error, dimensionless matrix / rad | Anisotropic or off-diagonal error is A, direction dependent; common positive scalar gain is C | VENDOR PRIOR cross-axis 0.05 deg, [P04](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p04), H4 section 2.3 Table 4 p6 (2023); installed compensation/residual matrix TBD. |
| Measurement noise, quantization and filter correlation, T; T/sqrt(Hz); s | Transverse noise is A; radial part C to first order. Filtering may also produce D | VENDOR PRIOR 14 nT/sqrt(Hz), 0.15 microtesla resolution, 200 Hz capability; P04/H4. Selected filtering, PSD convention, packet rate, discrete covariance and correlation TBD. No density-to-sigma conversion. |
| Spacecraft magnetic contamination, T at sensor location | A/C depending on local field direction; operational/temperature dependence can be systematic, not white noise | TBD magnitude/history, [P07](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p07) and [magnetometer validity](MAGNETIC_CONTROL_EVIDENCE.md#magnetometer-validity), evidence revision 2026-09-06/07. Static compensation does not prove dynamic cleanliness. |
| Torquer residual/recovery and filter memory, T and s | A/C plus D if the effective acquisition history includes contaminated field | CONFIRMED qualitative coil-off intent, ICD-ADCS-001 rev 5 sections 4.1.3/4.2.4. Physical settling/current/field/filter bounds TBD; existing 0.4 s off-history is TEST-ONLY, Phase 6A. [Coil-off evidence](MAGNETIC_CONTROL_EVIDENCE.md#coil-off-and-cycle-timing). |
| Acquisition/reference timing and reference-field mismatch, s; rad or T projected tangentially | D; can include B only in the special common-rotation case | CONFIRMED software epoch contract, 8B-2B at cfcf724; installed clock/aperture/filter age and field-reference uncertainty TBD. [Interface timing](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#interfaces-rates-and-measurement-epochs), ICD rev 5; [ADCS-MAG-006](ADCS_MAGNETIC_TEST_PLAN.md#adcs-mag-006-magnetometer-sampling-filtering-and-timestamp-characterization). |

### Sun-vector path

| ERROR INPUT / UNITS / FRAME | MAPPING INTO RESIDUAL | EVIDENCE CLASS AND EXISTING SOURCE / REVISION |
|---|---|---|
| CSS mounting/boresight and deployed geometry, rad; sensor normals in B | A for relative CSS/TAM misalignment; B for a shared body-frame error | TBD/TBC: [F03](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#f03), ICD rev 5, 2026-09-01 and unreleased B3 budget. Locations are not numeric calibrated normals. |
| Per-channel dark offset, gain, ADC and thermal calibration, V/counts; dimensionless; C | Generally A after array inversion; only a uniform gain with a homogeneous reconstruction can reduce to C | TBD installed coefficients; VENDOR PRIOR compensation capability/examples, [P09](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p09)/[P12](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p12), H5 v1.0.2c, 2025-08-06. Example curves are not unit calibrations. |
| Irradiance, response law, albedo and temperature, W/m^2; V; C | A through unequal response/calibration; ideal uniform irradiance may be C, not generally | VENDOR PRIOR analog irradiance/angle dependence, [P08](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p08), H5 sections 1.2/2/2.3, 2025-08-06. Installed response envelope TBD. |
| FOV edge/nonlinearity and channel validity, rad | A and loss of usable channels; can be invalid rather than a small error | VENDOR PRIOR TensorCSS full FOV 120 deg (+/-60), typical angle error 5 deg with no sigma/RMS definition, P08/H5. Not demonstrated array coverage/accuracy or applicable to the unresolved SLCD alternative. |
| Reconstruction geometry and algorithm, dimensionless direction / rad | A, including anisotropic/correlated errors and unobservable directions | TBD/TBC [S05](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#s05)/[S06](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#s06)/F03. Conflicting 2/3/6 TensorCSS-10 versus 4 SLCD61N8; ICD rev 5 intensity channels versus Sun-vector port leaves inversion undefined. B3/PDR/budget are unreleased or have unresolved applicability. |
| Shadow, occlusion, eclipse and stray illumination, illumination fraction; geometry in B/N | A for partial/unequal obscuration; loss of measurement for insufficient illumination | TBD installed masks/validity and reconstruction behavior, F03/P08/[P10](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#p10). No nonzero vector may be fabricated in eclipse; two-vector cold start needs a separate availability decision. |
| Noise, readout/ADC/filter correlation, V/counts; rad after reconstruction | A, possibly correlated between channels and with calibration/illumination; radial C only locally | TBD installed covariance, P09/P10; nominal 12-bit ADC interface is documented in S06, not an angular-noise distribution. Current ideal Sun is TEST-ONLY, Phase 7C config revision 2026-09-07. |
| Channel scan skew, aperture, vector/reference epoch, s | D; a common attitude-age rotation can be B and escape this test | TBD/TBC S06/interface timing, ICD rev 5. Manager rate does not establish simultaneous channels, exposure/filter midpoint or synchronized reconstructed vector. |

The sources establish error mechanisms and missing evidence, not their numerical
dominance. H4 is DS100-CR-70-R1 / footer 25-100-CR-70-R1, copyright 2023.
H5 is TensorCSS document v1.0.2c, product v1.0, revision 2025-08-06. Neither
establishes an installed HS-2 calibration. TensorCSS typical 5 deg is **not a
Gaussian sigma**, a reconstructed three-axis error, or a threshold recommendation.

## Joint small-angle derivation and signs

At one physical epoch let true unit directions in B be m and s, with
`c=m^T s=cos(theta)`, `k=norm(m cross s)=sin(theta)`. Ideal common-epoch reference
directions in N have the same c under the proper rotation C_BN. For `0<theta<pi`,
let `n=(m cross s)/k`. Use **active vector-error rotations**, in radians:

```text
m_meas = m + delta_phi_m cross m + O(phi^2)
s_meas = s + delta_phi_s cross s + O(phi^2)
delta_c = (delta_phi_m cross m) dot s + m dot (delta_phi_s cross s)
        = delta_phi_m dot (m cross s) + delta_phi_s dot (s cross m)
        = (delta_phi_m - delta_phi_s) dot (m cross s)
        = k * n dot (delta_phi_m - delta_phi_s)
```

These are vector perturbations, not an assertion about MEKF's passive attitude
error-state sign. Rotations about each vector itself do not change that vector.
Only relative angular error **along n** contributes at first order. This rotation
moves a vector within the m/s plane. Rotational errors lying in that plane move
vectors out of plane and have zero first-order contribution to the dot product.

With reference cosine error `nu=delta_c_reference`:

```text
r approximately k*n dot (delta_phi_m - delta_phi_s) - nu
D = abs(r)
```

Linearize signed r, not the nondifferentiable absolute value at zero. A positive
active rotation of m about n moves it toward s and increases c; a positive
rotation of s about n moves it away from m and decreases c. The helper checks
all measured/reference rotation derivatives using finite Rodrigues rotations.

An equivalent tangent form makes out-of-plane effects explicit:

```text
t_m = (s-c*m)/k; t_s = (m-c*s)/k
u = alpha_m*t_m + beta_m*n; v = alpha_s*t_s + beta_s*n
m_meas = normalize(m+u); s_meas = normalize(s+v)
r = k*(alpha_m+alpha_s) + u dot v - c*(|u|^2+|v|^2)/2 - nu + O(error^3)
u dot v = -c*alpha_m*alpha_s + beta_m*beta_s
```

Here each positive alpha means *toward the other vector*, so the alphas add;
this agrees with the difference of active rotations above. Beta effects begin
at second order, including a TAM/Sun cross term. Tangent coefficients equal
angular errors only to first order. Pure radial perturbation cancels exactly
under positive scaling; zero and reversed directions are separate cases.

For a magnetic additive error e_B in Tesla, with `g=norm(B)`:
`delta_m=(I-m*m^T)*e_B/g`. Thus `delta_r=s^T*(I-m*m^T)*e_B/g`.
CSS channel errors must instead pass through the reconstruction Jacobian and
direction normalization; a per-channel voltage error is not an angular sigma.

Timing errors contribute via `delta_m approximately m_dot*delta_t_m` and the
corresponding Sun/reference terms, with a declared timestamp-error sign. Relative
age, channel skew, attitude motion and reference evolution all matter. A common
delay is invisible only to the extent that it produces the same rigid rotation
and preserves the reference included angle. Common delay is not generally safe:
orbital field evolution or different reference epochs can change that angle.

## Common-rotation blind spot

For any proper rotation R, `(R*m)^T*(R*s)=m^T*s` **exactly**, not just to first
order. If measured vectors are `R*C_BN*m_N` and `R*C_BN*s_N`, a two-vector
attitude solution can be `R*C_BN`: pair consistency passes while attitude is
wrong relative to the intended B frame.

Consequently D does not establish absolute alignment, absolute attitude accuracy,
common mounting/frame correctness or common attitude-age correctness. Magnitude
and some directional errors are also invisible; out-of-plane errors are locally
second order, not universally undetectable. Even two individually wrong vectors
with the correct included angle admit a common rotation of the reference pair.
Independent mounting/clock/frame checks and attitude-quality evidence are needed.
Changing from cosine to angle or normalizing the same residual cannot remove
this blind spot. A best-fit two-vector statistical test also cannot detect a
shared rotation without independent information.

## Geometry and observability

TRIAD normalizes `m cross s`; errors in that normal are amplified roughly as
`1/k`. Both **near-parallel** and **near-antiparallel** pairs have small k and
weak sensitivity of r to differential rotation, while rotation about their
nearly common line becomes poorly observable. At exact collinearity one attitude
degree of freedom is unobservable. A small D there is especially weak evidence.
Well-separated lines (near 90 deg, not merely a large included angle) give better
conditioning and larger first-order sensitivity.

For independent, isotropic small tangent-angle errors with inverse variances
w_m and w_s, the local attitude information matrix is

```text
I_att = w_m*(I-m*m^T) + w_s*(I-s*s^T)
eigenvalues = w_m+w_s,
 (w_m+w_s +/- sqrt((w_m-w_s)^2 + 4*w_m*w_s*c^2))/2
equal weights w: {2*w, w*(1+abs(c)), w*(1-abs(c))}
```

The smallest eigenvalue tends to zero at either collinear limit. With equal
weights its condition number is `2/(1-abs(c))`, approximately `4/k^2` near those
limits. A sine barely above the current TEST-ONLY `1e-6` can thus correspond to
conditioning of order `4e12`; that floor prevents a numerical singularity, not
an acceptable attitude error. Installed anisotropy/correlation changes this
simple matrix; the full measurement Jacobian/covariance is then required.

A future rule needs a separately justified observability/attitude-uncertainty
condition, over the acquisition envelope, as well as residual consistency.
Near collinearity the first-order scalar variance can vanish: do not interpret
that as perfect knowledge, divide by it, or silently invent a variance floor.

## Candidate forms — comparison, not a selected flight design

All options retain finite/validity/epoch/frame prerequisites and separate
observability checks. Complexity below concerns a future, separately authorized
implementation; the present gate remains unchanged.

| FORM / WHAT IT TESTS | STRENGTHS | FAILURE MODES / BLIND SPOTS | EVIDENCE REQUIRED | COMPLEXITY / CURRENT ARCHITECTURE COMPATIBILITY |
|---|---|---|---|---|
| Fixed absolute D: cosine discrepancy | Simple, bounded, no inverse trig; useful ideal-fixture regression | Same numeric limit allows very different angle errors with geometry; ignores sensor quality, correlation and common rotations | A supported worst-case error envelope over all admitted geometry and operating conditions; independent observability rule | Low; existing pre-TRIAD scalar predicate. Not sufficient alone for flight acquisition. |
| Angle difference: abs(theta_meas-theta_ref), preferably theta=atan2(cross norm,dot) | Direct included-angle units, more uniform in-plane interpretation away from collinearity | Still one scalar with common-rotation blind spot; near collinearity nonlinear/non-Gaussian angle errors, absent attitude observability | Joint relative-angle bounds/distribution, reference/timing errors, geometry envelope | Low; same four vectors and pre-TRIAD location; validity/conditioning still separate. |
| Uncertainty-normalized scalar: z=(r-mu_r)/sigma_r | Scales consistency by expected quality; can account for geometry and both sensors | Wrong uncertainty or bias assumptions give misleading acceptance; zero/poorly conditioned variance and outliers need explicit handling; common rotation invisible | Directional covariances/bounds, bias treatment and correlations at sample epoch, reference uncertainty, decision policy | Moderate; pre-TRIAD scalar calculation possible, but acquisition inputs need uncertainty/provenance metadata beyond a bare TEST-ONLY scalar. |
| Geometry-dependent bound: D_limit(theta, supported error envelope) | Can map deterministic angular bounds without claiming Gaussian statistics; explicit directional sensitivity | Geometry alone does not supply error magnitude or restore observability; permissive bound may accept bad pairs; nonlinear extrema near endpoints | Calibrated bounds per mode/illumination/cleanliness, joint angle envelope and robust reference-geometry uncertainty | Moderate; same geometric inputs plus supported envelope. Could use exact cosine interval for theta +/- bounded angle error (clipped to [0,pi]); not an arbitrary sin multiplier. |
| Covariance/statistical consistency: scalar NIS or joint weighted residual after best-fit attitude | Models anisotropy/correlation; pairs naturally with predicted attitude uncertainty; allows stated probabilistic false-reject design under verified assumptions | Non-Gaussian bias, wrong covariance, model mismatch and data-dependent selection invalidate nominal probabilities; coherent wrong pairs still fit | Full calibrated joint error model, systematic bounds, availability conditioning, accepted statistical/fault model and independent attitude-quality criteria | Moderate for scalar NIS, higher for weighted attitude fit/Jacobian and quality policy; may require acquisition-interface/initialization design changes beyond substituting one tolerance. |

These forms overlap: a calibrated geometry-dependent limit can be equivalent to
a normalized residual. None is universally best without the supported error
model and operational decision policy. A fixed D can remain a regression test or
a conservatively justified subcheck over a restricted envelope; it is not a
sufficient long-term acquisition criterion by itself.

## Symbolic uncertainty-aware design

Let `eta_m=n^T*delta_phi_m`, `eta_s=n^T*delta_phi_s` and `x=eta_m-eta_s`.
For small errors at nondegenerate geometry, signed `r approximately k*x-nu`.
Retain systematic mean and correlations:

```text
mu_r = k*(mu_eta_m-mu_eta_s) - mu_nu
sigma_r^2 = k^2*(var(eta_m)+var(eta_s)-2*cov(eta_m,eta_s))
            + var(nu) - 2*k*cov(x,nu)
z = (r-mu_r)/sigma_r                    [candidate only]
T = (r-mu_r)^2/sigma_r^2                [candidate only]
```

More generally stack active rotation errors in their declared coordinates:
`e=[phi_mB,phi_sB,phi_mN,phi_sN]`. At the ideal pair,
`J=[(m_B cross s_B)^T, -(m_B cross s_B)^T,
    -(m_N cross s_N)^T, (m_N cross s_N)^T]` and `sigma_r^2=J*P_joint*J^T`.
Cross-covariances must be represented in these specific B/N coordinates; they
cannot be mixed without transformations. Reference-field, Sun-reference and
timing uncertainty must be propagated into P_joint or treated as bounded terms.

If references are treated as exact and TAM/Sun errors are independent isotropic
small angles, then `sigma_r^2=k^2*(sigma_m^2+sigma_s^2)`. Here each sigma is the
**one-axis tangent angular standard deviation**, not total directional RMS.
For two isotropic tangent axes, directional RMS is sqrt(2) times that sigma.
This simplification is a conditional algebra result, not a supported HS-2 model.
Common-mode covariance can cancel in r even while contributing to attitude error.

Away from collinearity, the signed angle residual
`a=theta_meas-theta_ref approximately -r/k` has
`sigma_a^2 approximately sigma_r^2/k^2`. This division fails as a reliable local
description at degeneracy; use supported nonlinear error propagation and an
observability decision there.

Only under a justified locally linear, correctly centered Gaussian model with
known variance would T approximately follow chi-square with one degree of freedom.
The scalar redundancy is consistent with four tangent measurement components
minus three attitude unknowns for a noncollinear pair. This is not justification
for treating unknown bias/contamination as Gaussian. Bounded systematic errors
need explicit bounds or nuisance-parameter treatment. A false-reject quantile
under the good-sample model does not establish false-accept performance against
faults; fault hypotheses, common-mode blind spots and independent evidence remain.
No numerical sigma, quantile, bound or new acquisition gate is selected here.

## Saved 0.4 s geometry — illustration only

Reuse the [committed 8B-2B observation](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8b-2b-cold-start-acquisition-consistency-boundary---2026-10-02),
source `validate_acquisition_boundary.py`, 2026-10-02 at `cfcf724`: B magnitude
22.7306052815 microtesla, theta 2.08876241514 rad, c=-0.495114043767.
These are CONFIRMED software-fixture observations, not installed flight values.
Using the rounded cosine gives `k=0.8688279942915567` and
`theta=2.08876241513526 rad`; the last digits reflect rounding, not new simulation.

Thus `r approximately 0.868827994292*(eta_m-eta_s)-nu`. The maximal local
magnetic-only transverse sensitivity is `k/g approximately 0.038223 microtesla^-1`;
this is a directional derivative, not an allowed field-error budget.

**ILLUSTRATIVE ONLY / NOT HS-2 SENSOR PERFORMANCE.** Set a deterministic angular
offset `a=1e-4 rad` (about 0.00572958 deg). Status ASSUMED / TEST-ONLY; source
[analysis helper](../basilisk_runner/analyze_acquisition_criterion.py), revision
2026-10-02. References are ideal; no injected runtime noise or calibrated sigma.
Exact finite rotations/unit vectors give:

| HYPOTHETICAL PERTURBATION IN THE B/S PLANE BASIS | FIRST NONZERO APPROXIMATION TO D | FINITE-GEOMETRY D (dimensionless) |
|---|---|---|
| TAM rotates toward Sun by a; Sun unchanged | k*a | 8.68852748546e-5 |
| Sun rotates toward TAM by a; TAM unchanged | k*a | 8.68852748546e-5 |
| Both rotate toward each other by a | 2*k*a | 1.73775499981e-4 |
| Both undergo the same active rotation by a about n | Exactly zero | 5.55e-17 numerical roundoff |
| Only TAM tilts out of plane toward n by a | abs(c)*a^2/2 | 2.47557019328e-9 |
| Both tilt out of plane toward n by a | (1-c)*a^2 | 1.49511403258e-8 |

The last row is not a common rigid rotation; two independent tilts toward the
same normal change the included angle at second order. These results do not
recommend thresholds or repeat acquisition-boundary testing. The equal-weight
information eigenvalues divided by w are `[0.504885956233,1.495114043767,2]`:
the saved geometry is separated, but angular quality still depends on unknown
sensor errors and alignment, not these eigenvalues alone.

## Minimum evidence before selecting a number

### REQUIRED BEFORE NUMERIC FLIGHT THRESHOLD

1. **Applicable configuration and frame chain:** installed TAM/CSS identity,
   firmware/settings/channel map, deployment geometry, calibrated normals/C_SB,
   polarity, relative and common alignment residuals with units, revision and
   uncertainty. Resolve the CSS count/model conflict; a candidate framework may
   remain parametric but a flight threshold needs an applicable configuration.
2. **TAM calibrated residual envelope:** hard/soft-iron, scale/cross-axis and
   temperature residuals; local-field dependence; random noise, quantization,
   filtering/bandwidth and temporal correlation at the actual sample rate.
3. **Post-quiet cleanliness:** measured sensor-local field/current/recovery across
   relevant coil/polarity/power/temperature states and the complete filter/aperture
   history; approved valid/invalid and residual-error bounds. See
   [ADCS-MAG-005/006](ADCS_MAGNETIC_TEST_PLAN.md#proposed-tests). Commanded zero and
   elapsed off-time alone do not meet this evidence need.
4. **Sun-vector uncertainty:** CSS geometry, per-channel calibration and analog/ADC
   response/noise, temperature/irradiance/FOV effects, occlusion/eclipse/channel
   validity, reconstruction algorithm and its directional error/correlation over
   usable illumination. A calibrated end-to-end vector envelope can supply the
   needed uncertainty without a high-fidelity model of every internal effect.
5. **Timing and reference uncertainty:** common clock, aperture/filter midpoint,
   per-channel scan skew, delay/jitter, acquisition/reference epoch binding and
   motion-induced error; supported magnetic/Sun reference and navigation/model
   errors over the intended envelope. Delays must not be counted twice.
6. **Acquisition-quality definition:** admitted geometry/availability envelope,
   observability or initialized-attitude uncertainty requirement, required
   success/deferral behavior and treatment of eclipse/near-collinearity. The
   current minimum sine and P0 are not these requirements.
7. **Decision philosophy and validation:** allocated false rejection and unsafe
   false acceptance, time-to-acquire/retry policy, fault hypotheses and independent
   checks for shared frame/alignment/time faults; calibrated correlations/bounds
   and evidence for distribution assumptions, independently validated on cases
   covering the declared envelope. A fit to one nominal example is insufficient.

These items require applicable, reproducible calibration/test records and
accepted requirements, not just a list of error names. Existing sources
[V01](ATTITUDE_SENSOR_ESTIMATOR_EVIDENCE.md#v01), F02/F03/P07/P09 and the magnetic
test plan leave them open. This phase neither invents acceptance probabilities
nor authorizes hardware tests or criterion implementation.

### USEFUL BUT NOT STRICTLY REQUIRED

- Detailed device-internal physics when calibrated end-to-end error/timing bounds
  already cover the declared acquisition envelope.
- Large mission-wide Monte Carlo/long-duration detumble or pointing campaigns;
  targeted independent criterion/geometry verification is still required.
- An additional independent attitude sensor and extended HIL/flight correlation
  beyond the accepted acquisition verification scope. Independent truth suitable
  to establish calibration/accuracy remains required; no particular new flight
  sensor is mandated by this design basis.
- Aging and environmental extrapolation beyond the initially accepted envelope.
  Conditions inside that envelope are required, not deferred as optional.

## CSS dependency and next branch

**Can a defensible numeric criterion be chosen before CSS/Sun-vector modeling?**
Not from the current repository evidence. Sun error enters at the same first
order as TAM error; the present ideal Sun supplies no physical directional error
or availability model. TAM-only tuning would assume away half the measurement
problem and would not close common-mode errors.

What is indispensable is a defensible Sun-vector **error/availability model or
equivalent installed end-to-end calibration evidence**, not a particular software
implementation. Such evidence could justify a criterion before a detailed CSS
simulator existed. Neither is available here, and adding a TEST-ONLY framework
alone will not establish installed uncertainty or release a numeric threshold.

**Single recommended next phase: CSS/Sun-vector measurement model framework.**
Define per-channel illumination/response, mounting, reconstruction, validity,
acquisition/reference epochs and provenance; retain unresolved geometry and
calibration as explicit candidate/test-only inputs. Do not adopt 5 deg as sigma,
choose a hardware winner or tune acquisition. The reason is the missing Sun
measurement contract despite an existing parametric TAM framework. No reviewed
evidence quantifies which sensor dominates; TAM contamination/recovery remains
a prerequisite for a later flight criterion and must not be called resolved.

## Phase 8C-1 Sun measurement contract connection - 2026-10-02

The [parametric Sun framework](ATTITUDE_ESTIMATOR_ARCHITECTURE.md#phase-8c-1-parametric-css--sun-vector-measurement-framework---2026-10-02)
now separates truth, channel measurements, calibrated reconstruction and
availability/epoch provenance. IDEAL_REGRESSION still gives the previous direct
Sun input exactly; the synthetic array is isolated from live actuator control.
All supplied profiles remain ASSUMED / TEST-ONLY, not installed performance.

The architectural uncertainty path is now explicit:

```text
channel response/calibration errors + uncertain normals + visibility/timing
  -> calibrated least-squares solution x
  -> normalized Sun direction s_hat_B
  -> Sun directional error delta_phi_s
  -> relative TAM/Sun signed residual (delta_phi_m-delta_phi_s) dot (m cross s)
```

For a fixed, full-rank active channel set and small response perturbations,
`delta_s approximately (I-s*s^T)*A_pseudoinverse*delta_b/norm(x)`.
Here delta_b is calibrated response error; mounting/calibration-matrix uncertainty
adds its own terms, and changing FOV/occlusion/channel selection invalidates a
single fixed Jacobian across that boundary. Common-mode alignment/time errors
remain a blind spot of pair consistency. Reconstruction structural validity is
not a calibrated accuracy estimate or proof of acquisition acceptance.

No covariance, installed Sun uncertainty, acquisition threshold, Q/R/P0 or MEKF
equation is changed. The framework makes a later uncertainty study possible;
it does not close the required installed evidence listed above. The original
8B-2C derivation and provenance findings remain unchanged.

## Verification and preservation

The standalone helper imports no production/scenario/MEKF modules and writes no
files. It reads only the unchanged test policy for a preservation assertion.
Eleven focused checks cover all measured/reference derivative signs, joint
in-plane errors, second-order out-of-plane effects, a general tangent expansion,
exact common-rotation invariance, normalization/invalid vectors, correlated
uncertainty propagation against finite rotations, information eigenvalues/rank
loss, collinear sensitivity, the saved rounded geometry and unchanged policy.
Numerical assertion tolerances are TEST-ONLY analysis precision, not gate values.

```powershell
.\.venv\Scripts\python.exe -B basilisk_runner\analyze_acquisition_criterion.py
.\.venv\Scripts\python.exe -m compileall -q basilisk_runner
git diff --check
```

The focused checks pass (11/11), as do compilation and whitespace checks.
Documentation verification passes for 27 new local links/anchors and six new
tables; the five-file allowlist, unchanged tracked runtime/configuration and
unchanged index are verified. No previous characterization, adapter suite, full
simulation or performance study is needed for these isolated changes.
Existing runtime/configuration files are unchanged. Git's ordinary diff stat
omits new untracked files; report those separately without staging or committing.
