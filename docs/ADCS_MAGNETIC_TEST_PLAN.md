# HS-2 magnetic hardware-input and test plan — Recovery Phase 6C

Plan revision: 2026-09-06. Evidence baseline: `8053e464a0b322688a786f2187db0ed70224e02c` (committed Phase 6B).
**Planning only. All tests are NOT EXECUTED. No flight-candidate magnetic cycle is created.**

## Purpose and evidence boundary

Convert the [Phase 6B reconciliation](MAGNETIC_CONTROL_EVIDENCE.md) and [structured evidence register](../basilisk_runner/config/magnetic_control_evidence.json) into an actionable hardware/team input plan. Those sources retain the candidate values, statuses, original source IDs, revisions and contradictions. The [companion Phase 6C JSON](../basilisk_runner/config/adcs_magnetic_test_requirements.json) contains the same 39-item matrix, eight tests, 13 model-output definitions, timing distinctions, sweep rules and exit gates, with source-file hashes.

No new repository/Drive audit, web research or physical measurement is used. The plan does not change gains, profiles, native actuation, WMM/Earth orientation, sensor/estimator behavior, scheduling, pointing or outputs. Continuous regression remains default and Phase 6A timing remains TEST-ONLY.

On continuation, no Phase 6C files existed; the 39-row extraction and test definitions drafted in the conversation were carried forward. No completed extraction or audit was restarted.

All proposed test methods and output schemas are planning assumptions, not measured engineering values. Current engineering statuses are copied from Phase 6B; null means unsupported/TBD, never zero. Teams below are inferred owners requiring assignment confirmation, not contacted individuals. A procedure or tracker entry does not close a physical parameter.

## Current blockers and priority matrix

There are **39 mandatory items: 31 P0, 5 P1, 3 P2**, with no merges or dropped IDs. Selection is every Phase 6B row treated as BLOCKED PENDING HARDWARE INPUT or PARAMETRIC SWEEP, plus all directly relevant TBD/TBC rows (including NOT MODELED YET A10/E04). Eleven other Phase 6B rows are retained as context/anchors in the companion JSON.

| Priority | Definition |
|---|---|
| P0 | Blocks physically credible magnetic-control simulation. |
| P1 | Needed for realistic performance prediction, beyond basic architecture. |
| P2 | Later refinement/uncertainty reduction or evidence-record housekeeping; never a substitute for required measurements. |


P2 F01-F03 concern legacy record/traceability housekeeping: they do not authorize accepting missing physical measurements. The required measured packages remain P0 F05. Timing timestamps, electrical decay, magnetic settling, aperture and delays are separately defined below even where a source register row originally grouped them.

Each row retains its Phase 6B ID. Model-output links specify the minimum data/schema and exact future use; test links provide the resolution procedure. Detailed candidate objects and original source/revision references remain in the companion JSON and Phase 6B register.

### P0 items

| ID / PARAMETER | CURRENT STATUS / CANDIDATE | WHY IT MATTERS | EXACT EVIDENCE TO RESOLVE | LIKELY OWNER / EVIDENCE TYPE | MINIMUM MODEL OUTPUT / TEST |
|---|---|---|---|---|---|
| <a id="blocker-a02"></a>A02: Intended actuator models | TBC; ICD: 2 CR0002 +1 MT01A; MDD: 3 in-house cells. Units: model/count. | Actuator identity determines applicable torque/electrical physics. | Released installed BOM with part/variant/serial/PCB revisions; approved disposition of COTS versus in-house and MT01A versus MT01. | ADCS / Systems / procurement; document confirmation, schematic/BOM confirmation | [M01](#m01); [ADCS-MAG-001](#adcs-mag-001) |
| <a id="blocker-a03"></a>A03: Physical actuator axes, origin and polarity | TBD; Installed mapping/sign TBD; regression identity/X-Y rods/Z coil ASSUMED. Units: body unit vectors / rotation matrix. | Wrong axes or polarity can redirect/reverse torque. | Released body datum and as-built coil/sensor survey; positive-command/current/dipole sign and pin-channel records with uncertainty. | ADCS / Structures / CDH; document confirmation, schematic/BOM confirmation, bench measurement, calibration | [M01](#m01), [M02](#m02); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-002](#adcs-mag-002) |
| <a id="blocker-a05"></a>A05: Air-coil dipole and nominal/saturation voltage claims | TBC; MT01A >0.395 A m^2 at3.3 V; >0.790 at9 V versus heading1.25-5 V. Older MT01 >0.19 nominal/>0.85 saturation. Units: A m^2; V. | Variant-mixed dipole claims cannot define a physical clamp. | Confirm air-coil variant and permitted envelope; measure signed moment/current versus command and conditions; retain vendor lower bounds as lower bounds. | ADCS / supplier liaison; document confirmation, bench measurement, calibration, schematic/BOM confirmation | [M02](#m02), [M03](#m03); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-a06"></a>A06: Air-coil resistance | TBC; Claim4.1-4.7 ohm at25 degC; regression4.4 ohm ASSUMED. Units: ohm; degC. | Resistance affects current, heating and delivered moment. | Per-winding resistance with lead/contact correction, winding temperature and measurement uncertainty. | ADCS / EPS / Thermal; bench measurement, calibration | [M03](#m03); [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-a08"></a>A08: Air-coil voltage, current and power | TBC; ICD12 V,0.07-1.1 A,0.84-13.2 W; budget5 V/1.75 W; thermal1.75 W; H2 50-1500 mW and invalid mAh current label. Units: V; A; W; mW; mAh (invalid for current). | Rail/winding and peak/average ambiguity invalidates power/actuation limits. | Released measurement boundaries; synchronous bus/winding V/I; peak/steady/cycle power and limits; disposition of conflicting voltages, powers and variants. | ADCS / EPS / Thermal; document confirmation, schematic/BOM confirmation, bench measurement, calibration | [M03](#m03); [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-a09"></a>A09: Installed magnetic gain and dipole transfer function | TBD; Measured per-axis gain/m(I,T), Z gain TBD. Units: A m^2/A; function of current/temperature. | Actual command-current-dipole response sets control authority. | Independent vector dipole measurements versus signed command/current, temperature, history/background; held-out validation and uncertainty. | ADCS / magnetic calibration lead; bench measurement, calibration | [M02](#m02); [ADCS-MAG-002](#adcs-mag-002) |
| <a id="blocker-a12"></a>A12: Driver/H-bridge selection | TBC; B3 DRV8231; ICD voltage amplifiers TBR; released PCB/BOM absent. Units: model. | Driver identity/mode determines current path and OFF behavior. | Released driver schematic/BOM and register/pin truth table; confirm/reject DRV8231 claim against observed switching states. | ADCS / CDH / EPS; document confirmation, schematic/BOM confirmation, bench measurement | [M01](#m01), [M03](#m03); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-a13"></a>A13: Power rails and coil-drive boundary | TBC; Rails3.3/5/12 V; rod5 V; coil-driver boost12 V; PWM0-5 V description. Units: V. | Supply and logic labels cannot substitute for winding voltage. | Trace rail/connector/logic levels to driver/winding; simultaneous supply/winding measurements in declared modes. | EPS / ADCS / CDH; document confirmation, schematic/BOM confirmation, bench measurement | [M03](#m03); [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-a15"></a>A15: Thermal/continuous-duty envelope | TBC; Rod qualification -20..80 degC; coil operating claim -55..85 degC; thermal calculation100% duty; flight envelope TBD. Units: degC; %. | Usable duty/burst must fit a real operating envelope. | Signed safe electrical/temperature test limits; measured heating/steady response versus burst/duty with thermal boundary conditions; approved restricted envelope/extrapolation. | Thermal / EPS / ADCS; document confirmation, bench measurement, calibration | [M03](#m03); [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-b01"></a>B01: Flight control/manager rate | TBC; 5 Hz MDD/SW1 versus10 Hz ICD (0.2/0.1 s ticks). Units: Hz; s. | Manager rate alone does not establish acquisition/control timing. | Approved 5/10 Hz disposition, event mapping, firmware/config hash and measured cadence under representative load. | CDH / ADCS / Systems; document confirmation, timing measurement | [M09](#m09), [M10](#m10); [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-b03"></a>B03: IMU transport and configured serial rate | TBC; LVTTL UART; UART/RS-232 at115200 bit/s; I2C manager; MDD RS-422 TBR. Units: protocol; bit/s. | Conflicting sensor interfaces prevent reproducible acquisition. | Exact sensor variant, connector/transceiver/protocol settings; decoded packet/register captures and UART/I2C/RS-422 disposition. | CDH / ADCS; document confirmation, schematic/BOM confirmation, bench measurement, timing measurement | [M07](#m07), [M09](#m09); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-b05"></a>B05: Actual magnetometer acquisition/output settings | TBD; Configured raw rate, filters, packet/trigger/acquisition epoch TBD. Units: Hz; configuration; clock/epoch. | Packets may contain stale or contaminated acquisition history. | Register/filter/output readback and timed independent field stimulus establishing cadence, effective aperture/filter history and epoch/availability bounds. | ADCS / CDH / sensor liaison; document confirmation, bench measurement, calibration, timing measurement | [M07](#m07), [M09](#m09); [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-b06"></a>B06: Magnetic measurement software path | TBD; IMU table names rate/acceleration/time; BDot requires current/previous B; producer/B epoch unclosed. Units: port payload / epoch. | Unclosed current-field source/epoch invalidates freshness reasoning. | Flight topology/schema/code revision identifying B producer, units/frame/validity, acquisition epoch, previous/current sample IDs and actual consumption trace. | CDH / ADCS flight software; document confirmation, timing measurement | [M07](#m07), [M09](#m09), [M10](#m10); [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-b07"></a>B07: Flight scheduler jitter and end-to-end timing trace | TBD; Measured execution order, queue latency, jitter and period TBD. Units: s; execution order. | Latency and jitter change field/rate age at actuation. | Synchronized acquisition-availability-compute-command-driver-current trace with conditional joint delay bounds/distributions and clock uncertainty. | CDH / ADCS integration; timing measurement, integrated/HIL test | [M09](#m09), [M10](#m10); [ADCS-MAG-007](#adcs-mag-007), [ADCS-MAG-008](#adcs-mag-008) |
| <a id="blocker-c02"></a>C02: Required quiet period | TBD; Quiet duration TBD; diagnostic0.4 s zero-command history TEST-ONLY. Units: s. | Zero command history cannot establish a valid sample. | Approved magnetic-error allocation and earliest entire valid acquisition window after OFF over amplitude/history/temperature/geometry envelope. | ADCS / magnetic test lead / Systems; document confirmation, bench measurement, calibration, timing measurement | [M05](#m05), [M06](#m06), [M12](#m12); [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-c03"></a>C03: Current-decay time and off-state behavior | TBD; Current decay/threshold and off mode TBD. Units: s; A; brake/coast/recirculation state. | Current may persist after OFF depending on recirculation mode. | Command-off, driver-off and winding-current traces for supported modes/polarities; link any acceptable current threshold to magnetic validity. | ADCS / EPS / CDH; document confirmation, schematic/BOM confirmation, bench measurement, timing measurement | [M04](#m04), [M06](#m06), [M09](#m09); [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004) |
| <a id="blocker-c04"></a>C04: Magnetic settling time | TBD; Field settling/threshold TBD; RESET_WAIT_TICKS is reset recovery only. Units: s; T; ticks. | Residual field/filter recovery can outlast current decay. | Local vector field and flight-sensor recovery versus baseline with repeated histories and approved residual-field criterion; reset wait remains separate. | ADCS / magnetic test lead; document confirmation, bench measurement, calibration, timing measurement | [M05](#m05), [M06](#m06), [M12](#m12); [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-c05"></a>C05: Magnetometer sample aperture and phase | TBD; Physical aperture/filter window/phase/reference TBD. Units: s; epoch definition. | Output timestamp/rate does not specify a physical acquisition window. | Device-setting confirmation plus timed field stimuli identifying aperture/filter response and timestamp reference; retain bounds if acquisition is not directly observable. | ADCS / CDH / sensor liaison; document confirmation, bench measurement, calibration, timing measurement | [M07](#m07), [M09](#m09); [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-c06"></a>C06: Sample-to-compute delay | TBD; Flight sample-to-compute TBD; diagnostic0.1 s TEST-ONLY. Units: s. | Aging field/rate inputs affect computed commands. | Acquisition reference, availability, compute start/end and consumed sample ID; separate transport, queue and execution delays. | CDH / ADCS; timing measurement | [M09](#m09); [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-c07"></a>C07: Compute-to-actuation delay | TBD; Flight compute-to-application TBD; diagnostic0.1 s TEST-ONLY. Units: s. | Publication and physical moment onset need not coincide. | Compute completion, command publication, PWM latch/enable and winding-current rise on a common clock; moment onset where independently resolved. | CDH / ADCS / EPS; bench measurement, timing measurement | [M04](#m04), [M09](#m09); [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-c09"></a>C09: Burst duration and actuation window | TBD; BURST_TICKS absent; manager10 Hz; burst length/start/retrigger TBD. Units: ticks; s; Hz. | Burst window and retrigger policy govern duty and command age. | Configured BURST_TICKS/rate, replace/queue/retrigger/expiry policy, measured window and admissible duty envelope. | ADCS / CDH / EPS / Thermal; document confirmation, schematic/BOM confirmation, bench measurement, timing measurement, integrated/HIL test | [M03](#m03), [M09](#m09), [M10](#m10); [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-007](#adcs-mag-007), [ADCS-MAG-008](#adcs-mag-008) |
| <a id="blocker-d01"></a>D01: PWM/GPIO channel allocation | TBC; 6 PWM software versus3 PWM +3 GPIO pin table/comment; EPS corroborates GPIO/PWM. Units: signal channels. | Incorrect channel mapping can change sign or inhibit actuation. | Released schematic/connector/pinmux truth table tied to firmware; trace every used PWM/GPIO and dispose of 6 PWM versus 3 PWM +3 GPIO. | CDH / ADCS / EPS; document confirmation, schematic/BOM confirmation, bench measurement | [M01](#m01), [M03](#m03), [M10](#m10); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-d02"></a>D02: PWM polarity, magnitude, enable and off semantics | TBC; Six-PWM text defines +/- pair/duty/enable; actual3PWM+3GPIO truth table TBD. Units: truth table / signed duty / enable state. | Zero duty, disable, brake and coast differ physically. | Per-pin truth table and winding response for positive/negative, idle/OFF/reset/recovery states; apply polarity exactly once. | CDH / ADCS / EPS; document confirmation, schematic/BOM confirmation, bench measurement | [M01](#m01), [M03](#m03), [M04](#m04); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004) |
| <a id="blocker-d03"></a>D03: PWM carrier frequency and resolution | TBD; PWM_FREQUENCY/carrier/resolution/waveforms TBD. Units: Hz; bits. | Carrier/quantization/latch behavior affects current and contamination. | PrmDb/firmware settings plus carrier period, duty resolution, latch/switching traces and instrumentation bandwidth/uncertainty. | CDH / ADCS / EPS; document confirmation, schematic/BOM confirmation, bench measurement, timing measurement | [M03](#m03), [M10](#m10); [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-d05"></a>D05: Per-coil current sensing and dipole feedback | TBD; Per-coil sensing, regulation/feedback bandwidth and dipole feedback TBD; rail monitor is distinct. Units: topology; Hz; A; A m^2. | Rail monitoring is not per-coil feedback/regulation evidence. | Schematic/firmware confirmation of sensing/regulation paths or their documented absence; independent sensor calibration and limiting/feedback measurements. | EPS / ADCS / CDH; document confirmation, schematic/BOM confirmation, bench measurement, calibration | [M03](#m03); [ADCS-MAG-003](#adcs-mag-003) |
| <a id="blocker-e01"></a>E01: Magnetometer placement and separation | TBD; IMMU payload lateral plate; MTA between PDS/transceiver; separation/transform TBD;15.24 mm stack pitch only. Units: location; m; rotation; mm. | Contamination and transforms depend on actual installation. | Survey coil/sensor positions/orientations, harness return paths and relevant configuration; establish magnetic separation independently of stack pitch. | Structures / ADCS; document confirmation, schematic/BOM confirmation, bench measurement, calibration | [M01](#m01), [M05](#m05); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-005](#adcs-mag-005) |
| <a id="blocker-e02"></a>E02: Alignment knowledge requirement versus budget predictions | TBC; RVM orientation knowledge3 deg; B3 actuator prediction/allocation0.10/0.25 deg and sensor0.25/0.50 deg. Units: deg. | Nominal alignment and knowledge uncertainty differ. | Survey/calibration and covariance/bounds tied to a body datum; disposition of 3 degree knowledge obligation and tighter budget allocations. | ADCS / Structures / Systems; document confirmation, calibration | [M01](#m01), [M12](#m12); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-e03"></a>E03: Magnetic cleanliness and allowable contamination | TBD; Magnetic-error/cleanliness metric, allowable bias/residual/gradient and threshold TBD. Units: T; T/m; acceptance rule. | No accepted magnetic-error allocation means no accepted quiet time. | Approved magnetic-error metric/limit, frame/window/coverage/margin and operating envelope; independent baseline/reference definition. | ADCS / Systems / sensor lead; document confirmation | [M06](#m06), [M12](#m12); [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-e04"></a>E04: Remanence, residual field and interference map | TBD; Remanence/local residual, hard/soft-iron and coupling map TBD. Units: A m^2; T; calibration matrices. | Residual/hysteretic fields corrupt B even with zero current. | Measured per-axis/polarity/history coupling and post-OFF residual maps; ambient reference correction that retains remanence and uncertainty. | ADCS / magnetic calibration lead; bench measurement, calibration | [M05](#m05), [M06](#m06); [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006) |
| <a id="blocker-e06"></a>E06: External magnetometers and total installed count | TBD; VN-100 internal3-axis sensor documented; additional external models/count TBD. Units: model/count. | Consumed B must match the calibrated physical sensor. | Installed inventory/serials, connector and firmware selection; demonstrate physical source of each consumed sample; generic headings do not prove external devices. | ADCS / CDH / Systems; document confirmation, schematic/BOM confirmation, bench measurement | [M01](#m01), [M07](#m07); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-f05"></a>F05: Completed measurements constraining a flight cycle | TBD; No qualifying measured dipole/current/power/noise/interference/settling/timing/HIL records located. Units: measurement records. | No qualified measurements means no supported physical cycle. | Complete calibrated measured packages with conditions/uncertainty/independent comparisons/sign-off; preserve failures and exclusions. | ADCS / Systems / test leads; bench measurement, calibration, timing measurement, integrated/HIL test | [M01](#m01), [M02](#m02), [M03](#m03), [M04](#m04), [M05](#m05), [M06](#m06), [M07](#m07), [M09](#m09), [M10](#m10), [M11](#m11), [M12](#m12), [M13](#m13), [M08](#m08); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007), [ADCS-MAG-008](#adcs-mag-008) |

### P1 items

| ID / PARAMETER | CURRENT STATUS / CANDIDATE | WHY IT MATTERS | EXACT EVIDENCE TO RESOLVE | LIKELY OWNER / EVIDENCE TYPE | MINIMUM MODEL OUTPUT / TEST |
|---|---|---|---|---|---|
| <a id="blocker-a10"></a>A10: Actuator dynamics and residual dipole | TBD; Inductance, rise/history/hysteresis/temperature dynamics TBD. Units: H; s; A m^2; degC dependence. | Rise dynamics, magnetic memory and temperature response refine prediction. | Measured turn-on/off waveform families and history loops; identify R-L or other dynamic coefficients only after model adequacy checks. Basic off validity remains P0 in C03/E04. | ADCS / EPS / Thermal; bench measurement, calibration, timing measurement | [M02](#m02), [M04](#m04), [M05](#m05); [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-005](#adcs-mag-005) |
| <a id="blocker-a14"></a>A14: ADCS power and energy allocation | TBR; RVM<2 W detumble/<0.4 W standby and <=11 Wh, TBR; derivation60 Wh; ICDpeak12 W; PDR2.97 W/6.27 Wh estimates. Units: W; Wh. | Power/energy compliance requires scoped allocations. | Approve bus/winding and peak/average/cycle boundaries; reconcile power allocation and 11/60 Wh conflict before mission energy verification; measured cycle power. | Systems / ADCS / EPS; document confirmation, bench measurement | [M03](#m03), [M12](#m12); [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-008](#adcs-mag-008) |
| <a id="blocker-c08"></a>C08: Serial latency interface allocation | TBC; ICD serial latency <one10 Hz loop period; conditional<0.1 s. Units: s. | Serial allocation cannot be used as nominal sample age. | Approved loop/latency endpoints and measured transport; confirm applicability of the ICD less-than-one-period allocation. | CDH / ADCS / Systems; document confirmation, timing measurement | [M09](#m09), [M12](#m12); [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007) |
| <a id="blocker-c10"></a>C10: Historical mode duty assumptions | ASSUMED; PDR80%; budget detumble50%, experiment20%, standby10%; ASSUMED. Units: %. | Historical duty anchors are not allowable operating limits. | Mode-specific power/duty definition and measured approved envelope; separate carrier duty, burst fraction and nonzero-current time. | ADCS / EPS / Systems; document confirmation, bench measurement, integrated/HIL test | [M03](#m03), [M10](#m10); [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-008](#adcs-mag-008) |
| <a id="blocker-f04"></a>F04: Budget MC/HIL/calibration assertions | ASSUMED; Budget calibration/MC/HIL assertions; supporting completed records not located. Units: verification claim. | Budget verification assertions need independent substantiation. | Reserved integrated/HIL records and predeclared criteria, or retain claim as unverified; no tuning on verification data. | ADCS / Systems / HIL lead; document confirmation, integrated/HIL test | [M11](#m11), [M13](#m13); [ADCS-MAG-008](#adcs-mag-008) |

### P2 items

| ID / PARAMETER | CURRENT STATUS / CANDIDATE | WHY IT MATTERS | EXACT EVIDENCE TO RESOLVE | LIKELY OWNER / EVIDENCE TYPE | MINIMUM MODEL OUTPUT / TEST |
|---|---|---|---|---|---|
| <a id="blocker-f01"></a>F01: ADCS hardware test tracker | CONFIRMED; PCB delivered/done No; rods/IMU/sun sensors ordered/done No; amplifier/coil/integrated entries incomplete. Units: tracker states. | Tracker flags cannot validate physical parameters. | Link executed record IDs/configurations to tracker; obsolete entries may be replaced without redundant legacy testing. | ADCS / Systems verification; document confirmation | [M13](#m13); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-008](#adcs-mag-008) |
| <a id="blocker-f02"></a>F02: Current ADCS verification procedure | CONFIRMED; Current written procedures; measurements/results/initials unfilled. Units: procedure state. | A written procedure is not a measured result. | Crosswalk procedure IDs to signed executed packages; reuse qualified results rather than duplicate tests. | ADCS / Systems verification; document confirmation | [M13](#m13); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-008](#adcs-mag-008) |
| <a id="blocker-f03"></a>F03: Older ADCS verification record | CONFIRMED; Older template; results/signatures/data/graphs unfilled. Units: procedure state. | Blank old templates are not acceptance records. | Link the actual current measured record and classify the legacy template; no requirement to fill an obsolete template. | ADCS / Systems verification; document confirmation | [M13](#m13); [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-008](#adcs-mag-008) |

## Minimum characterization set

| CATEGORY / TEST | MUST BE KNOWN BEFORE USE | CAN REMAIN PARAMETRIC | COMPLETELY BLOCKED | MODEL OUTPUT / FORMAT |
|---|---|---|---|---|
| CHAR-01: Actuator identity and geometry; [ADCS-MAG-001](#adcs-mag-001) | Released installed identities, body datum, channel/polarity and surveyed sensor/coil transforms/positions. | Only residual measured survey uncertainty inside the approved envelope. | Choosing COTS/in-house/air-coil variant or actual axes from assumption. | [M01](#m01); geometry.json and signed survey/BOM |
| CHAR-02: Per-axis dipole/current calibration; [ADCS-MAG-002](#adcs-mag-002) | Signed command-to-current-to-vector-moment curves; direction, nonlinearity/history and validated domain. | Measured calibration uncertainty and accepted model discrepancy. | Unmeasured gain, saturation clamp, assumed linearity or variant-mixed capability. | [M02](#m02); long-form calibration table and fit metadata |
| CHAR-03: Electrical limits and thermal/duty behavior; [ADCS-MAG-003](#adcs-mag-003) | Driver/feedback truth table, supply/winding limits, carrier, power boundary and admissible thermal/burst/duty envelope. | Measured variability within that envelope; later environmental extension remains excluded. | Using rail voltage as coil DC drive or treating historical duty as a thermal rating. | [M03](#m03), [M10](#m10), [M12](#m12); electrical.json plus synchronous waveforms/envelope tables |
| CHAR-04: Coil current decay; [ADCS-MAG-004](#adcs-mag-004) | Separate command-off/driver-off events and actual winding-current persistence for used off modes. | Measured conditional transient repeatability; detailed R-L fit optional if a bounded equivalent is justified. | Numeric acceptable decay without threshold/uncertainty or proof that zero command equals zero current. | [M04](#m04), [M09](#m09); current/logic traces and conditional transient tables |
| CHAR-05: Interference and magnetic settling; [ADCS-MAG-005](#adcs-mag-005) | Installed local coupling/remanence and entire valid acquisition window against an approved magnetic-error allocation. | Measured residual uncertainty/condition dependence; explicitly excluded untested geometries. | Contamination magnitude, magnetic separation or accepted quiet duration from intuition. | [M05](#m05), [M06](#m06), [M12](#m12); interference table, raw B traces and validity.json |
| CHAR-06: Magnetometer configuration and timestamps; [ADCS-MAG-006](#adcs-mag-006) | Actual sensor/data source, packet/filter/calibration settings, acquisition window/epoch/availability and enough baseline error characterization to assess cleanliness. | Measured residual bias/noise and epoch uncertainty; complex long-term stochastic tails need more data. | Hardware maximum as flight output rate/aperture, receipt time as acquisition time. | [M07](#m07), [M08](#m08), [M09](#m09); sensor.json, register dump and stimulus/packet records |
| CHAR-07: Flight software control/timing/jitter; [ADCS-MAG-007](#adcs-mag-007) | Approved 5/10 Hz disposition and full event map; measured latency/phase/jitter/staleness/retrigger behavior. | Measured joint timing distribution or bounded variability, not unknown delays. | Treating manager rate as whole cycle or inventing ranges around 0.1 s test delays. | [M09](#m09), [M10](#m10); schedule.json and common-clock events table |
| CHAR-08: Integrated cycle verification; [ADCS-MAG-008](#adcs-mag-008) | Independent reserved-case evidence that real assembly/software obeys accepted magnetic/electrical/timing rules. | Only supported model discrepancy within an explicit candidate validity envelope. | Passing on duplicated equations, calibrated-current dipole used as independent dipole, or simulated rate treated as physical measurement. | [M11](#m11), [M13](#m13); validation manifest, reserved raw/replay traces and signed decision |

The named JSON/table/waveform formats below are **proposed future measurement packages**, not files created in this phase. A characterized, explicitly restricted envelope is required; any conditions needed by the intended flight-candidate envelope cannot be deferred as later refinement.

## Separate timing quantities

| ID / QUANTITY | DISTINCT OBSERVABLE | MODEL USE | BLOCKERS |
|---|---|---|---|
| TIME-01: Actuator command-off timestamp | t_cmd_off and associated command ID in mapped clock; t_driver_off measured separately | M09 event stream; command hold endpoint, not a decay duration | [C03](#blocker-c03), [D02](#blocker-d02), [B07](#blocker-b07) |
| TIME-02: Coil-current decay | I(t) after command/driver OFF and stable crossing of an approved current criterion | M04 transient response; M06 validity contribution | [C03](#blocker-c03) |
| TIME-03: Local magnetic-field decay/settling | Independent local Delta B(t) and flight sensor response versus background/history | M05 contamination and M06 valid acquisition rule | [C04](#blocker-c04), [E04](#blocker-e04), [E03](#blocker-e03) |
| TIME-04: Magnetometer aperture/filter history | Effective acquisition start/end or bounded response kernel from independent timed stimulus | M07 finite acquisition/filter behavior, not packet interval | [C05](#blocker-c05), [B05](#blocker-b05) |
| TIME-05: Magnetometer sample timestamp | Acquisition reference/bounds, sample ID and clock mapping distinct from availability | M07/M09 measurement epoch and age | [B05](#blocker-b05), [B06](#blocker-b06), [B07](#blocker-b07) |
| TIME-06: Controller compute delay | t_compute_start/end and consumed sample ID; queue and execution separated from transport | M09 future sample_to_compute contract | [C06](#blocker-c06), [B07](#blocker-b07) |
| TIME-07: Actuator application delay | Compute end, command publication, driver latch/on and physical current rise | M09 plus M04; future compute_to_actuate/actuator response | [C07](#blocker-c07), [B07](#blocker-b07) |


The chronological chain is acquisition start/reference/end, sample availability, compute start/end, command publication, driver latch/activation and winding response. Command-off and actual driver-off are separately observed on the opposite transition. Preserve actual event IDs and clock mappings.

For a chosen acquisition reference `t_acq`, the age at current onset includes transport/filter availability, queue wait, compute execution, publication, driver latch and current rise. Split these measured components explicitly; do not charge one component twice or relabel a publication time as acquisition time.

The minimum usable off interval is the earliest age at which the **entire effective acquisition window/filter history** meets the approved magnetic-error rule across the declared conditions, including uncertainty and observed rebound. Current decay and magnetic settling are different observations that may overlap; adding two durations measured from the same OFF origin can double count elapsed time. Numeric thresholds, coverage and margin remain TBD.

## Common measurement and evidence contract

- **status:** ASSUMED / proposed test-record structure; all results NOT EXECUTED
- **raw trace schema:** Run/case/config ID; sample/command/event ID; physical channel and origin_kind (measured, inferred, simulated); clock ID/raw ticks/scale; mapped reference time and uncertainty; value/units/frame; acquisition start/reference/end or bounds; validity/saturation/missing flags; condition/history IDs.
- **manifest:** Installed serial/BOM/PCB/firmware/register/pinmux/PrmDb revisions; fixture/body/reference transforms and harness geometry; instrument serial/calibration/date/bandwidth/noise floor; operating limits; thermal/background/command histories; clock mapping/offset/drift; raw-data and processing/fit checksums; source/requirement criteria; repeats, failures/exclusions and reviewer disposition.
- **resolution:** No numerical timestamp resolution, probe bandwidth, sample rate, repetition count, dwell duration or equipment tolerance is invented. Test owner allocates these from the smallest event/magnetic-error margin to be resolved and instrument uncertainty. A pilot trace may establish bandwidth/dwell needs inside approved limits; justify settings before acceptance. Integer t_ns is a storage unit, not a claim of nanosecond accuracy.
- **repetition:** Repeat signed amplitudes/histories and independent sessions over the declared envelope; choose sample count/coverage from the approved uncertainty objective. Preserve correlated timing/magnetic histories and censored/no-settle cases; do not silently select only favorable repeats.
- **independence:** Independent reference measurement and held-out cases are required for calibration/model acceptance. Never overwrite measured current/field or native torque with expected calculations. HIL simulated rate/torque is labeled simulated; actual physical torque/rate needs independent measurement.
- **units and frames:** SI data plus preserved original units/conversions; declared right-handed body datum, coil sign, C_SB B-to-S transform, external B_N and simulated C_BN N-to-B direction. Sensor local contamination never substitutes for external magnetic field in plant torque.
- **future use:** Record formats named here are proposed future evidence packages only; none are created now except the two planning artifacts. Candidate parameter source/revision/status/config/frame/uncertainty must reference executed evidence; no unmeasured runtime parameters.

## Test sequence and dependencies

1. 001-D: document identity/body datum/schematic and initial fixture survey; ADCS/EPS/Thermal approve the initial bench envelope and instrumentation/criteria plan.
2. 003 initial bring-up: verify power/pin/OFF limits; then 001-P verifies physical polarity. 002 dipole calibration and full 003 electrical/thermal characterization proceed within approved limits.
3. 006 sensor acquisition/calibration may proceed in parallel with actuator work once sensor identity/power and independent field reference are established.
4. 004 current decay and 005 local-field/sensor recovery use characterized drive, geometry and aperture; Systems/ADCS allocation must exist before accepting a quiet interval.
5. 007 confirms flight schedule and measured age/jitter. 008 uses frozen packages and reserved data; no recalibration/tuning on that validation set.

No hardware test is executed by this phase. Initial safe bench limits must be issued from identified parts/schematic/thermal review before powered steps; the plan does not use unresolved datasheet/ICD extremes as authorized test settings.

## Proposed tests

<a id="adcs-mag-001"></a>
### ADCS-MAG-001: Actuator identity, polarity and body-axis verification

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Identify the actual assembly and establish physical frames/channel signs used by the model. |
| Required hardware | Actual or explicitly configuration-equivalent MTA/driver/selected sensor/harness; released BOM/drawings; survey instruments; independent calibrated vector magnetic reference; bounded power/current/logic instrumentation. |
| Setup | Survey body datum, coil axes/centers and sensor location/orientation in the intended configuration; record fixture-to-body transform and magnetic-reference poses. |
| Controlled variables | Part/serial/PCB/firmware, mounting/harness, background field, temperature and approved polarity-test commands. |
| Measured channels | Identity/pin records; g_Bi, C_SB, r_B; command/driver state, signed winding current, independent field/moment direction; survey uncertainty. |
| Timestamp resolution/synchronization | Static configuration/time records plus mapped command/current/field acquisition IDs for powered polarity; actual acquisition epochs, not receipt time. |
| Prerequisites | 001-D is unpowered. Initial safe bench limits and MAG-003 bring-up precede 001-P; full thermal-envelope characterization is not a prerequisite for the static survey. |
| Outputs and exact future Basilisk mapping | [M01](#m01), [M07](#m07), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [A02](#blocker-a02), [A03](#blocker-a03), [A05](#blocker-a05), [A12](#blocker-a12), [B03](#blocker-b03), [D01](#blocker-d01), [D02](#blocker-d02), [E01](#blocker-e01), [E02](#blocker-e02), [E06](#blocker-e06), [F01](#blocker-f01), [F02](#blocker-f02), [F03](#blocker-f03), [F05](#blocker-f05) |
| Existing criteria / qualification | RVM ADCS-6/7 (Phase 6B E02) document orientation knowledge within 3 degrees. Applicability/tighter budget allocations remain TBC; knowledge uncertainty is not mounting tolerance. Proposed identity/sign comparison must match the approved configuration; numeric survey acceptance beyond the applicable obligation is TBD. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Complete document/survey stage 001-D; resolve COTS/in-house and air-coil variant, actual sensor inventory and data source.
2. Define B origin/handedness, survey coil/sensor transforms and current-return routing; do not substitute stack pitch for magnetic separation.
3. After approved initial limits/MAG-003 bring-up, perform powered stage 001-P: correlate +/- commands, actual current direction and independent vector field direction.
4. Check C_SB direction/right-handedness/orthonormality within measured uncertainty. Preserve nonorthogonality of separate coil axes; do not require G_B to be an orthogonal DCM.
5. Close discrepancies by signed disposition; cross-check moment direction in MAG-002. Record legacy-procedure/record crosswalk without requiring obsolete template execution.

<a id="adcs-mag-002"></a>
### ADCS-MAG-002: Per-axis current-to-dipole and command calibration

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Measure command-to-current-to-vector-dipole response and test linearity, saturation and history dependence. |
| Required hardware | Identified actuator/driver and actual firmware command source; approved supply; calibrated current/voltage and temperature instruments; independent vector reference magnetometer(s) with surveyed fixture or calibrated torque magnetometry; controlled/monitored background field. |
| Setup | Use multiple surveyed poses/orientations with an identifiable field-to-moment inverse; check dipole approximation against independent poses rather than assume near-field dipolar behavior. Retain pre/post zero-command baselines and ambient reference. |
| Controlled variables | Signed amplitude, rising/falling sequence, dwell/history, voltage, winding temperature, driver/carrier, background vector and reference pose. |
| Measured channels | Command/latch, I/V, independent field vectors/poses or torque, temperature and zero/remanent/transient records. |
| Timestamp resolution/synchronization | Map command/current/reference-field clocks; record physical acquisition windows and actual dwell. Stability/dwell acceptance is defined before selecting steady intervals; no invented duration. |
| Prerequisites | MAG-001 document/geometry/polarity and approved initial electrical conditions; full thermal extension may follow. Test accuracy/coverage criteria must be approved before declaring acceptance. |
| Outputs and exact future Basilisk mapping | [M01](#m01), [M02](#m02), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [A03](#blocker-a03), [A05](#blocker-a05), [A09](#blocker-a09), [A10](#blocker-a10), [E04](#blocker-e04), [F05](#blocker-f05) |
| Existing criteria / qualification | Dipole accuracy, cross-axis/nonlinearity/hysteresis residual, fit coverage and saturation acceptance require approved ADCS allocations. Vendor nominal ratings alone do not establish installed acceptance. Failed identifiability or inadequate held-out fit blocks the chosen reduced model. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Use MAG-001 identity/geometry and MAG-003 initial limits; all points stay inside an approved test envelope.
2. Collect zero and +/- commands over that envelope with ascending/descending paths, repeats and different magnetic histories/background orientations; test per-axis direction and coupling.
3. Infer moment independently from calibrated reference field/geometry or torque; propagate instrument, background and geometry uncertainty and retain remanence.
4. Fit current versus command separately from vector moment versus measured current; assess linear/nonlinear and temperature/history dependence using reserved poses/sequences.
5. Publish curves, interpolation domain, uncertainty and held-out residuals. Adopt scalar gain/fixed axis only against a predeclared model-error allocation; do not turn vendor lower bounds into clamps.

<a id="adcs-mag-003"></a>
### ADCS-MAG-003: Electrical/current/power/thermal characterization

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Resolve driver and sensing semantics and release a measured operating envelope within approved hardware limits. |
| Required hardware | Actual MTA/CDH drive and EPS-equivalent supply; released schematic/BOM; calibrated per-coil current and differential winding-voltage probes, bus power measurement, logic/oscilloscope and temperature instruments; representative thermal fixture. |
| Setup | Instrument bus and windings independently without unrecorded return-path changes. Compare installed feedback against independent probes; capture true driver mode/carrier. |
| Controlled variables | Voltage, temperature/thermal boundary, signed command, carrier/duty/enable/OFF state, burst/retrigger, individual/simultaneous coils and firmware load. |
| Measured channels | Bus/winding V/I, PWM/GPIO/latch, temperature, installed monitor telemetry, limiting/fault status and actual on/off windows. |
| Timestamp resolution/synchronization | Bandwidth/sample rates are selected from observed switching/transients and required uncertainty, with anti-aliasing/probe limits recorded. Peak/mean/RMS windows and clock skew explicit; faster switching and slow heating share record IDs. |
| Prerequisites | MAG-001 document/fixture identification plus signed initial bench envelope. Initial bring-up enables 001-P/002; fuller electrical/thermal work follows without a circular prerequisite. |
| Outputs and exact future Basilisk mapping | [M03](#m03), [M04](#m04), [M10](#m10), [M12](#m12), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [A05](#blocker-a05), [A06](#blocker-a06), [A08](#blocker-a08), [A10](#blocker-a10), [A12](#blocker-a12), [A13](#blocker-a13), [A14](#blocker-a14), [A15](#blocker-a15), [C03](#blocker-c03), [C07](#blocker-c07), [C09](#blocker-c09), [C10](#blocker-c10), [D01](#blocker-d01), [D02](#blocker-d02), [D03](#blocker-d03), [D05](#blocker-d05), [F05](#blocker-f05) |
| Existing criteria / qualification | Actual assembly voltage/current/thermal/duty limits and averaging boundaries are presently unresolved: acceptance TBD. RVM <2 W detumble / <0.4 W standby and <=11 Wh remain TBR; 60 Wh and coil/ADCS peak conflicts need disposition. This test does not verify 24-hour mission energy. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Release initial safe bench conditions from actual parts/schematic/thermal review before powering. Conflicting 9/12 V claims or 100% thermal-table duty are not test authorizations.
2. Verify pin truth table, levels, sign/magnitude/enable/idle/reset and supported recovery states; measure carrier and duty/latch resolution.
3. Measure winding resistance with lead/contact correction and recorded temperature; characterize command-current relation and regulation/limiting. Distinguish per-coil sensing from rail INA3221 readings; confirm absence where applicable.
4. Capture bus/winding instantaneous v*i before averaging/integrating; retain ripple, regeneration, baseline load and driver losses. Use I_rms^2 R only for applicable winding heat.
5. Measure heating and repeated-burst/duty behavior inside approved bounds; Thermal/EPS sign a restricted operating envelope. Room-air cooling is not automatically a flight thermal result.

<a id="adcs-mag-004"></a>
### ADCS-MAG-004: Magnetorquer command-off and coil-current decay

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Measure physical current persistence after OFF separately from command and driver timestamps. |
| Required hardware | Identified driver/coil/CDH, approved supply/fixture; calibrated current/voltage probes, synchronized logic/scope and temperature; optional independent fast field reference. |
| Setup | Trigger on command marker and actual driver edge; measure current through the true recirculation path with adequate documented bandwidth/dynamic range. |
| Controlled variables | Prior signed amplitude/history, burst, temperature, supply, brake/coast/disable/reset mode and simultaneous coils within approved limits. |
| Measured channels | t_cmd_off, t_driver_off, command IDs, I(t)/V(t), temperature, mode; optional independent field/moment response. |
| Timestamp resolution/synchronization | Preserve both OFF origins, clock/probe skew, trigger uncertainty, noise floor and record end. Timing resolution is justified by the measured transient and acceptance uncertainty; numeric requirement TBD. |
| Prerequisites | Verified driver truth table and initial electrical limits; a threshold is not needed to collect raw decay traces but is required to accept a quiet-time contribution. |
| Outputs and exact future Basilisk mapping | [M04](#m04), [M06](#m06), [M09](#m09), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [A10](#blocker-a10), [C02](#blocker-c02), [C03](#blocker-c03), [D02](#blocker-d02), [F05](#blocker-f05) |
| Existing criteria / qualification | No accepted decay duration/current threshold exists. Threshold, stable-window definition and uncertainty/margin must come from approved magnetic-error allocation or a justified current-to-local-field bound. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Exercise supported off states after representative signed burst/history cases inside the characterized electrical envelope.
2. Capture pre-OFF conditions and complete observable decay; repeat and check rebound/history dependence.
3. Report I(t) and threshold-crossing bounds; the acceptance current level/stable window must be approved or derived from magnetic validity.
4. Treat below-instrument-floor and beyond-record decay as bounded/censored evidence, not zero current or invented finite settling.
5. Correlate any independent magnetic response; current decay does not close the local-field/sensor recovery test.

<a id="adcs-mag-005"></a>
### ADCS-MAG-005: Actuator-to-magnetometer interference and local-field settling

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Measure local contamination/remanence and establish when an entire flight-sensor acquisition becomes usable. |
| Required hardware | Flight-like MTA/sensor/structure/harness, selected magnetic sensor, calibrated independent ambient/local vector reference, synchronous current/logic capture, controlled field and temperature. |
| Setup | Use surveyed intended geometry. Characterize the reference probe's own coil pickup; preserve raw/corrected flight data and independent local field so subtraction cannot erase contamination. |
| Controlled variables | Per-coil and simultaneous-coil signed command/current/moment, prior history, temperature/background vector, mounting state and burst conditions. |
| Measured channels | Command/current/driver events, independent ambient/local B, flight raw/calibrated B and acquisition/availability, temperature/config IDs. |
| Timestamp resolution/synchronization | Synchronize all channels and use MAG-006 physical aperture/filter history, not nearest-time pairing without age bounds. Record command-off, current decay and field/sensor recovery separately. |
| Prerequisites | MAG-001 geometry, MAG-002/003 command envelope, MAG-004 off behavior and MAG-006 acquisition semantics. Data collection may precede allocation; accepted quiet time may not. |
| Outputs and exact future Basilisk mapping | [M01](#m01), [M05](#m05), [M06](#m06), [M07](#m07), [M08](#m08), [M12](#m12), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [A10](#blocker-a10), [C02](#blocker-c02), [C04](#blocker-c04), [E01](#blocker-e01), [E03](#blocker-e03), [E04](#blocker-e04), [F05](#blocker-f05) |
| Existing criteria / qualification | ACCEPTANCE CRITERION TBD for contamination magnitude, coverage, margin and valid window; ADCS/Systems must supply the allocation. I1's qualitative burst-then-zero-current intent (C01) is documented, but software zero command does not prove physical validity. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Measure off-state baseline/drift/noise; repeat after positive, negative and alternating histories; retain any persistent change as remanence until explained.
2. Exercise individual axes/polarities and approved simultaneous cases across the characterized envelope; test rather than assume linear superposition.
3. Measure local vector contamination during/after drive with independent reference and actual flight sensor; separate physical field recovery from sensor filter recovery.
4. Apply the approved error metric/frame/window/coverage to the entire effective acquisition history. Find earliest valid acquisition with uncertainty, rebound and repeatability included.
5. Publish conditional validity bounds, raw traces, failures and geometry limits. Without a threshold or identifiable acquisition window, report traces only and retain quiet/settling TBD.

<a id="adcs-mag-006"></a>
### ADCS-MAG-006: Magnetometer sampling, filtering and timestamp characterization

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Identify the real magnetic data source and measure its calibration, effective acquisition/filter response and epoch semantics. |
| Required hardware | Flight sensor/firmware/interface; calibrated controlled vector field or equivalent stimulus with independent reference, survey fixture, common-clock packet/logic capture and temperature measurement. |
| Setup | Read back actual registers/packets/calibration state. Apply independently measured field changes/rotations at varied phase relative to output; distinguish physical response from transport. |
| Controlled variables | Sensor/config/firmware, output/filter settings, field magnitude/direction/stimulus phase, temperature/orientation, static/dynamic fields and recovery history. |
| Measured channels | Reference B, raw/corrected sensor B, packet sequence/content, data-ready if available, transmit/receive/decode, device/host clock events and temperature. |
| Timestamp resolution/synchronization | Measure clock offsets/drift/wrap, acquisition start/reference/end or bounded response kernel, filter history and availability separately. 200 Hz capability is not a 5 ms aperture. |
| Prerequisites | Identified sensor/fixture and permitted power/interface settings; independently calibrated stimulus/reference. May run alongside actuator tests once these prerequisites exist. |
| Outputs and exact future Basilisk mapping | [M01](#m01), [M07](#m07), [M08](#m08), [M09](#m09), [M12](#m12), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [B03](#blocker-b03), [B05](#blocker-b05), [B06](#blocker-b06), [C02](#blocker-c02), [C04](#blocker-c04), [C05](#blocker-c05), [C06](#blocker-c06), [C08](#blocker-c08), [E02](#blocker-e02), [E03](#blocker-e03), [E04](#blocker-e04), [E06](#blocker-e06), [F05](#blocker-f05) |
| Existing criteria / qualification | Sensor/model error and epoch uncertainty acceptance TBD from ADCS allocation. H4 rates/noise are vendor capability, not installed validation. I1 serial latency <one 10 Hz period is conditional <0.1 s; applicability/endpoints and 5/10 Hz conflict remain TBC. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Confirm actual VN-100/internal or external source and flight selection; dispose of interface contradictions through hardware/firmware evidence.
2. Preserve register dumps/raw readings and calibration state; measure scale/cross-axis/bias with known vector fields and quiet residual noise/drift with an independent reference.
3. Identify/bound aperture and filter response using phase-varied timed stimuli; reference actual generated field, not the field-generator command.
4. Trace sample IDs, publication/transport age, losses/repeats/out-of-order behavior and recovery; retain acquisition uncertainty if no direct marker exists.
5. Validate calibration/response on held-out stimuli; publish supported bias/noise/timing models and limitations. No estimator/noise code is implemented now.

<a id="adcs-mag-007"></a>
### ADCS-MAG-007: Flight-software magnetic-control timing and jitter

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Resolve the schedule and measure actual sample-to-compute-to-physical-application timing. |
| Required hardware | Flight-like CDH, actual sensor and driver/MTA (initial dummy load explicitly labeled), firmware/topology/PrmDb/pinmux, common-clock trace/logic/packet capture and current probes. |
| Setup | Instrument actual code events with sample/command IDs and cross-checked GPIO markers; quantify instrumentation overhead and correlate real driver/current transitions. |
| Controlled variables | Supported approved 5/10 Hz candidate configurations, flight workload/mode/queues, sensor settings, burst/retrigger and late/stale/reset cases. |
| Measured channels | Acquisition reference/bounds, availability, compute start/end and consumed sample/nav IDs, publication, PWM latch/on/off/current rise, scheduler and carrier events. |
| Timestamp resolution/synchronization | Mapped monotonic clocks and measured skew/drift/overhead. Retain actual ordered events and correlated jitter, not intended scheduler ticks; numeric resolution allocated before acceptance. |
| Prerequisites | MAG-006 acquisition semantics, MAG-003 driver envelope and released candidate firmware settings. Existing TEST-ONLY delays are not acceptance limits. |
| Outputs and exact future Basilisk mapping | [M04](#m04), [M07](#m07), [M09](#m09), [M10](#m10), [M12](#m12), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [B01](#blocker-b01), [B03](#blocker-b03), [B06](#blocker-b06), [B07](#blocker-b07), [C06](#blocker-c06), [C07](#blocker-c07), [C08](#blocker-c08), [C09](#blocker-c09), [D01](#blocker-d01), [D03](#blocker-d03), [E06](#blocker-e06), [F05](#blocker-f05) |
| Existing criteria / qualification | Loop mapping, worst-case age/jitter/expiry and recovery criteria are TBD pending ADCS/CDH/Systems approval. Conditional I1 serial allocation retains its endpoints and conflict; it cannot substitute for total acquisition-to-current acceptance. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Obtain a version-controlled map distinguishing manager, acquisition, control and complete-cycle rate; compare 5/10 Hz only as controlled supported alternatives.
2. Trace acquisition-to-availability-to-compute-to-publication-to-driver/current under representative workload.
3. Exercise supported late/stale/duplicate input and replace/retrigger/expiry/reset behavior; demonstrate which sample generated each applied command.
4. Separate transport, queue, compute, publication/latch/current-rise delays and their joint variability; retain MAG-006 bounds on unobservable epochs.
5. Review simulation representability without rounding away offsets or changing source timestamps; unsupported hardware timing requires separately authorized architecture work.

<a id="adcs-mag-008"></a>
### ADCS-MAG-008: Integrated magnetic-control cycle / HIL validation

**Status:** NOT EXECUTED; proposed method ASSUMED; no result record.

| FIELD | DEFINITION |
|---|---|
| Objective | Independently verify clean sampling, admissible actuation and correct event timing with real assembly/software on reserved cases. |
| Required hardware | Identified flight controller/sensor/driver/MTA; calibrated field stimulus and independent reference; current/voltage/timing instruments; separately authorized Basilisk replay/HIL harness. Physical torque/rate claims additionally need calibrated independent torque or free-body/rate instrumentation. |
| Setup | Freeze calibration before validation. Begin integrated bench cycles; later HIL maps external B_N through truth attitude into cage B_B and independently verifies stimulus tracking. Cage feedback must not silently cancel coil contamination. |
| Controlled variables | Reserved field/command/polarity histories, electrical/thermal conditions, firmware loads and supported stale/reset cases inside the characterized envelope. |
| Measured channels | Actual sensor acquisitions/availability, command/current, independent background/local field or moment, bus/winding power/temperature and events. Native/analytical torque and simulated state explicitly labeled. |
| Timestamp resolution/synchronization | Carry clock maps/epoch bounds end-to-end; measure cage/HIL transport separately from flight delay. Preserve exact event/sample IDs, with no hidden time shifts to improve fit. |
| Prerequisites | Accepted configuration/MAG-001..007 measurements and independent reserved-case instrumentation/criteria. This document authorizes planning only, not code or hardware execution. |
| Outputs and exact future Basilisk mapping | [M01](#m01), [M02](#m02), [M03](#m03), [M05](#m05), [M06](#m06), [M07](#m07), [M09](#m09), [M10](#m10), [M11](#m11), [M12](#m12), [M13](#m13); each linked definition lists deterministic values, curves, uncertainty/timing distributions or validation-only evidence and the existing-model limitation. |
| Blockers resolved | [A14](#blocker-a14), [B07](#blocker-b07), [C09](#blocker-c09), [C10](#blocker-c10), [F01](#blocker-f01), [F02](#blocker-f02), [F03](#blocker-f03), [F04](#blocker-f04), [F05](#blocker-f05) |
| Existing criteria / qualification | Acceptance TBD until calibration, magnetic, electrical/duty and timing criteria are approved. Duplicated equations or simulator self-consistency cannot pass physical validation. No 24-hour detumble/pointing requirement verification here; uninstrumented torque/rate and injected channels remain explicit limitations. |
| Acceptance | ACCEPTANCE CRITERION TBD until the stated approvals/allocations are supplied. A complete dataset may be useful characterization without being a passed flight acceptance test. |

Procedure outline:

1. Freeze accepted MAG-001..007 packages, configuration and predeclared physical validation limits; reserve cases never used for fitting or tuning.
2. Run real integrated cycles over reserved histories; verify actual OFF-current history and entire clean acquisition window plus power/duty and freshness.
3. For separately authorized HIL, label injected gyro/nav as substitutions when a fixed fixture cannot rotate. Physical sensor sees applied environment plus local contamination; native torque uses external environment only.
4. Compare predictions against independent reserved field/dipole/current/power/timing evidence. Fitted m=f(I) is not independently measured m; computed m x B/state are not physical torque/rate observations.
5. Evaluate approved criteria with uncertainty; preserve failures and exclusions. Sign a restricted validity envelope or leave acceptance TBD; do not tune on validation data.

## Basilisk input mapping

These are future mappings and data contracts, not runtime values or authorization for code changes. Every output remains TBD until its measurement/approval package exists. A simplified model is acceptable only if independently validated to its allocated error inside the intended envelope.

The input classes are deterministic scalar/vector/matrix; calibration curve / lookup table; uncertainty distribution; timing distribution; validation-only evidence. Distributions are supported by measured data or bounds; none are invented from nominal vendor values.

<a id="m01"></a>
### M01: Identity, geometry and polarity

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | geometry.json: hardware/serial/revision, body datum, g_Bi unit columns, r_Bi, sensor C_SB/r_BS, channel/sign map, survey uncertainty; signed drawing/BOM links. Units: 1; m; rad (source degrees retained). |
| Frame / time contract | B is a released right-handed body datum; C_SB maps B components to S. G_B columns are individual coil axes, not necessarily an orthogonal DCM. Origin is explicit. |
| Exact future model destination | Future MagnetorquerConfig.count/axes_B and SensorConfig.magnetometer_dcm_SB after body-vector-to-coil allocation is verified; polarity applied exactly once. Positions support contamination modeling, not a changed mass/COM profile. |
| Uncertainty / validation treatment | Measured correlated survey uncertainty; do not invent a Gaussian mounting distribution. |

<a id="m02"></a>
### M02: Command-current-vector-dipole calibration

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | calibration table: command/units, coil, I_A, m_B_Am2[3], V_V, T_degC, driver mode, history, ambient_B_T, time, uncertainty; fit/held-out case IDs. Units: A; A m^2; V; degC; T; s. |
| Frame / time contract | Positive current/moment tied to M01; independently infer vector moment from calibrated reference field/geometry or torque. Test background, history and direction dependence. |
| Exact future model destination | Future dipole_gains only if scalar linear gain/fixed direction meets allocation. Otherwise separately authorized calibration adapter/lookup and allocation work is needed; native MtbEffector still receives per-axis dipole. Do not force direction-changing response into a fixed axis. |
| Uncertainty / validation treatment | Fit and instrument/geometry/background/repeatability uncertainty, correlations and interpolation domain; reserve independent poses/sequences. Vendor lower bounds are not maximum clamps. |

<a id="m03"></a>
### M03: Electrical, power and thermal/duty envelope

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | electrical.json plus waveforms/tables: bus/winding V/I, resistance versus temperature, limiting/feedback topology, driver modes/carrier, peak/mean/RMS windows, allowed burst/duty/temperature. Units: V; A; ohm; W; J; Hz; s; degC; fraction. |
| Frame / time contract | Supply, logic and winding boundaries distinct; instantaneous signed p(t)=v(t)i(t) before averaging/integrating. Record regeneration, ripple and baseline loads. I_rms^2 R is winding heat when applicable, not bus power. |
| Exact future model destination | Future current/voltage/power limits only where existing MagnetorquerConfig derivations match measured behavior. Curves/driver losses/regulation/thermal constraints may require separate code authorization; no electrical constants introduced now. |
| Uncertainty / validation treatment | Instrument/ripple/temperature and run variability; admissible envelope excludes untested thermal/voltage/duty cases. |

<a id="m04"></a>
### M04: Physical current rise/decay

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; timing distribution; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | transient traces: command/driver event IDs and times, coil I/V versus elapsed time, polarity, history, temperature, mode, uncertainty and finite-record bounds. Units: A; V; s; optional independently measured A m^2. |
| Frame / time contract | Separate software command-off, driver-off and winding current response. Current crossing needs approved threshold/stability definition; unresolved below-floor current is bounded, not zero. |
| Exact future model destination | Future switching/actuator-response model or validated bounded approximation; no R-L model without adequacy tests. Feeds M06 but electrical decay alone does not define magnetic quiet time. |
| Uncertainty / validation treatment | Conditional waveform family and repeatability/threshold timing bounds, probe bandwidth and trigger skew; retain censoring/nonsettled cases. |

<a id="m05"></a>
### M05: Local contamination, remanence and magnetic recovery

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | interference table/raw traces: coil/command/I, polarity/history, B_S_raw_T, B_S_cal_T, independent ambient/local B, off times, sensor acquisition/availability, geometry and temperature. Units: T; A; s; m; degC; A m^2 where independently measured. |
| Frame / time contract | S is flight sensor frame; independent ambient transformed into S. Characterize reference probe's coil pickup; retain changed post-burst baseline/remanence. Test simultaneous-coil response, not assumed superposition. |
| Exact future model destination | Future explicit sensor contamination/bias model, separate from WMM B_N and native torque. Do not apply local coil self-field as external Earth field producing spacecraft torque. |
| Uncertainty / validation treatment | Measured vector coupling, history/background/temperature dependence and correlated residuals; no invented contamination magnitude. |

<a id="m06"></a>
### M06: Quiet/settling validity rule

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; timing distribution; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | validity.json: approved criterion ID, entire effective acquisition window, earliest command-off-to-valid-window age by condition, current/field thresholds if applicable, coverage/margin, failures/exclusions. Units: s; T; A; declared error metric/coverage. |
| Frame / time contract | Quiet and settling use a common explicit origin; require full acquisition/filter history to meet criterion. Independent current/field transients may overlap; do not blindly sum measured durations. |
| Exact future model destination | Future MagneticCycleConfig.quiet/settling split represents one measured off-to-valid-acquisition requirement without double counting. Finite aperture/filter history needs explicit modeling or an approved bounded approximation; instantaneous TAM is not measured aperture. |
| Uncertainty / validation treatment | Repeatability, sensor filter memory and timing/measurement uncertainty enter margin by approved method; no numeric threshold/margin from this plan. |

<a id="m07"></a>
### M07: Flight sensor configuration/acquisition semantics

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; timing distribution; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | sensor.json/register dump plus packet/stimulus traces: source/variant, interface, raw/calibration state, output/filter settings, sample IDs, acquisition bounds/window and availability, staleness flags. Units: Hz; T; s; bit/s; packet units. |
| Frame / time contract | Distinguish acquisition start/reference/end, data-ready, transmission, receipt and decode; record clock origin/drift/wrap. If acquisition is unobservable, retain identified bounds. |
| Exact future model destination | Future SensorConfig/acquisition driver settings; present ideal TAM lacks real packet/filter/finite aperture behavior. Vendor 200 Hz is not selected output cadence or a 5 ms aperture. |
| Uncertainty / validation treatment | Measured filter/step response and epoch/window uncertainty; device capability alone cannot determine filter history or group delay. |

<a id="m08"></a>
### M08: Magnetometer calibration, bias and noise

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | calibration/residual table: raw/corrected B_S, reference B, orientation, temperature, scale/cross-axis/bias, residual covariance/PSD/autocorrelation, independent validation cases. Units: T; T^2 covariance; 1; Hz; T/sqrt(Hz); degC. |
| Frame / time contract | S with traceable C_SB; separate sensor offset/noise from background drift and actuator remanence; retain internal corrections and raw outputs. |
| Exact future model destination | Future bias/scale/noise or richer sensor model only after separate authorization. No estimator/noise implementation now; vendor density is not per-sample standard deviation without filter/bandwidth. |
| Uncertainty / validation treatment | Only measured supported distributions; preserve bias drift/correlation and distinguish instrument floor. Wider untested tails remain unsupported. |

<a id="m09"></a>
### M09: Acquisition-to-application events/delay/jitter

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; timing distribution; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | events table: run/cycle/sample/command IDs, event name, clock ID/ticks, mapped t_ns and uncertainty, load/mode; raw logic/current/packet/software traces. Units: s; integer ticks with declared clock scale. |
| Frame / time contract | Record acquisition, availability, compute start/end, publication, driver latch/on/off and current rise separately; calibrate clock skew/drift and trace overhead. Actual events, not target tick times. |
| Exact future model destination | Future sample_to_compute/compute_to_actuate after explicit event mapping; separate transport, queue, execution, publication, latch/current response to avoid double counting. Nonrepresentable timing requires separate work, not rounding. |
| Uncertainty / validation treatment | Empirical joint/conditional timing distributions or bounds with autocorrelation and tails supported by records; no independent Gaussian jitter invented. |

<a id="m10"></a>
### M10: Scheduler/burst/PWM command policy

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; calibration curve / lookup table; timing distribution; uncertainty distribution; validation-only evidence; TBD |
| Minimum record / units | schedule.json: firmware/topology/PrmDb/pinmux hashes, acquisition/manager/control/cycle clocks, BURST_TICKS/reset settings, carrier, replace/retrigger/expiry/OFF rules; observed traces. Units: Hz; ticks; s; duty fraction. |
| Frame / time contract | Manager tick, PWM carrier, burst fraction and physical current-on duration distinct. RESET_WAIT_TICKS is reset recovery, not magnetic settling. |
| Exact future model destination | Future cycle parameters and scheduling contract after feasibility review. Existing 0.1 s plant grid requires four distinct positive phases and >=0.4 s whole cycle; 5/10 Hz manager rates do not imply feasible complete cycles. |
| Uncertainty / validation treatment | Cadence/phase uncertainty tied to M09 and real firmware modes; ideal test cycle remains unchanged. |

<a id="m11"></a>
### M11: Independent integrated/HIL validation

| FIELD | DEFINITION |
|---|---|
| Class / current result status | validation-only evidence; TBD |
| Minimum record / units | validation manifest plus reserved input/output traces: measured current/field/dipole/power/events, inferred values labeled separately, native torque/analytical torque/simulated state; residuals and acceptance decisions. Units: T; A; A m^2; N m; rad/s; s; W; J. |
| Frame / time contract | Measured versus simulated truth explicit; gyro/nav injection on fixed fixture labeled substitution. Cage tracking/latency independently measured; feedback must not silently cancel coil contamination. |
| Exact future model destination | Validation only; no fitting/tuning on this set. m=f(I) is not independent measured dipole; m x B is not measured torque. Simulated rate is not hardware rigid-body validation. |
| Uncertainty / validation treatment | Use predeclared allocated residual/coverage limits with instrument/model uncertainty; physical torque/rate requires independent instrumentation. |

<a id="m12"></a>
### M12: Approved acceptance/requirement definitions

| FIELD | DEFINITION |
|---|---|
| Class / current result status | deterministic scalar/vector/matrix; validation-only evidence; TBD |
| Minimum record / units | acceptance register: criterion ID/revision/owner, metric/frame/window/limit/coverage/margin/envelope and source/conflict disposition. Units: T or declared metric; rad; s; W; J; fraction. |
| Frame / time contract | Magnetic allocation, alignment knowledge versus mounting, serial versus end-to-end latency and power boundaries explicitly defined. |
| Exact future model destination | Future validation/configuration metadata only after approval. RVM 3 degree orientation knowledge is not a measured mount tolerance; TBR energy budget does not define duty or 24-hour compliance. |
| Uncertainty / validation treatment | Approved uncertainty/coverage treatment; no numeric limit, confidence level or equipment tolerance invented. |

<a id="m13"></a>
### M13: Reproducible executed evidence package

| FIELD | DEFINITION |
|---|---|
| Class / current result status | validation-only evidence; TBD |
| Minimum record / units | manifest.json links immutable raw records, hardware/firmware/config/calibration revisions, procedure/results/signatures, frame/time/channel schema, analysis hash, fits/held-out sets, failures/exclusions. Units: Metadata; units declared for every measured channel. |
| Frame / time contract | Common event IDs and clock mappings retained; original raw data preserved along with derived tables and exact processing version. |
| Exact future model destination | Provenance/validation only. Every later candidate parameter needs measured/approved source, status, configuration/frame and revision; tracker/procedure flags never promote values. |
| Uncertainty / validation treatment | Instrument uncertainty/calibration validity and sampling/coverage limitations included; missing results remain TBD. |

## Sweepable versus blocked uncertainties

No sweep is run, no new range is generated, and no flight value is selected. **RESPONSIBLE PARAMETRIC SWEEP** means a proposed sensitivity design using the stated non-flight anchors; it does not demonstrate hardware admissibility.

Every mandatory row has one of the requested classifications. Multiple missing prerequisites may apply; the displayed class identifies the immediate blocker. Administrative record items are not numerical parameters and cannot be resolved by a sweep.

| BLOCKER | CLASSIFICATION | QUALIFICATION |
|---|---|---|
| [A02](#blocker-a02) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A03](#blocker-a03) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A05](#blocker-a05) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A06](#blocker-a06) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A08](#blocker-a08) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A09](#blocker-a09) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A10](#blocker-a10) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A12](#blocker-a12) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A13](#blocker-a13) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A14](#blocker-a14) | BLOCKED — NEEDS ACCEPTANCE THRESHOLD | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [A15](#blocker-a15) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [B01](#blocker-b01) | RESPONSIBLE PARAMETRIC SWEEP | Only the documented 5/10 Hz scheduler alternatives; not full-cycle periods or measured jitter. |
| [B03](#blocker-b03) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [B05](#blocker-b05) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [B06](#blocker-b06) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [B07](#blocker-b07) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C02](#blocker-c02) | BLOCKED — NEEDS ACCEPTANCE THRESHOLD | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C03](#blocker-c03) | BLOCKED — NEEDS ACCEPTANCE THRESHOLD | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C04](#blocker-c04) | BLOCKED — NEEDS ACCEPTANCE THRESHOLD | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C05](#blocker-c05) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C06](#blocker-c06) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C07](#blocker-c07) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C08](#blocker-c08) | BLOCKED — NEEDS ACCEPTANCE THRESHOLD | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C09](#blocker-c09) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [C10](#blocker-c10) | RESPONSIBLE PARAMETRIC SWEEP | Historical mode duty anchors only; non-flight, not thermal limits and not automatically feasible cycles. |
| [D01](#blocker-d01) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [D02](#blocker-d02) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [D03](#blocker-d03) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [D05](#blocker-d05) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [E01](#blocker-e01) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [E02](#blocker-e02) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [E03](#blocker-e03) | BLOCKED — NEEDS ACCEPTANCE THRESHOLD | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [E04](#blocker-e04) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [E06](#blocker-e06) | BLOCKED — NEEDS HARDWARE IDENTITY FIRST | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [F01](#blocker-f01) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [F02](#blocker-f02) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [F03](#blocker-f03) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [F04](#blocker-f04) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |
| [F05](#blocker-f05) | BLOCKED — NO DEFENSIBLE RANGE | No numeric range is authorized by this row. Documentary/record items require evidence, not numerical sweeps; calibration/threshold/identity prerequisites may also apply. |


| STUDY / QUANTITY | CLASSIFICATION / ANCHORS | DESIGN | LIMITS / METRICS |
|---|---|---|---|
| SW-01: Competing scheduler/control assumptions | RESPONSIBLE PARAMETRIC SWEEP; 5 Hz versus 10 Hz, with their respective documented contexts; no new intermediate physical rates. | Future controlled comparison changes only the declared scheduler hypothesis while freezing profile/gains/environment and replay/initial cases. First define manager, sample, control and complete-cycle rates independently. | Not a physical rate uncertainty distribution. Present 0.1 s-grid cycle requires >=0.4 s total; do not implement full 0.1/0.2 s cycles through silent rounding. Event/sample age, command hold/on fraction, validation failures and later short-run rate/energy sensitivity; no requirement-compliance claim. |
| SW-02: Existing architecture anchors | RESPONSIBLE PARAMETRIC SWEEP; Unchanged continuous regression and opt-in TEST-ONLY diagnostic: quiet0.2 s + settling0.2 s; instant sample0.4 s; compute0.5 s; apply0.6 s; burst0.4 s; period1.0 s; duty40%. | Only existing cases provide anchors; preserve their full phase relationship and label non-flight. No invented perturbation band around 0.1 s delays. | A single diagnostic delay does not define a physical sweep range. An ideal sensor comparison cannot quantify real contamination or accepted quiet time. Architecture invariants, actual command timing and labeled development response; simulations are not run in Phase 6C. |
| SW-03: Historical burst-duty hypotheses | RESPONSIBLE PARAMETRIC SWEEP; PDR80%; budget detumble50%, experiment20%, standby10%; retain mode/provenance. Phase6A40% is separately TEST-ONLY. | Future feasible burst/period pairs must honor every phase and clock constraint. Compare one effect at a time, then relevant coupled effects; do not fix period and silently shorten quiet/compute phases. | These are budget/analysis hypotheses, not permitted hardware duty/thermal ranges or PWM carrier duty. Reject infeasible timing combinations; do not force every anchor into the 1 s diagnostic cycle. Time-averaged applied moment, power/duty, sample age/validity and rate/energy sensitivity; no assumed flight acceptability. |
| SW-04: Actuator numerical capability envelope | BLOCKED — NEEDS HARDWARE IDENTITY FIRST; Conditional CR0002 vendor 0.2 A m^2 at5 V/20 degC, gain2.3 A m^2/A, R51 ohm. MT01A >0.395 at3.3 V / >0.790 at9 V conflict with heading; older MT01 >0.85 is separate. | Retain categorical evidence branches and exact source specifications. No numerical flight range for the in-house or MT01A branch is established. | Vendor lower bounds and incompatible variants/operating points cannot form min/max clamps or an uncertainty distribution. Actual selection/transfer/limits need measurements. After identity/calibration: delivered torque/saturation and power sensitivity over measured bounds. |
| SW-05: Quiet time, current/field settling and contamination | BLOCKED — NEEDS ACCEPTANCE THRESHOLD; No accepted physical range. Existing TEST-ONLY zero-command history is not such a range. | Collect independent current/local-field/filter response and approve a validity metric first; sensitivity only over supported measured conditions thereafter. | Do not invent residual T/A thresholds, decay constants, remanence or quiet-time bounds. Full-acquisition validity, residual field and time-to-valid-window with uncertainty. |
| SW-06: Aperture, sample age, compute/application delay and jitter | BLOCKED — NO DEFENSIBLE RANGE; No measured distribution/range; diagnostic0.1 s delays and vendor200 Hz capability do not provide one. | Use synchronized physical event records first. A later sensitivity generator preserves measured joint timing and phase constraints. | Phase6B identified potential sensitivity topics; Phase6C distinguishes a topic from an evidence-supported numerical range. Do not generate Gaussian jitter or arbitrary latency bands. Acquisition-to-current age, worst observed/bounded delay, command expiry and controller stability indicators. |
| SW-07: Physical mounting/alignment and sensor selection | BLOCKED — NEEDS HARDWARE IDENTITY FIRST; RVM3 deg knowledge bound and tighter B3 allocations are requirements/predictions, not measured actual error distributions. | Survey actual geometry/source first; later apply measured uncertainty and explicitly separate requirement-budget sensitivity. | No guessed separation, axis rotation, sensor count or stochastic mounting distribution. Frame/polarity consistency, delivered torque direction, magnetic validity and eventual pointing readiness. |
| SW-08: Sensor noise/bias and electrical/thermal dynamic extensions | BLOCKED — NO DEFENSIBLE RANGE; Vendor noise/ranges and thermal claims remain contextual capability evidence; no installed stochastic/dynamic envelope. | Future use of measured calibration/noise/transient distributions is conditional on MAG-002..006; no estimator implementation now. | Noise density requires filter/bandwidth before RMS conversion; approved limits and measured conditions must precede thermal/current sweeps. Residual validity/error, power/duty and model discrepancy; no ranges or simulations created here. |


The historical/test-only cases provide sensitivity anchors, not measured uncertainty ranges. In particular, Phase 6B's suggestion to study sample age or burst duration does not supply numerical bounds. Those physical ranges remain blocked here until measurements exist.

The current grid and distinct nonzero quiet/sample-compute/compute-apply/burst phases impose a minimum complete cycle of 0.4 s. A future experiment generator must reject infeasible combinations, preserve sample epochs, keep carrier duty separate from burst duty and report unsupported cases rather than forcing every historical duty onto the diagnostic 1 s period.

## Priority by expected model impact

These rankings are engineering judgments about uncertainty reduction, not measured sensitivity results. A wrong polarity or invalid sample may dominate otherwise smooth parameter sensitivities; rankings are conditional, not quantitative guarantees.

| MODEL OUTCOME | RANK | BLOCKERS | EXPECTED EFFECT |
|---|---|---|---|
| Detumble time | 1 | [A02](#blocker-a02), [A05](#blocker-a05), [A09](#blocker-a09), [A15](#blocker-a15), [C09](#blocker-c09) | Delivered magnetic moment and usable duty determine average control authority. |
| Detumble time | 2 | [B07](#blocker-b07), [C06](#blocker-c06), [C07](#blocker-c07), [E04](#blocker-e04) | Sample age/contamination alters command usefulness during a held burst. |
| Detumble time | 3 | [A03](#blocker-a03), [E02](#blocker-e02) | Axis/polarity errors redirect torque; a sign error can dominate every other term. |
| Magnetic-sample validity | 1 | [E03](#blocker-e03), [C04](#blocker-c04), [E04](#blocker-e04), [C05](#blocker-c05) | Need an error allocation and physical field/filter recovery across the whole aperture. |
| Magnetic-sample validity | 2 | [C03](#blocker-c03), [E01](#blocker-e01), [B05](#blocker-b05) | Current recirculation, installation geometry and device settings set observed contamination/history. |
| Magnetic-sample validity | 3 | [B06](#blocker-b06), [B07](#blocker-b07) | Correct source/epoch identifies whether the apparently clean sample is actually fresh. |
| Controller stability | 1 | [A03](#blocker-a03), [D02](#blocker-d02) | Wrong polarity/frame can reverse the intended control action. |
| Controller stability | 2 | [B01](#blocker-b01), [B07](#blocker-b07), [C06](#blocker-c06), [C07](#blocker-c07), [C09](#blocker-c09) | Cadence, age, jitter and hold/retrigger behavior affect closed-loop phase and authority. |
| Controller stability | 3 | [E04](#blocker-e04), [A09](#blocker-a09) | Biased field and nonlinear/history-dependent actuation change the assumed feedback/plant relationship. |
| Actuator saturation | 1 | [A05](#blocker-a05), [A09](#blocker-a09), [A12](#blocker-a12), [D05](#blocker-d05) | Actual variant, calibration, driver limiting and feedback determine achievable moment. |
| Actuator saturation | 2 | [A08](#blocker-a08), [A13](#blocker-a13), [A15](#blocker-a15) | Supply/temperature/duty limits constrain usable current and headroom. |
| Actuator power/duty | 1 | [A08](#blocker-a08), [A13](#blocker-a13), [A15](#blocker-a15) | Measured voltage/current/power and thermal envelope establish admissible operation. |
| Actuator power/duty | 2 | [C09](#blocker-c09), [C10](#blocker-c10), [D02](#blocker-d02), [D03](#blocker-d03) | Burst/carrier/off-state semantics change average power and transient heat. |
| Actuator power/duty | 3 | [A14](#blocker-a14) | System allocation/boundary reconciliation is needed before a compliance claim. |
| Eventual pointing readiness | 1 | [A03](#blocker-a03), [E02](#blocker-e02), [A09](#blocker-a09) | Known actuator/sensor frames, polarity and controllable moment are prerequisites. |
| Eventual pointing readiness | 2 | [E03](#blocker-e03), [E04](#blocker-e04), [B05](#blocker-b05), [B06](#blocker-b06) | Valid magnetic measurements and known data path are prerequisites for later estimation. |
| Eventual pointing readiness | 3 | [B07](#blocker-b07), [C06](#blocker-c06), [C07](#blocker-c07), [F04](#blocker-f04) | Timing and independent integrated evidence are needed before acquisition/hold development. |


Highest-return sequence:

1. Resolve installed actuator/driver/sensor identity and body-axis/polarity first; these define which tests and vendor statements apply.
2. Measure command-current-vector-dipole and electrical/thermal envelope together; this resolves delivered torque, saturation and usable duty.
3. Approve magnetic-error allocation and measure independent current/local-field/sensor recovery with a known acquisition window; this makes quiet time physically meaningful.
4. Trace actual acquisition-availability-compute-command-driver-current timing under flight-like workload; resolve the 5/10 Hz mapping and joint jitter.
5. Validate the frozen characterization on independent reserved integrated cycles; retain model discrepancy, failures and exclusions.

## Hard exit gate for hs2_candidate magnetic_cycle

**Do not create hs2_candidate magnetic_cycle until every REQUIRED gate is satisfied for a declared flight-relevant validity envelope and every P0 blocker is closed by measured/approved evidence. HIGHLY DESIRABLE items cannot substitute for REQUIRED ones; CAN REMAIN PARAMETRIC means bounded supported uncertainty, not invented values. A test with undefined criteria remains characterization-only, not PASS.**

All gates below remain unsatisfied; this plan records no new confirmation or measurement. Creating a provenance-bearing candidate later is separate from flight qualification, mission requirement verification and permission to modify runtime code.

| GATE / PREREQUISITE | CLASSIFICATION | MINIMUM EVIDENCE | TESTS / REASON |
|---|---|---|---|
| G01: Confirmed installed actuator/driver/sensor BOM | REQUIRED | Signed part/variant/serial/BOM/PCB/firmware/configuration and conflict dispositions. | [ADCS-MAG-001](#adcs-mag-001); Identity cannot remain a categorical guess in a named flight-candidate profile. |
| G02: Body axes, sensor/actuator transforms and polarity | REQUIRED | Released datum, survey/uncertainty, channel map and independently checked sign/direction; allocation and transforms consistent. | [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-006](#adcs-mag-006); An identity matrix or known design label is insufficient. |
| G03: Per-axis command-current-dipole calibration | REQUIRED | Measured usable transfer function, direction/nonlinearity/history limits and held-out validation with uncertainty within the declared domain. | [ADCS-MAG-002](#adcs-mag-002); No guessed gain or vendor lower-bound maximum clamp. |
| G04: Electrical interface and operating limits | REQUIRED | Verified driver/pin/OFF/carrier/feedback semantics and approved bus/winding voltage/current/power boundaries and limits. | [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004); Open-loop current control is permissible only if documented and characterized; a closed feedback loop is not invented as a requirement. |
| G05: Identified flight magnetometer and configuration | REQUIRED | Actual source/variant, connector/protocol/packet/filter/raw/calibration settings and minimum baseline error characterization for validity. | [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-006](#adcs-mag-006); Hardware maximum sample rate and generic sensor names do not close this gate. |
| G06: Acquisition aperture and timestamp semantics | REQUIRED | Measured or bounded physical acquisition/filter history and timestamp reference/availability, clock mapping and uncertainty. | [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007); Packet receipt cannot substitute for acquisition epoch. |
| G07: Magnetic allocation and measured current/field settling | REQUIRED | Approved cleanliness/error metric and independent current/local-field/sensor recovery establishing a full valid acquisition window with margins over the intended envelope. | [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006); Zero command/current alone does not prove clean magnetic data. |
| G08: Approved control rate and complete event map | REQUIRED | 5/10 Hz disposition plus separate manager/sample/control/burst mapping, reset/expiry/retrigger behavior and measured cadence. | [ADCS-MAG-007](#adcs-mag-007); A manager tick is not a whole-cycle definition. |
| G09: Measured sample-compute-application timing and jitter | REQUIRED | Actual component delays and supported joint jitter bounds/coverage under relevant load, including latch/current response and clock/instrument uncertainty. | [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007); Unmeasured timing cannot remain parametric simply to pass the gate. |
| G10: Thermal/burst/duty envelope and cycle power admissibility | REQUIRED | Signed admissible envelope under intended flight-relevant thermal boundary or justified conservative bound, with scope/exclusions and applicable cycle power allocation. | [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-008](#adcs-mag-008); Room-air data or historical duty estimates alone cannot approve a flight-like burst. |
| G11: Representable calibrated model and independent integrated cycle evidence | REQUIRED | Frozen calibration and predeclared limits pass reserved integrated physical/interface cases; current model represents measured behavior or a reviewed bounded approximation. Any needed software changes are separately authorized and verified. | [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007), [ADCS-MAG-008](#adcs-mag-008); No candidate is created through silent time rounding, omitted aperture, self-referential torque checks or a failed scalar actuator reduction. |
| G12: Provenance, reproducibility and team disposition | REQUIRED | Executed record packages, calibration/analysis revisions, raw traces, failures/exclusions and ADCS/Systems/CDH/EPS/Structures/Thermal review as applicable. | [ADCS-MAG-001](#adcs-mag-001), [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-004](#adcs-mag-004), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007), [ADCS-MAG-008](#adcs-mag-008); All values retain provenance/status; every applicable P0 blocker has an accepted resolution. |
| G13: Independent physical torque/rate and expanded dynamical HIL correlation | HIGHLY DESIRABLE | Calibrated physical torque/free-body/rate measurements with fixture inertia/friction uncertainty and independent truth; accurately scoped channel substitutions. | [ADCS-MAG-008](#adcs-mag-008); Strengthens plant/performance validation. Absence forbids physical rigid-body performance claims; the limited integrated-cycle gate still requires real sensor/driver evidence. |
| G14: Longer-term drift/aging and wider environmental characterization | HIGHLY DESIRABLE | Additional measured environmental/aging/long-duration records beyond the accepted candidate envelope. | [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-008](#adcs-mag-008); May extend validity later; any condition actually required by the initial candidate envelope is REQUIRED, not deferred here. |
| G15: Residual calibrated parameter and timing uncertainty | CAN REMAIN PARAMETRIC | Measured uncertainty/bounds/distributions and declared correlations inside the accepted envelope; sensitivity demonstrates robustness to those supported variations. | [ADCS-MAG-002](#adcs-mag-002), [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-005](#adcs-mag-005), [ADCS-MAG-006](#adcs-mag-006), [ADCS-MAG-007](#adcs-mag-007), [ADCS-MAG-008](#adcs-mag-008); The distribution's supported variability may remain parametric; nominal identity, limits, clean window and timing bounds may not remain unknown. |
| G16: Later mission-level energy/performance hypotheses | CAN REMAIN PARAMETRIC | Retain unresolved global energy/mission assumptions with statuses; release before the relevant mission verification campaign. | [ADCS-MAG-003](#adcs-mag-003), [ADCS-MAG-008](#adcs-mag-008); No 24-hour detumble, energy or pointing compliance is claimed. This does not relax required local cycle power/thermal limits. |


The twelve REQUIRED gates include confirmed BOM/axes/polarity, per-axis calibration, electrical and thermal/duty limits, identified/configured flight magnetometer, physical aperture/timestamps, measured contamination/settling against an approved allocation, approved control rate/event map, measured delays/jitter, model representability, independent integrated evidence and reproducible signed disposition. Unknown timing jitter cannot be reclassified as harmless parametric freedom.

## Verification scope

Validate the companion JSON structure; unique/exact coverage of all 39 Phase 6B mandatory IDs and the 31/5/3 priorities; source candidates/statuses/revisions preserved; all tests linked to blockers; every P0 row has a concrete evidence/owner/test/model-output path; timing quantities distinct; allowed sweep/input/gate classes; internal links/anchors and Markdown/JSON consistency; source hashes, whitespace and the two-file allowlist.

Only [this plan](ADCS_MAGNETIC_TEST_PLAN.md) and [the structured plan](../basilisk_runner/config/adcs_magnetic_test_requirements.json) are added. No simulations/regression runs are needed for this documentation-only change. No test execution, successful hardware result, flight-candidate cycle or commit is claimed.
