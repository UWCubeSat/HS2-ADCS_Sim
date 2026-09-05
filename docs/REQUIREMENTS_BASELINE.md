# ADCS requirements baseline and conflicts

Audit-derived requirements register, 2026-09-05. This is a record of candidate requirements and contradictions, **not an approved consolidated flight baseline**. No simulation thresholds were changed.

Document titles, exact Drive files, revisions, dates, and approval limitations for R1-R3, M0-M2, I1-I5, B1-B4, H1-H5, SW1, OP1, P1, T1, and T2 are in the [source register](PHYSICAL_PARAMETERS.md#source-register).

## Authority and interpretation

Approved/revision-controlled requirements govern obligations; released ICD/CAD/BOM records govern configuration; applicable manufacturer specifications govern component capability; calibrated signed tests establish demonstrated performance. Working drafts and analysis budgets can expose needed changes, but do not automatically amend approved requirements. Newest modification time is not authority.

R1 is the controlled-folder verification matrix, yet relevant ADCS checks remain incomplete. R2 has approved older history entries but blank D/E approval fields and an inconsistent REQ-001/REQ-002 cover. The reviewed R3 ADCS/orbit values agree with R1. I1 is detailed and revision-tracked but internally contradictory and unsigned in its approval fields. B3 supplies a detailed pointing derivation without a located release/acceptance record.

Use CONFIRMED only for the stated documented obligation/fact, not achieved compliance. TBR/TBC/TBD/ASSUMED follow AGENTS.md. Requirement IDs are source-specific: M1 renumbers some ADCS entries relative to R1 after omitting the energy row. Preserve the source and wording, not the ID alone.

## Requirements and conflicting candidates

| AREA / REQUIREMENT | REQUIREMENT OR CANDIDATE A | CONFLICTING / RELATED CANDIDATE B | UNITS | STATUS | SOURCE / SECTION / REVISION | INTERPRETATION / OPEN DECISION |
|---|---|---|---|---|---|---|
| Detumble, R1 ADCS-1 | From at least10 to0.5 within24 h after deployment | RVM comment mentions30 deg/s theoretical release rate; MDD calls10 worst case | deg/s;h | CONFIRMED | R1 ADCS-1;R2 section4.5;M1 section4.1; source register | The requirement statement agrees. Initial norm/per-axis definition, direction distribution, deployment delay, and sustained success criteria remain TBC. |
| Detumble duration prediction | MDD approximately110 min | PDR average2.11 h | min;h | ASSUMED | M1 commissioning;P1 slide19 | Predictions, not requirement changes or verified worst-case bounds. |
| Experiment angular rate, R1 ADCS-3 | Acquire/maintain <0.5 TBC in experiment mode | Repo comparisons use0.05 rad/s first crossing (approximately2.865 deg/s) | deg/s;rad/s | TBC | R1 ADCS-3;R2 section4.5; compare_reference_vs_basilisk.py at833b015 | Repo threshold is a different development check. Define vector norm versus components, dwell/hold time, exposure interval, and confidence level. |
| Pointing, R1 ADCS-4 | 20 deg, worded accuracy of at least20 degrees | B3 10 deg;B1/B2 total allocation15 deg;R2 derivation42 deg TBR;B4/MDD prose43 deg | deg | TBC | R1 ADCS-4;R2 section4.5;B1 Pointing Budget F12;B3 section2.1;B4 section2.1;M1 section4.1 | No approved supersession found. Define error <= limit, boresight versus full attitude, reference motion, confidence, duration, and keep-out context. |
| Pointing prediction | B3 combined5.5517;control residual5 at3-sigma | Spreadsheet total0.303924 with thermal/control terms zero | deg | ASSUMED | B3 sections2.1-2.9;B1/B2 Pointing Budget | Neither is demonstrated performance. Zero modeled errors cannot be interpreted as physical absence. |
| Sun keep-out, R1 ADCS-5 | >=55 when cameras on | R2/T2 >=70;RVM legacy Twinkle note90;payload keep-out fields TBD | deg | TBC | R1 ADCS-5;M1 section4.1;R2 section4.5;T2;R1 PAY-19 | Confirm selected cameras/lenses/baffles/Sagitta and define centerline versus FOV-edge exclusion, uncertainty margin, and maneuver safety. |
| Slew-rate ceiling | I1 software enforces0.04 | No identified parent requirement; repo development rates/gains are not that requirement | deg/s | TBC | I1 sections4.1.7-4.1.8,Drive rev16403 | Clarify whether commanded, actual, axiswise, or norm ceiling and which modes it covers. |
| 180-degree slew | No explicit spacecraft180 deg slew requirement found | References to180 deg orbital sampling locations are not spacecraft attitude slews | deg;time TBD | TBD | Reviewed R1/R2/M0/M1/I1/OP1 | Need target rotation, completion time, trajectory constraints, and post-slew settling if such a requirement is adopted. |
| Control-loop rate | MDD/SW1 5 Hz,200 ms tick | I1 10 Hz,100 ms;repo0.1 s task | Hz;ms | TBC | M1 section2.4;SW1 sections2.2-3.1;I1 section4;repo833b015 | I1 is more specific current interface evidence, SW1 explicitly outdated; approval and measured schedule still needed. |
| Actuator alignment, R1 ADCS-6 | Orientation relative to body known within3 | B3 prediction0.10/allocation0.25 | deg | TBC | R1 ADCS-6;I1 section2.1;B3 section2.6 | The3 deg documented bound is clear; proposed tighter allocations are not an approved replacement or measured result. Knowledge error is not automatically mounting tolerance. |
| Sensor alignment, R1 ADCS-7 | Orientation relative to body known within3 | B3 prediction0.25/allocation0.50 | deg | TBC | R1 ADCS-7;I1 section2.2;B3 section2.4 | Define all sensor-to-body transforms and calibration uncertainties; body axes themselves remain TBD. |
| LOST/FOUND physical angle | 90 +/-1 | Body mounting face stillTBD | deg | CONFIRMED | R1 PAY table row11;R2 section4.7;I3 section2.2 | Documented relative mounting requirement only; inspection/calibration not completed. |
| LOST/FOUND knowledge | R1 PAY-8:0.05 | R1 PAY-9:0.04 TBR;older R2:0.05 TBR | deg | TBC | R1 PAY-8/9;R2 section4.7 | Contradiction exists within current RVM; preserve both entries and their derivations. |
| Camera/truth knowledge | R1 FOUND/TM0.04 TBR;LOST/TMTBD | R2 FOUND/TM0.1 TBR,other passages0.05/TBD;B3 truth alignment0.05 predicted/0.10 allocated | deg | TBC | R1 PAY-10/11;R2 section4.7;B3 section2.5 | Different pairs and budget terms must not be combined as if they were the same quantity. |
| Detumble energy, R1 ADCS-2 | <=11 TBR | R2 derivation60 TBR;PDR estimated6.27 | Wh | TBR | R1 ADCS-2;R2 section4.5;P1 slide20;T2 | Need an approved energy allocation and measurement boundary. EPS charging60 Wh is a separate obligation. |
| ADCS power, R1 ADCS-10 | <2 TBR detumble;<0.4 TBR standby | PDR torquer estimate2.97;I1 air-coil maximum13.2 versus total ADCS peak12 | W | TBR | R1 ADCS-10;P1 slide20;I1 section1.1 | Peak versus time-average and winding versus bus power are not reconciled. Resolve variant, drive, duty cycle, losses, and thermal limits. |
| Solar recovery/charging |60 Wh over72 h after deployment, assuming10 deg/s aboutZ | Battery budget75.6 Wh;solar deployment/attitude assumptions unresolved | Wh;h;deg/s | TBC | M1 EPS requirements;B1 Power Generated | Define initial state of charge, eclipse/deployment timing, panel angles, usable battery energy, and generation model. |
| Solar pointing | Two solar faces oriented toward Sun | B1 +X/Y labels;B3 approximately30 deg deployed sensor axes;no pointing-error bound | orientation | TBD | M1 commissioning;R1 STR-2;B1;B3 section2.4 | Requires actual panel normal vectors and an attitude/power acceptance definition. |
| COMMS pointing | Establish ground link with two patch antennas | No numerical pointing-error/coverage requirement found;mounts TBD | deg/link margin TBD | TBD | R1 SAT/COM;M1 commissioning;I4;OP1 section6.4 | Derive attitude-dependent antenna coverage and link constraints from installed patterns and mission passes. |
| Experiment targets | FOUND Earth limb;LOST/truth tracker star field | No complete simultaneous body-frame attitude/roll and keep-out solution | vectors | CONFIRMED | M1 section2.4;I3 | Mission intent clear; numerical acquisition/hold trajectory and visibility criteria remain TBD. |
| Camera FOV | LOST20-25 TBR;FOUND78-90 TBR | B3 25/76;M1 FOUND110-140 | deg | TBR | R1 PAY-2/4;B3 section2.1;M1 section4.1 | Must resolve selected optical configuration before deriving allowable pointing error. |
| LOST accuracy | B3 threshold30 cross/60 around,objectives9/50 at3-sigma | MDD minimum-success passages retain180 cross/360 around and other full-success values | arcsec | TBC | B3 section3;M1 experiment/success criteria | Distinguish minimum/full/objective criteria and approve the intended experiment-level comparison; not spacecraft pointing degrees. |
| FOUND position accuracy | RVM500 TBR;B3 threshold500/objective300 at3-sigma | Older MDD passages10 km TBR;B3 prediction291.60 | m;km | TBC | R1 PAY-5/14;B3 section4;M1 integration | Clarify applicable success tier and estimator/camera/truth chain;prediction is not compliance. |
| Attitude knowledge | B3 pointing estimate10;FOUND input5 | RVM attitude knowledge fieldsTBD;no located independent estimator/HIL results | arcsec | ASSUMED | B3 sections2.8,4.3;R1 PAY-20 | These are analytical predictions, not demonstrated VN-100/solar/magnetic estimator capability. |
| LOST/FOUND exposure timing | Within10 | B3 camera/attitude timing prediction0.50 is a different chain/term | ms | TBC | R1 PAY-12;B3 section4.4 | Define exposure midpoint, triggering, timestamp source, synchronization versus latency, and jitter. |
| LOST/truth synchronization |15 angular error allocation | Not15 seconds or15 milliseconds | arcsec | CONFIRMED | R1 PAY-13;B3 section3 | Convert to time only using a defined angular-rate envelope and measurement epochs. |
| FOUND/GNSS synchronization | Less than +/-65 | B3 prediction3.0;VPP not wired,BBB/F' timestamps proposed | ms | TBC | R1 PAY-15;B3 section4.6 | Measure clock offset, transport delay, rate-group jitter, and GNSS/exposure epochs. |
| FOUND/attitude-truth synchronization | Less than +/-10 | B3 prediction0.50 | ms | TBC | R1 PAY-16;B3 section4.4 | Budget predictions need a traceable hardware timing test;not arbitrary resampling. |
| Magnetic sampling quiet time | Coils idle after burst for clean measurement | No numeric burst/settling values;active repo continuously holds algebraic current | ticks;s | TBD | I1 sections4.1.3,4.2.4;repo833b015 | Behavioral requirement exists;sample phase, current-decay threshold, and contamination limit not defined. |
| Deployment delays | RVM30 min for deployables;45 min before RF | Final UNP/launch-provider delays pending;MDD approximate5 min deployment phase is not a panel-release time | min | TBC | R1 CalPoly rows53-54;UNP12-96;M1 commissioning | Incorporate actual release sequencing in detumble and power verification. |
| ADCS mass/volume | <300 TBR; MTA <=10x10x3 | B1 ADCS subtotal158.5 is an estimate, not weighed flight hardware | g;cm | TBR | R1 ADCS-8/9; B1 Mass Budget | Mass/volume allocation, not confirmed as-built compliance. |
| Thermal operation | TensorCSS operating-25 to70 | HS-2 tables label-40 to100 operating,which manufacturer identifies as survival | C | TBC | H5 section2;I1 section3;B2 Thermal Budget | Manufacturer operational/survival distinction governs the component;system documents still need correction/approval. |

## Present verification does not close this register

- R1 ADCS verification entries are incomplete; T1 marks ADCS PCB, rods, IMU, and sun-sensor unit tests not done. T2 result fields are blank.
- The repo's 0.05 rad/s threshold, 3000-3500 s time window, <=2.6 W coil-power check, and direct-torque <1 deg pointing check are ASSUMED development acceptance thresholds, not adopted HS-2 requirements.
- Detumble energy must be integrated over the agreed duration and electrical boundary. A peak I^2 R check is not an energy requirement test.
- First threshold crossing is not sustained rate/pointing compliance. Record dwell duration, reacquisition, exclusions, and violations.
- A budget paragraph stating verification by Monte Carlo/HIL does not substitute for linked inputs, code revision, measurements, calibration, raw results, and signed acceptance.
- Input truth contamination, self-referential applied-torque columns, unknown frame definitions, and inconsistent epochs prevent treating current passes as independent validation.

## Open decisions required for release

The responsible project authorities must establish applicable requirement revisions and success tiers; settle pointing/Sun limits; define rate norm/hold times and confidence levels; approve optical/hardware configuration; resolve 5/10 Hz and quiet-time schedules; and define power/timing measurement boundaries.

No arbitrary winner was selected in this documentation pass. These questions do not block an accurate conflict register; they block releasing a flight-performance baseline. Follow the ordered phases in [RECOVERY_ROADMAP.md](RECOVERY_ROADMAP.md).
