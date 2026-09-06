# Physical parameters and provenance

Audit-derived record, 2026-09-05. Repository anchor: 833b015 on adcs-sim-recovery-2026. This document preserves the completed repository/follow-up and HS-2 Shared Drive audits. It does not select a released flight configuration or alter any constant.

Statuses follow [AGENTS.md](../AGENTS.md): CONFIRMED, TBR, TBC, TBD, ASSUMED. A CONFIRMED vendor rating or written requirement is not confirmation of installed hardware or achieved flight performance. All repository-only spacecraft/control values without HS-2 design traceability remain ASSUMED. Contradictions are retained as separate values.

Phase 4 runtime source: [hs2_sim_config.py](../basilisk_runner/hs2_sim_config.py) centralizes the active detumble inputs and provenance while preserving Phase 3 (`c1b6021`) numerical values. Saved runs include a configuration JSON and fingerprint. The audit tables below remain historical evidence; candidate requirements and the 3.72911 kg estimate are not automatic runtime replacements.

## Phase 5 explicit physical profiles

`regression_baseline` remains the default with its original configuration JSON/fingerprint and 2.6 kg box. `--profile hs2_candidate` explicitly selects a sensitivity case in [hs2_sim_config.py](../basilisk_runner/hs2_sim_config.py). Every other runtime section remains identical. Names are recorded outside the fingerprint in separate `_run.json` manifests; candidate CSV/config/plot names are separate from the regression outputs. This is **NOT FLIGHT VALIDATED**.

| CANDIDATE PARAMETER | VALUE / UNITS | STATUS | SOURCE / REVISION / LIMITATION |
|---|---|---|---|
| Mass | 3.72911 kg | ASSUMED | B1 Mass Budget E43, modified 2026-09-03; B2 same roll-up. Release revision not established; estimated/TBD/old hardware entries remain. Not released measured mass. |
| Form factor / envelope | 3U intent; 0.100 x 0.100 x 0.3405 m used in this case | CONFIRMED intent; TBC exact dimensions | R1 STR-9 (tracker rev 9, 2026-03-21); M1 4.6 (log rev 8, 2026-08-17). Conflicts retained: MDD 340 mm and B2 volume-budget structure 338.6 mm; no arbitrary reconciliation. |
| Model COM | [0,0,0] m | ASSUMED | Origin equals the uniform-prism centroid only for this model. Actual COM remains TBD under I2/S1; no measurement exists in the audited evidence. |
| Model inertia about COM | diag(0.03913708713979167, 0.03913708713979167, 0.006215183333333334) kg m^2; products zero | ASSUMED | Phase 5 derivation, 2026-09-06: uniform rectangular prism from the candidate mass/envelope, shown at runtime float precision. Geometry-only placeholder, independently checked by volume integration. Does not use B1's isolated 0.05 kg m^2 entry. |

Dimensional assignment is mathematical B X=0.100 m, Y=0.100 m, Z=0.3405 m; it does not establish physical HS-2 axes. Uniform density, principal-axis alignment, centroid=COM, and zero products are assumptions. Released CAD mass properties must still establish COM, full tensor, reference point, axis mapping, and deployed configuration. A one-orbit comparison does not verify R1 ADCS-1 (at least 10 deg/s to 0.5 deg/s within 24 h); norm/per-axis interpretation, direction envelope, and sustained-success criteria remain TBC.

## Source register

Source codes below carry the exact Drive file, title, engineering revision, date, and authority limitations for the parameter and requirements tables. Drive modification dates are observations, not release dates. All Drive evidence was read within Husky Satellite Lab (0AC8raEVvalZjUk9PVA); no Drive changes were made.

| CODE | DOCUMENT / GOOGLE DRIVE FILE | REVISION / DOCUMENT DATE | AUTHORITY / LIMITATION |
|---|---|---|---|
| R1 | [Working RVM](https://docs.google.com/spreadsheets/d/11IF6Z258qsOcPkMAP1GGssFSR6CSEYRHTrowt0nhDzM/edit) | Cover 2025-09-28; tracker through rev 9, 2026-03-21; modified 2026-09-03 | Controlled-folder traceability source; unsynchronized dates; ADCS verification incomplete. |
| R2 | [HS-2 Requirements Document](https://docs.google.com/document/d/1ndJqcboajU4Jl_xzJY8hvECOCxZaLUuu2n2bvZ4lSzM/edit) | REQ-001/REQ-002 both appear; rev E 2025-10-05; Drive rev 34576, 2026-07-08 | A-C have named approvals; D/E approval fields blank. Tables and derivations conflict. |
| R3 | [CDR RVM](https://docs.google.com/spreadsheets/d/1IdPXY-JAhdWJwn6VxP6_rIqVG_30bHNx-A5q65oHmkU/edit) | CDR copy; modified 2026-09-03; approved release not established | Reviewed ADCS/orbit values match R1. |
| M1 | [Working HS-2 Mission Design Document](https://docs.google.com/document/d/1ERQ7L8lpUAOpvaVY-v4of7ee1vO7vtSrgt6Sk4kwgAU/edit) | Cover MIS-00-06, 2025-11-05; log through rev 8, 2026-08-17; modified 2026-09-02 | Controlled-folder mission source with stale/conflicting hardware and requirements. |
| M2 | [CDR HS-2 Mission Design Document](https://docs.google.com/document/d/1vc8vGf2CylgD8v8OVZiItkl5buHTeDvqFJDnShdzgDY/edit) | Same MIS-00-06 cover; modified 2026-09-01 | Checked passages retain principal M1 conflicts. |
| M0 | [HS-2 MDD 8-5-25.pdf](https://drive.google.com/file/d/1hXeDt21JFeOmiuBLt5lsYiR4msW9CmcZ/view) | HS2-MIS-00-03; log rev 3.4, 2025-08-03; file 2025-08-05 | Historical controlled-folder snapshot; contains reaction-wheel modes. |
| I1 | [ADCS ICD](https://docs.google.com/document/d/1hLtnfIFSr0lpA1G06h1QUmzP6TUkJwez6NtpzFYbtBY/edit) | ICD-ADCS-001; cover 2026-02-06; internal rev 5, 2026-09-01; Drive rev 16403, 2026-09-04 | Strong model-specific selection/interface evidence; approval fields blank and internal conflicts unresolved. |
| I2 | [STR ICD](https://docs.google.com/document/d/19OYepvNrMGA2Q5ppwQZsvyE5AeaLBhPCMuBoY2KrNd0/edit) | ICD-STR-001; cover 2026-07-07; rev 3 dated 2026-07-15; modified 2026-08-25 | Largely template; detailed mounting sections cited elsewhere absent. |
| I3 | [PAYLOAD ICD](https://docs.google.com/document/d/1lNaU3924o3YauIkrCbEoAxai42gTTooRNMTJWuaQLno/edit) | ICD-PAYLOAD-001/ICD-PAY-001; cover 2025-10-06; Drive rev 13628, 2026-09-03 | Rev 3 history says 2026-09-29, later than audit; camera model variants conflict; geometry incomplete. |
| I4 | [COM ICD](https://docs.google.com/document/d/103fdkT5sri2t1deSvI38SxKiVpJd6wFFb2oIYmZJlxw/edit) | ICD-COM-001 rev 3, 2026-07-07; modified 2026-09-03 | Two antennas described; mounting brackets/directions pending. |
| I5 | [EPS ICD](https://docs.google.com/document/d/1bnfNJGvO283E8RFnzkQVy5H58ZAzqiigyk1cTYZS-WQ/edit) | ICD-EPS-001 rev 3, 2026-07-15; modified 2026-09-03 | Defines electrical/deployment interfaces, not complete deployed geometry. |
| B1 | [OFFICIAL UNP BUDGETS](https://docs.google.com/spreadsheets/d/1QjB3b6N_8ApnokAY7mx5-lb7i-Ln06A943hU4ePlcwg/edit) | Template cover 2025-07-31; modified 2026-09-03; HS-2 release revision not established | HS-2 entries mixed with educational ShipSat template text, estimates, old hardware, and unresolved cells. Filename does not establish approval. |
| B2 | [HuskySat2 CDR Technical Budgets Draft](https://docs.google.com/spreadsheets/d/1hy7b_YhNAb9_2k_P1YpDi65O4OrNPHfZdlsP3fRRqwE/edit) | Draft; modified 2026-08-27 | Mass/pointing agree with B1; experiment power differs. |
| B3 | [Pointing_Payload_Budgets_CDR.docx](https://drive.google.com/file/d/1ruAMFUOjwCCmeaTXRkqRnvfjvYuLqsIY/view) | No explicit release ID/date/approval found; modified 2026-09-03 | Detailed budget candidate; asserted verification not backed by located completed MC/HIL records. |
| B4 | [Pointing_Payload_budgets.pdf](https://drive.google.com/file/d/1x53I07dr98MjgMUzaPj3JCJwbdOZVdl8/view) | Internal revision/date not established; modified 2025-09-27 | Historical derivation, still linked by B1 despite differing numbers. |
| S1 | [Structural Analysis Report (CDR)](https://docs.google.com/document/d/1GQgMkH5wiUpsO43QypF9Y3pa84cakhFgIAd8xlZazQ4/edit) | Released ID/revision/date not established; modified 2026-08-18 | Body describes preliminary simplified ISISpace analysis, not certified current GranSystems mass properties. |
| S2 | [Assembly Procedure](https://docs.google.com/document/d/1N8c_Cdke-shybAeaORkgEBtXeVX5PAKX4PMQoI_tHNo/edit) | Cover 2026-09-18, future-dated at audit; modified 2026-08-18 | Empty Axis Definition section confirmed from native document structure. |
| SW1 | [ADCS Software Design Document](https://docs.google.com/document/d/15a2GrN6oSJ5obUN1S6sxZ1JvkFbdp3SOwh22T2Br-Ug/edit) | No numbered release/date; modified 2026-04-21 | RTEMS/F' proposal explicitly marked slightly outdated and not wholly accurate. |
| OP1 | [CDR Mission Operations Plan](https://docs.google.com/document/d/1qFJCzb4mRiTEUGQtsd-Gb1qKJJWku_0albkJb_S2UpE/edit) | MOP-001, cover 2026-08-20; history through 2026-08-28 | ADCS, power/thermal, payload operations sections unfilled; flight rules preliminary. |
| P1 | [ADCS Preliminary Design Review](https://docs.google.com/presentation/d/1bWG-Q-g4478csJb8kl6YpiZLU0KtIes1wHr5YpLo8NI/edit) | No numbered release/cover date established; modified 2025-12-13 | Historical simulation and sizing estimates, not acceptance evidence. |
| H1 | [CubeSpace CR0002 datasheet](https://drive.google.com/file/d/1aPEP8ZH1pR268A1Y9b7ZGr2sMazNASKM/view) | Internal revision/date not found; modified 2026-07-29 | Vendor family-table CR0002 values; I1 separately establishes candidate selection. |
| H2 | [MT01 datasheet.pdf](https://drive.google.com/file/d/1LU4-HwiNK-LN4jqoV1HWQ0DZbjjopd1W/view) | MT01A content; internal revision/date not found; modified 2026-07-29 | Linked by I1; internally inconsistent voltage/current/performance statements. |
| H3 | [MT01 User Guide](https://drive.google.com/file/d/1ScHLr4ZUKw3jq0-InU8G66odAAlWNRJl/view) | MT01 Compact rev B, 2020 | Older/different variant; not interchangeable with H2 without confirmation. |
| H4 | [VN-100 Rugged datasheet](https://drive.google.com/file/d/1LEekbzz2LuTFYMBSQvNY-sX1FVPAtqm2/view) | DS100-CR-70-R1; footer 25-100-CR-70-R1; copyright 2023; issue date not established | Vendor specification for rugged model selected in I1. |
| H5 | [TensorCSS-10/10S datasheet](https://docs.google.com/document/d/1eavCpJA7YkNcuhxe0hNQhgPKT_kznHYu5U_ah01YpyQ/edit) | v1.0.2c; product v1.0; 2025-08-06 history entry | Manufacturer revision/approval present; separates operating and survival limits. |
| T1 | [Hardware Test Tracking](https://docs.google.com/spreadsheets/d/1RwvsRkzPaZftcbcFuLMexglIOU14P74Qi7rl-4YHkvs/edit) | No released revision; modified 2026-09-01 | Recorded procurement/test state, not signed acceptance results. |
| T2 | [ADCS Avionics Verification Procedures](https://docs.google.com/document/d/1MsIS6hL0CcfJpajEntyaY438vQCkFLmYpLsk5qAA-rA/edit) | HS2_ADCS_TEST-001 through -007; modified 2026-07-30; execution dates/results blank | Planned verification, not completed performance evidence. |

Local source aliases: L1 = [minimal](../basilisk_runner/scenario_huskysat2_minimal.py), L2 = [detumble](../basilisk_runner/scenario_huskysat2_detumble.py), L3 = [adapter](../basilisk_runner/basilisk_adcs_adapter.py), L4 = [pointing](../basilisk_runner/scenario_huskysat2_pointing.py), L5 = [standalone context](../reference_standalone/original_project/include/sim_context.hpp) and [initialization](../reference_standalone/original_project/src/satellite.cpp). These references are evaluated at 833b015, not at a future checkout.

## Spacecraft, frames, and geometry

| PARAMETER | VALUE | UNITS | STATUS | SOURCE | REVISION/DATE | NOTES/CONFLICTS |
|---|---|---|---|---|---|---|
| Current repo mass | 2.6 | kg | ASSUMED | L1/L2 constants; L5 initialization | 833b015 | Legacy ASSUMED value, not released flight mass. Pointing reuses L2. |
| Current repo dimensions | 0.1 x 0.1 x 0.2 | m | ASSUMED | L1/L2; L5 | 833b015 | Legacy ASSUMED uniform 2U-sized box; conflicts with documented HS-2 3U configuration. |
| HS-2 form factor | 3U | U | CONFIRMED | M1 section 4.6; R1 STR-9 | M1 rev 8 / R1 register | Documented design intent, not complete as-built geometry. |
| Mass-budget estimate | 3.72911 | kg | ASSUMED | B1 Mass Budget E43; B2 same roll-up | B1 modified 2026-09-03 | Explicitly NOT released flight mass. Assumed PCB masses, TBD cells, and old camera/lens hardware remain. |
| Mass-budget margin / limit | 4.6613875 including 25%; maximum allowable 6 | kg | ASSUMED | B1 Mass Budget E44:E45 | B1 register | Margin is not physical mass; final launch/dispenser limit applicability TBC. |
| Primary structure mass | 0.54 in budget; approximately 0.580 in MDD | kg | TBC | B1 row 4; M1 section 4.6 | B1 / M1 register | Both describe GranSystems; not silently reconciled. |
| Structural subtotal/allocation | 0.97792 subtotal; 0.400 TBR allocation; 0.440 budget limit with margin | kg | TBC | B1 rows 3,55; R1 STR-10 | B1 / R1 register | Subtotal exceeds structural allocation/limit; no release decision found. |
| External dimensions | 100 x 100 x 340.5 requirement; 100 x 100 x 340 MDD; 100 x 100 x 338.6 volume-budget structure | mm | TBC | R1 STR-9; M1 section 4.6; B2 Volume Budget | Source register | Envelope and component dimensions need configuration-specific reconciliation. |
| Actual spacecraft COM | TBD | m in declared body frame | TBD | I2; S1; B1 | Source register | No authoritative numerical COM found. |
| Repo COM offset | [0,0,0] | m | ASSUMED | L1/L2 configure_spacecraft, r_BcB_B | 833b015 | Modeling origin/COM coincidence only, not HS-2 measurement. |
| Actual Ixx, Iyy, Izz | TBD, TBD, TBD | kg m^2 | TBD | I2; S1; B1 | Source register | Need full mass-properties export with frame/origin/configuration. |
| Actual full inertia tensor/products | TBD, including Ixy, Ixz, Iyz and tensor sign convention | kg m^2 | TBD | I2; S1 | Source register | Do not substitute a diagonal box matrix for the physical tensor. |
| Repo inertia | diag(0.0108333333,0.0108333333,0.0043333333); off-diagonals zero | kg m^2 | ASSUMED | L1/L2/L5 uniform-box formula m/12 times squared side lengths | 833b015 | Derived from legacy 2.6 kg box; not CAD/flight mass properties. |
| Budget inertia input | Largest moment 0.05 | kg m^2 | ASSUMED | B1 Pointing Budget I13 | B1 register | Axis/origin/CAD provenance not supplied; not a complete tensor. |
| Coordinate origin requirement | Geometric center via CubeSat coordinate-system requirement | — | CONFIRMED | R1 CalPoly CAL2.2.1, row 14 | R1 register | Geometric center need not equal COM. |
| Physical +X/+Y/+Z directions | TBD | frame definition | TBD | I2; S2 Axis Definition | I2 / S2 register | Body labels in code/budgets do not establish physical mapping; S2 section empty. |
| LOST boresight | Lateral-plate mounting; body vector TBD | unit vector / rotation | TBD | I3 section 2.1 | I3 register | No complete optical-to-body transform. |
| FOUND boresight | 90 deg relative to LOST intended; chosen +X/-X/+Y/-Y face TBD | deg / unit vector | TBC | I3 section 2.2; R1 PAY row 11 | I3 / R1 register | Physical mounting 90 +/-1 deg and knowledge uncertainty are different requirements. |
| Camera alignment knowledge | 0.05 versus 0.04 TBR LOST/FOUND; camera/truth values 0.04, 0.05, 0.1, or TBD by source | deg | TBC | R1 PAY-8/9/10/11; R2 section 4.7; B3 | Source register | Preserve each requirement mapping; see requirements baseline. |
| Sagitta body orientation | Lateral-plate brackets; boresight transform TBD | unit vector / rotation | TBD | I3 section 2.3; B3 section 2.5 | Source register | Budget calibration claims are not a surveyed transform. |
| Camera hardware / FOV | I3 xiC MC031CG-SY versus MC031MG-SY-FL; budgets retain xiQ-S7; LOST 20-25 TBR /25; FOUND 78-90 TBR /76 /110-140 | model; deg | TBC | I3 hardware/mechanical tables; B1; R1 PAY-2/4; B3 section 2.1; M1 | Source register | Camera/lens selection changes mass, geometry, power, keep-outs, and error budgets. |
| Antenna orientation | Two patch antennas; bracket normals/body transforms TBD | count / unit vectors | TBD | I4 sections 2.3-2.4; M1 section 2.2.2 | Source register | Broad-coverage intent does not define installed radiation geometry. |
| Solar/deployed geometry | Budget labels +X and Y; two solar faces intended; one-wing hardware; deployed sensor axes approximately 30 deg in B3 | configuration / deg | TBC | B1 Power Generated and Mass Budget row 15; M1; B3 section 2.4; I5 | Source register | Deployed angles, hinge geometry, areas, normals, and deployed mass properties not established. |
| Structural analysis applicability | Simplified ISISpace model, subsystem centers assumed, external panels/antennas omitted | — | ASSUMED | S1 section 4 | S1 register | Does not release mass properties for intended GranSystems assembly. |
| Latest located CAD archive | Full_Structure_version_9.4.6.7pm.zip | file | CONFIRMED | [CAD archive](https://drive.google.com/file/d/151ni7o7F19cZFzEwxwT0OUDhdFyFe2cC/view) | Modified 2026-09-05; release revision not established | Existence/name verified; not a computed or released mass-properties report. |

## Orbit and environment

| PARAMETER | VALUE | UNITS | STATUS | SOURCE | REVISION/DATE | NOTES/CONFLICTS |
|---|---|---|---|---|---|---|
| Repo altitude/inclination | 600 / 56 | km / deg | ASSUMED | L1/L2/L5 and standalone main | 833b015 | Legacy development case, not confirmed deployment orbit. |
| Mission altitude | 400-600 TBR; RVM endpoints TBD; budget 500 | km | TBR | M1 section 2.1.1; R1/R3 ORB; B1 Orbit Info | Source register | Budget 500 km is ASSUMED; final launch-provider inputs missing. |
| Mission inclination | 47.6-51.6 TBR; RVM lower bound >=47.6 TBR | deg | TBR | M1 section 2.1.1; R1/R3 ORB | Source register | No accepted upper bound/launch solution established. |
| Eccentricity | Repo 0; MDD <0.01 TBR; RVM limit TBD | dimensionless | TBR | L1/L2; M1; R1/R3 | 833b015 / source register | Circular repo initialization is ASSUMED, not an adopted mission requirement. |
| Repo initial position/velocity | r=[R+h,0,0]; v=[0,sqrt(mu/(R+h)) cos(i),sqrt(mu/(R+h)) sin(i)] | m; m/s | ASSUMED | L1/L2 configure_spacecraft | 833b015 | Initial inertial +X position; no mission epoch/RAAN registration. Periapsis is undefined for an exactly circular orbit. |
| Mission epoch/RAAN/phase | TBD | UTC / deg | TBD | Reviewed R1/M1/OP1 and empty orbital-simulations folder | Drive audit 2026-09-05 | Target launch year is not a defined simulation epoch or Earth orientation. |
| Repo initial attitude/rate | sigma_BN=[0,0,0]; omega=[0.8,-0.2,0.3] | dimensionless MRP; rad/s | ASSUMED | L1/L2 | 833b015 | Rate norm approximately 0.8775 rad/s or 50.28 deg/s, not the 10 deg/s requirement case. |
| Deployment-rate envelope | 10 design case; EPS specifies Z; RVM comment mentions 30 theoretical | deg/s | TBC | M1 commissioning/EPS; R1 ADCS-1 comment | Source register | No confirmed release distribution, norm/per-axis definition, or initial-direction ensemble. |
| Earth mu | Basilisk 3.986004418e14; standalone 6.67e-11 times 5.972e24 | m^3/s^2 | ASSUMED | L1/L2; L5 initialization | 833b015 | Distinct model constants; parity requires deliberate alignment, not silent substitution. |
| Earth radius | Orbit 6371 km; WMM assignment 6371.2 km; budgets 6378 km | km | ASSUMED | L1/L2; B1/B3 | 833b015 / source register | Different uses/reference surfaces must be named; not necessarily interchangeable radii. |
| Active gravity | Earth point-mass central gravity | model | ASSUMED | L1/L2 configure_spacecraft | 833b015 | No attached gravity-gradient torque or higher-order/disturbance suite. |
| Active magnetic model | WMM module; find_wmm2025_path selects first WMM2025.COF match in installed package then reference tree | model/file | ASSUMED | L2 find_wmm2025_path and run | 833b015 | File search is not a pinned coefficient hash/version manifest. Effective model configuration requires verification. |
| WMM epoch assignment | EPOCH_FRACTIONAL_YEAR=2026.0; assigned to epochDateFractionalYear | year | ASSUMED | L2 | 833b015 | Record the effective module epoch/API behavior; assignment alone does not prove an Earth-orientation/time baseline. |
| Earth orientation | No explicit planet ephemeris/orientation input wired by active scenario | — | TBD | L2 run; completed follow-up audit | 833b015 | Mission-consistent inertial-to-fixed orientation/rotation remains unestablished; field norm checks cannot validate it. |
| Standalone environment | Embedded WMM2025; decimal year 2026.0; Greenwich angle 0; Earth rate 7.2921150e-5 | year; rad; rad/s | ASSUMED | L5; standalone frames.hpp and satellite.cpp | 833b015 | Independent vector checks and Earth-rotation sign review required; historical reference is not a truth oracle. |
| HS-2 magnetic-model requirement | No approved WMM/IGRF version/epoch found; PDR uses IGRF | — | TBD | P1 simulation section; reviewed mission sources | Source register | Historical model choice does not select the recovered environment. |
| Active disturbances | No modeled drag, SRP, residual magnetic, or gravity-gradient torque | model coverage | ASSUMED | L2/L4 effector setup | 833b015 | Omission is an idealization, not evidence disturbances are zero. |
| Standalone disturbance inputs | Cd=2.2; Cr=1.3; SRP=4.56e-6; residual dipole=[2e-4,2e-4,4.5e-3]; aero/SRP offsets zero | dimensionless; N/m^2; A m^2; m | ASSUMED | L5; standalone disturbance.cpp | 833b015 | Approximate atmosphere/Sun/eclipse and unmeasured magnetic/geometry assumptions; not flight-calibrated parameters. |

## Actuators and power

| PARAMETER | VALUE | UNITS | STATUS | SOURCE | REVISION/DATE | NOTES/CONFLICTS |
|---|---|---|---|---|---|---|
| Selected torquer candidates | 2 CR0002 rods +1 EXA MT01A; MDD says 3 in-house cells | count/model | TBC | I1 Table 01; M1 section 4.1 | I1 rev 5 / M1 rev 8 | I1 is more specific and corroborated by component/test references, but no approved reconciliation/installed BOM. |
| Repo actuator directions | Identity three-axis layout; X/Y rods, Z air coil assumed | unit vectors | ASSUMED | L2 configure_mtb_config_message; L3; L5 | 833b015 | Does not establish physical spacecraft mounting. |
| Repo maximum dipoles | [0.2,0.2,0.85] | A m^2 | ASSUMED | L3 ADCSConfig; L5 | 833b015 | Z value derives from older MT01 material; not confirmed MT01A installed capability. |
| CR0002 vendor capability | 0.2 at 5 V,20 C; gain 2.3; linear voltage +/-5; R=51 at 20 C | A m^2; A m^2/A; V; ohm | CONFIRMED | H1 p.2 | H1 register | Component rating only. 5/51 times 2.3 does not exactly reproduce nominal 0.2; calibration needed. |
| Repo current limits | [5/51,5/51,sqrt(1.75/4.4)]; effective rods clipped further to 0.2/2.3 | A | ASSUMED | L3; L5 | 833b015 | Current and dipole limits both apply; effective rod clamp about 0.08696 A. |
| Optional C++ default Z current | 0.6301260378126048 | A | ASSUMED | cpp_adcs_core/include/adcs/adcs_core.hpp | 833b015 | Separate hard-coded default versus Python sqrt expression. Adapter passes configuration; verify actual backend/configuration rather than assuming default parity. |
| Repo resistance/gain | R=[51,51,4.4]; gain=[2.3,2.3,0.85/sqrt(1.75/4.4)] | ohm; A m^2/A | ASSUMED | L3/L5 | 833b015 | Z gain inferred from a power limit, not an installed calibration. |
| MT01A dipole claims | >0.395 at 3.3 V; >0.790 saturation at 9 V | A m^2 | TBC | H2 performance table | H2 register | 9 V statement conflicts with 1.25-5 V heading; do not promote to accepted model constants. |
| Older MT01 dipole claims | >0.19 nominal; >0.85 saturation | A m^2 | TBC | H3 performance table | Rev B,2020 | Variant mismatch with I1-linked MT01A. |
| Air-coil voltage/power/current | ICD 12 V,0.07-1.1 A,up to13.2 W; budget 5 V/1.75 W; H2 50-1500 mW | V; A; W | TBC | I1 section 1.1; B1 Power Draw; H2/H3 | Source register | Different variants/operating points and duty intervals not reconciled. ICD also says total ADCS peak12 W. Datasheet current labels mAh are dimensionally inconsistent. |
| Air-coil resistance/mass | R=4.1-4.7 at25 C; H2/B1 mass7.5 versus H3 10.5 | ohm; g | TBC | H2/H3/B1 | Source register | Establish actual winding/variant and temperature dependence. |
| Rod current/power | 0.098 /0.49 per rod at5 V | A; W | ASSUMED | I1 section1.1 | I1 rev5 | Calculated nominal input, not measured HS-2 current or driver limit. |
| H-bridge | DRV8231 in B3; voltage amplifiers TBR in ICD | model | TBC | B3 section2.6; I1 Table01 | Source register | No released PCB/BOM/driver characterization established. |
| PWM interface | Six PWM channels versus comment saying three PWM plus three GPIO | channels | TBC | I1 sections4.1.3,4.2.3 and comment[i] | Drive rev16403 | Internal contradiction; actual wiring/firmware must decide. |
| PWM carrier/burst/settling | PWM_FREQUENCY,BURST_TICKS,RESET_WAIT_TICKS named; values TBD | Hz; ticks;s | TBD | I1 section4.1.3 | I1 register | No reproducible numeric actuation schedule or measured current decay. |
| Magnetic quiet interval | Burst then zero current for uncontaminated magnetic sampling | behavior | CONFIRMED | I1 section4.2.4 | I1 register | Duration/settling threshold and sensor phase remain TBD; active repo does not model it. |
| Duty assumptions | PDR80%; budget detumble50%,experiment20%,standby10% | % | ASSUMED | P1 slide20; B1 Power Draw | Source register | Analysis assumptions, not verified hardware thermal duty limits. |
| Actuator thermal limits | Rod thermal-vacuum qualification -20 to80; air-coil operating -55 to85 | C | TBC | H1/H2/H3; I1 section3 | Source register | Flight drive/temperature/duty envelope unverified. |
| Battery budget | 75.6 | Wh | ASSUMED | B1 Power Generated row16 | B1 register | Nominal capacity, not verified usable energy. |
| Solar generation/cells | 15.30 peak; generation model13 cells versus mass budget16 | W; count | ASSUMED | B1 Power Generated/Mass Budget | B1 register | Deployment, orientation, efficiency, shadowing, and count unresolved. |
| Mode power | Safe5.93; standby7.17; experiment16.19; detumble5.46; CDR draft experiment11.53 | W | ASSUMED | B1/B2 Power Draw totals | B1/B2 register | Older camera inputs explain part of discrepancy; no unified verified power model. |

## Sensors and timing

| PARAMETER | VALUE | UNITS | STATUS | SOURCE | REVISION/DATE | NOTES/CONFLICTS |
|---|---|---|---|---|---|---|
| Active navigation/sensors | Ideal SimpleNav; TAM dcm_SB=identity,scale1,bias0,noise0 | configuration | ASSUMED | L2 run | 833b015 | No flight gyro/estimator, sun sensor, tracker, latency, quantization, or MTQ contamination model. |
| IMU candidate | One VN-100 Rugged | count/model | CONFIRMED | I1 Table01; B1 Mass Budget row19 | Source register | Selection documented; tracker marks ordered, not accepted installed hardware. |
| Gyro range | +/-2000 vendor versus 400 in MDD | deg/s | TBC | H4 p.5; M1 section4.1 | Source register | H4 is authoritative for component capability; MDD discrepancy retained. |
| Gyro bias/noise | Bias instability5 typical/10 maximum; noise density0.0035 | deg/h; deg/s/sqrt(Hz) | CONFIRMED | H4 p.5 | H4 register | Vendor ratings, not complete installed stochastic distributions. |
| Gyro rates/resolution | Internal sample800; bandwidth265; resolution0.02 | Hz; Hz; deg/s | CONFIRMED | H4 p.5 | H4 register | Internal sensor rate is not the control or published-message rate. |
| Magnetometer selection/count | VN-100 internal three-axis magnetometer; additional hardware/count TBD | model/count | TBC | H4; M1 section4.1; B3 section2.8 | Source register | Three axes do not mean three separate magnetometers. |
| Magnetometer capability | +/-2.5 G (=+/-250 uT); noise140 uG/sqrt(Hz) (=14 nT/sqrt(Hz)); resolution1.5 mG (=0.15 uT); sample200 Hz | stated units | CONFIRMED | H4 p.6 | H4 register | Unit conversions only; not installation/calibration verification. |
| IMU placement | Payload lateral plate; minimum MTA separation/body transform TBD | m/rotation | TBD | I1 sections2.2-2.3 | I1 register | Volume-budget notes differ; hard/soft-iron and dynamic interference map missing. |
| IMU interface | UART RS-232/TTL hardware; ICD detailed section115200 bps; other text I2C; MDD RS-422 TBR | protocol;bps | TBC | H4; I1 section4.2.1/software; M1;SW1 | Source register | Confirm actual port/wiring/register output configuration. |
| Sun-sensor model/count | TensorCSS-10: PDR2,ICD3,budget6; B3 four SLCD61N8 | model/count | TBC | P1 slide11;I1;B1 row21;B3 section2.4 | Source register | Latest ICD adds six-channel fields but retains three-channel descriptions; no silent winner. |
| Sun-sensor directions | Solar +X,+Y,FOUND camera in ICD; approximately30 deg deployed axes in B3 | unit vectors/deg | TBC | I1 section3;B3 section2.4 | Source register | Calibrated installed vectors missing. |
| TensorCSS-10 specifications | FOV120;typical angular error5;nominal3.3 V;0.1 mA;0.5 g | deg;V;mA;g | CONFIRMED | H5 sections1.2,2,2.1 | v1.0.2c,2025-08 | Applies to selected variant only; per-unit response/temperature calibration required. |
| TensorCSS temperature | Operating -25 to70; survival -40 to100 | C | CONFIRMED | H5 section2 | H5 register | HS-2 ICD/thermal tables use wider survival range as operating limits; unresolved discrepancy. |
| Sun ADC/sample | 12-bit MCP3204/3208;10 Hz candidate schedule | bits;Hz | TBC | I1 sections4.1.2,4.2.2 | Drive rev16403 | Exact variant/channel population/calibration not closed. |
| Truth sensor role | Sagitta independent truth in B3; star-tracker feedback in MDD/ICD/SW1 | estimator architecture | TBC | B3 sections2.5,2.8;M1;I1;SW1 | Source register | Independence of future validation depends on resolving this input policy. |
| Repo task/control/record | 0.1 task/control;1 main record | s | ASSUMED | L1/L2/L4 | 833b015 | Physical sample epochs can precede the recorded propagated state. Command data nearest-joined to output grid. |
| Flight control rate | MDD/SW1 5 versus ICD10 | Hz | TBC | M1 section2.4;SW1 section2.2;I1 section4 | Source register | Effective execution timing and source-age trace not established. |
| Image/GNSS timing | RVM LOST/FOUND10 ms,FOUND/GNSS +/-65 ms,FOUND/truth +/-10 ms; B3 camera0.50 ms,GNSS3.0 ms predictions | ms | TBC | R1 PAY-12/15/16;B3 sections4.4,4.6 | Source register | Requirement limits and predicted timing errors are distinct quantities; exposure midpoint/clock/latency need tests. |
| General thermal environment | MDD internal -30 to40,external -40 to90,battery0 to45; B3 alignment environment -30 to85 | C | TBC | M1 section4.5;B3 section2.7 | Source register | Component-specific limits and thermoelastic/power constraints must be reconciled. |

## Unclosed decisions

Actual COM/full tensor, body axes, all installed mounting transforms, released BOM/air-coil driver limits, sun-sensor model/count, effective Earth orientation/epoch, sensor quiet timing, and accepted pointing requirements remain open. They did not block recording the audit; they block treating the simulation as a validated HS-2 model.

Use [REQUIREMENTS_BASELINE.md](REQUIREMENTS_BASELINE.md) for obligations and confidence/hold-time questions, and [RECOVERY_ROADMAP.md](RECOVERY_ROADMAP.md) for ordered closure. No value in this document is an instruction to overwrite simulation constants.
