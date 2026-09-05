# HS-2 ADCS controlled recovery

## Goal and current authorization

The project goal is a COMPLETE, validated HuskySat-2 ADCS simulation in Basilisk, suitable for engineering decisions, controller development, detumble and pointing analysis, requirements verification, and mission planning. A direct-torque pointing proof of concept is not completion of that goal.

The current recovery authorization permits documentation changes only to this file and:

- docs/REPO_MAP.md
- docs/PHYSICAL_PARAMETERS.md
- docs/REQUIREMENTS_BASELINE.md
- docs/SCENARIO_STATUS.md
- docs/RECOVERY_ROADMAP.md

Until the user expands scope, do not change simulation behavior, Python, C++, CMake, dependencies, Git configuration/index, .gitignore, editor configuration, generated outputs, or any other file. Do not commit. The roadmap describes future work; it is not authorization to execute it.

Google Drive is a read-only evidence source. Do not create, edit, copy, upload, move, rename, comment on, share, or delete Drive files. Restrict any authorized follow-up searches to Husky Satellite Lab, Shared Drive ID 0AC8raEVvalZjUk9PVA, and the relevant HS-2 material. Reuse the completed audits; do not repeat broad exploration.

## Engineering evidence

Every engineering value, requirement, calibration, and model assumption must carry units, source, revision/date, applicable configuration/frame, and one of these statuses:

| STATUS | MEANING |
|---|---|
| CONFIRMED | The cited evidence establishes the stated requirement, component specification, or configuration fact. State exactly what is confirmed; documentation of a requirement is not demonstrated compliance. |
| TBR | Explicitly to be reviewed, including an unresolved source review flag. |
| TBC | To be confirmed, including contradictory selections/values or uncertain applicability to the installed spacecraft. |
| TBD | Not yet defined or supported by the reviewed evidence. |
| ASSUMED | Analytical input, placeholder, estimate, legacy constant, or unverified approximation. |

A number in code is not automatically confirmed HS-2 engineering data. Vendor ratings do not prove hardware selection or installation. Distinguish budget estimates, design allocations, predictions, measurements, and released flight values.

Never silently replace or reconcile conflicting engineering values. Preserve both values, their sources and revisions, and the unresolved decision. Do not rank sources by modification time alone. Use approved requirements for obligations, released ICD/CAD/BOM for configuration, applicable vendor documentation for component capability, and signed calibrated tests for achieved performance. If that hierarchy does not settle a conflict, retain TBC/TBR/TBD.

## Before physics or control changes

When such changes are authorized, first record and check:
1. Source and applicable hardware/configuration revision, including conflicting evidence.
2. Frames: physical body axes, origin, COM, sensor/actuator mounting transforms, and inertial/Earth-fixed definitions.
3. Units: SI conversions, radians versus degrees, nT versus T, current versus charge, and tensor reference point.
4. Signs: DCM direction, quaternion/MRP conventions, handedness, coil polarity, cross-product order, and disturbance direction.
5. Timing: acquisition epoch, message timestamp, sample age, task order, control/plant rates, hold intervals, quiet periods, latency, and logging alignment.
6. Independent validation and requirement-specific acceptance criteria.

Do not hide timing defects by copying commands into applied-quantity columns or using nearest-time joins without checking actual sample epochs. Keep truth, measured, estimated, commanded, and independently applied telemetry distinct.

Basilisk owns active spacecraft truth propagation. Optional project C++ should implement controller/estimator logic without duplicating the plant. Never attach two effectors that apply the same commanded torque.

## Verification and commits

Tests are required before commits. Run meaningful checks appropriate to the change, inspect their results, and review the diff for scope and unintended behavior. Documentation-only work requires provenance/consistency checks, local-link and table checks, and a diff/allowlist review; it does not require running simulations that overwrite outputs. Do not claim a test ran when it did not.

For future code changes, select tests that exercise independent expected behavior: frame/sign cases, timestamps, actuator saturation and polarity, plant invariants, and affected scenario/requirement checks. A successful build, smoke test, matching duplicate equations, or a printed pass message is insufficient physics validation. If required checks cannot run, record the limitation and do not present the change as verified.

Do not import scenario modules merely to inspect constants: current scripts create output directories during import. Existing scenario/comparison scripts write results and may replace or remove previous artifacts.

## Reporting discipline

Never report proof-of-concept outputs as HS-2 flight performance. WORKING DEVELOPMENT BASELINE describes software bring-up, not flight qualification. Keep FAILED EXPERIMENT and LEGACY REFERENCE material clearly labeled. All current scenarios are NOT FLIGHT VALIDATED.

Report what changed, why, what was checked, and what remains unresolved. Preserve failed cases and reproducibility metadata when future work is authorized. Read the recovery documents together; they are an audit-derived record, not a release of spacecraft parameters.
