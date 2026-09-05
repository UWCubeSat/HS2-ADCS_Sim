# Repository map

Audit-derived recovery record, 2026-09-05. Scope: the completed repository audit, verified follow-up, and HS-2 Shared Drive audit. Documentation records existing behavior; it does not change or approve the physics.

## Recovery anchor and history

The documentation task found a clean checkout on **adcs-sim-recovery-2026** at **833b015** (Document setup and validation runbook). The local basilisk-migration-baseline-v2 and origin/basilisk-migration-baseline-v2 refs point to the same commit. These are observed local refs; no fetch or remote-state update was performed.

| REF | OBSERVED COMMIT | INTERPRETATION |
|---|---|---|
| adcs-sim-recovery-2026 | 833b015 | Current recovery anchor before these documentation changes. |
| basilisk-migration-baseline-v2 / origin/basilisk-migration-baseline-v2 | 833b015 | Same migration baseline. |
| mtq-limited-pointing-v2 | ecf3cab | Ancestor of the recovery anchor; its name is not evidence of successful magnetic pointing. Commit adds direct-torque pointing metrics. |
| origin/main | 2bb6545 | Ancestor, seven commits behind the recovery anchor in the inspected history. |
| local main | 491722e | Different local line; includes afae66e and 491722e absent from the recovery anchor. Do not equate it with origin/main. |

The recovery line includes migration bring-up (5a09334), detumble math checks (7f68700), direct-torque pointing (7a9a669), pointing metrics (ecf3cab), and the runbook (833b015). Other visible remote refs, including cohesive-refactor and codex/huskysat2-realism-pass, are historical development lines, not active runtime selection.

## Directory and execution map

| PATH | ROLE | ACTIVE OR HISTORICAL |
|---|---|---|
| [basilisk_runner/](../basilisk_runner/) | Python scenario orchestration, Basilisk messages/effectors, controller adapter, CSV/plot/metric generation. | Active development path. |
| [cpp_adcs_core/](../cpp_adcs_core/) | Small independent C++17 controller library, smoke executable, optional pybind11 module named adcs_core. | Optional controller computation; not the spacecraft plant. |
| [reference_standalone/original_project/](../reference_standalone/original_project/) | Preserved 13-state C++ orbit/attitude simulation with its own environment, sensors, navigation, controller, and RK4. | LEGACY REFERENCE; separately executed. |
| [src/](../src/), [include/](../include/), root [CMakeLists.txt](../CMakeLists.txt) | Older GeographicLib-dependent orbit/magnetic-field program, target MyProject. | Historical; root CMake does not build the active Basilisk scenarios or optional controller core. |
| [scripts/](../scripts/) | Plotting for older root C++ text output. | Historical plotting utilities. |
| archive/old_attempts/huskysat2_bsk/ | First Basilisk detumble attempt with ExternalModules/mtqDetumble and SWIG/source-build assumptions. | Ignored local archive, not a portable tracked dependency. |
| archive/experiments/ | Two failed magnetorquer-pointing Python experiments. | Ignored local evidence, not the current pointing implementation. |
| [docs/](./) | Earlier migration reports/runbooks plus this recovery record. | Existing runbook claims must be read with the audit limitations below. |
| [requirements.txt](../requirements.txt), [.vscode/tasks.json](../.vscode/tasks.json) | Dependency declaration and developer tasks. | Workflow inputs, not engineering parameter authority; unchanged. |
| .venv/, build/, binary extensions | Installed Basilisk/Python dependencies and generated build products. | Local environment, not spacecraft design evidence. |

## Active Basilisk paths

| ENTRY POINT | EXECUTION / PURPOSE |
|---|---|
| [scenario_huskysat2_minimal.py](../basilisk_runner/scenario_huskysat2_minimal.py) | Spacecraft plus central gravity; 600 s propagation without ADCS actuation. |
| [scenario_huskysat2_detumble.py](../basilisk_runner/scenario_huskysat2_detumble.py) | Spacecraft, WMM, ideal SimpleNav, ideal body-aligned magnetometer, PythonBdotMTQController, and ExtForceTorque magnetic-torque bridge; one nominal orbit. |
| [scenario_huskysat2_pointing.py](../basilisk_runner/scenario_huskysat2_pointing.py) | Reuses detumble spacecraft setup, overrides initial attitude/rates, and applies ideal direct body torque from an MRP PD controller for 1800 s. It does not attach WMM or magnetic actuators. |
| [basilisk_adcs_adapter.py](../basilisk_runner/basilisk_adcs_adapter.py) | Basilisk Python SysModel, current/dipole/power conversion, optional C++ dispatch, message writing, controller diagnostics. |
| [compare_reference_vs_basilisk.py](../basilisk_runner/compare_reference_vs_basilisk.py) | Reads saved CSVs and writes range/invariant metrics. It is not a trajectory-equivalence or HS-2 requirement verifier. |
| [plot_results.py](../basilisk_runner/plot_results.py) | Visualizes existing detumble output; not a separate simulation. |

Basilisk is used as an installed Python dependency. Active code does not import Basilisk.ExternalModules. Spacecraft truth translation and rotation are propagated by Basilisk, not by cpp_adcs_core or the standalone executable.

The detumble schedule runs WMM, navigation, magnetometer, controller, ExtForceTorque, then spacecraft at decreasing priorities 900, 800, 700, 600, 500, 100. The task period is 0.1 s and principal message recorders use 1 s. Input-source age and recorder phase must be checked explicitly; equal output-row labels do not establish equal physical epochs.

### Controller computation

The class name includes Bdot, but the present law is rate-cross-field damping, not a finite-difference B-dot implementation:

- desired dipole = K times (omega_B cross B_B), K = 67200 (ASSUMED development gain).
- allowed current on axis i = min(current limit, dipole limit / dipole gain).
- current = componentwise clamp(desired dipole / dipole gain).
- dipole = dipole gain times current.
- coil power = resistance times current squared.
- torque command = dipole cross the measured body magnetic field.

No RL winding dynamics, PWM switching, duty-cycle quiet intervals, thermal derating, or measured current feedback is modeled.

The adapter attempts to import adcs_core. If available and enabled, it calls its step function; import or call exceptions fall back to Python. Therefore Python is the available fallback, but backend selection is environment-dependent. The using_cpp_core field records availability/preference, not proof that an individual C++ call succeeded. Do not claim C++ execution from that flag alone.

### Native MtbEffector versus current actuation

Native MtbEffector is available in the audited Basilisk environment and imported by detumble. The scenario creates and subscribes an instance but neither attaches it to the spacecraft nor schedules it. ExtForceTorque is the sole active control effector.

The current three-bar configuration places an identity matrix in the first nine entries of GtMatrix_B (row-major 3 by numMTB). The old archive incorrectly used a MAX_EFF_CNT stride. Historical zero-output observations do not establish that the native capability is unavailable.

USE_DIRECT_TORQUE_FALLBACK is not an implemented branch switch: the active construction remains unconditional. Native actuation recovery must verify message layout, frame conversion, scheduling, clipping, and independently applied torque, and avoid applying torque twice.

## Optional C++ core and standalone reference

The [core CMake project](../cpp_adcs_core/CMakeLists.txt) builds adcs_core_lib and adcs_core_smoke. It builds adcs_core bindings only if pybind11 is found and enabled. This is independent of a Basilisk source rebuild. The [smoke test](../cpp_adcs_core/tests/test_adcs_core.cpp) checks a valid result and finite total power for one input; it is not broad unit or physics coverage.

The standalone reference's [main.cpp](../reference_standalone/original_project/src/main.cpp) executes environment -> sampled sensors/navigation -> controller -> CSV logging -> RK4 truth propagation. It uses scalar-first quaternions and its own central gravity, embedded WMM2025, approximate disturbances, noisy sensors, and magnetic-correction estimator. Its CSV BNx/BNy/BNz fields are navigation-filtered body magnetic fields, not Basilisk inertial B_N.

It shares legacy box and actuator assumptions with the active migration, so agreement can be regression consistency rather than independent physical validation. See [SCENARIO_STATUS.md](SCENARIO_STATUS.md) for sign, fidelity, observability, and timing limitations.

## Generated outputs and provenance

| LOCATION | CONTENT / LIMIT |
|---|---|
| basilisk_runner/output_data/ | minimal_output.csv, detumble_output.csv, pointing_output.csv, comparison_metrics.json, pointing_metrics.json. Generated, not a released verification package. |
| basilisk_runner/output_plots/ | Generated plots. Minimal and detumble share some filenames, so one run can replace another's plots. |
| reference_standalone/original_project/adcs_output.csv and output_plots/ | Standalone run outputs. |
| reference_standalone/adcs_output.csv | Copy consumed by the comparison script; may be stale relative to source/build. |
| build/, reference_standalone/original_project/build/, .venv/ | Build/dependency artifacts. |
| output/ and ../output/ relative to legacy executable working directory | Root C++ test.txt and bfield.txt output. |
| [generated_file_manifest.json](generated_file_manifest.json) | Existing inventory metadata; not a complete run manifest or proof of regeneration. |

Scenario imports create output directories. Runs and plotting write artifacts; the comparison script can remove stale comparison output. Do not execute them during documentation-only recovery.

A major telemetry defect remains: detumble initially reads an effector logger, then overwrites applied_torque_B_* and mtb_torque_B_* with controller command history. Those final CSV columns are not an independent applied-torque measurement. The cross-product check consequently rechecks related command-side equations.

The comparison gate mostly checks Basilisk scalar ranges/invariants and reference finiteness, not reference-to-Basilisk tolerances. Saved passes do not establish current-source reproducibility, frame direction, actuator fidelity, 0.5 deg/s requirement compliance, or pointing feasibility.

## Related recovery records

- [Physical parameters and source register](PHYSICAL_PARAMETERS.md)
- [Requirements and conflicts](REQUIREMENTS_BASELINE.md)
- [Scenario status](SCENARIO_STATUS.md)
- [Ordered recovery roadmap](RECOVERY_ROADMAP.md)

Earlier README phrases such as validated fallback and known-good reference describe development bring-up. They do not supersede these limitations or establish HS-2 flight performance.
