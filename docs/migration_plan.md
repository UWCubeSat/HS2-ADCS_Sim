# Migration plan

## Milestone 0 — Preserve and document standalone reference

Status: **done in this scaffold**

- Clean copy of standalone source placed in `reference_standalone/original_project/`.
- Known-good generated CSV placed in `reference_standalone/adcs_output.csv`.
- Known-good plots placed in `reference_standalone/output_plots/`.
- Source inspection report written in `docs/source_inspection_report.md`.

## Milestone 1 — Minimal Basilisk spacecraft/orbit/attitude simulation

Status: **created, needs local run with installed `bsk`**

Script:

```text
basilisk_runner/scenario_huskysat2_minimal.py
```

Purpose:

- verify pip-installed Basilisk imports
- verify VS Code/PowerShell workflow
- create spacecraft
- attach central gravity
- run one short 6-DOF simulation
- log position, velocity, MRP attitude, body rates
- generate first body-rate and attitude plots

No WMM, magnetometer, magnetorquer, or custom C++ is required.

## Milestone 2 — Add Basilisk WMM magnetic field

Status: **created inside detumble scenario, needs local run**

Script:

```text
basilisk_runner/scenario_huskysat2_detumble.py
```

Actions:

- add `magneticFieldWMM.MagneticFieldWMM`
- search installed Basilisk package for `WMM2025.COF`
- fall back to local `reference_standalone/original_project/reference/WMM2025.COF`
- log inertial magnetic field
- add magnetometer for body/sensor-frame magnetic field

Validation:

- compare magnetic field magnitude against standalone
- verify frame/sign convention before controller tuning

## Milestone 3 — Add magnetic torque-bar application

Status: **created, needs local run**

Use:

```python
from Basilisk.simulation import MtbEffector
```

The controller produces commanded dipole. Basilisk `MtbEffector` applies the torque to the spacecraft.

TODO:

- confirm `GtMatrix_B` row-major layout against current installed Basilisk version
- confirm field frame used by `MtbEffector` and logged WMM output

## Milestone 4 — Add detumble control

Status: **created in Python; optional C++ core created**

Primary fast path:

```text
basilisk_runner/basilisk_adcs_adapter.py
```

This defines a Python Basilisk module that:

- reads `NavAttMsg`
- reads `TAMSensorMsg`
- computes rate-cross-field detumble command
- applies current and dipole saturation
- writes `MTBCmdMsg`
- stores diagnostic history for currents, power, and saturation flags

Optional C++ fast path:

```text
cpp_adcs_core/
```

Builds independently of Basilisk and can expose `adcs_core.step(...)` through pybind11 when available.

## Milestone 5 — Add pointing control

Status: **scaffold only**

Script:

```text
basilisk_runner/scenario_huskysat2_pointing.py
```

Do not force pointing before detumble is stable. First decision after detumble:

- If reaction wheels are used later, evaluate Basilisk `mrpFeedback` + RW effectors.
- If magnetorquer-only pointing is required, keep custom project-specific control because pure magnetic torque is underactuated instantaneously.

## Milestone 6 — Add environmental disturbances

Status: **planned**

Candidate Basilisk native path:

- SRP dynamic effector
- drag effector / atmosphere model
- gravity-gradient effector if appropriate
- `extForceTorque` for residual custom disturbance terms

Do not tune disturbances before the minimal + WMM + MTB detumble chain is verified.

## Milestone 7 — Compare against standalone reference

Status: **comparison script created**

Script:

```text
basilisk_runner/compare_reference_vs_basilisk.py
```

Metrics:

- initial angular speed magnitude
- final angular speed magnitude
- detumble time
- body-rate decay
- magnetic field magnitude/history
- commanded dipole behavior
- commanded current behavior
- coil power mean and peak
- disturbance torque magnitude if available
- numerical sanity checks

## Milestone 8 — Clean up, test, VS Code workflow

Status: **initial files created**

Created:

- `.gitignore`
- `.vscode/tasks.json`
- `requirements.txt`
- docs
- optional C++ core smoke test

Next:

- run locally with installed Basilisk
- record exact Basilisk version
- update TODOs in `interface_contract.md`
- add pytest for controller math and CSV sanity
- decide whether any true Basilisk plugin is justified later
