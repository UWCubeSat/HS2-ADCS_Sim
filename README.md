# HS2 ADCS Sim — Basilisk migration baseline

This repository contains the HuskySat-2 ADCS simulation work and a Basilisk migration baseline.

The main workflow goal is:

```text
Use Basilisk as an installed Python dependency, not as a source tree that must be rebuilt for every controller edit.
```

Normal scenario/controller updates should run through Python scripts in `basilisk_runner/` and should not require rebuilding Basilisk or importing `Basilisk.ExternalModules`.

## Current status

Validated in the Basilisk migration baseline:

- Fast pip-installed Basilisk workflow.
- Minimal Basilisk spacecraft propagation.
- Detumble scenario with magnetic field, body-rate, current, dipole, coil-power, applied-torque, Euler-angle, and summary plots.
- Detumble comparison against the standalone C++ reference using metric/invariant checks.
- Direct-torque pointing proof-of-concept with pointing-error, body-rate, torque, Euler-angle, and summary plots.
- Standalone C++ reference build fixed for Windows/MSVC.
- Math validation for detumble and pointing.

Known follow-ups:

- Native Basilisk `MtbEffector` wiring is not resolved yet.
- Magnetorquer-limited full-attitude pointing is not solved yet.
- Direct-torque pointing is a guidance/control proof-of-concept, not final flight actuator behavior.

## Required software

Install these before running the project on Windows:

1. **Git**
   - Required for cloning, branches, commits, and PR workflow.

2. **Python**
   - Use a Python version supported by the installed Basilisk wheel.
   - The current workflow was tested with a project virtual environment and `bsk` installed from `requirements.txt`.

3. **PowerShell**
   - Commands below are written for PowerShell on Windows.

4. **CMake**
   - Required to build the preserved standalone C++ reference and optional `cpp_adcs_core/`.

5. **Visual Studio 2022 Community or Build Tools for Visual Studio 2022**
   - Required for MSVC C++ compilation on Windows.
   - Make sure the **Desktop development with C++** workload is installed.

6. **VS Code** *(optional but recommended)*
   - Useful for editing, terminal workflow, and provided `.vscode/tasks.json` tasks.

7. **Internet access**
   - Needed for the first `pip install -r requirements.txt`.

## Python dependencies

Python dependencies are listed in:

```text
requirements.txt
```

Current major dependencies include:

```text
bsk[examples]
numpy
pandas
matplotlib
pytest
pybind11
```

`bsk[examples]` installs Basilisk from PyPI. Do not clone or rebuild Basilisk for normal scenario/controller edits.

## Fresh clone and setup

Until the Basilisk migration PR is merged, use the PR branch:

```powershell
git clone https://github.com/UWCubeSat/HS2-ADCS_Sim.git
cd HS2-ADCS_Sim
git switch basilisk-migration-baseline-v2
```

After the PR is merged, use `main` instead:

```powershell
git clone https://github.com/UWCubeSat/HS2-ADCS_Sim.git
cd HS2-ADCS_Sim
```

Create and activate the virtual environment:

```powershell
python -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Confirm Basilisk imports:

```powershell
python -c "import Basilisk; print(Basilisk.__path__)"
python -c "from Basilisk.utilities import SimulationBaseClass, macros; from Basilisk.simulation import spacecraft, gravityEffector; print('Basilisk imports OK')"
```

## Repository layout

```text
HS2-ADCS_Sim/
├── basilisk_runner/
│   ├── scenario_huskysat2_minimal.py
│   ├── scenario_huskysat2_detumble.py
│   ├── scenario_huskysat2_pointing.py
│   ├── basilisk_adcs_adapter.py
│   ├── plot_results.py
│   ├── compare_reference_vs_basilisk.py
│   ├── output_data/      # generated, ignored by git
│   └── output_plots/     # generated, ignored by git
├── cpp_adcs_core/
│   ├── CMakeLists.txt
│   ├── include/adcs/adcs_core.hpp
│   ├── src/adcs_core.cpp
│   ├── tests/test_adcs_core.cpp
│   └── bindings/pybind11_module.cpp
├── reference_standalone/
│   └── original_project/ # preserved standalone C++ reference source
├── docs/
│   ├── source_inspection_report.md
│   ├── basilisk_capability_audit.md
│   ├── migration_plan.md
│   ├── interface_contract.md
│   ├── validation_plan.md
│   └── build_workflow.md
└── requirements.txt
```

## Run the minimal Basilisk scenario

From the repository root with the virtual environment active:

```powershell
python .\basilisk_runner\scenario_huskysat2_minimal.py
```

Expected behavior:

- Writes `basilisk_runner/output_data/minimal_output.csv`.
- Prints initial and final angular speed.
- Angular speed should stay nearly constant because there is no active detumble controller in this minimal scenario.

## Build and run the standalone C++ reference

The large generated reference CSV is intentionally not committed to Git. To run detumble comparison, regenerate it locally.

From the repository root:

```powershell
cd .\reference_standalone\original_project
cmake -S . -B build
cmake --build build --config Release
.\build\Release\adcs.exe
python .\plot_all.py
Copy-Item .\adcs_output.csv ..\adcs_output.csv -Force
cd ..\..
```

Expected outputs:

```text
reference_standalone/original_project/adcs_output.csv
reference_standalone/adcs_output.csv
reference_standalone/original_project/output_plots/
```

Do not commit generated CSVs or plots.

## Run the Basilisk detumble scenario

From the repository root with the virtual environment active:

```powershell
python .\basilisk_runner\scenario_huskysat2_detumble.py
```

Expected generated files:

```text
basilisk_runner/output_data/detumble_output.csv
basilisk_runner/output_plots/body_rates.png
basilisk_runner/output_plots/nav_euler.png
basilisk_runner/output_plots/mag_field.png
basilisk_runner/output_plots/currents.png
basilisk_runner/output_plots/dipole.png
basilisk_runner/output_plots/coil_power.png
basilisk_runner/output_plots/mtb_torque.png
basilisk_runner/output_plots/summary_plots.png
```

The current detumble baseline uses direct application of the computed magnetic torque through Basilisk body torque:

```text
tau_B = m_B x B_B
```

This is a validated fallback path. Native Basilisk `MtbEffector` integration remains a follow-up task.

## Compare standalone reference vs. Basilisk detumble

First make sure the standalone reference CSV exists:

```text
reference_standalone/adcs_output.csv
```

Then run:

```powershell
python .\basilisk_runner\compare_reference_vs_basilisk.py
```

Expected terminal line:

```text
Validation passed?: True
```

The comparison is metric-based, not CSV-column-exact. It checks:

- finite numeric values,
- final angular speed less than initial angular speed,
- rotational kinetic energy decrease,
- mean mechanical control power is negative,
- `tau_B = m_B x B_B` consistency,
- magnetic-field magnitude plausibility,
- body/inertial magnetic-field norm preservation,
- peak coil-power limit,
- detumble time to `0.05 rad/s` within the expected window.

Metrics are written to:

```text
basilisk_runner/output_data/comparison_metrics.json
```

## Run the direct-torque pointing proof-of-concept

From the repository root with the virtual environment active:

```powershell
python .\basilisk_runner\scenario_huskysat2_pointing.py
```

Expected terminal line:

```text
Pointing validation passed?: True
```

Expected generated files:

```text
basilisk_runner/output_data/pointing_output.csv
basilisk_runner/output_data/pointing_metrics.json
basilisk_runner/output_plots/pointing_error.png
basilisk_runner/output_plots/pointing_body_rates.png
basilisk_runner/output_plots/pointing_torque.png
basilisk_runner/output_plots/pointing_euler.png
basilisk_runner/output_plots/pointing_summary.png
```

This scenario validates guidance/control behavior with an ideal direct body-torque actuator. It is not the final magnetorquer-only pointing controller.

## Full local validation sequence

Run this from the repository root after setting up the virtual environment.

```powershell
# 1. Regenerate standalone reference CSV
cd .\reference_standalone\original_project
cmake -S . -B build
cmake --build build --config Release
.\build\Release\adcs.exe
Copy-Item .\adcs_output.csv ..\adcs_output.csv -Force
cd ..\..

# 2. Run Basilisk detumble and comparison
python .\basilisk_runner\scenario_huskysat2_detumble.py
python .\basilisk_runner\compare_reference_vs_basilisk.py

# 3. Run pointing proof-of-concept validation
python .\basilisk_runner\scenario_huskysat2_pointing.py

# 4. Confirm git tree is clean
 git status
```

Expected validation lines:

```text
Validation passed?: True
Pointing validation passed?: True
```

Expected Git status:

```text
nothing to commit, working tree clean
```

## Plot existing detumble output

If `detumble_output.csv` already exists, regenerate plots with:

```powershell
python .\basilisk_runner\plot_results.py
```

## Optional: build the standalone ADCS C++ core

Only do this when custom C++ controller logic changes:

```powershell
cmake -S cpp_adcs_core -B build\cpp_adcs_core -DCMAKE_BUILD_TYPE=Release
cmake --build build\cpp_adcs_core --config Release --parallel
```

If CMake cannot find pybind11:

```powershell
$pybindDir = python -m pybind11 --cmakedir
cmake -S cpp_adcs_core -B build\cpp_adcs_core -DCMAKE_BUILD_TYPE=Release -Dpybind11_DIR=$pybindDir
cmake --build build\cpp_adcs_core --config Release --parallel
```

This builds only the small project-owned C++ core. It does not rebuild Basilisk.

## Normal development workflow

For most changes:

```text
1. Edit Python scenario/controller code in basilisk_runner/.
2. Run the relevant scenario.
3. Run validation scripts.
4. Inspect generated plots/metrics.
5. Commit source/docs/config changes only.
```

If C++ controller code changes:

```text
1. Edit cpp_adcs_core/.
2. Rebuild cpp_adcs_core only.
3. Run Basilisk scenario and validation.
```

Do not rebuild Basilisk for normal work.

## Files that should not be committed

These are generated or local-only artifacts:

```text
.venv/
build/
archive/
basilisk_runner/output_data/
basilisk_runner/output_plots/
reference_standalone/adcs_output.csv
reference_standalone/output_plots/
reference_standalone/original_project/build/
reference_standalone/original_project/adcs_output.csv
reference_standalone/original_project/output_plots/
```

## Troubleshooting

### `Basilisk is not installed`

Activate the virtual environment and install requirements:

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

### `Reference CSV not found`

Regenerate the standalone reference:

```powershell
cd .\reference_standalone\original_project
cmake -S . -B build
cmake --build build --config Release
.\build\Release\adcs.exe
Copy-Item .\adcs_output.csv ..\adcs_output.csv -Force
cd ..\..
```

Then rerun:

```powershell
python .\basilisk_runner\compare_reference_vs_basilisk.py
```

### `.uild
dcs.exe is not recognized` or executable not found

With the Visual Studio CMake generator, the executable is usually under the configuration folder:

```powershell
.\build\Release\adcs.exe
```

not:

```powershell
.\build\adcs.exe
```

### C++ build cannot find `M_PI`

The standalone reference CMake file defines `_USE_MATH_DEFINES` for MSVC. If this error appears, confirm this line exists in `reference_standalone/original_project/CMakeLists.txt`:

```cmake
target_compile_definitions(adcs PRIVATE _USE_MATH_DEFINES)
```

### Git shows generated files

Generated output should be ignored. If generated files appear, do not commit them. Confirm `.gitignore` includes output folders and local archives.

## Follow-up work

Keep these out of the baseline PR unless explicitly requested:

1. Magnetorquer-limited pointing controller.
2. Native Basilisk `MtbEffector` isolated debugging.
3. GitHub Actions / CI smoke testing.
4. More complete disturbance-model parity with the standalone reference.
