# satellite_adcs_basilisk_project

Fast Basilisk migration scaffold for the standalone HuskySat-style C++ ADCS/WMM simulation.

The main design rule is: **Basilisk is a dependency, not the project you rebuild for every controller edit.** Use the prebuilt `bsk` wheel for spacecraft dynamics, message passing, WMM, sensors, logging, and native effectors. Keep only project-specific ADCS logic in a small standalone C++ core or in Python while testing.

## What this bundle contains

```text
satellite_adcs_basilisk_project/
├── README.md
├── .gitignore
├── requirements.txt
├── reference_standalone/
│   ├── original_project/          # cleaned copy of the working standalone source
│   ├── adcs_output.csv            # known-good generated output, ignored by git
│   └── output_plots/              # known-good generated plots, ignored by git
├── basilisk_runner/
│   ├── scenario_huskysat2_minimal.py
│   ├── scenario_huskysat2_detumble.py
│   ├── scenario_huskysat2_pointing.py
│   ├── basilisk_adcs_adapter.py
│   ├── plot_results.py
│   ├── compare_reference_vs_basilisk.py
│   ├── output_data/
│   └── output_plots/
├── cpp_adcs_core/
│   ├── CMakeLists.txt
│   ├── include/adcs/adcs_core.hpp
│   ├── src/adcs_core.cpp
│   ├── tests/test_adcs_core.cpp
│   └── bindings/pybind11_module.cpp
├── docs/
│   ├── source_inspection_report.md
│   ├── basilisk_capability_audit.md
│   ├── migration_plan.md
│   ├── interface_contract.md
│   ├── validation_plan.md
│   └── build_workflow.md
└── archive/old_attempts/
    └── huskysat2_bsk/             # old source-level attempt, not the primary workflow
```

## Windows / VS Code setup

Open PowerShell in VS Code:

```powershell
cd C:\Users\chine\Documents\satellite_adcs_basilisk_project
python -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Basilisk is installed by the `bsk[examples]` requirement. Do **not** clone or rebuild Basilisk for normal controller edits.

## Run the preserved standalone reference

```powershell
cd C:\Users\chine\Documents\satellite_adcs_basilisk_project\reference_standalone\original_project
cmake -S . -B build
cmake --build build
.\build\adcs.exe
python .\plot_all.py
```

The prepared bundle also includes the known-good `reference_standalone\adcs_output.csv` and `reference_standalone\output_plots\` from the uploaded standalone run. These are ignored by git so they do not get accidentally committed.

## Run the minimal Basilisk scenario

```powershell
cd C:\Users\chine\Documents\satellite_adcs_basilisk_project
.\.venv\Scripts\Activate.ps1
python .\basilisk_runner\scenario_huskysat2_minimal.py
```

Expected outputs:

```text
basilisk_runner/output_data/minimal_output.csv
basilisk_runner/output_plots/body_rates.png
basilisk_runner/output_plots/nav_euler.png
```

## Run the Basilisk detumble scenario

```powershell
cd C:\Users\chine\Documents\satellite_adcs_basilisk_project
.\.venv\Scripts\Activate.ps1
python .\basilisk_runner\scenario_huskysat2_detumble.py
```

This uses Basilisk-native spacecraft dynamics, gravity, WMM magnetic field, magnetometer, and `MtbEffector`. The controller is a Python Basilisk module by default, with an optional separate pybind C++ core if built. It does not import `Basilisk.ExternalModules.mtqDetumble`.

Expected outputs:

```text
basilisk_runner/output_data/detumble_output.csv
basilisk_runner/output_plots/body_rates.png
basilisk_runner/output_plots/mag_field.png
basilisk_runner/output_plots/currents.png
basilisk_runner/output_plots/dipole.png
basilisk_runner/output_plots/coil_power.png
basilisk_runner/output_plots/summary_plots.png
```

## Plot existing Basilisk output

```powershell
python .\basilisk_runner\plot_results.py
```

The plot script prefers `detumble_output.csv`; if that is not present it falls back to `minimal_output.csv`.

## Compare reference vs. Basilisk output

```powershell
python .\basilisk_runner\compare_reference_vs_basilisk.py
```

The comparison is metric-based, not CSV-column-exact. It checks initial/final angular speed, detumble time if applicable, magnetic field magnitude, dipole/current/power behavior, disturbance torque if available, and NaN/Inf sanity.

## Build the optional standalone C++ ADCS core

Only do this when C++ controller logic changes:

```powershell
cd C:\Users\chine\Documents\satellite_adcs_basilisk_project
cmake -S cpp_adcs_core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

This builds the small `adcs_core` library and smoke test independently of Basilisk. If CMake finds pybind11, it also builds a Python module named `adcs_core` that `basilisk_adcs_adapter.py` can use automatically.

If pybind11 is installed but CMake cannot find it, use:

```powershell
$pybindDir = python -m pybind11 --cmakedir
cmake -S cpp_adcs_core -B build -DCMAKE_BUILD_TYPE=Release -Dpybind11_DIR=$pybindDir
cmake --build build --parallel
```

## Normal fast workflow

1. Open this project in VS Code.
2. Edit `basilisk_runner/*.py` for scenario and adapter work.
3. Edit `cpp_adcs_core/src/adcs_core.cpp` for custom ADCS logic only.
4. Rebuild only `cpp_adcs_core` if C++ changed.
5. Run the Basilisk scenario.
6. Plot and compare metrics.
7. Commit/push source, docs, and small config files only.

No Basilisk source rebuild is needed unless you deliberately decide to create a true Basilisk C++ external module later.

## Current capability state

Current prepared code status:

- Minimal Basilisk spacecraft/orbit/attitude scenario: **created**.
- Basilisk WMM magnetic field integration: **created in detumble scenario, pending local run with installed `bsk`**.
- Detumble scenario: **created with Python Basilisk controller and optional pybind core, pending local Basilisk run**.
- Pointing scenario: **scaffold only**.
- Full standalone equivalence: **not claimed**. The correct next step is local execution and metric comparison.
