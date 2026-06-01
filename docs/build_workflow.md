# Fast build workflow

## What gets rebuilt

| Change | Rebuild needed? | Command |
|---|---:|---|
| Edit Basilisk scenario Python | No | Run Python script again. |
| Edit plotting/comparison Python | No | Run plot/compare script again. |
| Edit `basilisk_adcs_adapter.py` Python controller | No | Run scenario again. |
| Edit `cpp_adcs_core/src/adcs_core.cpp` | Yes, small C++ core only | `cmake --build build --parallel` |
| Edit Basilisk source or true ExternalModules plugin | Yes, Basilisk rebuild | Avoid unless clearly necessary. |

## Standard setup

```powershell
cd C:\Users\chine\Documents\satellite_adcs_basilisk_project
python -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Standard run loop

```powershell
python .\basilisk_runner\scenario_huskysat2_detumble.py
python .\basilisk_runner\plot_results.py
python .\basilisk_runner\compare_reference_vs_basilisk.py
```

## Optional C++ ADCS core loop

Configure once:

```powershell
cmake -S cpp_adcs_core -B build -DCMAKE_BUILD_TYPE=Release
```

Rebuild after C++ ADCS changes:

```powershell
cmake --build build --parallel
```

Run smoke test:

```powershell
.\build\adcs_core_smoke.exe
```

If pybind is found, copy or keep the generated `adcs_core` module where Python can import it. The adapter will automatically use it if importable; otherwise it uses the Python fallback.

## What not to do by default

Do not make this the first normal path:

```python
from Basilisk.ExternalModules import mtqDetumble
```

That path requires a Basilisk external C++ module build and links normal controller changes to the large Basilisk build/SWIG pipeline.
