"""Pointing-control scaffold.

This file exists intentionally, but pointing should not be treated as complete
until the detumble scenario has been locally verified. The next implementation
choice depends on hardware:

- reaction-wheel pointing: use Basilisk-native MRP guidance/control examples and RW effectors
- magnetorquer-only pointing: keep custom control logic because magnetic torque is instantaneously underactuated
"""

from __future__ import annotations

from pathlib import Path
import pandas as pd

HERE = Path(__file__).resolve().parent
OUT_DATA = HERE / "output_data"
OUT_DATA.mkdir(parents=True, exist_ok=True)


def run():
    status = pd.DataFrame([
        {
            "status": "pointing_scaffold_only",
            "reason": "Detumble + WMM + MtbEffector chain should be validated before pointing control is added.",
            "next_step": "Decide reaction-wheel MRP feedback vs custom magnetorquer-only pointing logic.",
        }
    ])
    out = OUT_DATA / "pointing_status.csv"
    status.to_csv(out, index=False)
    print(f"Pointing scenario is a scaffold only. Wrote {out}")
    return status


if __name__ == "__main__":
    run()
