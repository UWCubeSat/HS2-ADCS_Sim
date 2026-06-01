from __future__ import annotations

from pathlib import Path
import json

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REF = ROOT / "reference_standalone" / "adcs_output.csv"
OUT_DATA = HERE / "output_data"
COMPARE_OUT = OUT_DATA / "comparison_metrics.json"


def mag3(df: pd.DataFrame, cols):
    return np.sqrt(sum(np.asarray(df[c], dtype=float) ** 2 for c in cols))


def detumble_time(t, wmag, threshold):
    idx = np.where(wmag <= threshold)[0]
    return None if len(idx) == 0 else float(t[idx[0]])


def finite_numeric(df: pd.DataFrame) -> bool:
    numeric = df.select_dtypes(include=[np.number])
    return bool(np.isfinite(numeric.to_numpy(dtype=float)).all())


def summarize_reference(df: pd.DataFrame):
    t = df["t"].to_numpy(dtype=float)
    wmag = mag3(df, ["p", "q", "r"])
    out = {
        "rows": int(len(df)),
        "cols": int(len(df.columns)),
        "finite_numeric": finite_numeric(df),
        "initial_omega_mag_rad_s": float(wmag[0]),
        "final_omega_mag_rad_s": float(wmag[-1]),
        "detumble_time_0p05_s": detumble_time(t, wmag, 0.05),
        "detumble_time_0p02_s": detumble_time(t, wmag, 0.02),
        "detumble_time_0p01_s": detumble_time(t, wmag, 0.01),
    }
    if "pcoil_total" in df.columns:
        out["mean_coil_power_W"] = float(df["pcoil_total"].mean())
        out["peak_coil_power_W"] = float(df["pcoil_total"].max())
    if all(c in df.columns for c in ["BBx", "BBy", "BBz"]):
        bmag = mag3(df, ["BBx", "BBy", "BBz"])
        out["mean_B_body_T"] = float(bmag.mean())
        out["peak_B_body_T"] = float(bmag.max())
    if all(c in df.columns for c in ["Tdist_x", "Tdist_y", "Tdist_z"]):
        tdist = mag3(df, ["Tdist_x", "Tdist_y", "Tdist_z"])
        out["mean_disturbance_torque_Nm"] = float(tdist.mean())
    return out


def summarize_basilisk(df: pd.DataFrame):
    t = df["time_s"].to_numpy(dtype=float)
    if "omega_mag_rad_s" in df.columns:
        wmag = df["omega_mag_rad_s"].to_numpy(dtype=float)
    else:
        wmag = mag3(df, ["omega_B_x_rad_s", "omega_B_y_rad_s", "omega_B_z_rad_s"])
    out = {
        "rows": int(len(df)),
        "cols": int(len(df.columns)),
        "finite_numeric": finite_numeric(df),
        "initial_omega_mag_rad_s": float(wmag[0]),
        "final_omega_mag_rad_s": float(wmag[-1]),
        "detumble_time_0p05_s": detumble_time(t, wmag, 0.05),
        "detumble_time_0p02_s": detumble_time(t, wmag, 0.02),
        "detumble_time_0p01_s": detumble_time(t, wmag, 0.01),
    }
    if "pcoil_total_W" in df.columns:
        out["mean_coil_power_W"] = float(df["pcoil_total_W"].mean())
        out["peak_coil_power_W"] = float(df["pcoil_total_W"].max())
    if "B_B_mag_T" in df.columns:
        out["mean_B_body_T"] = float(df["B_B_mag_T"].mean())
        out["peak_B_body_T"] = float(df["B_B_mag_T"].max())
    elif all(c in df.columns for c in ["B_B_x_T", "B_B_y_T", "B_B_z_T"]):
        bmag = mag3(df, ["B_B_x_T", "B_B_y_T", "B_B_z_T"])
        out["mean_B_body_T"] = float(bmag.mean())
        out["peak_B_body_T"] = float(bmag.max())
    if "applied_torque_B_mag_Nm" in df.columns:
        out["mean_applied_magnetic_torque_Nm"] = float(df["applied_torque_B_mag_Nm"].mean())
        out["peak_applied_magnetic_torque_Nm"] = float(df["applied_torque_B_mag_Nm"].max())
    elif "mtb_torque_B_mag_Nm" in df.columns:
        out["mean_mtb_torque_Nm"] = float(df["mtb_torque_B_mag_Nm"].mean())
        out["peak_mtb_torque_Nm"] = float(df["mtb_torque_B_mag_Nm"].max())
    elif all(c in df.columns for c in ["mtb_torque_B_x_Nm", "mtb_torque_B_y_Nm", "mtb_torque_B_z_Nm"]):
        tmtb = mag3(df, ["mtb_torque_B_x_Nm", "mtb_torque_B_y_Nm", "mtb_torque_B_z_Nm"])
        out["mean_mtb_torque_Nm"] = float(tmtb.mean())
        out["peak_mtb_torque_Nm"] = float(tmtb.max())
    return out


def main():
    if not REF.exists():
        raise SystemExit(f"Reference CSV not found: {REF}")
    bsk_csv = OUT_DATA / "detumble_output.csv"
    if not bsk_csv.exists():
        bsk_csv = OUT_DATA / "minimal_output.csv"
    if not bsk_csv.exists():
        raise SystemExit("No Basilisk output found. Run scenario_huskysat2_detumble.py or scenario_huskysat2_minimal.py first.")

    ref_df = pd.read_csv(REF)
    bsk_df = pd.read_csv(bsk_csv)
    ref = summarize_reference(ref_df)
    bsk = summarize_basilisk(bsk_df)

    comparison = {
        "reference_file": str(REF),
        "basilisk_file": str(bsk_csv),
        "reference": ref,
        "basilisk": bsk,
        "notes": [
            "This comparison is physical-metric based, not CSV schema exact.",
            "Exact agreement is not expected until WMM epoch, frames, controller timing, disturbances, and actuator models are aligned.",
        ],
    }

    COMPARE_OUT.parent.mkdir(parents=True, exist_ok=True)
    COMPARE_OUT.write_text(json.dumps(comparison, indent=2), encoding="utf-8")

    print(json.dumps(comparison, indent=2))
    print(f"Wrote {COMPARE_OUT}")


if __name__ == "__main__":
    main()
