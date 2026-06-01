from __future__ import annotations

from pathlib import Path
import json
from typing import Any

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REF = ROOT / "reference_standalone" / "adcs_output.csv"
OUT_DATA = HERE / "output_data"
COMPARE_OUT = OUT_DATA / "comparison_metrics.json"

# Must match the current HuskySat-style baseline used by the Basilisk scenarios.
# TODO: replace with verified HuskySat-2 inertia once the final vehicle properties are available.
MASS_KG = 2.6
LX_M, LY_M, LZ_M = 0.10, 0.10, 0.20
IXX = (MASS_KG / 12.0) * (LY_M**2 + LZ_M**2)
IYY = (MASS_KG / 12.0) * (LX_M**2 + LZ_M**2)
IZZ = (MASS_KG / 12.0) * (LX_M**2 + LY_M**2)
INERTIA_DIAG_KG_M2 = np.array([IXX, IYY, IZZ], dtype=float)


def mag3(df: pd.DataFrame, cols: list[str]) -> np.ndarray:
    return np.sqrt(sum(np.asarray(df[c], dtype=float) ** 2 for c in cols))


def vec3(df: pd.DataFrame, cols: list[str]) -> np.ndarray:
    return np.column_stack([np.asarray(df[c], dtype=float) for c in cols])


def detumble_time(t: np.ndarray, wmag: np.ndarray, threshold: float) -> float | None:
    idx = np.where(wmag <= threshold)[0]
    return None if len(idx) == 0 else float(t[idx[0]])


def finite_numeric(df: pd.DataFrame) -> bool:
    numeric = df.select_dtypes(include=[np.number])
    return bool(np.isfinite(numeric.to_numpy(dtype=float)).all())


def rotational_energy_J(omega_B_rad_s: np.ndarray) -> np.ndarray:
    """Rigid-body rotational kinetic energy using the scenario diagonal inertia."""
    return 0.5 * np.sum((omega_B_rad_s**2) * INERTIA_DIAG_KG_M2.reshape((1, 3)), axis=1)


def maybe_remove_stale_comparison() -> None:
    """Avoid leaving stale comparison_metrics.json after a failed compare run."""
    try:
        COMPARE_OUT.unlink(missing_ok=True)
    except TypeError:  # Python < 3.8 defensive fallback
        if COMPARE_OUT.exists():
            COMPARE_OUT.unlink()


def summarize_reference(df: pd.DataFrame) -> dict[str, Any]:
    t = df["t"].to_numpy(dtype=float)
    omega = vec3(df, ["p", "q", "r"])
    wmag = np.linalg.norm(omega, axis=1)
    energy = rotational_energy_J(omega)
    out: dict[str, Any] = {
        "rows": int(len(df)),
        "cols": int(len(df.columns)),
        "finite_numeric": finite_numeric(df),
        "initial_omega_mag_rad_s": float(wmag[0]),
        "final_omega_mag_rad_s": float(wmag[-1]),
        "detumble_time_0p05_s": detumble_time(t, wmag, 0.05),
        "detumble_time_0p02_s": detumble_time(t, wmag, 0.02),
        "detumble_time_0p01_s": detumble_time(t, wmag, 0.01),
        "initial_rotational_energy_J": float(energy[0]),
        "final_rotational_energy_J": float(energy[-1]),
        "rotational_energy_reduction_fraction": float((energy[0] - energy[-1]) / energy[0]) if energy[0] > 0 else None,
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


def summarize_basilisk(df: pd.DataFrame) -> dict[str, Any]:
    t = df["time_s"].to_numpy(dtype=float)
    omega = vec3(df, ["omega_B_x_rad_s", "omega_B_y_rad_s", "omega_B_z_rad_s"])
    if "omega_mag_rad_s" in df.columns:
        wmag = df["omega_mag_rad_s"].to_numpy(dtype=float)
    else:
        wmag = np.linalg.norm(omega, axis=1)

    energy = rotational_energy_J(omega)
    out: dict[str, Any] = {
        "rows": int(len(df)),
        "cols": int(len(df.columns)),
        "finite_numeric": finite_numeric(df),
        "initial_omega_mag_rad_s": float(wmag[0]),
        "final_omega_mag_rad_s": float(wmag[-1]),
        "detumble_time_0p05_s": detumble_time(t, wmag, 0.05),
        "detumble_time_0p02_s": detumble_time(t, wmag, 0.02),
        "detumble_time_0p01_s": detumble_time(t, wmag, 0.01),
        "initial_rotational_energy_J": float(energy[0]),
        "final_rotational_energy_J": float(energy[-1]),
        "rotational_energy_reduction_fraction": float((energy[0] - energy[-1]) / energy[0]) if energy[0] > 0 else None,
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

    torque_cols = None
    if all(c in df.columns for c in ["applied_torque_B_x_Nm", "applied_torque_B_y_Nm", "applied_torque_B_z_Nm"]):
        torque_cols = ["applied_torque_B_x_Nm", "applied_torque_B_y_Nm", "applied_torque_B_z_Nm"]
        torque_label = "applied_magnetic_torque"
    elif all(c in df.columns for c in ["mtb_torque_B_x_Nm", "mtb_torque_B_y_Nm", "mtb_torque_B_z_Nm"]):
        torque_cols = ["mtb_torque_B_x_Nm", "mtb_torque_B_y_Nm", "mtb_torque_B_z_Nm"]
        torque_label = "mtb_torque"
    else:
        torque_label = "none"

    if torque_cols is not None:
        torque = vec3(df, torque_cols)
        torque_mag = np.linalg.norm(torque, axis=1)
        mech_power = np.sum(omega * torque, axis=1)
        nonzero_torque = torque_mag > 1e-14
        out[f"mean_{torque_label}_Nm"] = float(torque_mag.mean())
        out[f"peak_{torque_label}_Nm"] = float(torque_mag.max())
        out["mean_mechanical_control_power_W"] = float(mech_power.mean())
        out["min_mechanical_control_power_W"] = float(mech_power.min())
        out["max_mechanical_control_power_W"] = float(mech_power.max())
        out["negative_mechanical_power_fraction"] = float(np.mean(mech_power[nonzero_torque] < 0.0)) if np.any(nonzero_torque) else None

    if all(c in df.columns for c in ["mcmd_x_Am2", "mcmd_y_Am2", "mcmd_z_Am2", "B_B_x_T", "B_B_y_T", "B_B_z_T"]):
        m_B = vec3(df, ["mcmd_x_Am2", "mcmd_y_Am2", "mcmd_z_Am2"])
        B_B = vec3(df, ["B_B_x_T", "B_B_y_T", "B_B_z_T"])
        tau_cross = np.cross(m_B, B_B)
        if torque_cols is not None:
            torque = vec3(df, torque_cols)
            err = torque - tau_cross
            err_mag = np.linalg.norm(err, axis=1)
            tau_mag = np.linalg.norm(torque, axis=1)
            out["torque_cross_product_mean_abs_error_Nm"] = float(err_mag.mean())
            out["torque_cross_product_max_abs_error_Nm"] = float(err_mag.max())
            out["torque_cross_product_max_rel_error"] = float(err_mag.max() / max(float(tau_mag.max()), 1e-30))

    if all(c in df.columns for c in ["B_N_x_T", "B_N_y_T", "B_N_z_T", "B_B_x_T", "B_B_y_T", "B_B_z_T"]):
        B_N = vec3(df, ["B_N_x_T", "B_N_y_T", "B_N_z_T"])
        B_B = vec3(df, ["B_B_x_T", "B_B_y_T", "B_B_z_T"])
        b_norm_err = np.abs(np.linalg.norm(B_N, axis=1) - np.linalg.norm(B_B, axis=1))
        out["max_B_frame_norm_error_T"] = float(b_norm_err.max())
        out["mean_B_frame_norm_error_T"] = float(b_norm_err.mean())

    return out


def check(name: str, passed: bool, value: Any = None, limit: Any = None) -> dict[str, Any]:
    return {"passed": bool(passed), "value": value, "limit": limit}


def build_validation(ref: dict[str, Any], bsk: dict[str, Any]) -> dict[str, Any]:
    checks: dict[str, Any] = {}
    checks["reference_numeric_finite"] = check("reference_numeric_finite", ref.get("finite_numeric") is True)
    checks["basilisk_numeric_finite"] = check("basilisk_numeric_finite", bsk.get("finite_numeric") is True)
    checks["basilisk_final_omega_less_than_initial"] = check(
        "basilisk_final_omega_less_than_initial",
        bsk["final_omega_mag_rad_s"] < bsk["initial_omega_mag_rad_s"],
        bsk["final_omega_mag_rad_s"],
        f"< {bsk['initial_omega_mag_rad_s']}",
    )
    checks["basilisk_final_energy_less_than_initial"] = check(
        "basilisk_final_energy_less_than_initial",
        bsk["final_rotational_energy_J"] < bsk["initial_rotational_energy_J"],
        bsk["final_rotational_energy_J"],
        f"< {bsk['initial_rotational_energy_J']}",
    )
    checks["basilisk_mean_mechanical_power_negative"] = check(
        "basilisk_mean_mechanical_power_negative",
        bsk.get("mean_mechanical_control_power_W", 1.0) < 0.0,
        bsk.get("mean_mechanical_control_power_W"),
        "< 0 W",
    )
    checks["torque_matches_m_cross_B"] = check(
        "torque_matches_m_cross_B",
        bsk.get("torque_cross_product_max_abs_error_Nm", float("inf")) < 5e-8
        or bsk.get("torque_cross_product_max_rel_error", float("inf")) < 0.05,
        {
            "max_abs_error_Nm": bsk.get("torque_cross_product_max_abs_error_Nm"),
            "max_rel_error": bsk.get("torque_cross_product_max_rel_error"),
        },
        "max abs < 5e-8 Nm OR max rel < 5%",
    )
    checks["B_body_mean_LEO_plausible"] = check(
        "B_body_mean_LEO_plausible",
        2.0e-5 <= bsk.get("mean_B_body_T", 0.0) <= 7.0e-5,
        bsk.get("mean_B_body_T"),
        "2e-5 T <= mean <= 7e-5 T",
    )
    checks["B_frame_norm_preserved"] = check(
        "B_frame_norm_preserved",
        bsk.get("max_B_frame_norm_error_T", float("inf")) < 1e-10,
        bsk.get("max_B_frame_norm_error_T"),
        "< 1e-10 T",
    )
    checks["peak_coil_power_within_expected_limit"] = check(
        "peak_coil_power_within_expected_limit",
        bsk.get("peak_coil_power_W", float("inf")) <= 2.6,
        bsk.get("peak_coil_power_W"),
        "<= 2.6 W",
    )
    checks["detumble_to_0p05_within_expected_window"] = check(
        "detumble_to_0p05_within_expected_window",
        bsk.get("detumble_time_0p05_s") is not None and 3000.0 <= bsk["detumble_time_0p05_s"] <= 3500.0,
        bsk.get("detumble_time_0p05_s"),
        "3000 s to 3500 s",
    )
    return {
        "passed": all(item["passed"] for item in checks.values()),
        "checks": checks,
    }


def main() -> None:
    if not REF.exists():
        maybe_remove_stale_comparison()
        raise SystemExit(f"Reference CSV not found: {REF}")
    bsk_csv = OUT_DATA / "detumble_output.csv"
    if not bsk_csv.exists():
        bsk_csv = OUT_DATA / "minimal_output.csv"
    if not bsk_csv.exists():
        maybe_remove_stale_comparison()
        raise SystemExit("No Basilisk output found. Run scenario_huskysat2_detumble.py or scenario_huskysat2_minimal.py first.")

    ref_df = pd.read_csv(REF)
    bsk_df = pd.read_csv(bsk_csv)
    ref = summarize_reference(ref_df)
    bsk = summarize_basilisk(bsk_df)
    validation = build_validation(ref, bsk)

    comparison = {
        "reference_file": str(REF),
        "basilisk_file": str(bsk_csv),
        "reference": ref,
        "basilisk": bsk,
        "validation": validation,
        "notes": [
            "This comparison is physical-metric based, not CSV schema exact.",
            "The validation checks are range/invariant tests, not claims of exact standalone equivalence.",
            "Exact agreement is not expected until WMM epoch, frames, controller timing, disturbances, and actuator models are aligned.",
        ],
    }

    COMPARE_OUT.parent.mkdir(parents=True, exist_ok=True)
    COMPARE_OUT.write_text(json.dumps(comparison, indent=2), encoding="utf-8")

    print(json.dumps(comparison, indent=2))
    print(f"Wrote {COMPARE_OUT}")
    print(f"Validation passed?: {validation['passed']}")
    if not validation["passed"]:
        failed = [name for name, item in validation["checks"].items() if not item["passed"]]
        print("Failed checks:")
        for name in failed:
            print(f"  - {name}")


if __name__ == "__main__":
    main()
