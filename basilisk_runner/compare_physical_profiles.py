"""Phase 5 mass/geometry/inertia sensitivity, using saved native detumble runs.

No simulation is run here. Each CSV is bound to its provenance-bearing config
and run manifest. All other inputs must match. Results are sampled development
metrics, NOT HS-2 requirement verification or flight validation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

import compare_reference_vs_basilisk as validation
from hs2_sim_config import HS2SimConfig, physical_profile_name

HERE = Path(__file__).resolve().parent
OUT_DATA = HERE / "output_data"


def load_run(csv_path: Path) -> tuple[pd.DataFrame, HS2SimConfig, dict[str, Any]]:
    config = HS2SimConfig.load(csv_path.with_name(csv_path.stem + "_config.json"))
    manifest = json.loads(csv_path.with_name(csv_path.stem + "_run.json").read_text(encoding="utf-8"))
    if (manifest["configuration_sha256"] != config.fingerprint()
            or HS2SimConfig.from_dict(manifest["configuration"]) != config
            or manifest["physical_profile"] != physical_profile_name(config)
            or manifest["csv_file"] != csv_path.name
            or manifest["csv_sha256"] != hashlib.sha256(csv_path.read_bytes()).hexdigest()):
        raise ValueError(f"Run manifest/configuration/CSV mismatch: {csv_path}")
    frame = pd.read_csv(csv_path)
    frame.attrs["simulation_config"] = config.to_dict()
    validation.config_for_telemetry(frame, config)
    return frame, config, manifest


def inertia_sanity(config: HS2SimConfig) -> dict[str, Any]:
    """Independent volume integral, exact for the quadratic inertia integrand.

    Two-point Gauss quadrature in each prism dimension uses eight equal mass
    weights at +/-side/(2*sqrt(3)). Integrate r^2*identity - outer(r,r), rather
    than reusing the profile's three moment formulas. Inputs are kg and metres;
    each weighted squared distance has units kg m^2 about the assumed centroid.
    """
    sc = config.spacecraft
    tensor = np.asarray(sc.inertia.value, dtype=float)
    half_nodes = np.asarray(sc.dimensions.value, dtype=float) / (2 * np.sqrt(3))
    integral = np.zeros((3, 3))
    for index in np.ndindex(2, 2, 2):
        r = (2 * np.asarray(index) - 1) * half_nodes
        integral += (sc.mass.value / 8) * ((r @ r) * np.eye(3) - np.outer(r, r))
    moments = np.linalg.eigvalsh(tensor)
    checks = {
        "symmetric": bool(np.array_equal(tensor, tensor.T)),
        "positive_definite": bool(np.all(moments > 0)),
        "principal_moment_triangle_inequalities": bool(np.all(2 * moments <= moments.sum() + 1e-14)),
        "dimensionally_consistent": sc.mass.units == "kg" and sc.dimensions.units == "m" and sc.inertia.units == "kg m^2",
        "matches_independent_volume_integral": bool(np.allclose(tensor, integral, rtol=1e-14, atol=1e-16)),
    }
    return {"passed": all(checks.values()), "checks": checks,
            "principal_moments_kg_m2": moments.tolist(),
            "volume_integral_tensor_kg_m2": integral.tolist(),
            "max_volume_integral_error_kg_m2": float(np.max(np.abs(tensor - integral)))}


def threshold_observation(times: np.ndarray, rates: np.ndarray, threshold: float) -> dict[str, Any]:
    """Strictly below at saved samples; no interpolation or claimed dwell rule.

    'below_through_end_from_s' excludes transient crossings, but only establishes
    the remaining observed window. HS-2 sustained-success criteria remain TBC.
    """
    below = rates < threshold
    crossings = np.flatnonzero(below)
    first = None if not len(crossings) else float(times[crossings[0]])
    above = np.flatnonzero(~below)
    index = 0 if not len(above) else int(above[-1]) + 1
    sustained = float(times[index]) if index < len(times) else None
    return {"threshold_rad_s": threshold, "first_below_s": first,
            "below_through_end_from_s": sustained,
            "remaining_observed_window_s": None if sustained is None else float(times[-1]) - sustained}


def attitude_separation_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Relative rotation angle, invariant to MRP shadow-set sign changes."""
    def quaternion(sigma):
        s2 = np.sum(sigma * sigma, axis=1, keepdims=True)
        return np.column_stack(((1 - s2) / (1 + s2), 2 * sigma / (1 + s2)))
    qa, qb = quaternion(a), quaternion(b)
    vector = qa[:, :1] * qb[:, 1:] - qb[:, :1] * qa[:, 1:] - np.cross(qa[:, 1:], qb[:, 1:])
    return np.degrees(2 * np.arctan2(np.linalg.norm(vector, axis=1), np.abs(np.sum(qa * qb, axis=1))))


def summarize_profile(df: pd.DataFrame, config: HS2SimConfig) -> dict[str, Any]:
    metrics = validation.summarize_basilisk(df, config)
    # No standalone data is consumed or purportedly validated by this study.
    checks = validation.build_validation({}, metrics)["checks"]
    del checks["reference_numeric_finite"]
    rates = df[[f"omega_B_{axis}_rad_s" for axis in "xyz"]].to_numpy(dtype=float)
    speed = np.linalg.norm(rates, axis=1)
    times = df.time_s.to_numpy(dtype=float)
    saturation = df[[f"saturation_{axis}" for axis in "xyz"]].to_numpy()
    # Existing telemetry deliberately stores diagnostic flags as numeric 0/1.
    # Validate that representation before interpreting it; do not alter the CSV.
    if saturation.dtype.kind not in "biuf" or not np.isin(saturation, (0, 1)).all():
        raise ValueError("Expected binary controller saturation flags")
    saturation = saturation.astype(bool)
    return {
        "physical_profile": physical_profile_name(config),
        "configuration_sha256": config.fingerprint(),
        "physical_parameters": config.to_dict()["spacecraft"],
        "inertia_sanity": inertia_sanity(config),
        "initial_rotational_energy_J": metrics["initial_rotational_energy_J"],
        "initial_rate_deg_s": float(np.degrees(speed[0])),
        "final_rate_rad_s": metrics["final_omega_mag_rad_s"],
        "final_rate_deg_s": float(np.degrees(speed[-1])),
        "requested_duration_s": config.duration_s,
        "last_recorded_time_s": float(times[-1]),
        "thresholds": {name: threshold_observation(times, speed, limit) for name, limit in
                       (("0.5_deg_s", float(np.radians(0.5))), ("0.05_rad_s", 0.05), ("0.02_rad_s", 0.02))},
        "mean_magnetic_torque_Nm": metrics["mean_applied_magnetic_torque_Nm"],
        "peak_magnetic_torque_Nm": metrics["peak_applied_magnetic_torque_Nm"],
        "mean_coil_power_W": metrics["mean_coil_power_W"],
        "peak_coil_power_W": metrics["peak_coil_power_W"],
        "saturation": {"any_axis_sample_fraction": float(np.any(saturation, axis=1).mean()),
                       "per_axis_sample_fraction": dict(zip("xyz", saturation.mean(axis=0).tolist())),
                       "samples_with_any_axis_clipped": int(np.any(saturation, axis=1).sum()),
                       "meaning": "Controller clipping flags on the saved 1 s grid; not measured actuator duty cycle"},
        "validation": {"passed": all(item["passed"] for item in checks.values()), "checks": checks},
    }


def compare_runs(baseline_path: Path, candidate_path: Path) -> dict[str, Any]:
    baseline, base_config, base_manifest = load_run(baseline_path)
    candidate, candidate_config, candidate_manifest = load_run(candidate_path)
    if (physical_profile_name(base_config), physical_profile_name(candidate_config)) != ("regression_baseline", "hs2_candidate"):
        raise ValueError("Expected regression_baseline followed by hs2_candidate")
    if replace(base_config, spacecraft=candidate_config.spacecraft) != candidate_config:
        raise ValueError("Sensitivity runs must differ only in spacecraft physical parameters")
    if not np.array_equal(baseline.time_ns, candidate.time_ns):
        raise ValueError("Sensitivity runs require identical exact sample epochs")
    for frame in (baseline, candidate):
        if not ((frame.applied_torque_source == "MtbEffector.torqueExternalPntB_B").all()
                and (frame.command_mode == "controller").all()):
            raise ValueError("Sensitivity study requires native actuation with controller feedback")

    def vectors(frame, prefix, suffix):
        return frame[[f"{prefix}_{axis}_{suffix}" for axis in "xyz"]].to_numpy(dtype=float)
    rates_a, rates_b = vectors(baseline, "omega_B", "rad_s"), vectors(candidate, "omega_B", "rad_s")
    sigma_columns = [f"sigma_BN_{i}" for i in (1, 2, 3)]
    sigma_a, sigma_b = baseline[sigma_columns].to_numpy(dtype=float), candidate[sigma_columns].to_numpy(dtype=float)
    if not (np.array_equal(rates_a[0], rates_b[0]) and np.array_equal(sigma_a[0], sigma_b[0])):
        raise ValueError("Initial attitude/rate must match")
    rate_delta = np.linalg.norm(rates_b - rates_a, axis=1)
    angle_delta = attitude_separation_deg(sigma_a, sigma_b)
    position_delta = float(np.linalg.norm(vectors(candidate, "r_N", "m") - vectors(baseline, "r_N", "m"), axis=1).max())
    field_delta = float(np.linalg.norm(vectors(candidate, "B_N", "T") - vectors(baseline, "B_N", "T"), axis=1).max())
    summaries = {"regression_baseline": summarize_profile(baseline, base_config),
                 "hs2_candidate": summarize_profile(candidate, candidate_config)}
    shared_environment = position_delta < 1e-6 and field_delta < 1e-15
    return {
        "passed": shared_environment and all(s["validation"]["passed"] and s["inertia_sanity"]["passed"] for s in summaries.values()),
        "scope": "Assumed physical-configuration sensitivity; NOT FLIGHT VALIDATED; no requirement compliance claim",
        "inputs": {"baseline": {"csv": str(baseline_path), "csv_sha256": base_manifest["csv_sha256"]},
                   "candidate": {"csv": str(candidate_path), "csv_sha256": candidate_manifest["csv_sha256"]}},
        "profiles": summaries,
        "differences": {
            "max_rate_vector_difference_rad_s": float(rate_delta.max()),
            "rms_rate_vector_difference_rad_s": float(np.sqrt(np.mean(rate_delta**2))),
            "final_rate_vector_difference_rad_s": float(rate_delta[-1]),
            "max_rate_magnitude_difference_rad_s": float(np.abs(np.linalg.norm(rates_b, axis=1) - np.linalg.norm(rates_a, axis=1)).max()),
            "max_attitude_separation_deg": float(angle_delta.max()),
            "final_attitude_separation_deg": float(angle_delta[-1]),
            "max_position_difference_m": position_delta,
            "max_inertial_field_difference_T": field_delta,
            "shared_environment_within_roundoff_tolerance": shared_environment,
            "any_axis_saturation_sample_fraction_delta": summaries["hs2_candidate"]["saturation"]["any_axis_sample_fraction"] - summaries["regression_baseline"]["saturation"]["any_axis_sample_fraction"],
        },
        "requirement_context": {
            "statement": "Detumble from at least 10 deg/s to 0.5 deg/s within 24 hours after deployment",
            "status": "CONFIRMED", "source": "docs/REQUIREMENTS_BASELINE.md: R1 ADCS-1; R2 section 4.5; M1 section 4.1",
            "source_revision": "R1 tracker rev 9, 2026-03-21; R2 rev E, 2025-10-05; M1 log rev 8, 2026-08-17",
            "interpretation": "Study uses angular-rate vector norm; requirement norm/per-axis, direction envelope and dwell criteria remain TBC",
            "requirement_verified": False, "24_hour_evaluation_performed": False,
            "24_hour_run_constraint": "Existing EarthOrientation.UpdateState and configuration validation require t < 86400 s. Extending the reviewed time contract is outside Phase 5; no environment implementation was changed.",
            "limitation": "One orbit cannot establish the 24-hour obligation. Synthetic initial direction, ideal sensors, provisional actuators and box inertia do not establish flight performance.",
        },
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=OUT_DATA / "detumble_output.csv")
    parser.add_argument("--candidate", type=Path, default=OUT_DATA / "detumble_output_hs2_candidate.csv")
    parser.add_argument("--report", type=Path, default=OUT_DATA / "physical_profile_comparison.json")
    args = parser.parse_args(argv)
    try:
        report = compare_runs(args.baseline, args.candidate)
    except (ValueError, KeyError, IndexError, OSError) as exc:
        report = {"passed": False, "error": str(exc)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, indent=2, allow_nan=False))
    print(f"Wrote {args.report}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
