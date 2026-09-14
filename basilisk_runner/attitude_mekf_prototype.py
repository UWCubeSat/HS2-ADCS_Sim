"""Opt-in Phase 7C shadow harness. Never supplies attitude to production control.

All sensor injections and covariance weights are TEST-ONLY (2026-09-07).
Basilisk's unchanged cycled scenario owns truth. Synthetic 100 Hz gyro uses
four-point cubic interpolation of recorded angular RATE at subinterval midpoints,
never attitude differences. Offline lookahead belongs to sensor synthesis only;
it is not an installed packet model. Interpolation/integration error remains.
Ideal Sun is a synthetic vector,
not a CSS model. Magnetic input is the actual frozen TAM acquisition.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
from typing import Mapping, cast

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf import MEKF, LossEvent, NumericalPolicy, ReplayEstimator, VectorSample

HERE = Path(__file__).resolve().parent
TEST_CONFIG = HERE / "config" / "attitude_mekf_test_only.json"


def load_test_policy(path: Path = TEST_CONFIG) -> tuple[NumericalPolicy, dict]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw["flight_runtime_usable"] or any(raw[key]["value"] != 0 for key in
                                          ("injected_noise", "injected_alignment_error")):
        raise ValueError("Phase 7C requires an isolated ideal-observation test profile")
    policy = NumericalPolicy(np.diag(raw["p0_diagonal"]["value"]), np.diag(raw["qc_diagonal"]["value"]),
        raw["vector_weight"]["value"], raw["maximum_residual"]["value"],
        raw["minimum_acquisition_sine"]["value"], raw["history"]["value"],
        raw["max_events"]["value"], raw["label"], raw["acquisition_pair_tolerance"]["value"])
    return policy, raw


def magnetic_cycle_sample(row: Mapping[str, object], cycle) -> VectorSample:
    """Convert an ACQUISITION row, retaining the original TAM and WMM epochs.

    Later receipt is allowed through replay; passing an ACTUATE row containing
    held data is not an acquisition. Physical coil decay/filter effects remain
    absent exactly as in Phase 6A. No current B_B truth column is accessed.
    """
    def number(key: str) -> float:
        return float(cast(float, row[key]))

    def values(prefix: str, suffix: str = ""):
        return np.array([number(f"{prefix}_{a}{suffix}") for a in "xyz"])

    tick = int(cast(int, row["time_ns"]))
    acquisition = int(cast(int, row["cycle_sample_epoch_ns"]))
    reference_epoch = int(cast(int, row["field_evaluation_time_ns"]))
    valid = (bool(row["cycle_sample_valid"]) and bool(row["cycle_sample_event"])
             and row["cycle_phase"] == "SAMPLE" and tick == acquisition
             and tick % cycle.period_ns == cycle.sample_offset_ns
             and int(cast(int, row["tam_message_time_ns"])) == acquisition
             and reference_epoch == acquisition
             and number("cycle_time_since_disabled_ns") >= cycle.sample_offset_ns
             and not np.any(values("cycle_pre_native_command"))
             and not np.any(values("cycle_pre_native_effective_dipole"))
             and not np.any(values("applied_torque_B", "_Nm")))
    measured = values("tam_sample_B_B", "_T")
    # Separate consistency witness, not a replacement for the TAM sample.
    valid = valid and np.array_equal(measured, values("cycle_sample_B_B"))
    return VectorSample(f"cycle-mag-{tick}", "magnetic", acquisition, reference_epoch,
        measured, values("cycle_sample_B_N"), "Phase6A frozen TAM + sample-epoch WMM; ideal S=B",
        valid=valid, invalid_reason="magnetic_cycle_invalid")


def independent_error(estimated_q, truth_sigma):
    """Stable quaternion principal angle and DCM small-angle error; no MEKF math."""
    truth_q = np.asarray(rbk.MRP2EP(truth_sigma), dtype=float)
    q = np.asarray(estimated_q, dtype=float)
    q = q/np.linalg.norm(q)
    if q @ truth_q < 0:
        q = -q
    angle = 4*np.arctan2(np.linalg.norm(q-truth_q), np.linalg.norm(q+truth_q))
    relative = np.asarray(rbk.MRP2C(truth_sigma)) @ np.asarray(rbk.EP2C(q)).T
    axis_small = -0.5*np.array([relative[2, 1]-relative[1, 2], relative[0, 2]-relative[2, 0],
                              relative[1, 0]-relative[0, 1]])
    return float(angle), axis_small


def interpolated_gyro(rates: np.ndarray, interval: int, fraction: float) -> np.ndarray:
    """Offline uniform-grid cubic interpolation of truth angular rate only."""
    start = max(0, min(interval-1, len(rates)-4))
    nodes = np.arange(start, start+4, dtype=float)
    point = interval+fraction
    result = np.zeros(3)
    for j, node in enumerate(nodes):
        others = np.delete(nodes, j)
        result += np.prod((point-others)/(node-others))*rates[int(node)]
    return result


def run_truth_cases():
    # Imports are intentionally lazy: inspecting/loading the estimator does not
    # import a scenario or touch its output directories.
    from magnetic_control_cycle import diagnostic_cycle_config
    from scenario_huskysat2_detumble import run

    policy, fixtures = load_test_policy()
    cycle = diagnostic_cycle_config()
    with contextlib.redirect_stdout(io.StringIO()):
        truth = run(stop_time_s=fixtures["truth_duration"]["value"], cycle=cycle,
                    write_outputs=False, make_plots=False)
    records = truth.to_dict("records")
    sun_n = np.asarray(fixtures["sun_reference_n"]["value"], dtype=float)
    sigma_cols = [f"sigma_BN_{j}" for j in (1, 2, 3)]
    omega_cols = [f"omega_B_{a}_rad_s" for a in "xyz"]
    rates = truth[omega_cols].to_numpy(dtype=float)
    substeps = fixtures["gyro_substeps"]["value"]
    sigma0 = truth[sigma_cols].iloc[0].to_numpy(dtype=float)
    # Explicit imperfect-prior fixture only; acquisition cases start uninitialized.
    prior_error = np.asarray(fixtures["prior_error"]["value"], dtype=float)
    prior_c = np.asarray(rbk.PRV2C(prior_error)) @ np.asarray(rbk.MRP2C(sigma0))
    prior_q = np.asarray(rbk.C2EP(prior_c), dtype=float)
    cases = {}
    for case in range(1, 6):
        bias = np.asarray(fixtures["synthetic_three_axis_bias" if case == 5 else "basic_true_bias"]["value"], dtype=float)
        estimator = ReplayEstimator(MEKF(policy, prior_q=None if case in (1, 4) else prior_q))
        errors, bias_errors, eig_history, samples = [], [], [], []
        delayed: list[tuple[int, VectorSample]] = []
        dispositions: dict[str, int] = {}
        for i, row in enumerate(records):
            tick = int(row["time_ns"])
            if i:
                previous = records[i-1]
                start = int(previous["time_ns"])
                if (tick-start) % substeps:
                    raise ValueError("synthetic gyro subintervals must resolve to integer ns")
                interval_ns = (tick-start)//substeps
                for part in range(substeps):
                    gyro = interpolated_gyro(rates, i-1, (part+0.5)/substeps)+bias
                    estimator.advance(start+part*interval_ns, start+(part+1)*interval_ns, gyro)
            sigma = np.array([row[c] for c in sigma_cols])
            c_bn_truth = np.asarray(rbk.MRP2C(sigma))
            # CASE 4: explicitly commanded loss followed by genuine two-vector
            # reacquisition at 14.4 s; no numerical flight loss threshold.
            if case == 4 and tick == 14_400_000_000:
                estimator.submit(LossEvent("case4-explicit-loss", tick), tick)
            dropout = case == 4 and 8_000_000_000 <= tick < 14_400_000_000
            if bool(row["cycle_sample_event"]) and case != 3 and not dropout:
                sample = magnetic_cycle_sample(row, cycle)
                # Intermittent magnetic-only case; no hidden Sun update.
                if case != 2 or int(row["cycle_index"]) % 3 == 0:
                    delay = 200_000_000 if case in (1, 5) and tick > 1_000_000_000 else 0
                    delayed.append((tick+delay, sample))
            if case == 3 and row["cycle_phase"] == "ACTUATE" and tick % 1_000_000_000 == 600_000_000:
                result = estimator.submit(magnetic_cycle_sample(row, cycle), tick)
                dispositions[result] = dispositions.get(result, 0)+1
            # 0.7 s Sun cadence, offset 0.4 s, distinct from the magnetic cadence.
            if case != 2 and not dropout and tick >= 400_000_000 and (tick-400_000_000) % 700_000_000 == 0:
                sample = VectorSample(f"sun-{tick}", "sun", tick, tick, c_bn_truth @ sun_n, sun_n,
                                      "ideal synthetic Sun vector; no CSS reconstruction")
                estimator.submit(sample, tick)
            for arrival, sample in delayed[:]:
                if arrival <= tick:
                    estimator.submit(sample, tick)
                    delayed.remove((arrival, sample))
            if estimator.core.initialized:
                angle, axes = independent_error(estimator.core.q, sigma)
                errors.append(angle)
                bias_errors.append(float(np.linalg.norm(estimator.core.bias-bias)))
                eig_history.append(np.linalg.eigvalsh(estimator.core.p))
                samples.append({"epoch_ns": tick, "angle_rad": angle, "small_angle_B_rad": axes.tolist()})
        if not errors or delayed:
            raise AssertionError("truth case lacks valid estimates or leaves undelivered samples")
        eigenvalues = np.asarray(eig_history)
        rejected = estimator.ingress_rejections + estimator.core.rejections
        cases[f"case_{case}"] = {
            "initial_attitude_error_rad": errors[0], "final_attitude_error_rad": errors[-1],
            "maximum_attitude_error_rad": max(errors), "final_small_angle_B_rad": samples[-1]["small_angle_B_rad"],
            "first_valid_epoch_ns": samples[0]["epoch_ns"], "output_epoch_ns": estimator.core.epoch_ns,
            "true_bias_B_rad_s": bias.tolist(), "final_estimated_bias_B_rad_s": estimator.core.bias.tolist(),
            "initial_bias_error_norm_rad_s": bias_errors[0], "final_bias_error_norm_rad_s": bias_errors[-1],
            "final_covariance_eigenvalues": eigenvalues[-1].tolist(),
            "minimum_covariance_eigenvalue_over_run": float(eigenvalues.min()),
            "maximum_covariance_eigenvalue_over_run": float(eigenvalues.max()),
            "updates": dict(estimator.core.updates), "rejections": dict(rejected),
            "acquisitions": estimator.core.acquisitions, "reacquisitions": max(0, estimator.core.acquisitions-1),
            "delayed_replays": estimator.replay_count, "maximum_stored_events": policy.max_events,
            "retained_events": len(estimator.gyros)+len(estimator.events),
            "finite_and_psd": bool(np.isfinite(eigenvalues).all() and eigenvalues.min() >= -1e-12),
            "covariance_meaning": "TEST-ONLY mixed-unit error-state covariance; not calibrated confidence"}
    checks = {
        "all_cases_finite_psd": all(c["finite_and_psd"] for c in cases.values()),
        "two_vector_ideal_case_small_residual": cases["case_1"]["final_attitude_error_rad"] < 0.002,
        "magnetic_only_no_sun_updates": cases["case_2"]["updates"].get("sun", 0) == 0,
        "actuation_magnetic_rejected": cases["case_3"]["rejections"].get("magnetic_cycle_invalid", 0) > 0,
        "sun_only_no_magnetic_updates": cases["case_3"]["updates"].get("magnetic", 0) == 0,
        "explicit_reacquisition": cases["case_4"]["reacquisitions"] == 1,
        "three_axis_bias_recovered": cases["case_5"]["final_bias_error_norm_rad_s"] < 0.0001,
        "bias_case_attitude_recovered": cases["case_5"]["final_attitude_error_rad"] < 0.002,
        "delayed_samples_replayed": cases["case_1"]["delayed_replays"] > 0,
    }
    return {"scope": "Phase 7C algorithmic truth cases; NOT HS-2 PERFORMANCE", "passed": all(checks.values()),
        "checks": checks, "acceptance_basis": "TEST-ONLY integration/convergence checks, not requirement thresholds",
        "test_configuration_sha256": hashlib.sha256(TEST_CONFIG.read_bytes()).hexdigest(),
        "truth_configuration_sha256": str(truth.simulation_config_sha256.iloc[0]),
        "truth_cycle_sha256": cycle.fingerprint(), "noise_free_caveat": __doc__, "cases": cases}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, help="Optional separate prototype JSON report; no production CSV writes")
    args = parser.parse_args()
    report = run_truth_cases()
    text = json.dumps(report, indent=2, allow_nan=False)
    if args.report:
        args.report.write_text(text+"\n", encoding="utf-8")
    print(text)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
