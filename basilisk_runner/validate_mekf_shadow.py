"""Phase 7D opt-in live/offline interface verification, not flight performance."""
from __future__ import annotations

import argparse
import contextlib
import io
import json
from pathlib import Path

import numpy as np
import pandas as pd

from attitude_mekf import MEKF, ReplayEstimator
from attitude_mekf_adapter import ShadowOptions
from attitude_mekf_prototype import load_test_policy
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run


def chronological_reference(inputs, options: ShadowOptions):
    """Independent chronological application of the recorded live input events.

    Reuse the verified core, not the Basilisk adapter or its event delivery code.
    A delayed event available by the final output epoch is applied at acquisition.
    Initial acquisition follows the explicitly restricted fresh/common-pair rule.
    """
    policy, _ = load_test_policy()
    events = [sample for batch in inputs for sample in batch.deliveries]
    events.sort(key=lambda item: (item.epoch_ns, 0 if item.sensor == "magnetic" else 1, item.event_id))
    if options.initial_q_BN is not None:
        start = inputs[0].sim_epoch_ns
        reference = ReplayEstimator(MEKF(policy, epoch_ns=start, prior_q=options.initial_q_BN))
    else:
        reference = None
        for batch in inputs:
            candidate = ReplayEstimator(MEKF(policy, epoch_ns=batch.sim_epoch_ns))
            for sample in batch.deliveries:
                if sample.epoch_ns == batch.sim_epoch_ns:
                    candidate.submit(sample, batch.sim_epoch_ns)
            if candidate.core.initialized:
                start = batch.sim_epoch_ns
                # Replay initial measurements below in chronological order too.
                reference = ReplayEstimator(MEKF(policy, epoch_ns=start))
                break
        if reference is None:
            return None
    events = [event for event in events if event.epoch_ns >= start]
    position = 0
    while position < len(events) and events[position].epoch_ns == start:
        reference.submit(events[position], start)
        position += 1
    for batch in inputs:
        for begin, end, rate in batch.intervals:
            if end <= start:
                continue
            while position < len(events) and events[position].epoch_ns <= end:
                event = events[position]
                if reference.core.epoch_ns < event.epoch_ns:
                    reference.advance(reference.core.epoch_ns, event.epoch_ns, rate)
                reference.submit(event, event.epoch_ns)
                position += 1
            if reference.core.epoch_ns < end:
                reference.advance(reference.core.epoch_ns, end, rate)
    if position != len(events):
        raise AssertionError("offline event lacks gyro support")
    return reference


def equivalence(frame, options):
    reference = chronological_reference(frame.attrs["shadow_input_trace"], options)
    actual = frame.attrs["shadow_estimator_trace"][-1]
    if reference is None:
        return {"passed": actual["core"] is None, "initialized": False}
    if actual["core"] is None:
        return {"passed": False, "initialized": False}
    observed = actual["core"]
    q = observed["q_BN"]
    q_difference = min(np.linalg.norm(q-reference.core.q), np.linalg.norm(q+reference.core.q))
    bias_difference = float(np.max(np.abs(observed["bias_B_rad_s"]-reference.core.bias)))
    p_difference = float(np.max(np.abs(observed["P"]-reference.core.p)))
    epoch_equal = observed["epoch_ns"] == reference.core.epoch_ns
    counts_equal = actual["updates"] == dict(reference.core.updates)
    return {"passed": bool(q_difference < 1e-12 and bias_difference < 1e-12 and p_difference < 1e-12
                             and epoch_equal and counts_equal),
            "initialized": True, "quaternion_component_difference_norm": float(q_difference),
            "bias_max_abs_difference_rad_s": bias_difference, "P_max_abs_difference": p_difference,
            "epochs_equal": epoch_equal, "counts_equal": counts_equal,
            "numerical_tolerance": "TEST-ONLY 1e-12 absolute; not an attitude-accuracy requirement"}


def run_development_cases(duration=8.):
    options = {
        "two_vectors": ShadowOptions(ideal_sun=True),
        "magnetic_only": ShadowOptions(),
        "sun_only": ShadowOptions(ideal_sun=True, magnetic_enabled=False),
        "gyro_only_uninitialized": ShadowOptions(magnetic_enabled=False),
        "gyro_only_explicit_prior": ShadowOptions(magnetic_enabled=False, initial_q_BN=(1., 0., 0., 0.)),
        "lost_sun": ShadowOptions(ideal_sun=True, drop_sun_after_ns=2_000_000_000),
        "delayed_magnetic": ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000),
        "delayed_sun": ShadowOptions(ideal_sun=True, sun_delay_ns=200_000_000),
    }
    with contextlib.redirect_stdout(io.StringIO()):
        baseline = run(stop_time_s=duration, cycle=diagnostic_cycle_config(), write_outputs=False, make_plots=False)
    report = {"scope": "SHADOW DEVELOPMENT INTEGRATION / NOT FLIGHT VALIDATED", "cases": {}}
    for name, option in options.items():
        with contextlib.redirect_stdout(io.StringIO()):
            host = run(stop_time_s=duration, cycle=diagnostic_cycle_config(), shadow=option,
                       write_outputs=False, make_plots=False)
        pd.testing.assert_frame_equal(baseline, host, check_exact=True)
        telemetry = host.attrs["shadow_telemetry"]
        valid = telemetry.loc[telemetry.valid]
        status = host.attrs["shadow_status"][-1]
        expected_initialized = name not in ("magnetic_only", "sun_only", "gyro_only_uninitialized")
        matches = equivalence(host, option)
        epoch_consistent = bool(((valid.nav_written_ns == valid.time_ns)
                                 & (valid.estimator_state_epoch_ns == valid.time_ns)
                                 & (valid.gyro_epoch_ns == valid.time_ns)).all())
        checked = {"passed": bool(matches["passed"] and epoch_consistent
                                  and status["initialized"] == expected_initialized and not status["fault"]),
                   "production_frame_exact": True, "epoch_consistent": epoch_consistent,
                   "valid_rows": len(valid), "rows": len(telemetry), "final_state": status["state"],
                   "updates": status["updates"], "acquisitions": status["acquisitions"],
                   "replays": status["replays"], "rejections": status["rejected"], "equivalence": matches,
                   "initial_valid_epoch_ns": int(valid.time_ns.iloc[0]) if len(valid) else None,
                   "final_development_attitude_error_rad": float(valid.attitude_error_rad.iloc[-1]) if len(valid) else None,
                   "minimum_covariance_eigenvalue": float(valid.covariance_eigenvalue_min.min()) if len(valid) else None}
        report["cases"][name] = checked
    report["passed"] = all(case["passed"] for case in report["cases"].values())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = run_development_cases(args.duration)
    text = json.dumps(report, indent=2, allow_nan=False)
    if args.report:
        args.report.write_text(text+"\n", encoding="utf-8")
    print(text)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
