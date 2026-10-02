"""8A-1 short live shadow equivalence/provenance, TEST-ONLY, 2026-10-01.

Load the committed adapter as well as compare the current default bridge to the
opt-in IDEAL_REGRESSION source. Reuse existing chronological replay and production
checks; no new estimator equations, acceptance accuracy or flight noise tuning.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict, is_dataclass
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import numpy as np

import attitude_mekf_adapter as current_adapter
from attitude_mekf_adapter import ShadowOptions
from gyro_sensor_model import PROFILES, SCOPE, profile_config
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_mekf_closed_loop import csv_bytes, preservation
from validate_mekf_shadow import equivalence

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DURATION_S = 3.0


def normalized(value):
    """Compare original values exactly, excluding only additive sensor metadata."""
    if is_dataclass(value) and not isinstance(value, type):
        value = asdict(value)
    if isinstance(value, dict):
        return {k: normalized(v) for k, v in value.items() if k != "gyro_sample"}
    if isinstance(value, (list, tuple)):
        return [normalized(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def old_adapter_run(options, *, control_source="SIMPLE_NAV_REFERENCE"):
    """Read-only HEAD source loading, no checkout/temp files or installed edits."""
    name = "attitude_mekf_adapter"
    old = ModuleType(name)
    old.__file__ = str(HERE/(name+".py"))
    source = subprocess.check_output(["git", "show", f"HEAD:basilisk_runner/{name}.py"], cwd=ROOT, text=True, encoding="utf-8")
    try:
        sys.modules[name] = old
        exec(compile(source, old.__file__, "exec"), old.__dict__)
        return run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), shadow=old.ShadowOptions(**options), control_source=control_source)
    finally:
        sys.modules[name] = current_adapter


def validate():
    options = dict(ideal_sun=True, magnetic_delay_ns=200_000_000)
    kwargs = dict(stop_time_s=DURATION_S, write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config())
    config = profile_config()
    with contextlib.redirect_stdout(io.StringIO()):
        committed = old_adapter_run(options)
        baseline = run(**kwargs, shadow=ShadowOptions(**options))
        ideal = run(**kwargs, shadow=ShadowOptions(**options, gyro_model=config))
        biased = run(**kwargs, shadow=ShadowOptions(**options, gyro_model=profile_config("TEST_BIAS_ONLY")))
        # Existing closed-loop baseline is unchanged and remains unmodeled.
        old_control = old_adapter_run(dict(ideal_sun=True), control_source="MEKF_DEVELOPMENT")
        current_control = run(**kwargs, shadow=ShadowOptions(ideal_sun=True), control_source="MEKF_DEVELOPMENT")
        preserved = preservation()
    old_inputs = committed.attrs["shadow_input_trace"]
    inputs = ideal.attrs["shadow_input_trace"]
    changed = biased.attrs["shadow_input_trace"]
    truth = ideal[[f"omega_B_{axis}_rad_s" for axis in "xyz"]].to_numpy(dtype=float)
    replay = equivalence(ideal, ShadowOptions(**options, gyro_model=config))
    checks = {
        "committed_default_inputs_bit_exact": normalized(old_inputs) == normalized(baseline.attrs["shadow_input_trace"]),
        "ideal_mode_all_inputs_intervals_events_exact": normalized(inputs) == normalized(old_inputs),
        "ideal_measurements_match_recorded_live_truth": all(np.array_equal(batch.gyro_point_B, rate)
            and batch.gyro_sample is not None and np.array_equal(batch.gyro_sample.measurement_B_rad_s, rate)
            for batch, rate in zip(inputs, truth)),
        "ideal_acquisition_publication_processing_epochs_identical": all(b.gyro_sample is not None
            and b.gyro_sample.valid and b.gyro_sample.acquisition_ns == b.gyro_sample.publication_ns
            == b.gyro_epoch_ns == b.sim_epoch_ns and b.gyro_sample.configuration_fingerprint == config.fingerprint()
            for b in inputs),
        "all_ideal_estimator_states_covariance_bias_counts_exact": normalized(committed.attrs["shadow_estimator_trace"])
            == normalized(baseline.attrs["shadow_estimator_trace"]) == normalized(ideal.attrs["shadow_estimator_trace"]),
        "all_ideal_status_events_exact": committed.attrs["shadow_status"] == baseline.attrs["shadow_status"] == ideal.attrs["shadow_status"],
        "all_ideal_shadow_telemetry_exact": committed.attrs["shadow_telemetry"].equals(baseline.attrs["shadow_telemetry"])
            and committed.attrs["shadow_telemetry"].equals(ideal.attrs["shadow_telemetry"]),
        "delayed_vector_replay_unchanged_with_modeled_gyro": replay["passed"] and ideal.attrs["shadow_status"][-1]["replays"] > 0,
        "shadow_host_CSV_byte_identical": csv_bytes(committed) == csv_bytes(baseline) == csv_bytes(ideal) == csv_bytes(biased),
        "synthetic_bias_reaches_MEKF_once_in_B": all(np.array_equal(b.gyro_point_B, rate+np.array([.003, -.002, .001]))
            and b.gyro_sample is not None and b.gyro_sample.valid for b, rate in zip(changed, truth)),
        "no_development_owner_for_modeled_gyro": "mekf_command_owner" not in ideal.attrs and "mekf_command_owner" not in biased.attrs,
        "unmodeled_closed_loop_CSV_and_estimator_unchanged": csv_bytes(old_control) == csv_bytes(current_control)
            and normalized(old_control.attrs["shadow_estimator_trace"]) == normalized(current_control.attrs["shadow_estimator_trace"]),
    }
    checks = {k: bool(v) for k, v in checks.items()}
    sources = ("gyro_sensor_model.py", "attitude_mekf_adapter.py", "test_gyro_sensor_model.py",
        "test_gyro_shadow_integration.py", "validate_gyro_sensor_model.py", "attitude_mekf.py",
        "scenario_huskysat2_detumble.py", "config/attitude_mekf_test_only.json")
    report = dict(scope=SCOPE, passed=all(checks.values()) and preserved["passed"], checks=checks,
        base_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={n: hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        profiles={name: profile_config(name).to_dict() for name in PROFILES},
        duration_s=DURATION_S, live_samples=len(inputs), vector_replay=replay,
        host_sha256={name: hashlib.sha256(csv_bytes(frame)).hexdigest() for name, frame in
            (("committed", committed), ("current_default", baseline), ("ideal_model", ideal), ("bias_shadow", biased))},
        production_preservation=preserved, simulation_config=ideal.attrs["simulation_config"],
        gyro_samples=[asdict(b.gyro_sample) for b in inputs],
        estimator_trace=normalized(ideal.attrs["shadow_estimator_trace"]),
        gyro_timing_boundary="Live: existing cadence, zero latency. Delayed gyro is isolated-only; existing replay supports delayed vectors.")
    return report, ideal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase8a1_gyro_model.json")
    args = parser.parse_args()
    report, ideal = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    args.report.with_suffix(".csv").write_bytes(csv_bytes(ideal))
    print(json.dumps({key: report[key] for key in ("passed", "checks", "live_samples", "vector_replay", "host_sha256", "production_preservation")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
