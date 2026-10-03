"""8C-1 short live ideal Sun equivalence and production/authority preservation.

Only three-second wiring fixtures; no detumble-performance study. Load committed
scenario AND adapter in memory for comparison, never edit/checkout runtime files.
Report is separate from production CSVs; the scenario runs with write_outputs=False.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict, is_dataclass, replace
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
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from sun_sensor_model import SCOPE, SunAvailability, SunConfig, SunModel, profile_config
from validate_mekf_closed_loop import csv_bytes

HERE, ROOT = Path(__file__).resolve().parent, Path(__file__).resolve().parents[1]
DURATION_S = 3.0  # Existing sensor wiring horizon; ASSUMED TEST-ONLY.
OPTIONS = dict(ideal_sun=True, magnetic_delay_ns=200_000_000, sun_delay_ns=200_000_000)


def normalized(value, *, omit_sun_source=False):
    # Canonicalize only the two documented producer labels, including the core's
    # retained acquisition-source list. Unknown provenance is NEVER stripped.
    if isinstance(value, str) and omit_sun_source and value == "SunModel IDEAL_REGRESSION development direct-vector mode; not physical CSS reconstruction":
        return "TEST-ONLY ideal Sun direction from current truth; no CSS model"
    if is_dataclass(value) and not isinstance(value, type):
        value = asdict(value)
    if isinstance(value, dict):
        return {k: normalized(v, omit_sun_source=omit_sun_source) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [normalized(v, omit_sun_source=omit_sun_source) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def committed_run(options=None, *, cycled=True):
    """Exact HEAD runtime source, including lazy scenario adapter imports."""
    old_adapter = ModuleType("attitude_mekf_adapter")
    old_scenario = ModuleType("phase8c1_committed_scenario")
    old_adapter.__file__ = str(HERE / "attitude_mekf_adapter.py")
    old_scenario.__file__ = str(HERE / "scenario_huskysat2_detumble.py")
    try:
        sys.modules[old_adapter.__name__] = old_adapter
        sys.modules[old_scenario.__name__] = old_scenario
        for name, module in (("attitude_mekf_adapter", old_adapter), ("scenario_huskysat2_detumble", old_scenario)):
            source = subprocess.check_output(["git", "show", f"HEAD:basilisk_runner/{name}.py"], cwd=ROOT, text=True, encoding="utf-8")
            exec(compile(source, module.__file__, "exec"), module.__dict__)
        return old_scenario.run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config() if cycled else None,
            shadow=old_adapter.ShadowOptions(**options) if options is not None else None)
    finally:
        sys.modules[old_adapter.__name__] = current_adapter
        sys.modules.pop(old_scenario.__name__, None)


def run_case(modeled=False, rejected=False):
    options = dict(OPTIONS, drop_sun_after_ns=1_500_000_000 if rejected else None)
    return run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
               cycle=diagnostic_cycle_config(), control_source="SIMPLE_NAV_REFERENCE",
               shadow=ShadowOptions(**options, sun_model=SunConfig() if modeled else None))


def events(frame):
    return [(b.sim_epoch_ns, e) for b in frame.attrs["shadow_input_trace"] for e in b.deliveries if e.sensor == "sun"]


def compare(a, b):
    left, right = events(a), events(b)
    same_count = len(left) == len(right) > 0
    vector_error = max((float(np.max(np.abs(np.array(x.measured)-y.measured)))
                        for (_, x), (_, y) in zip(left, right)), default=float("inf"))
    maxima = dict(q_BN=0., bias_B_rad_s=0., P=0.)
    for x, y in zip(a.attrs["shadow_estimator_trace"], b.attrs["shadow_estimator_trace"]):
        if x["core"] is not None and y["core"] is not None:
            for key in maxima:
                maxima[key] = max(maxima[key], float(np.max(np.abs(np.array(x["core"][key])-y["core"][key]))))
    checks = dict(
        same_nonempty_sun_count=same_count,
        vectors_bit_exact=same_count and vector_error == 0.,
        inputs_epochs_validity_order_exact=normalized(a.attrs["shadow_input_trace"], omit_sun_source=True)
            == normalized(b.attrs["shadow_input_trace"], omit_sun_source=True),
        estimator_attitude_bias_covariance_trace_exact=normalized(a.attrs["shadow_estimator_trace"], omit_sun_source=True)
            == normalized(b.attrs["shadow_estimator_trace"], omit_sun_source=True),
        status_acquisition_accept_reject_sequence_exact=normalized(a.attrs["shadow_status"], omit_sun_source=True)
            == normalized(b.attrs["shadow_status"], omit_sun_source=True),
        nav_telemetry_exact=a.attrs["shadow_telemetry"].equals(b.attrs["shadow_telemetry"]),
        spacecraft_actuator_CSV_byte_identical=csv_bytes(a) == csv_bytes(b))
    return dict(passed=all(checks.values()), checks=checks, max_vector_difference=vector_error,
                estimator_max_abs_differences=maxima, sun_events=len(right))


def synthetic_geometry():
    config = profile_config("TEST_ARRAY_GEOMETRY")
    rows = []
    directions = [*(sign*axis for sign in (-1, 1) for axis in np.eye(3)), np.array([.2, -.3, .7]), np.array([.4, .8, -.1])]
    for direction in directions:
        sample = SunModel(config).acquire(direction, np.eye(3), truth_epoch_ns=0, acquisition_ns=0,
            availability=SunAvailability(0, True, "TEST-ONLY all channels unobscured"), source="synthetic independent known direction")
        result = sample.reconstruction
        error = float(np.arctan2(np.linalg.norm(np.cross(result.direction_B, direction)), np.dot(result.direction_B, direction)))
        rows.append(dict(direction=direction, error_rad=error, reconstruction=asdict(result)))
    return dict(scope="synthetic mathematics only; NOT HS-2 accuracy", rows=normalized(rows),
                max_direction_error_rad=max(row["error_rad"] for row in rows))


def validate():
    with contextlib.redirect_stdout(io.StringIO()):
        committed = committed_run(OPTIONS)
        baseline, modeled = run_case(), run_case(True)
        rejected_baseline, rejected_model = run_case(rejected=True), run_case(True, True)
        preservation = {}
        for label, cycled in (("continuous_native", False), ("diagnostic_cycle_native", True)):
            before = committed_run(cycled=cycled)
            after = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
                        cycle=diagnostic_cycle_config() if cycled else None)
            preservation[label] = dict(bytes_identical=csv_bytes(before) == csv_bytes(after), rows=len(after),
                committed_sha256=hashlib.sha256(csv_bytes(before)).hexdigest(),
                current_sha256=hashlib.sha256(csv_bytes(after)).hexdigest(),
                no_shadow="shadow_input_trace" not in after.attrs)
    nominal, rejected_case, disabled = compare(baseline, modeled), compare(rejected_baseline, rejected_model), compare(committed, baseline)
    records = modeled.attrs["sun_model_samples"]
    actual_events = events(modeled)
    authority_guards, live_guards = [], []
    array = profile_config("TEST_ARRAY_GEOMETRY")
    configs = (SunConfig(), array, replace(array, profile="IDEAL_REGRESSION"),
               replace(SunConfig(), latency_ns=replace(SunConfig().latency_ns, value=100_000_000)))
    for config in configs:
        if config != SunConfig():
            try:
                ShadowOptions(ideal_sun=True, sun_model=config).validate(100_000_000)
                live_guards.append(False)
            except ValueError as error:
                live_guards.append("live Sun permits only" in str(error))
        try:
            run(stop_time_s=1., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
                shadow=ShadowOptions(ideal_sun=True, sun_model=config), control_source="MEKF_DEVELOPMENT")
            authority_guards.append(False)
        except ValueError as error:
            authority_guards.append("MEKF_DEVELOPMENT requires" in str(error))
    geometry = synthetic_geometry()
    checks = dict(
        live_ideal_equivalence=nominal["passed"], rejected_event_equivalence=rejected_case["passed"],
        disabled_HEAD_equivalence=disabled["passed"],
        disabled_sources_and_metadata_unchanged=normalized(committed.attrs["shadow_input_trace"])
            == normalized(baseline.attrs["shadow_input_trace"]),
        production_preserved=all(v["bytes_identical"] and v["no_shadow"] for v in preservation.values()),
        cold_start_acquisition_unchanged=modeled.attrs["shadow_status"][4]["acquisition_event"]
            and modeled.attrs["shadow_status"][-1]["acquisitions"] == 1,
        epochs_and_provenance_preserved=len(records) == len(actual_events) == 4 and all(
            row["state_epoch_ns"] == row["sample"]["truth_epoch_ns"] == row["sample"]["acquisition_ns"]
            == row["sample"]["publication_ns"] == row["sample"]["reconstruction_ns"] == e.epoch_ns == e.reference_epoch_ns
            and row["bridge_delivery_ns"] == t and row["sample"]["configuration_fingerprint"] == SunConfig().fingerprint()
            for row, (t, e) in zip(records, actual_events)),
        delayed_processing_not_reacquisition=actual_events[1][0]-actual_events[1][1].epoch_ns == 200_000_000
            and modeled.attrs["shadow_status"][-1]["replays"] > 0,
        source_identifies_direct_mode=all("SunModel IDEAL_REGRESSION" in e.source for _, e in actual_events),
        no_physical_array_claim=all(not r["sample"]["channels"] and r["sample"]["reconstruction"]["method"] == "development_direct_vector" for r in records),
        rejected_delivery_keeps_finite_sample=any(not e.valid and np.isfinite(e.measured).all() for _, e in events(rejected_model)),
        all_model_control_selection_rejected=all(authority_guards),
        all_nonideal_live_selection_rejected=all(live_guards),
        no_development_actuator_owner=all("mekf_command_owner" not in f.attrs for f in (baseline, modeled, rejected_model)),
        shadow_rejection_does_not_change_actuation=csv_bytes(baseline) == csv_bytes(modeled) == csv_bytes(rejected_baseline) == csv_bytes(rejected_model),
        synthetic_known_direction_recovery=geometry["max_direction_error_rad"] < 1e-14,
        core_policy_dynamics_TAM_gyro_unchanged=subprocess.run(["git", "diff", "--quiet", "HEAD", "--",
            *["basilisk_runner/"+n for n in ("attitude_mekf.py", "attitude_mekf_prototype.py", "config/attitude_mekf_test_only.json",
                "hs2_sim_config.py", "basilisk_adcs_adapter.py", "magnetic_control_cycle.py", "magnetic_environment.py",
                "tam_sensor_model.py", "gyro_sensor_model.py")]], cwd=ROOT).returncode == 0)
    sources = ("sun_sensor_model.py", "test_sun_sensor_model.py", "validate_sun_sensor_model.py", "test_sun_shadow_integration.py",
        "attitude_mekf_adapter.py", "scenario_huskysat2_detumble.py", "attitude_mekf.py", "attitude_mekf_prototype.py",
        "config/attitude_mekf_test_only.json", "hs2_sim_config.py", "magnetic_control_cycle.py", "tam_sensor_model.py", "gyro_sensor_model.py")
    report = dict(scope=SCOPE, passed=bool(all(checks.values())), checks={k:bool(v) for k,v in checks.items()},
        base_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in sources},
        profiles={name:profile_config(name).to_dict() for name in ("IDEAL_REGRESSION", "TEST_ARRAY_GEOMETRY")},
        duration_s=DURATION_S, options=asdict(ShadowOptions(**OPTIONS, sun_model=SunConfig())),
        simulation_config=modeled.attrs["simulation_config"], cycle_config=diagnostic_cycle_config().to_dict(),
        nominal=nominal, rejected=rejected_case, disabled=disabled, preservation=preservation,
        synthetic_geometry=geometry, model_samples=records,
        ideal_inputs=normalized(modeled.attrs["shadow_input_trace"]),
        ideal_estimator_trace=normalized(modeled.attrs["shadow_estimator_trace"]),
        ideal_status=normalized(modeled.attrs["shadow_status"]),
        host_sha256=hashlib.sha256(csv_bytes(modeled)).hexdigest())
    return report, baseline, modeled


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase8c1_sun_model.json")
    args = parser.parse_args()
    report, _, _ = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(normalized(report), indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(normalized({key:report[key] for key in ("passed", "checks", "nominal", "rejected", "preservation", "host_sha256")}), indent=2))
    print("Synthetic maximum direction error, rad:", report["synthetic_geometry"]["max_direction_error_rad"])
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
