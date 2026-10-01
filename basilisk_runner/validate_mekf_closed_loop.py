"""Phase 7G-1 nominal six-second A/B, ASSUMED / TEST-ONLY, 2026-09-30.

Six complete existing 1 s cycles, 24 held actuation intervals, and the final
coil-off publication exercise startup and repeated sample/compute/use boundaries.
No injected faults, tuning, new sensor model or flight-performance acceptance.
All tolerances below are numerical development checks, not HS-2 requirements.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

import magnetic_control_cycle as cycle_module
from analyze_physical_profile_dynamics import rk_work
from attitude_mekf_adapter import ShadowOptions
from attitude_mekf_prototype import independent_error
from command_health_gate import CommandHealthGate
from compare_reference_vs_basilisk import build_validation, predict_magnetic_step, summarize_basilisk
from control_input_snapshot import capture_control_snapshot
from disconnected_command_observer import ObserverOptions
from disconnected_detumble_math import evaluate_snapshot
from hs2_sim_config import HS2SimConfig, get_profile_config
from magnetic_control_cycle import diagnostic_cycle_config
from mekf_command_owner import MEKF_DEVELOPMENT, SIMPLE_NAV_REFERENCE, SCOPE
from scenario_huskysat2_detumble import run

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DURATION_S = 6.0


def vector(frame, prefix, suffix):
    return frame[[f"{prefix}_{a}_{suffix}" for a in "xyz"]].to_numpy(dtype=float)


def mrp(frame, prefix="sigma_BN"):
    return frame[[f"{prefix}_{i}" for i in (1, 2, 3)]].to_numpy(dtype=float)


def csv_bytes(frame):
    return frame.to_csv(index=False).encode("utf-8")


def preservation():
    """Load BOTH changed runtime modules from HEAD, not old scenario/new driver."""
    def committed(name):
        return subprocess.check_output(["git", "show", f"HEAD:basilisk_runner/{name}.py"],
                                       cwd=ROOT, text=True, encoding="utf-8")
    name = "magnetic_control_cycle"
    old_cycle = ModuleType(name)
    old_cycle.__file__ = str(HERE/(name+".py"))
    previous = ModuleType("committed_scenario")
    previous.__file__ = str(HERE/"scenario_huskysat2_detumble.py")
    try:
        sys.modules[name] = old_cycle
        exec(compile(committed(name), old_cycle.__file__, "exec"), old_cycle.__dict__)
        exec(compile(committed("scenario_huskysat2_detumble"), previous.__file__, "exec"), previous.__dict__)
    finally:
        sys.modules[name] = cycle_module
    results = {}
    for label, profile, cycled in (("continuous", "regression_baseline", False),
                                   ("candidate", "hs2_candidate", False),
                                   ("cycled", "regression_baseline", True)):
        options = dict(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
                       config=get_profile_config(profile))
        old = previous.run(**options, cycle=old_cycle.diagnostic_cycle_config() if cycled else None)
        current = run(**options, cycle=diagnostic_cycle_config() if cycled else None)
        before, after = csv_bytes(old), csv_bytes(current)
        results[label] = {"bytes_identical": before == after,
            "committed_sha256": hashlib.sha256(before).hexdigest(),
            "working_sha256": hashlib.sha256(after).hexdigest(), "rows": len(current),
            "no_development_owner": "mekf_command_owner" not in current.attrs}
    return {"passed": all(r["bytes_identical"] and r["no_development_owner"] for r in results.values()),
            "cases": results}


def physics(frame):
    config = HS2SimConfig.from_dict(frame.attrs["simulation_config"])
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    dt = np.diff(ticks)*1e-9
    w, s = vector(frame, "omega_B", "rad_s"), mrp(frame)
    held_w, held_s = vector(frame, "held_omega_B", "rad_s")[1:], mrp(frame, "held_sigma_BN")[1:]
    held_m, held_b = vector(frame, "held_mcmd", "Am2")[1:], vector(frame, "held_B_N", "T")[1:]
    native = vector(frame, "native_mtbNetTorque_B", "Nm")[1:]
    m = vector(frame, "mcmd", "Am2")
    inertia = np.array(config.spacecraft.inertia.value)
    predicted, _, torque = predict_magnetic_step(held_s, held_w, held_m, held_b, dt, inertia)
    torque_free, _, _ = predict_magnetic_step(held_s, held_w, np.zeros_like(held_m), held_b, dt, inertia)
    work, work_torque, _, _, _ = rk_work(held_s, held_w, held_m, held_b, dt, inertia)
    energy = .5*np.sum(w*(w @ inertia.T), axis=1)
    zero = ~np.any(held_m, axis=1)
    limits = np.minimum(config.magnetorquers.dipole_limits.value,
        np.array(config.magnetorquers.current_limits.value)*np.array(config.magnetorquers.dipole_gains.value))
    stage_error = float(np.max(np.linalg.norm(native-torque, axis=1)))
    rate_error = float(np.max(np.linalg.norm(w[1:]-predicted[:, 3:], axis=1)))
    state_error = float(np.max(np.abs(np.column_stack((s[1:], w[1:]))-predicted)))
    work_error = float(np.max(np.abs(np.diff(energy)-work.sum(axis=1))))
    checks = build_validation({}, summarize_basilisk(frame))["checks"]
    del checks["reference_numeric_finite"]  # No standalone file is an input.
    result = {"existing_independent_telemetry_checks": all(c["passed"] for c in checks.values()),
        "finite_truth": bool(np.isfinite(frame.select_dtypes(include=np.number).to_numpy()).all()),
        "native_final_stage_torque": stage_error < 1e-15,
        "rigid_body_rate_step": rate_error < 1e-12,
        "coupled_attitude_state_step": state_error < 1e-12,
        "zero_held_command_zero_native_torque": bool(np.all(native[zero] == 0)),
        "command_bounds": bool(np.all(np.abs(m) <= limits+1e-15)),
        "nonzero_magnetic_state_response": bool(np.max(np.abs(w[1:]-torque_free[:, 3:])) > 1e-8),
        "next_interval_input_hold": bool(np.array_equal(held_m, m[:-1]) and np.array_equal(held_w, w[:-1])),
        "energy_work_balance": work_error < 1e-10,
        "work_stage_torque_witness": bool(np.max(np.abs(work_torque-native)) < 1e-15)}
    return {"passed": all(result.values()), "checks": result, "existing_checks": checks,
        "native_torque_error_Nm": stage_error, "rate_step_error_rad_s": rate_error,
        "state_step_max_error": state_error, "work_balance_max_error_J": work_error,
        "energy_change_J": float(energy[-1]-energy[0]), "integrated_magnetic_work_J": float(work.sum()),
        "maximum_interval_energy_increase_J": float(np.diff(energy).max()),
        "initial_rate_rad_s": float(np.linalg.norm(w[0])), "final_rate_rad_s": float(np.linalg.norm(w[-1])),
        "peak_native_torque_Nm": float(np.max(np.linalg.norm(native, axis=1))),
        "active_time_s": float(np.sum(dt[np.any(held_m, axis=1)])),
        "zero_command_intervals": int(zero.sum()),
        "first_command_s": float(frame.time_s.to_numpy()[np.flatnonzero(np.any(m, axis=1))[0]]),
        "first_native_torque_s": float(frame.time_s.to_numpy()[1:][np.flatnonzero(np.any(native, axis=1))[0]]),
        "completed_cycles": int(ticks[-1]//diagnostic_cycle_config().period_ns)}


def component_replay(frame):
    """Only call verified 7F components; do not duplicate any gate/controller math."""
    gate = CommandHealthGate()
    config = HS2SimConfig.from_dict(frame.attrs["simulation_config"])
    comparisons = []
    for row in frame.attrs["disconnected_command_trace"]:
        tick = row["tick"]
        comparisons.append(gate.observe(row["nav"], row["decision"], tick) == row["health"])
        if row["capture_result"] is not None:
            capture = capture_control_snapshot(row["capture_nav"], row["decision"], row["sample"], row["plan"], tick,
                navigation_provenance="Live MEKF NavAtt + Phase 7E quality/consumer; 7F-2C 2026-09-30")
            comparisons.append(capture == row["capture_result"])
        for key in ("compute", "application"):
            if row[key] is None:
                continue
            pending, nav, decision, actual = row[key]
            if key == "compute" and pending is not None:
                comparisons.append(evaluate_snapshot(pending.snapshot, tick, config) == pending.command)
            expected = gate.evaluate(pending.snapshot if pending else None, pending.command if pending else None,
                nav, decision, tick, **({"application_cycle": diagnostic_cycle_config()} if key == "application" else {}))
            comparisons.append(expected == actual)
    return {"passed": all(comparisons), "exact_comparisons": len(comparisons)}


def integration(frame):
    owners = frame.attrs["mekf_command_owner"]
    records = frame.attrs["disconnected_command_records"]
    rows = records["telemetry"]
    m = vector(frame, "mcmd", "Am2")
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    cores = [r["core"] for r in frame.attrs["shadow_estimator_trace"] if r["core"] is not None]
    checks = {"complete_tick_evidence": len(owners) == len(rows) == len(frame),
        "current_single_owner": len({o["publisher_module_id"] for o in owners}) == 1 and all(
        o["publisher_module_id"] == o["subscriber_module_id"] and o["subscriber_epoch_ns"] == t
        and o["publication_epoch_ns"] == o["gate_epoch_ns"] == t and not o["unused_cycle_message_written"]
        for t, o in zip(ticks, owners)),
        "native_input_equals_gated_output": all(np.array_equal(o["native_input_dipole_Am2"], cmd)
            and np.array_equal(cmd, row["clipped_dipole_B_Am2"] if row["command_usable"] else np.zeros(3))
            for o, row, cmd in zip(owners, rows, m)),
        "zero_unusable_and_forbidden_phase": all(not np.any(cmd) for row, cmd, t in zip(rows, m, ticks)
            if not row["command_usable"] or t % 1_000_000_000 < 600_000_000),
        "every_nominal_burst_usable": all(row["command_usable"] for row, t in zip(rows, ticks)
                                         if t % 1_000_000_000 >= 600_000_000),
        "provenance": all(row["sample_epoch_ns"] == row["tam_acquisition_epoch_ns"] == row["snapshot_capture_ns"]
            == row["cycle_index"]*1_000_000_000+400_000_000
            and row["command_computation_ns"] == row["sample_epoch_ns"]+100_000_000
            and row["generation_match"] and row["cycle_match"] and row["source_healthy"]
            and o["command_id"] == row["command_id"] and o["generation"] == row["command_generation"]
            for row, o in zip(rows, owners) if row["command_usable"]),
        "actual_order": all([e[1] for e in records["execution_order"] if e[0] == t]+[o["priority"]]
            == ([595, 587, 554, 552, 550, 548] if t % 1_000_000_000 == 500_000_000 else [595, 587, 554, 550, 548])
            for t, o in zip(ticks, owners)),
        "finite_covariance_and_quaternion": bool(cores) and all(np.isfinite(c["P"]).all()
            and np.isfinite(c["q_BN"]).all() and abs(np.linalg.norm(c["q_BN"])-1) < 1e-12 for c in cores),
        "covariance_positive_semidefinite": bool(cores) and all(np.linalg.eigvalsh(c["P"]).min() >= -1e-14 for c in cores),
        "nominal_no_faults_or_resets": all(not r["fault"] and r["acquisitions"] <= 1 for r in frame.attrs["shadow_status"])
            and all(not r["reset"] for r in frame.attrs["disconnected_command_trace"])}
    replay = component_replay(frame)
    checks["unchanged_components_exact"] = replay["passed"]
    first = next(o for o in owners if np.any(o["published_dipole_Am2"]))
    statuses = frame.attrs["shadow_status"]
    return {"passed": all(checks.values()), "checks": checks, "component_replay": replay,
            "first_command": first, "first_valid_mekf_ns": next(r["sim_epoch_ns"] for r in statuses if r["valid"]),
            "valid_mekf_ticks": sum(r["valid"] for r in statuses), "owner_trace": owners, "chain": records}


def validate():
    options = dict(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
                   cycle=diagnostic_cycle_config(), shadow=ShadowOptions(ideal_sun=True))
    with contextlib.redirect_stdout(io.StringIO()):
        a = run(**options, control_source=SIMPLE_NAV_REFERENCE, disconnected_commands=ObserverOptions())
        b = run(**options, control_source=MEKF_DEVELOPMENT)
        preserved = preservation()
    checks_a, checks_b, chain = physics(a), physics(b), integration(b)
    w_a, w_b = vector(a, "omega_B", "rad_s"), vector(b, "omega_B", "rad_s")
    s_a, s_b = mrp(a), mrp(b)
    angles = [independent_error(rbk.MRP2EP(sb), sa)[0] for sa, sb in zip(s_a, s_b)]
    initial = np.array_equal(w_a[0], w_b[0]) and np.array_equal(s_a[0], s_b[0])
    # Isolate input-estimator differences from accumulated plant divergence: run
    # the SAME dispatcher with each body's own SimpleNav sample rate and TAM.
    # This diagnoses the command delta; it is not the torque/plant validation.
    from basilisk_adcs_adapter import ADCSConfig, controller_step
    cfg = ADCSConfig.from_sim_config(HS2SimConfig.from_dict(b.attrs["simulation_config"]))
    input_deltas = []
    command_deltas = []
    bias_residuals = []
    for command in b.attrs["disconnected_command_records"]["commands"]:
        snapshot, math = command["snapshot"], command["command"]
        i = int(snapshot["navigation_state_ns"]//100_000_000)
        result = controller_step(math["evaluation_epoch_ns"]*1e-9, w_b[i], snapshot["tam_B_T"], cfg)
        input_deltas.append(np.linalg.norm(np.array(snapshot["omega_BN_B_rad_s"])-w_b[i]))
        bias = b.attrs["shadow_estimator_trace"][i]["core"]["bias_B_rad_s"]
        bias_residuals.append(np.linalg.norm(np.array(snapshot["omega_BN_B_rad_s"])-w_b[i]+bias))
        command_deltas.append(np.linalg.norm(np.array(math["clipped_dipole_B_Am2"])-result["commanded_magnetic_dipole_B_Am2"]))
    comparison = {"identical_configuration": a.attrs["simulation_config"] == b.attrs["simulation_config"],
        "identical_cycle_and_sensor_options": a.attrs["magnetic_cycle"] == b.attrs["magnetic_cycle"]
            and a.attrs["shadow_options"] == b.attrs["shadow_options"],
        "identical_initial_conditions": initial, "identical_translation": np.array_equal(vector(a, "r_N", "m"), vector(b, "r_N", "m")),
        "pre_first_application_identical": np.array_equal(w_a[:7], w_b[:7]) and np.array_equal(s_a[:7], s_b[:7]),
        "max_rate_vector_difference_rad_s": float(np.max(np.linalg.norm(w_a-w_b, axis=1))),
        "final_rate_magnitude_difference_rad_s": float(np.linalg.norm(w_b[-1])-np.linalg.norm(w_a[-1])),
        "max_attitude_difference_rad": float(max(angles)),
        "max_requested_dipole_difference_Am2": float(np.max(np.linalg.norm(
            a[[f"cycle_requested_dipole_{x}" for x in "xyz"]].to_numpy(dtype=float)
            -b[[f"cycle_requested_dipole_{x}" for x in "xyz"]].to_numpy(dtype=float), axis=1))),
        "max_clipped_dipole_difference_Am2": float(np.max(np.linalg.norm(vector(a, "mcmd", "Am2")-vector(b, "mcmd", "Am2"), axis=1))),
        "max_native_torque_difference_Nm": float(np.max(np.linalg.norm(vector(a, "native_mtbNetTorque_B", "Nm")
            -vector(b, "native_mtbNetTorque_B", "Nm"), axis=1))),
        "max_same_plant_mekf_rate_input_difference_rad_s": float(max(input_deltas)),
        "max_same_plant_command_difference_Am2": float(max(command_deltas)),
        "max_rate_difference_plus_estimated_bias_rad_s": float(max(bias_residuals)),
        "explanation": "Same measured TAM and unchanged rate-cross-field law. MEKF point rate subtracts estimated bias "
            "from the explicit ideal gyro; causal gyro reconstruction and estimator numerical residuals seed small "
            "command differences, followed by plant feedback. Attitude is not an input to this detumble law. "
            "Ideal initial acquisition explains the almost identical first command; equality is not forced."}
    matched = all(comparison[k] for k in ("identical_configuration", "identical_cycle_and_sensor_options",
        "identical_initial_conditions", "identical_translation", "pre_first_application_identical"))
    matched = matched and comparison["max_rate_difference_plus_estimated_bias_rad_s"] < 1e-14
    names = ("mekf_command_owner.py", "magnetic_control_cycle.py", "scenario_huskysat2_detumble.py",
             "validate_mekf_closed_loop.py", "test_mekf_closed_loop.py")
    report = {"scope": SCOPE, "duration_s": DURATION_S,
        "passed": matched and checks_a["passed"] and checks_b["passed"] and chain["passed"]
            and preserved["passed"],
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": {n: hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in names if (HERE/n).exists()},
        "configuration": b.attrs["simulation_config"], "cycle": b.attrs["magnetic_cycle"], "sensors": b.attrs["shadow_options"],
        "A_simple_nav": checks_a, "B_mekf": checks_b, "integration": chain, "comparison": comparison,
        "preservation": preserved, "output_sha256": {"A": hashlib.sha256(csv_bytes(a)).hexdigest(),
                                                      "B": hashlib.sha256(csv_bytes(b)).hexdigest()}}
    return report, a, b


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase7g1_nominal.json")
    args = parser.parse_args()
    report, a, b = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    for label, frame in (("A_simple_nav", a), ("B_mekf", b)):
        args.report.with_name(args.report.stem+"_"+label+".csv").write_bytes(csv_bytes(frame))
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"],
        "A": {k: v for k, v in report["A_simple_nav"].items() if k != "existing_checks"},
        "B": {k: v for k, v in report["B_mekf"].items() if k != "existing_checks"},
        "integration": report["integration"]["checks"], "comparison": report["comparison"],
        "preservation": report["preservation"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
