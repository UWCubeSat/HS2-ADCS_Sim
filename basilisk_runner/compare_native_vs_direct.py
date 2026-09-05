"""Controlled native/direct runs; writes only generated validation artifacts.

Run native closed loop, the unchanged Phase 2B direct closed loop, and direct
actuation replaying EVERY native dipole command at its exact 0.1 s tick. The
latter isolates actuation from controller feedback. The direct bridge holds
body torque while native rotates held B_N at RK4 stages: finite-step trajectory
identity is NOT an acceptance criterion. Independent physics and step-refinement
tests in test_native_magnetic_actuation establish the common magnetic law.
This script reports the remaining hold-law difference without concealing it.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone

import Basilisk
import matplotlib
matplotlib.use("Agg")  # Batch validation writes plots; it has no GUI event loop.
import numpy as np
import pandas as pd

import compare_reference_vs_basilisk as comparison
from scenario_huskysat2_detumble import run, OUT_DATA, ROOT


def trajectory_difference(native, direct):
    if not np.array_equal(native.time_ns, direct.time_ns):
        raise ValueError("A/B state samples must have identical exact epochs")
    vectors = lambda df, p, u: comparison.vec3(df, comparison.vector_columns(p, u))
    rate = np.linalg.norm(vectors(native, "omega_B", "rad_s") - vectors(direct, "omega_B", "rad_s"), axis=1)
    # Use relative rotation, invariant to MRP shadow switching.
    def quaternions(df):
        sigma = df[[f"sigma_BN_{i}" for i in (1, 2, 3)]].to_numpy()
        s2 = np.sum(sigma*sigma, axis=1, keepdims=True)
        return np.column_stack(((1-s2)/(1+s2), 2*sigma/(1+s2)))
    a, b = quaternions(native), quaternions(direct)
    vector = a[:, :1]*b[:, 1:] - b[:, :1]*a[:, 1:] - np.cross(a[:, 1:], b[:, 1:])
    angle = np.degrees(2*np.arctan2(np.linalg.norm(vector, axis=1), np.abs(np.sum(a*b, axis=1))))
    return {
        "max_rate_vector_difference_rad_s": float(rate.max()),
        "final_rate_vector_difference_rad_s": float(rate[-1]),
        "max_attitude_difference_deg": float(angle.max()),
        "final_attitude_difference_deg": float(angle[-1]),
        "max_position_difference_m": float(np.linalg.norm(vectors(native, "r_N", "m") - vectors(direct, "r_N", "m"), axis=1).max()),
        "max_inertial_field_difference_T": float(np.linalg.norm(vectors(native, "B_N", "T") - vectors(direct, "B_N", "T"), axis=1).max()),
    }


def main(duration=None, make_plots=True):
    started_utc = datetime.now(timezone.utc).isoformat()
    native = run(duration, actuator="native", capture_commands=True, make_plots=make_plots)
    direct = run(duration, actuator="direct", make_plots=False)
    commands = native.attrs["command_history"]
    command_file = OUT_DATA / "detumble_native_commands.csv"
    pd.DataFrame(commands, columns=["time_ns", "mcmd_x_Am2", "mcmd_y_Am2", "mcmd_z_Am2"]).to_csv(command_file, index=False)
    replay = run(duration, actuator="direct", replay_commands=commands, capture_commands=True, make_plots=False)
    summaries = {name: comparison.summarize_basilisk(df) for name, df in
                 (("native", native), ("direct_closed_loop", direct), ("direct_native_command_replay", replay))}
    validations = {name: comparison.build_validation({"finite_numeric": True}, summary)
                   for name, summary in summaries.items()}
    for result in validations.values():
        # No standalone CSV is consumed in this A/B experiment.
        del result["checks"]["reference_numeric_finite"]
        result["passed"] = all(item["passed"] for item in result["checks"].values())
    identical_commands = bool(np.array_equal(commands, replay.attrs["command_history"]))
    differences = {"closed_loop": trajectory_difference(native, direct),
                   "identical_dipole_commands": trajectory_difference(native, replay)}
    initial_columns = [f"{p}_{a}_{u}" for p, u in (("r_N", "m"), ("v_N", "m_s"), ("omega_B", "rad_s")) for a in "xyz"]
    initial_columns += [f"sigma_BN_{i}" for i in (1, 2, 3)]
    identical_initial_states = all(np.array_equal(native[initial_columns].iloc[0], frame[initial_columns].iloc[0])
                                   for frame in (direct, replay))
    shared_environment = all(d["max_position_difference_m"] < 1e-6 and d["max_inertial_field_difference_T"] < 1e-15
                             for d in differences.values())
    passed = (identical_commands and identical_initial_states and shared_environment
              and all(v["passed"] for v in validations.values()))
    sources = [Path(__file__), Path(comparison.__file__)] + [Path(__file__).parent / name for name in
               ("scenario_huskysat2_detumble.py", "magnetic_actuation.py", "magnetic_environment.py", "basilisk_adcs_adapter.py")]
    report = {
        "passed": passed,
        "started_utc": started_utc,
        "scope": "Independent actuator physics, exact input replay and measured hold-law differences; NOT finite-step trajectory identity or HS-2 verification",
        "equivalence_basis": "test_native_magnetic_actuation: identical force-evaluation times using test-only Euler integration; direct-hold refinement toward native RK4. Production RK4 remains unchanged.",
        "basilisk_version": getattr(Basilisk, "__version__", "unknown"),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "command_file": str(command_file),
        "command_sha256": hashlib.sha256(command_file.read_bytes()).hexdigest(),
        "command_rows": len(commands),
        "output_sha256": {name: hashlib.sha256((OUT_DATA / name).read_bytes()).hexdigest()
                          for name in ("detumble_output.csv", "detumble_direct_output.csv", "detumble_direct_replay.csv")},
        "identical_initial_states": identical_initial_states,
        "identical_replayed_dipole_commands": identical_commands,
        "shared_orbit_and_inertial_field": shared_environment,
        "summaries": summaries, "validation": validations, "trajectory_differences": differences,
        "limitations": ["Native torque varies with body attitude inside each step; legacy direct torque is held in B.",
                        "Replay uses reference truth B_B for direct m x B; controller power/current diagnostics remain hypothetical closed-loop diagnostics in replay mode.",
                        "Standalone reference is not the A/B oracle. Limits, axes and spacecraft parameters remain provisional."],
    }
    output = OUT_DATA / "native_direct_comparison.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"passed": passed, "trajectory_differences": differences,
                      "validation_failed": {name: [key for key, value in result["checks"].items() if not value["passed"]]
                                            for name, result in validations.items()}}, indent=2))
    print(f"Wrote {output}")
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    try:
        result = main(args.duration, not args.no_plots)
    except (ValueError, OSError, RuntimeError) as exc:
        failure = {"passed": False, "error": str(exc), "failed_utc": datetime.now(timezone.utc).isoformat()}
        (OUT_DATA / "native_direct_comparison.json").write_text(json.dumps(failure, indent=2), encoding="utf-8")
        print(json.dumps(failure, indent=2))
        result = 2
    raise SystemExit(result)
