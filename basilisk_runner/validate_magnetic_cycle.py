"""Independent Phase 6A timing, actuator and energy checks on complete tick records.

Phase expectations are reconstructed from duration inputs, not driver phase()
or its validity flags. Native torque and propagated states retain their source.
Electrical estimates use the existing provisional resistance/current model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from hs2_sim_config import HS2SimConfig
from magnetic_control_cycle import MagneticCycleConfig

HERE = Path(__file__).resolve().parent


def cycle_checks(df: pd.DataFrame, config: HS2SimConfig):
    from compare_reference_vs_basilisk import rotate_inertial_to_body
    cycle = MagneticCycleConfig.from_dict(df.attrs["magnetic_cycle"])
    h = round(config.timing.dynamics_step.value*1e9)
    cycle.validate(h)
    d = cycle.durations_ns()
    # Separate construction of the event grid; never call the driver's planner.
    sample_offset = d["quiet"] + d["settling"]
    compute_offset = sample_offset + d["sample_to_compute"]
    act_offset = compute_offset + d["compute_to_actuate"]
    period = act_offset + d["actuation"]
    ticks = df.time_ns.to_numpy(dtype=np.int64)
    start = ticks // period * period
    offset = ticks - start
    sample = offset == sample_offset
    compute = offset == compute_offset
    actuate = offset >= act_offset
    valid = offset >= sample_offset
    checks = {}

    def check(name, condition):
        checks["cycle_"+name] = {"passed": bool(np.all(condition))}

    def vector(prefix, suffix=""):
        return df[[f"{prefix}_{a}{suffix}" for a in "xyz"]].to_numpy(dtype=float)

    for col in (c for c in df if c.startswith("cycle_") and c.endswith("_ns")):
        values = df[col].to_numpy(dtype=float)
        if not np.isfinite(values).all() or not np.equal(values, np.rint(values)).all():
            raise ValueError(f"Cycle timestamp must be integer nanoseconds: {col}")
    if len(ticks) < 2 or ticks[0] != 0 or not np.all(np.diff(ticks) == h):
        raise ValueError("Cycle validation requires every exact plant tick, beginning at zero")
    check("configuration", (df.cycle_config_sha256 == cycle.fingerprint()).all()
          and (df.cycle_period_ns == period).all() and (df.command_mode == "cycled").all())
    check("truth_field_label", (df.B_B_source == "current_truth_C_BN_times_WMM_B_N; not a sensor sample").all())
    expected_phase = np.select([offset < h, offset < d["quiet"], offset < sample_offset,
                               offset < compute_offset, offset < act_offset],
                              ["COIL_OFF", "QUIET", "SETTLING", "SAMPLE", "COMPUTE"], default="ACTUATE")
    lo = np.select([offset < h, offset < d["quiet"], offset < sample_offset,
                    offset < compute_offset, offset < act_offset],
                   [0, h, d["quiet"], sample_offset, compute_offset], default=act_offset)
    hi = np.select([offset < h, offset < d["quiet"], offset < sample_offset,
                    offset < compute_offset, offset < act_offset],
                   [h, d["quiet"], sample_offset, compute_offset, act_offset], default=period)
    check("phase_progression", df.cycle_phase.to_numpy() == expected_phase)
    check("phase_epochs", (df.cycle_start_ns.to_numpy() == start)
          & (df.cycle_index.to_numpy() == ticks//period)
          & (df.cycle_phase_start_ns.to_numpy() == start+lo)
          & (df.cycle_phase_end_ns.to_numpy() == start+hi)
          & (df.cycle_actuation_start_ns.to_numpy() == start+act_offset)
          & (df.cycle_actuation_end_ns.to_numpy() == start+period))
    check("sample_events", df.cycle_sample_event.to_numpy() == sample)
    check("sample_validity", df.cycle_sample_valid.to_numpy() == valid)
    check("compute_events", df.cycle_controller_consumed_sample.to_numpy() == compute)
    sample_epochs = np.maximum.accumulate(np.where(sample, ticks, -1))
    compute_epochs = np.maximum.accumulate(np.where(compute, ticks, -1))
    check("sample_epochs", (df.cycle_sample_epoch_ns.to_numpy() == sample_epochs)
          & (df.tam_message_time_ns.to_numpy() == np.maximum(sample_epochs, 0)))
    check("compute_epochs", df.cycle_compute_epoch_ns.to_numpy() == compute_epochs)
    check("sample_counts", (df.cycle_valid_sample_count.to_numpy() == np.cumsum(sample))
          & (df.cycle_rejected_sample_count.to_numpy() == 0))
    m, held = vector("mcmd", "_Am2"), vector("held_mcmd", "_Am2")
    pre, effective = vector("cycle_pre_native_command"), vector("cycle_native_input_effective_dipole")
    limits = np.asarray(config.magnetorquers.dipole_limits.value)
    axes = np.asarray(config.magnetorquers.axes_B.value)
    check("actual_subscriber_transport", np.allclose(pre, held, rtol=0, atol=1e-15)
          and np.allclose(vector("cycle_pre_native_effective_dipole"), np.clip(held, -limits, limits) @ axes.T, rtol=0, atol=1e-15)
          and np.allclose(effective, np.clip(m, -limits, limits) @ axes.T, rtol=0, atol=1e-15)
          and np.array_equal(held[1:], m[:-1]))
    on = np.any(m != 0, axis=1)
    disabled = np.maximum.accumulate(np.where(~on & np.r_[False, on[:-1]], ticks, 0))
    age = np.where(on, -1, ticks-disabled)
    check("disabled_epochs", (df.cycle_time_since_disabled_ns.to_numpy() == age)
          & (df.cycle_disabled_since_ns.to_numpy() == np.where(on, -1, disabled)))
    check("sample_coils_quiet", not np.any(m[sample]) and not np.any(pre[sample])
          and not np.any(vector("cycle_pre_native_effective_dipole")[sample])
          and np.all(age[sample] >= sample_offset))
    check("off_command_zero", not np.any(m[~actuate]))
    off_intervals = (ticks > 0) & ~np.any(held != 0, axis=1)
    native_torque = vector("applied_torque_B", "_Nm")
    check("native_off_torque_zero", not np.any(native_torque[off_intervals]))
    sample_b, sample_bn = vector("cycle_sample_B_B"), vector("cycle_sample_B_N")
    check("recorded_TAM_sample", np.array_equal(sample_b, vector("tam_sample_B_B", "_T")))
    sample_sigma, sample_w = vector("cycle_sample_sigma_BN"), vector("cycle_sample_omega_B")
    truth_sigma = df[[f"sigma_BN_{j}" for j in (1, 2, 3)]].to_numpy(dtype=float)
    check("sample_state_and_field", np.allclose(sample_bn[sample], vector("B_N", "_T")[sample], rtol=0, atol=1e-18)
          and np.allclose(sample_w[sample], vector("omega_B", "_rad_s")[sample], rtol=0, atol=1e-14)
          and np.allclose(sample_sigma[sample], truth_sigma[sample], rtol=0, atol=1e-14)
          and np.allclose(sample_b[sample], rotate_inertial_to_body(truth_sigma[sample], sample_bn[sample]), rtol=0, atol=1e-18))
    for prefix in ("cycle_sample_B_B", "cycle_sample_B_N", "cycle_sample_sigma_BN", "cycle_sample_omega_B"):
        values = vector(prefix)
        check(prefix+"_held", np.array_equal(values[1:][~sample[1:]], values[:-1][~sample[1:]]))
    expected_request = config.controller.dipole_command_gain.value*np.cross(sample_w, sample_b)
    expected_request[offset < compute_offset] = 0
    gains = np.asarray(config.magnetorquers.dipole_gains.value)
    max_current = np.minimum(config.magnetorquers.current_limits.value, limits/gains)
    expected_clip = np.clip(expected_request/gains, -max_current, max_current)*gains
    check("request_and_clipping", np.allclose(vector("cycle_requested_dipole"), expected_request, rtol=0, atol=1e-14)
          and np.allclose(vector("cycle_clipped_dipole"), expected_clip, rtol=0, atol=1e-14))
    expected_clip[~actuate] = 0
    check("command_epoch_and_hold", np.allclose(m, expected_clip, rtol=0, atol=1e-14))
    check("electrical_commands", np.allclose(vector("cycle_electrical_normalized"), m/(gains*max_current), rtol=0, atol=1e-14)
          and np.allclose(df[[f"i{a}_A" for a in "xyz"]].to_numpy(dtype=float), m/gains, rtol=0, atol=1e-14)
          and np.allclose(df.pcoil_total_W, np.sum((m/gains)**2*np.asarray(config.magnetorquers.resistance.value), axis=1), rtol=0, atol=1e-14))
    return checks


def load_run(path):
    frame = pd.read_csv(path)
    config = HS2SimConfig.load(path.with_name(path.stem+"_config.json"))
    frame.attrs["simulation_config"] = config.to_dict()
    if int(frame.telemetry_schema_version.iloc[0]) == 5:
        frame.attrs["magnetic_cycle"] = MagneticCycleConfig.load(path.with_name(path.stem+"_cycle.json")).to_dict()
    return frame, config


def analyze(frame, config, continuous):
    from compare_reference_vs_basilisk import summarize_basilisk, build_validation
    from analyze_physical_profile_dynamics import rk_work
    from compare_physical_profiles import threshold_observation
    metrics = summarize_basilisk(frame)
    validation = build_validation({}, metrics)
    del validation["checks"]["reference_numeric_finite"]
    validation["passed"] = all(c["passed"] for c in validation["checks"].values())
    def vec(prefix, suffix):
        return frame[[f"{prefix}_{a}_{suffix}" for a in "xyz"]].to_numpy(dtype=float)
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    dt = np.diff(ticks)*1e-9
    sigma = frame[[f"held_sigma_BN_{i}" for i in (1, 2, 3)]].to_numpy(dtype=float)[1:]
    work_axes, tau, stage_w, _, _ = rk_work(sigma, vec("held_omega_B", "rad_s")[1:],
        vec("held_mcmd", "Am2")[1:], vec("held_B_N", "T")[1:], dt, config.spacecraft.inertia.value)
    work = work_axes.sum(axis=1)
    omega = vec("omega_B", "rad_s")
    energy = 0.5*np.sum((omega @ np.asarray(config.spacecraft.inertia.value).T)*omega, axis=1)
    residual = float(np.abs(energy-energy[0]-np.r_[0., np.cumsum(work)]).max())
    native = vec("applied_torque_B", "Nm")[1:]
    native_error = float(np.abs(native-tau).max())
    m = vec("held_mcmd", "Am2")[1:]
    active = np.any(m != 0, axis=1)
    active_time, total_time = float(dt[active].sum()), float(dt.sum())
    validation["checks"]["cycle_energy_work_balance"] = {"passed": bool(residual < 1e-8*energy[0]), "max_residual_J": residual}
    validation["checks"]["cycle_work_native_torque"] = {"passed": native_error < 1e-15, "max_error_Nm": native_error}
    validation["passed"] = all(c["passed"] for c in validation["checks"].values())
    rate = np.linalg.norm(omega, axis=1)
    t = ticks*1e-9
    powers = frame.pcoil_total_W.to_numpy(dtype=float)[:-1]
    common_tick = min(int(ticks[-1]), int(continuous.time_ns.iloc[-1]))
    matches = np.flatnonzero(ticks == common_tick)
    if len(matches) != 1 or not (continuous.time_ns == common_tick).any():
        raise ValueError("Detumble comparison requires an exact shared final epoch")
    report = {"passed": validation["passed"], "validation": validation,
        "cycle_configuration": frame.attrs["magnetic_cycle"], "physical_configuration": config.to_dict(),
        "valid_samples": int(frame.cycle_valid_sample_count.iloc[-1]),
        "rejected_samples": int(frame.cycle_rejected_sample_count.iloc[-1]),
        "recorded_duration_s": total_time, "actuation_time_s": active_time,
        "actuation_time_fraction": active_time/total_time,
        "electrical_energy_estimate_J": float(powers @ dt), "mean_electrical_power_estimate_W": float(powers @ dt/total_time),
        "peak_electrical_power_estimate_W": float(powers.max()),
        "mean_dipole_norm_Am2": float(np.average(np.linalg.norm(m, axis=1), weights=dt)),
        "peak_dipole_norm_Am2": float(np.linalg.norm(m, axis=1).max()),
        "mean_native_final_stage_torque_norm_Nm": float(np.average(np.linalg.norm(native, axis=1), weights=dt)),
        "peak_native_final_stage_torque_norm_Nm": float(np.linalg.norm(native, axis=1).max()),
        "magnetic_work_J": float(work.sum()), "energy_removed_J": float(energy[0]-energy[-1]),
        "energy_removed_per_actuation_time_W": float(-work.sum()/active_time) if active_time else None,
        "energy_removed_per_wall_time_W": float(-work.sum()/total_time),
        "positive_native_stage_power_count_above_1e_15_W": int(np.sum(np.sum(native*stage_w, axis=1) > 1e-15)),
        "initial_rate_rad_s": float(rate[0]), "final_rate_rad_s": float(rate[-1]),
        "threshold_0p5_deg_s": threshold_observation(t, rate, float(np.radians(0.5))),
        "continuous_final_rate_rad_s": float(continuous.omega_mag_rad_s.iloc[-1]),
        "continuous_last_recorded_time_s": float(continuous.time_s.iloc[-1]),
        "comparison_epoch_s": common_tick*1e-9,
        "cycled_rate_at_comparison_epoch_rad_s": float(rate[matches[0]]),
        "continuous_rate_at_comparison_epoch_rad_s": float(continuous.loc[continuous.time_ns == common_tick, "omega_mag_rad_s"].iloc[0]),
        "scope": "ASSUMED timing architecture test; electrical I^2 R estimates are not flight power. Duty is time with nonzero native input dipole, not a PWM carrier. No 24-hour requirement verification."}
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=HERE/"output_data/detumble_output_cycled.csv")
    parser.add_argument("--continuous-csv", type=Path, default=HERE/"output_data/detumble_output.csv")
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase6a/cycle_validation.json")
    args = parser.parse_args(argv)
    try:
        frame, config = load_run(args.csv)
        continuous, continuous_config = load_run(args.continuous_csv)
        if config.to_dict() != continuous_config.to_dict():
            raise ValueError("Continuous/cycled comparison requires identical physical/runtime configurations")
        report = analyze(frame, config, continuous)
        report["csv_sha256"] = hashlib.sha256(args.csv.read_bytes()).hexdigest()
        report["continuous_csv_sha256"] = hashlib.sha256(args.continuous_csv.read_bytes()).hexdigest()
        report["analysis_source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    except (ValueError, KeyError, OSError, IndexError) as exc:
        report = {"passed": False, "error": str(exc)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("physical_configuration", "validation")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
