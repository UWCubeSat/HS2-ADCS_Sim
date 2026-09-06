"""Phase 5B diagnostic experiments; production profiles/law/hardware stay unchanged.

Run --cases A B C D E candidate, then convergence/alternate-direction cases, or
omit --cases for the complete suite. Completed raw runs are reused only when
their config, simulation-source signature and NPZ hash match. No 24-hour run.
Counterfactual inputs are ASSUMED experiments, not candidate/flight replacements.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time
from typing import Any

import numpy as np
import pandas as pd
import Basilisk
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import MtbEffector, svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros, RigidBodyKinematics

from hs2_sim_config import DEFAULT_CONFIG, HS2SimConfig, get_hs2_candidate_config
from basilisk_adcs_adapter import ADCSConfig, MAX_EFF_CNT, controller_step
import scenario_huskysat2_detumble as scenario
import compare_reference_vs_basilisk as validation
from compare_physical_profiles import threshold_observation, attitude_separation_deg

HERE = Path(__file__).resolve().parent
OUT = HERE / "output_data" / "phase5b"


def diagnostic_parameter(parameter, value, purpose):
    return replace(parameter, value=value, status="ASSUMED",
                   source="Phase 5B diagnostic experiment; original Phase 5 configuration inputs",
                   revision="2026-09-06", notes=purpose + "; not a normal/default physical profile")


def configurations() -> dict[str, HS2SimConfig]:
    base, candidate = DEFAULT_CONFIG, get_hs2_candidate_config()
    a, d = np.diag(base.spacecraft.inertia.value), np.diag(candidate.spacecraft.inertia.value)
    result = {"A": base, "candidate": candidate}
    for name, moments in (("B", (d[0], d[1], a[2])), ("C", (a[0], a[1], d[2])), ("D", d)):
        tensor = tuple(tuple(float(v) for v in row) for row in np.diag(moments))
        inertia = diagnostic_parameter(base.spacecraft.inertia, tensor, f"{name}: inertia-only counterfactual; all other inputs baseline")
        result[name] = replace(base, spacecraft=replace(base.spacecraft, inertia=inertia))
    # A/F preserve baseline anisotropy; G/D preserve candidate anisotropy.
    # This separates overall inertia scale from the changed gyroscopic ratio.
    scale = float(d[0]/a[0])
    for name, moments in (("F", a*scale), ("G", d/scale)):
        tensor = tuple(tuple(float(v) for v in row) for row in np.diag(moments))
        inertia = diagnostic_parameter(base.spacecraft.inertia, tensor,
            f"{name}: diagnostic separation of uniform inertia scale and anisotropy")
        result[name] = replace(base, spacecraft=replace(base.spacecraft, inertia=inertia))
    result["E"] = replace(candidate, spacecraft=replace(candidate.spacecraft,
        inertia=diagnostic_parameter(base.spacecraft.inertia, base.spacecraft.inertia.value,
                                     "E: candidate mass/envelope/COM metadata with BASELINE inertia")))
    for name, config in (("A", base), ("candidate", candidate)):
        for suffix, dt in (("h05", 0.05), ("h025", 0.025)):
            timing = config.timing
            for field in ("dynamics_step", "environment_step", "sensor_step", "control_step"):
                timing = replace(timing, **{field: diagnostic_parameter(getattr(timing, field), dt,
                    "Joint sample/hold and integration refinement; production equal-rate schedule unchanged")})
            result[name + "_" + suffix] = replace(config, timing=timing)
        rate = (0.0, 0.0, float(np.linalg.norm(base.initial.body_rate.value)))
        result[name + "_z"] = replace(config, initial=replace(config.initial,
            body_rate=diagnostic_parameter(config.initial.body_rate, rate, "Equal-norm +Z initial-rate sensitivity")))
    # Capture EVERY applied command interval. This changes recorder cadence only.
    return {name: replace(config, timing=replace(config.timing, record_step=diagnostic_parameter(
        config.timing.record_step, config.timing.dynamics_step.value,
        "Full-step independent work accounting; output analyses also retain original 1 s samples")))
        for name, config in result.items()}


def isolation_report() -> dict[str, Any]:
    a, b = DEFAULT_CONFIG.to_dict(), get_hs2_candidate_config().to_dict()
    differences = [f"{section}.{field}" for section in a for field in a[section]
                   if a[section][field] != b[section][field]]
    same = {section: a[section] == b[section] for section in a if section != "spacecraft"}
    passed = all(same.values()) and set(differences) == {
        "spacecraft.mass", "spacecraft.dimensions", "spacecraft.inertia", "spacecraft.com"}
    cases = configurations()
    inertia_only = all(replace(cases[n], spacecraft=replace(cases[n].spacecraft,
        inertia=cases["A"].spacecraft.inertia)) == cases["A"] for n in ("B", "C", "D", "F", "G"))
    return {"passed": passed and inertia_only, "changed_parameters_including_metadata": differences,
            "identical_nonphysical_sections": same, "BCDFG_change_inertia_only": inertia_only,
            "COM_note": "Numeric zero unchanged; candidate placeholder provenance differs"}


def simulation_signature() -> str:
    digest = hashlib.sha256(str(getattr(Basilisk, "__version__", "unknown")).encode())
    for name in ("scenario_huskysat2_detumble.py", "hs2_sim_config.py", "basilisk_adcs_adapter.py",
                 "magnetic_environment.py", "magnetic_actuation.py"):
        digest.update((HERE / name).read_bytes())
    return digest.hexdigest()


def raw_run(name: str, config: HS2SimConfig, folder: Path):
    archive, metadata = folder / f"{name}_raw.npz", folder / f"{name}_raw.json"
    signature = simulation_signature()
    if archive.exists() and metadata.exists():
        meta = json.loads(metadata.read_text())
        if (meta["config_sha256"] == config.fingerprint() and meta["simulation_signature"] == signature
                and meta["npz_sha256"] == hashlib.sha256(archive.read_bytes()).hexdigest()):
            with np.load(archive) as saved:
                return {key: saved[key] for key in saved.files}, meta
    started = time.perf_counter()
    frame = scenario.run(config=config, write_outputs=False, make_plots=False)
    checks = validation.build_validation({}, validation.summarize_basilisk(frame))["checks"]
    del checks["reference_numeric_finite"]
    if not all(check["passed"] for check in checks.values()):
        raise ValueError(f"Independent full-step validation failed: {name}: {checks}")
    # Match the original saved-sample horizon: 5792 s, not an unrecorded fraction.
    frame = frame[frame.time_s <= np.floor(config.duration_s)]
    def vec(prefix, suffix):
        return frame[[f"{prefix}_{axis}_{suffix}" for axis in "xyz"]].to_numpy(dtype=float)
    def mrp(prefix):
        return frame[[f"{prefix}_{i}" for i in (1, 2, 3)]].to_numpy(dtype=float)
    data = {"ticks": frame.time_ns.to_numpy(dtype=np.int64), "omega": vec("omega_B", "rad_s"),
            "sigma": mrp("sigma_BN"), "field_b": vec("B_B", "T"), "field_n": vec("B_N", "T"),
            "dipole": vec("mcmd", "Am2"), "torque": vec("applied_torque_B", "Nm"),
            "pre_omega": vec("held_omega_B", "rad_s"), "pre_sigma": mrp("held_sigma_BN"),
            "held_dipole": vec("held_mcmd", "Am2"), "held_field_n": vec("held_B_N", "T"),
            "saturation": frame[[f"saturation_{a}" for a in "xyz"]].to_numpy(dtype=bool),
            "power_electrical": frame.pcoil_total_W.to_numpy(dtype=float), "position": vec("r_N", "m")}
    if not (np.array_equal(data["pre_omega"][1:], data["omega"][:-1])
            and np.array_equal(data["held_dipole"][1:], data["dipole"][:-1])
            and np.all(np.diff(data["ticks"]) == round(config.timing.dynamics_step.value * 1e9))):
        raise ValueError("Full-step accounting requires every exact held command/state interval")
    np.savez_compressed(archive, **data)
    meta = {"configuration": config.to_dict(), "config_sha256": config.fingerprint(),
            "simulation_signature": signature, "npz_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            "wall_time_s": time.perf_counter() - started, "validation_checks": checks}
    metadata.write_text(json.dumps(meta, indent=2))
    return data, meta


def rk_work(sigma, omega, dipole, field_n, dt, inertia):
    """Independent held-input stage reconstruction, NOT energy differencing.

    Integrate tau dot omega with RK4 weights. Native final-stage torque is checked
    separately against the recorded effector output. Energy comes from propagated
    spacecraft states. Sparse 1 s final-stage readbacks alone cannot supply this.
    """
    moments = np.diag(inertia)
    h = dt[:, None]
    y0 = np.column_stack((sigma, omega))
    def evaluate(y):
        s, w = y[:, :3], y[:, 3:]
        b = validation.rotate_inertial_to_body(s, field_n)
        tau = np.cross(dipole, b)
        s2 = np.sum(s*s, axis=1, keepdims=True)
        sdot = ((1-s2)*w + 2*np.cross(s, w) + 2*s*np.sum(s*w, axis=1, keepdims=True))/4
        gyro = -np.cross(w, moments*w)/moments
        return np.column_stack((sdot, tau/moments + gyro)), tau*w, tau, w, gyro
    stages = [evaluate(y0)]
    stages.append(evaluate(y0 + h*stages[0][0]/2))
    stages.append(evaluate(y0 + h*stages[1][0]/2))
    stages.append(evaluate(y0 + h*stages[2][0]))
    work_axes = h * sum(weight * stage[1] for weight, stage in zip((1, 2, 2, 1), stages))/6
    return work_axes, stages[-1][2], stages[-1][3], stages[-1][4], stages[0][1].sum(axis=1)


def intervals(mask, times):
    """Half-open application intervals on the control clock, never sample counts alone."""
    starts = np.flatnonzero(np.diff(np.r_[False, mask, False].astype(int)) == 1)
    ends = np.flatnonzero(np.diff(np.r_[False, mask, False].astype(int)) == -1)
    return [{"start_s": float(times[a]), "end_s": float(times[b]), "duration_s": float(times[b]-times[a])}
            for a, b in zip(starts, ends)]


def accounting(data, config: HS2SimConfig):
    t = data["ticks"] * 1e-9
    w, b, m = data["omega"], data["field_b"], data["dipole"]
    tensor = np.asarray(config.spacecraft.inertia.value)
    moments = np.diag(tensor)
    if not np.array_equal(tensor, np.diag(moments)):
        raise ValueError("This diagnostic decomposition requires the specified diagonal tensors")
    momentum = w @ tensor.T
    energy = 0.5*np.sum(w*momentum, axis=1)
    speed = np.linalg.norm(w, axis=1)
    bnorm = np.linalg.norm(b, axis=1)
    bhat = b / bnorm[:, None]
    parallel = np.sum(w*bhat, axis=1)[:, None]*bhat
    perpendicular = w-parallel
    pn, qn = np.linalg.norm(parallel, axis=1), np.linalg.norm(perpendicular, axis=1)
    fraction = qn**2/np.maximum(speed**2, 1e-300)
    gain = config.controller.dipole_command_gain.value
    requested = gain*np.cross(w, b)
    acquisition_torque = np.cross(m, b)
    ideal_torque = -gain*bnorm[:, None]**2*perpendicular
    acquisition_power = np.sum(acquisition_torque*w, axis=1)
    ideal_power = np.sum(ideal_torque*w, axis=1)
    retained = np.divide(acquisition_power, ideal_power, out=np.ones_like(ideal_power), where=ideal_power != 0)
    dt = np.diff(t)
    work_axes, predicted_tau, stage_w, gyro, start_power = rk_work(
        data["pre_sigma"][1:], data["pre_omega"][1:], data["held_dipole"][1:],
        data["held_field_n"][1:], dt, tensor)
    native_tau = data["torque"][1:]
    native_power = np.sum(native_tau*stage_w, axis=1)
    cumulative_work = np.r_[0.0, np.cumsum(work_axes.sum(axis=1))]
    delta_energy = energy-energy[0]
    energy_residual = delta_energy-cumulative_work
    saturation = data["saturation"][:-1]
    any_sat = saturation.any(axis=1)
    work = work_axes.sum(axis=1)
    acquisition_gyro = -np.cross(w, momentum)/moments
    # All diagnostic tensors are axisymmetric: gyro redirects omega but its contribution
    # to d(|omega|^2/2)/dt is zero. It cannot directly remove energy or rate norm.
    gyro_norm_derivative = np.sum(acquisition_gyro*w, axis=1)
    instantaneous_axis_dissipation = -m*np.cross(w, b)
    unsat = ~data["saturation"].any(axis=1)
    native_tau_error = float(np.max(np.abs(native_tau-predicted_tau)))
    residual = float(np.max(np.abs(energy_residual)))
    history = pd.DataFrame({"time_s": t, "energy_J": energy, "delta_energy_J": delta_energy,
        "H_norm_kg_m2_s": np.linalg.norm(momentum, axis=1), "rate_norm_rad_s": speed,
        "angle_omega_B_deg": np.degrees(np.arccos(np.clip(np.sum(w*b, axis=1)/np.maximum(speed*bnorm, 1e-300), -1, 1))),
        "omega_parallel_norm_rad_s": pn, "omega_perpendicular_norm_rad_s": qn,
        "magnetically_controllable_rate_squared_fraction": fraction,
        "native_final_stage_mechanical_power_W": np.r_[0., native_power],
        "acquisition_mechanical_power_W": acquisition_power,
        "ideal_unclipped_same_state_power_W": ideal_power,
        "clipping_same_state_power_retained_fraction": retained,
        "cumulative_magnetic_work_J": cumulative_work, "energy_work_residual_J": energy_residual,
        "native_final_stage_angle_torque_omega_deg": np.r_[np.nan, np.degrees(np.arccos(np.clip(
            np.sum(native_tau*stage_w, axis=1)/np.maximum(np.linalg.norm(native_tau, axis=1)*np.linalg.norm(stage_w, axis=1), 1e-300), -1, 1)))],
        "gyroscopic_rate_squared_derivative_rad2_s3": gyro_norm_derivative})
    for axis, j in zip("xyz", range(3)):
        for prefix, values in (("H_B", momentum), ("omega_B", w), ("omega_parallel_B", parallel),
                               ("omega_perpendicular_B", perpendicular), ("requested_dipole", requested),
                               ("commanded_clipped_dipole", m), ("native_final_stage_torque", data["torque"])):
            history[f"{prefix}_{axis}"] = values[:, j]
    windows = {}
    for lo, hi in ((0, 1000), (1000, 3000), (3000, float(t[-1]))):
        selected = (t[:-1] >= lo) & (t[:-1] < hi)
        if not selected.any():
            continue
        weights = dt[selected]
        f = fraction[:-1][selected]
        windows[f"{lo:g}_{hi:g}_s"] = {
            "mean_controllable_rate_squared_fraction": float(np.average(f, weights=weights)),
            "fraction_within_15_deg_of_parallel_or_antiparallel": float(np.average(f < np.sin(np.radians(15))**2, weights=weights)),
            "fraction_angle_60_to_120_deg": float(np.average(f >= 0.75, weights=weights)),
            "magnetic_work_J": float(work[selected].sum()),
            "mean_native_mechanical_power_W": float(work[selected].sum()/weights.sum()),
            "saturation_time_fraction": float(np.average(any_sat[selected], weights=weights)),
            "mean_gyro_acceleration_rad_s2": float(np.average(np.linalg.norm(gyro[selected], axis=1), weights=weights)),
            "mean_torque_acceleration_rad_s2": float(np.average(np.linalg.norm(native_tau[selected]/moments, axis=1), weights=weights))}
    groups = {}
    for axis, j in zip("xyz", range(3)):
        mask = saturation[:, j]
        blocks = intervals(mask, t)
        groups[axis] = {
            "requested_dipole_max_abs_Am2": float(np.abs(requested[:-1, j]).max()),
            "commanded_clipped_dipole_max_abs_Am2": float(np.abs(m[:-1, j]).max()),
            "saturation_time_fraction": float(dt[mask].sum()/dt.sum()),
            "saturation_duration_s": float(dt[mask].sum()), "saturation_intervals": blocks,
            "longest_saturation_interval_s": max((x["duration_s"] for x in blocks), default=0.),
            "mean_native_torque_norm_when_saturated_Nm": float(np.average(np.linalg.norm(native_tau[mask], axis=1), weights=dt[mask])) if mask.any() else None,
            "mean_native_torque_norm_when_unsaturated_Nm": float(np.average(np.linalg.norm(native_tau[~mask], axis=1), weights=dt[~mask])) if (~mask).any() else None,
            "magnetic_work_during_axis_saturation_J": float(work[mask].sum()),
            "magnetic_work_outside_axis_saturation_J": float(work[~mask].sum()),
            "note": "Axis groups overlap; their total-work entries must not be summed across axes"}
    summary = {"inertia_tensor_kg_m2": tensor.tolist(), "principal_moments_kg_m2": np.linalg.eigvalsh(tensor).tolist(),
        "initial_energy_J": float(energy[0]), "final_energy_J": float(energy[-1]),
        "initial_H_norm_kg_m2_s": float(np.linalg.norm(momentum[0])), "final_H_norm_kg_m2_s": float(np.linalg.norm(momentum[-1])),
        "total_magnetic_work_J": float(cumulative_work[-1]), "energy_removed_J": float(energy[0]-energy[-1]),
        "mean_energy_removal_W": float((energy[0]-energy[-1])/t[-1]),
        "max_energy_work_residual_J": residual, "max_native_stage_torque_error_Nm": native_tau_error,
        "native_stage_endpoint_rectangle_work_J": float(np.sum(native_power*dt)),
        "held_interval_trapezoid_work_J": float(np.sum((start_power+native_power)*dt/2)),
        "accounting_horizon_s": float(t[-1]), "integration_step_s": config.timing.dynamics_step.value,
        "initial_rate_rad_s": float(speed[0]), "final_rate_rad_s": float(speed[-1]),
        "thresholds": {str(limit): threshold_observation(t, speed, limit) for limit in (0.05, 0.02, float(np.radians(0.5)))},
        "geometry_windows": windows, "final_parallel_rate_rad_s": float(pn[-1]), "final_perpendicular_rate_rad_s": float(qn[-1]),
        "mean_controllable_rate_squared_fraction": float(np.average(fraction[:-1], weights=dt)),
        "rate_squared_weighted_controllability_fraction": float(np.sum(qn[:-1]**2*dt)/np.sum(speed[:-1]**2*dt)),
        "saturation": groups, "any_axis_saturation_time_fraction": float(dt[any_sat].sum()/dt.sum()),
        "work_during_any_saturation_J": float(work[any_sat].sum()), "work_without_saturation_J": float(work[~any_sat].sum()),
        "instantaneous_law": {"unsaturated_max_torque_error_Nm": float(np.abs(acquisition_torque[unsat]-ideal_torque[unsat]).max()) if unsat.any() else None,
            "clipped_max_torque_error_vs_ideal_Nm": float(np.abs(acquisition_torque-ideal_torque).max()),
            "max_acquisition_power_W": float(acquisition_power.max()), "positive_acquisition_power_count_above_1e_15_W": int(np.sum(acquisition_power > 1e-15)),
            "max_native_stage_power_W": float(native_power.max()), "positive_native_stage_power_count_above_1e_15_W": int(np.sum(native_power > 1e-15)),
            "power_retention_when_saturated": float(np.sum(acquisition_power[:-1][any_sat]*dt[any_sat])/np.sum(ideal_power[:-1][any_sat]*dt[any_sat])) if any_sat.any() else 1.,
            "componentwise_sign_proof_max_residual_W": float(np.abs(acquisition_power-instantaneous_axis_dissipation.sum(axis=1)).max()),
            "proof": "P=-sum_i(m_clipped_i*m_requested_i)/K <= 0 at the same acquisition state. Independent component clipping preserves each sign and only reduces instantaneous dissipation. Held-command stage power is checked separately."},
        "max_abs_gyro_contribution_to_rate_squared_derivative": float(np.abs(gyro_norm_derivative).max()),
        "control_work_by_body_axis_J": work_axes.sum(axis=0).tolist(),
        "passed": residual < 1e-8*float(energy[0]) and native_tau_error < 1e-15}
    return summary, history


class BenchController(sysModel.SysModel):
    """Fixed 0.1 s controller clock on a separately refined dynamics bench."""
    def __init__(self, state, field, config, fixed_dipole=None):
        super().__init__()
        self.state, self.field, self.config = state, np.asarray(field), ADCSConfig.from_sim_config(config)
        self.fixed_dipole = fixed_dipole
        self.message = messaging.MTBCmdMsg().write(messaging.MTBCmdMsgPayload())

    def UpdateState(self, tick):
        if tick % 100_000_000:
            return
        state = self.state.read()
        field_b = np.asarray(RigidBodyKinematics.MRP2C(state.sigma_BN)) @ self.field
        dipole = self.fixed_dipole
        if dipole is None:
            dipole = controller_step(tick*1e-9, np.asarray(state.omega_BN_B), field_b, self.config)["commanded_magnetic_dipole_B_Am2"]
        payload = messaging.MTBCmdMsgPayload()
        payload.mtbDipoleCmds = list(dipole) + [0.]*(MAX_EFF_CNT-3)
        self.message.write(payload, tick)


def bench(config, omega, field, *, dt=0.1, duration=120., fixed_dipole=None, euler=False):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("diagnostic")
    proc.addTask(sim.CreateNewTask("bench", macros.sec2nano(dt)))
    sc = scenario.configure_spacecraft(config)
    sc.hub.omega_BN_BInit = list(omega)
    sc.hub.sigma_BNInit = [0., 0., 0.]
    if euler:
        stepper = svIntegrators.svIntegratorEuler(sc)
        sc.setIntegrator(stepper)
    driver = BenchController(sc.scStateOutMsg, field, config, fixed_dipole)
    native = MtbEffector.MtbEffector()
    native.mtbCmdInMsg.subscribeTo(driver.message)
    payload = messaging.MagneticFieldMsgPayload()
    payload.magField_N = list(field)
    field_msg = messaging.MagneticFieldMsg().write(payload)
    native.magInMsg.subscribeTo(field_msg)
    config_msg = scenario.configure_mtb_config_message(ADCSConfig.from_sim_config(config), config)
    native.mtbParamsInMsg.subscribeTo(config_msg)
    sc.addDynamicEffector(native)
    # Full-step recording supports the one-step inertia measurement too.
    state_log = sc.scStateOutMsg.recorder()
    torque_log = native.logger(["torqueExternalPntB_B"])
    for model, priority in ((sc, 1000), (native, 980), (torque_log, 975), (driver, 600), (state_log, 0)):
        sim.AddModelToTask("bench", model, ModelPriority=priority)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    return {"omega": np.asarray(state_log.omega_BN_B), "sigma": np.asarray(state_log.sigma_BN),
            "torque": np.asarray(torque_log.torqueExternalPntB_B).reshape(-1, 3),
            "time_s": np.asarray(state_log.times())*1e-9}


def bench_suite():
    report: dict[str, Any] = {"plant_inertia_measurements": {}, "principal_axis_cases": {}, "fixed_control_clock_convergence": {}}
    for name, config in (("baseline", DEFAULT_CONFIG), ("candidate", get_hs2_candidate_config())):
        measured = []
        for axis in range(3):
            dipole, field = np.eye(3)[(axis+1)%3]*0.1, np.eye(3)[(axis+2)%3]*4e-5
            data = bench(config, [0., 0., 0.], field, dt=0.001, duration=0.001, fixed_dipole=dipole, euler=True)
            expected_tau = np.eye(3)[axis]*4e-6
            np.testing.assert_allclose(data["torque"][-1], expected_tau, atol=1e-20, rtol=0)
            observed = float(data["torque"][-1, axis]*0.001/data["omega"][-1, axis])
            measured.append(observed)
            np.testing.assert_allclose(data["omega"][-1], expected_tau*0.001/np.diag(config.spacecraft.inertia.value), atol=1e-18, rtol=0)
        report["plant_inertia_measurements"][name] = {"inferred_diagonal_kg_m2": measured,
            "max_abs_error_kg_m2": float(np.max(np.abs(np.asarray(measured)-np.diag(config.spacecraft.inertia.value))))}
        for axis in range(3):
            unit, other = np.eye(3)[axis], np.eye(3)[(axis+1)%3]
            for geometry, field in (("perpendicular", other*4e-5),
                                    ("nearly_parallel", (np.cos(np.radians(3))*unit+np.sin(np.radians(3))*other)*4e-5)):
                data = bench(config, unit*0.01, field)
                energy = 0.5*np.sum(data["omega"]**2*np.diag(config.spacecraft.inertia.value), axis=1)
                report["principal_axis_cases"][f"{name}_{'xyz'[axis]}_{geometry}"] = {
                    "initial_rate_rad_s": 0.01, "field_N_T": field.tolist(), "duration_s": 120.,
                    "final_rate_rad_s": float(np.linalg.norm(data["omega"][-1])),
                    "initial_unsaturated_damping_time_constant_s": float(np.diag(config.spacecraft.inertia.value)[axis]/(config.controller.dipole_command_gain.value*(4e-5)**2)),
                    "peak_torque_Nm": float(np.linalg.norm(data["torque"], axis=1).max()),
                    "energy_change_J": float(energy[-1]-energy[0])}
        trajectories = {}
        for dt in (0.1, 0.05, 0.025):
            data = bench(config, [0.8, -0.2, 0.3], [1e-5, -2e-5, 3e-5], dt=dt, duration=30.)
            trajectories[str(dt)] = {"final_rate_vector_rad_s": data["omega"][-1].tolist(),
                                     "final_rate_rad_s": float(np.linalg.norm(data["omega"][-1]))}
        steps = [np.asarray(trajectories[str(h)]["final_rate_vector_rad_s"]) for h in (0.1, 0.05, 0.025)]
        changes = [float(np.linalg.norm(steps[i+1]-steps[i])) for i in (0, 1)]
        report["fixed_control_clock_convergence"][name] = {
            "trajectories": trajectories, "successive_rate_vector_differences_rad_s": changes,
            "refinement_ratio": changes[0]/changes[1], "passed": changes[1] < changes[0] < 1e-6}
    report["passed"] = (all(r["max_abs_error_kg_m2"] < 1e-14 for r in report["plant_inertia_measurements"].values())
        and all(r["passed"] for r in report["fixed_control_clock_convergence"].values())
        and all(r["energy_change_J"] < 0 for r in report["principal_axis_cases"].values()))
    report["scope"] = "Synthetic constant inertial B; unchanged gains/limits. Principal-axis inputs 0.01 rad/s avoid saturation. Euler is used only for one-step inertia identification. Convergence uses RK4 with a fixed 0.1 s control clock."
    return report


def assemble(folder):
    summaries = {path.stem[:-8]: json.loads(path.read_text()) for path in folder.rglob("*_summary.json")}
    differences = {}
    for other in ("B", "C", "D", "E", "F", "G", "candidate"):
        if "A" not in summaries or other not in summaries:
            continue
        a, b = pd.read_csv(folder / "A_timeseries.csv"), pd.read_csv(folder / f"{other}_timeseries.csv")
        columns = [f"omega_B_{axis}" for axis in "xyz"]
        delta = np.linalg.norm(b[columns].to_numpy()-a[columns].to_numpy(), axis=1)
        differences[other] = {"final_rate_difference_rad_s": summaries[other]["final_rate_rad_s"]-summaries["A"]["final_rate_rad_s"],
                              "max_rate_vector_difference_rad_s": float(delta.max()), "final_rate_vector_difference_rad_s": float(delta[-1])}
        with np.load(folder / "A_raw.npz") as ar, np.load(folder / f"{other}_raw.npz") as br:
            if not np.array_equal(ar["ticks"], br["ticks"]):
                raise ValueError("Inertia comparisons require identical acquisition epochs")
            angle = attitude_separation_deg(ar["sigma"], br["sigma"])
            differences[other].update({"max_attitude_separation_deg": float(angle.max()),
                "final_attitude_separation_deg": float(angle[-1]),
                "max_inertial_field_difference_T": float(np.linalg.norm(ar["field_n"]-br["field_n"], axis=1).max()),
                "max_position_difference_m": float(np.linalg.norm(ar["position"]-br["position"], axis=1).max())})
    bench_path = folder / "bench.json"
    if not bench_path.exists():
        bench_path = folder / "bench_checks" / "bench.json"
    isolation = isolation_report()
    bench_report = json.loads(bench_path.read_text()) if bench_path.exists() else None
    complete = set(configurations()).issubset(summaries) and bench_report is not None
    passed = (isolation["passed"] and (bool(summaries) or bench_report is not None)
        and all(s["passed"] and s["full_step_validation_passed"] for s in summaries.values())
        and (bench_report is None or bench_report["passed"]))
    return {"configuration_isolation": isolation, "cases": summaries, "differences_from_A": differences,
        "bench": bench_report, "complete_suite_present": complete, "available_checks_passed": passed,
        "analysis_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "csv_component_units": {"H_B": "kg m^2/s", "omega_B": "rad/s", "omega_parallel_B": "rad/s",
            "omega_perpendicular_B": "rad/s", "requested_dipole": "A m^2",
            "commanded_clipped_dipole": "A m^2", "native_final_stage_torque": "N m"},
        "dipole_contract": "Clipped dipole is the published command, whose torque effect is independently verified against native dynamics; this is not hardware readback. Saturation groups describe the following held-command interval, and overlap across axes.",
        "accounting_contract": "Cumulative work is reconstructed from every held-input RK4 interval, independently checked against native final-stage torque and propagated energy. Power readback uses reconstructed final-stage rate, not mismatched accepted attitude/rate. CSV time series retain every original 1 s sample. Controllability fraction is |omega_perp|^2/|omega|^2, not an anisotropic energy fraction.",
        "convergence_contract": "Full-orbit h05/h025 refine ALL equal-rate acquisition/control/environment/dynamics tasks together with unchanged gain/law. This is joint sample/hold+RK4 convergence. The isolated bench separately holds the control clock fixed while refining dynamics.",
        "scope": "Diagnostic counterfactuals; no defaults, physical-profile constants, hardware limits or controller gains changed; no 24-hour/flight validation."}


def plot_summary(folder):
    """Standalone scientific figure from saved diagnostics; no simulation rerun."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True, constrained_layout=True)
    for name, label in (("A", "Regression baseline"), ("candidate", "HS-2 candidate (provisional)")):
        data = pd.read_csv(folder / f"{name}_timeseries.csv")
        minutes = data["time_s"].to_numpy(dtype=float)/60
        axes[0].semilogy(minutes, np.degrees(data["rate_norm_rad_s"].to_numpy(dtype=float)), label=label)
        axes[1].plot(minutes, data["magnetically_controllable_rate_squared_fraction"].rolling(60, min_periods=1).mean())
        energy = data["energy_J"].to_numpy(dtype=float)
        axes[2].semilogy(minutes, energy/energy[0])
    axes[0].axhline(0.5, color="grey", linestyle="--", linewidth=0.8, label="0.5 deg/s observation threshold")
    axes[0].set_ylabel("Angular speed (deg/s)")
    axes[0].legend()
    axes[1].set_ylabel("Perpendicular rate squared fraction\n(trailing 60 saved 1 s samples)")
    axes[1].set_ylim(0, 1)
    axes[2].set_ylabel("Rotational energy / initial energy")
    axes[2].set_xlabel("Elapsed time (minutes)")
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Phase 5B: unchanged controller; two provisional inertia profiles")
    figure.savefig(folder / "physical_profile_dynamics.png", dpi=160)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", choices=tuple(configurations()), default=list(configurations()))
    parser.add_argument("--bench", action="store_true")
    parser.add_argument("--plot", action="store_true", help="Plot existing A/candidate diagnostic time series")
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    cases = configurations()
    for name in args.cases:
        print(f"Phase 5B case {name}", flush=True)
        data, meta = raw_run(name, cases[name], args.output)
        summary, history = accounting(data, cases[name])
        summary["configuration"] = cases[name].to_dict()
        summary["config_sha256"] = cases[name].fingerprint()
        summary["full_step_validation_passed"] = all(v["passed"] for v in meta["validation_checks"].values())
        summary["run_wall_time_s"] = meta["wall_time_s"]
        (args.output / f"{name}_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
        history.loc[data["ticks"] % 1_000_000_000 == 0].to_csv(args.output / f"{name}_timeseries.csv", index=False)
        print(json.dumps({k: summary[k] for k in ("final_rate_rad_s", "total_magnetic_work_J", "max_energy_work_residual_J", "passed")}), flush=True)
        if not summary["passed"]:
            raise ValueError(f"Work/torque accounting failed for {name}")
    if args.bench:
        (args.output / "bench.json").write_text(json.dumps(bench_suite(), indent=2, allow_nan=False))
    report = assemble(args.output)
    (args.output / "analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    if args.plot:
        plot_summary(args.output)
    print(f"Wrote {args.output / 'analysis.json'}", flush=True)
    return 0 if report["available_checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
