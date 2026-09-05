from __future__ import annotations

from pathlib import Path
import json
from typing import Any

import numpy as np
import pandas as pd
from datetime import datetime, timezone
from hs2_sim_config import DEFAULT_CONFIG, HS2SimConfig

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REF = ROOT / "reference_standalone" / "adcs_output.csv"
OUT_DATA = HERE / "output_data"
COMPARE_OUT = OUT_DATA / "comparison_metrics.json"

# Shared physical INPUTS; the equations below remain independent of Basilisk.
# Compatibility aliases for existing analytical regression fixtures.
MASS_KG = DEFAULT_CONFIG.spacecraft.mass.value
LX_M, LY_M, LZ_M = DEFAULT_CONFIG.spacecraft.dimensions.value
IXX, IYY, IZZ = np.diag(DEFAULT_CONFIG.spacecraft.inertia.value)
INERTIA_DIAG_KG_M2 = np.array([IXX, IYY, IZZ], dtype=float)

# CONFIRMED software timing in the Phase 2A detumble baseline, not a flight requirement.
CONTROL_STEP_NS = round(DEFAULT_CONFIG.timing.control_step.value * 1e9)
PUBLICATION_TIME_COLUMNS = [
    "state_time_ns", "field_evaluation_time_ns", "tam_message_time_ns",
    "nav_message_time_ns", "dipole_command_time_ns", "expected_torque_time_ns",
    "applied_torque_time_ns", "diagnostic_time_ns", "sensor_state_time_ns",
    "wmm_state_time_ns", "earth_orientation_time_ns",
]
HELD_TIME_COLUMNS = ["held_state_time_ns", "held_field_time_ns", "held_dipole_time_ns",
                     "held_expected_torque_time_ns"]


def reference_earth_matrix(et_s):
    """IAU Earth basis-vector construction, independent of publisher Euler matrices.

    pck00011 (2022-12-27), BODY399 coefficients. This tests the stated low-order
    software model, not agreement with measured terrestrial orientation/EOP.
    """
    days = np.asarray(et_s) / 86400.0
    ra = np.radians(-0.641 * days / 36525.0)
    dec = np.radians(90.0 - 0.557 * days / 36525.0)
    w = np.radians((190.147 + 360.9856235 * days) % 360.0)
    pole = np.column_stack((np.cos(ra) * np.cos(dec), np.sin(ra) * np.cos(dec), np.sin(dec)))
    node = np.column_stack((-np.sin(ra), np.cos(ra), np.zeros_like(ra)))
    transverse = np.cross(pole, node)
    x = np.cos(w)[:, None] * node + np.sin(w)[:, None] * transverse
    y = -np.sin(w)[:, None] * node + np.cos(w)[:, None] * transverse
    return np.stack((x, y, pole), axis=1)


def vector_columns(prefix: str, suffix: str) -> list[str]:
    return [f"{prefix}_{axis}_{suffix}" for axis in "xyz"]


def rotate_inertial_to_body(sigma: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Passive C_BN rotation via unit quaternions, independent of the TAM code."""
    s2 = np.sum(sigma * sigma, axis=1, keepdims=True)
    q0 = (1.0 - s2) / (1.0 + s2)
    q = 2.0 * sigma / (1.0 + s2)
    return ((q0 * q0 - np.sum(q * q, axis=1, keepdims=True)) * vectors
            + 2.0 * q * np.sum(q * vectors, axis=1, keepdims=True)
            - 2.0 * q0 * np.cross(q, vectors))


def rate_derivative(omega, torque, inertia):
    """Independent Euler equation; preserve Phase 3 arithmetic for diagonal I."""
    inertia = np.asarray(inertia)
    if inertia.ndim == 2 and np.array_equal(inertia, np.diag(np.diag(inertia))):
        inertia = np.diag(inertia)
    if inertia.ndim == 1:
        return (torque - np.cross(omega, inertia * omega)) / inertia
    return np.linalg.solve(inertia, (torque - np.cross(omega, omega @ inertia.T)).T).T


def config_for_telemetry(df, config=None):
    if config is None:
        saved = df.attrs.get("simulation_config")
        config = DEFAULT_CONFIG if saved is None else HS2SimConfig.from_dict(saved)
    config.validate()
    if "simulation_config_sha256" in df and not (df["simulation_config_sha256"] == config.fingerprint()).all():
        raise ValueError("Telemetry/configuration fingerprint mismatch; supply the matching saved configuration")
    return config


def predict_body_rate(omega: np.ndarray, torque: np.ndarray, dt: np.ndarray,
                      inertia=INERTIA_DIAG_KG_M2) -> np.ndarray:
    """Independent Euler rigid-body equation/RK4 over the recorded hold interval.

    Assumes the configured development inertia and constant applied
    body torque; this check must be revised if other effectors are introduced.
    It consumes effector readback and truth states, never controller internals.
    """
    def derivative(w):
        return rate_derivative(w, torque, inertia)

    h = np.asarray(dt).reshape(-1, 1)
    k1 = derivative(omega)
    k2 = derivative(omega + 0.5 * h * k1)
    k3 = derivative(omega + 0.5 * h * k2)
    k4 = derivative(omega + h * k3)
    return omega + h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def predict_magnetic_step(sigma, omega, dipole, field_n, dt, inertia=INERTIA_DIAG_KG_M2):
    """Independent coupled MRP/Euler RK4 with held m_B and B_N (SI units).

    Uses recorded inputs, not native output or the controller's torque. Returns
    accepted state, final RK stage state and torque for telemetry comparison.
    Inertia is the unchanged ASSUMED development tensor; no other torques apply.
    """
    h = np.asarray(dt).reshape(-1, 1)
    state = np.column_stack((sigma, omega))

    def derivative(y):
        s, w = y[:, :3], y[:, 3:]
        s2 = np.sum(s * s, axis=1, keepdims=True)
        sdot = ((1 - s2) * w + 2 * np.cross(s, w)
                + 2 * s * np.sum(s * w, axis=1, keepdims=True)) / 4
        tau = np.cross(dipole, rotate_inertial_to_body(s, field_n))
        wdot = rate_derivative(w, tau, inertia)
        return np.column_stack((sdot, wdot))

    k1 = derivative(state)
    k2 = derivative(state + h * k1 / 2)
    k3 = derivative(state + h * k2 / 2)
    last_stage = state + h * k3
    k4 = derivative(last_stage)
    result = state + h * (k1 + 2*k2 + 2*k3 + k4) / 6
    shadow = np.sum(result[:, :3]**2, axis=1) > 1
    result[shadow, :3] /= -np.sum(result[shadow, :3]**2, axis=1, keepdims=True)
    return result, last_stage, np.cross(dipole, rotate_inertial_to_body(last_stage[:, :3], field_n))


def require_detumble_telemetry(df: pd.DataFrame) -> None:
    """Reject legacy/partial evidence rather than passing on command aliases."""
    required = {
        "telemetry_schema_version", "time_s", "time_ns", "sensor_state_time_ns",
        "control_step_ns", "nav_time_tag_s", "applied_torque_source", "pcoil_total_W", "application_interval_valid",
        "earth_orientation_tdb_s", "earth_orientation_enabled", "earth_orientation_model",
        "wmm_coefficient_time_ns", *PUBLICATION_TIME_COLUMNS, *HELD_TIME_COLUMNS,
        *[f"earth_C_PN_{i}{j}" for i in (1, 2, 3) for j in (1, 2, 3)],
        *vector_columns("held_omega_B", "rad_s"), *vector_columns("held_B_N", "T"),
        *vector_columns("held_mcmd", "Am2"), *vector_columns("held_control_torque_B", "Nm"),
        *vector_columns("omega_B", "rad_s"),
        *vector_columns("sensor_state_omega_B", "rad_s"),
        *vector_columns("nav_omega_B", "rad_s"),
        *vector_columns("B_N", "T"), *vector_columns("B_B", "T"),
        *vector_columns("mcmd", "Am2"),
        *vector_columns("control_torque_B", "Nm"),
        *vector_columns("applied_torque_B", "Nm"),
        *[f"{prefix}_{i}" for prefix in ("sigma_BN", "sensor_state_sigma_BN", "nav_sigma_BN", "held_sigma_BN")
          for i in (1, 2, 3)],
    }
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f"Missing Phase 2B telemetry columns: {', '.join(missing)}")
    if len(df) < 2 or not finite_numeric(df):
        raise ValueError("Detumble telemetry must have at least two finite rows")
    if df["telemetry_schema_version"].nunique() != 1 or df["telemetry_schema_version"].iloc[0] not in (3, 4):
        raise ValueError("Unsupported or mixed telemetry schema; rerun the detumble scenario")
    source = df["applied_torque_source"].iloc[0]
    if df["applied_torque_source"].nunique() != 1 or source not in (
            "ExtForceTorque.torqueExternalPntB_B", "MtbEffector.torqueExternalPntB_B"):
        raise ValueError("Applied torque must identify one recorded dynamics effector")
    extra_times = []
    if df["telemetry_schema_version"].iloc[0] == 4:
        native = source.startswith("MtbEffector.")
        metadata = {"actuator_mode", "command_mode", "applied_torque_evaluation"}
        if not metadata.issubset(df.columns):
            raise ValueError("Missing schema 4 actuator contract")
        if (not (df["actuator_mode"] == ("native" if native else "direct")).all()
                or not (df["applied_torque_evaluation"] == ("final_RK4_stage" if native else "constant_body_hold")).all()
                or df["command_mode"].nunique() != 1 or df["command_mode"].iloc[0] not in ("controller", "replay")):
            raise ValueError("Inconsistent actuator/command/evaluation contract")
    if source.startswith("MtbEffector."):
        if not (df["telemetry_schema_version"] == 4).all():
            raise ValueError("Native torque requires schema 4 stage and input evidence")
        extra_times = ["native_output_time_ns", "native_input_dipole_time_ns", "native_input_field_time_ns"]
        required_native = {*extra_times, *vector_columns("native_mtbNetTorque_B", "Nm"),
                           *vector_columns("mtb_max_dipole", "Am2"),
                           *[f"mtb_Gt_B_{i}{j}" for i in (1, 2, 3) for j in (1, 2, 3)]}
        if not required_native.issubset(df.columns):
            raise ValueError("Missing native output/input/configuration evidence")
    for col in ["time_ns", "control_step_ns", "wmm_coefficient_time_ns", *PUBLICATION_TIME_COLUMNS, *HELD_TIME_COLUMNS, *extra_times]:
        values = df[col].to_numpy(dtype=float)
        if not np.isfinite(values).all() or not np.equal(values, np.rint(values)).all():
            raise ValueError(f"{col} must contain finite integer nanoseconds")


def telemetry_metrics(df: pd.DataFrame, config=None) -> dict[str, Any]:
    """Independent record-to-record checks, with explicit acquisition epochs."""
    require_detumble_telemetry(df)
    config = config_for_telemetry(df, config)
    step_ns = round(config.timing.control_step.value * 1e9)
    inertia = np.asarray(config.spacecraft.inertia.value)
    ticks = df["time_ns"].to_numpy(dtype=np.int64)
    source_ticks = df["sensor_state_time_ns"].to_numpy(dtype=np.int64)
    publication_aligned = all(np.array_equal(df[c].to_numpy(), ticks)
                              for c in PUBLICATION_TIME_COLUMNS)
    timing_valid = (ticks[0] == 0 and np.all(np.diff(ticks) > 0)
                    and np.all(ticks % step_ns == 0)
                    and (df["control_step_ns"] == step_ns).all()
                    and np.array_equal(df["application_interval_valid"], ticks > 0)
                    and all(np.array_equal(df[c], np.maximum(ticks - step_ns, 0))
                            for c in HELD_TIME_COLUMNS)
                    and np.array_equal(df["wmm_coefficient_time_ns"],
                                       ((ticks + 500_000_000) // 1_000_000_000) * 1_000_000_000)
                    and publication_aligned
                    and np.allclose(df["time_s"], ticks * 1e-9, rtol=0, atol=1e-9)
                    and np.allclose(df["nav_time_tag_s"], ticks * 1e-9, rtol=0, atol=1e-9))
    sigma = vec3(df, [f"sensor_state_sigma_BN_{i}" for i in (1, 2, 3)])
    B_N = vec3(df, vector_columns("B_N", "T"))
    B_B = vec3(df, vector_columns("B_B", "T"))
    reconstructed_B_B = rotate_inertial_to_body(sigma, B_N)
    b_error = np.linalg.norm(B_B - reconstructed_B_B, axis=1)
    b_angle = np.degrees(np.arctan2(np.linalg.norm(np.cross(B_B, reconstructed_B_B), axis=1),
                                  np.sum(B_B * reconstructed_B_B, axis=1)))
    b_norm_error = np.abs(np.linalg.norm(B_N, axis=1) - np.linalg.norm(B_B, axis=1))
    m = vec3(df, vector_columns("mcmd", "Am2"))
    applied = vec3(df, vector_columns("applied_torque_B", "Nm"))
    expected = vec3(df, vector_columns("control_torque_B", "Nm"))
    # Reconstruct the field from a separate WMM record and the actual consumed
    # truth attitude. Never use controller history or its computed torque here.
    current_command_error = np.linalg.norm(expected - np.cross(m, reconstructed_B_B), axis=1)
    held_sigma = vec3(df, [f"held_sigma_BN_{i}" for i in (1, 2, 3)])
    held_field_n = vec3(df, vector_columns("held_B_N", "T"))
    held_b = rotate_inertial_to_body(held_sigma, held_field_n)
    held_m = vec3(df, vector_columns("held_mcmd", "Am2"))
    held_expected = vec3(df, vector_columns("held_control_torque_B", "Nm"))
    interval_rows = ticks > 0  # initialization has no preceding command/field or applied interval
    cross_error = np.linalg.norm(applied[interval_rows] - np.cross(held_m, held_b)[interval_rows], axis=1)
    command_error = np.linalg.norm(applied[interval_rows] - held_expected[interval_rows], axis=1)
    pre_omega = vec3(df, vector_columns("held_omega_B", "rad_s"))
    input_omega = vec3(df, vector_columns("sensor_state_omega_B", "rad_s"))
    post_omega = vec3(df, vector_columns("omega_B", "rad_s"))
    nav_omega = vec3(df, vector_columns("nav_omega_B", "rad_s"))
    nav_sigma = vec3(df, [f"nav_sigma_BN_{i}" for i in (1, 2, 3)])
    dt = (df["state_time_ns"].to_numpy() - df["held_state_time_ns"].to_numpy()) * 1e-9
    propagated = dt > 0
    prediction = predict_body_rate(pre_omega[propagated], applied[propagated], dt[propagated], inertia)
    source = str(df["applied_torque_source"].iloc[0])
    native = source.startswith("MtbEffector.")
    native_metrics = {}
    power_omega = post_omega.copy()
    if native:
        limits = vec3(df, vector_columns("mtb_max_dipole", "Am2"))
        axes = df[[f"mtb_Gt_B_{i}{j}" for i in (1, 2, 3) for j in (1, 2, 3)]].to_numpy().reshape(-1, 3, 3)
        mapped_m = np.einsum("nij,nj->ni", axes, np.clip(held_m, -limits, limits))
        predicted_state, stage, predicted_torque = predict_magnetic_step(
            held_sigma[propagated], pre_omega[propagated], mapped_m[propagated],
            held_field_n[propagated], dt[propagated], inertia)
        # Torque readback is k4, NOT tau(t-dt) and NOT tau(accepted_state(t)).
        cross_error = np.linalg.norm(applied[propagated] - predicted_torque, axis=1)
        prediction = predicted_state[:, 3:]
        power_omega[propagated] = stage[:, 3:]
        timing_valid = (timing_valid and np.array_equal(df["native_output_time_ns"], ticks)
                        and all(np.array_equal(df[c], np.maximum(ticks - step_ns, 0))
                                for c in ("native_input_dipole_time_ns", "native_input_field_time_ns")))
        native_metrics = {
            # These compare two exports of ONE native quantity: transport only,
            # never described as an independent magnetic-physics validation.
            "native_output_transport_error_Nm": float(np.linalg.norm(
                vec3(df, vector_columns("native_mtbNetTorque_B", "Nm")) - applied, axis=1).max()),
            "magnetic_step_max_mrp_error": float(np.linalg.norm(
                vec3(df, [f"sigma_BN_{i}" for i in (1, 2, 3)])[propagated] - predicted_state[:, :3], axis=1).max()),
            "native_configuration_unchanged": bool(np.all(limits == config.magnetorquers.dipole_limits.value)
                                                     and np.all(axes == config.magnetorquers.axes_B.value)),
            "native_command_within_provisional_limits": bool(np.all(np.abs(m) <= limits + 1e-14)),
            "rate_prediction_source": "Independent coupled MRP/Euler RK4, recorded dipole/B_N; native final-stage torque checked separately",
            "mechanical_power_epoch": "Final RK4 stage; stage rate independently reconstructed",
        }
    step_error = np.linalg.norm(post_omega[propagated] - prediction, axis=1)
    torque_mag = np.linalg.norm(applied, axis=1)
    # Instantaneous power at the recorded torque evaluation: final RK stage for
    # native, accepted endpoint for direct; not a finite-difference energy balance.
    power = np.sum(power_omega * applied, axis=1)
    nonzero = torque_mag > 1e-14
    earth = df[[f"earth_C_PN_{i}{j}" for i in (1, 2, 3) for j in (1, 2, 3)]].to_numpy().reshape(-1, 3, 3)
    et = df["earth_orientation_tdb_s"].to_numpy(dtype=float)
    # Calendar difference 2000-01-01 noon -> 2026-01-01 midnight = 9496.5 days.
    # UTC->TT is 69.184 s; NAIF DELTET periodic TDB-TT is bounded by 1.657 ms.
    calendar_seconds = (datetime.fromisoformat(config.environment.epoch_utc.value)
                        - datetime(2000, 1, 1, 12, tzinfo=timezone.utc)).total_seconds()
    clock_error = np.abs(et - (calendar_seconds + ticks * 1e-9 + 69.184))
    orientation_valid = bool((df["earth_orientation_enabled"] == 1).all()
                             and (df["earth_orientation_model"] == config.environment.earth_orientation_model.value).all()
                             and np.max(clock_error) < 0.0017)
    earth_error = np.linalg.norm(earth - reference_earth_matrix(et), axis=(1, 2))
    return {
        "telemetry_timing_valid": bool(timing_valid),
        "max_sensor_state_age_s": float(np.max(ticks - source_ticks) * 1e-9),
        "max_B_frame_vector_error_T": float(b_error.max()),
        "max_B_frame_direction_error_deg": float(b_angle.max()),
        "max_B_frame_norm_error_T": float(b_norm_error.max()),
        "mean_B_frame_norm_error_T": float(b_norm_error.mean()),
        "earth_orientation_valid": orientation_valid,
        "max_earth_model_matrix_error": float(earth_error.max()),
        "max_earth_clock_tt_offset_s": float(clock_error.max()),
        "max_nav_rate_error_rad_s": float(np.linalg.norm(nav_omega - input_omega, axis=1).max()),
        "max_nav_mrp_error": float(np.linalg.norm(nav_sigma - sigma, axis=1).max()),
        "torque_source": source,
        "command_mode": str(df["command_mode"].iloc[0]) if "command_mode" in df else "controller",
        "torque_cross_product_mean_abs_error_Nm": float(cross_error.mean()),
        "torque_cross_product_max_abs_error_Nm": float(cross_error.max()),
        "torque_cross_product_max_rel_error": float(cross_error.max() / max(float(torque_mag.max()), 1e-30)),
        "expected_vs_applied_max_error_Nm": float(command_error.max()),
        "current_command_cross_product_max_error_Nm": float(current_command_error.max()),
        "rigid_body_step_rows": int(np.count_nonzero(propagated)),
        "rigid_body_step_max_rate_error_rad_s": float(step_error.max()) if len(step_error) else None,
        "mean_applied_magnetic_torque_Nm": float(torque_mag.mean()),
        "peak_applied_magnetic_torque_Nm": float(torque_mag.max()),
        "mean_mechanical_control_power_W": float(power.mean()),
        "min_mechanical_control_power_W": float(power.min()),
        "max_mechanical_control_power_W": float(power.max()),
        "negative_mechanical_power_fraction": float(np.mean(power[nonzero] < 0)) if np.any(nonzero) else None,
        **native_metrics,
    }


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


def rotational_energy_J(omega_B_rad_s: np.ndarray, inertia=INERTIA_DIAG_KG_M2) -> np.ndarray:
    """Rigid-body rotational kinetic energy using the scenario diagonal inertia."""
    inertia = np.asarray(inertia)
    if inertia.ndim == 2 and np.array_equal(inertia, np.diag(np.diag(inertia))):
        inertia = np.diag(inertia)
    if inertia.ndim == 1:
        return 0.5 * np.sum((omega_B_rad_s**2) * inertia.reshape((1, 3)), axis=1)
    return 0.5 * np.sum(omega_B_rad_s * (omega_B_rad_s @ inertia.T), axis=1)


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


def summarize_basilisk(df: pd.DataFrame, config=None) -> dict[str, Any]:
    config = config_for_telemetry(df, config)
    independent = telemetry_metrics(df, config)
    t = df["time_s"].to_numpy(dtype=float)
    omega = vec3(df, ["omega_B_x_rad_s", "omega_B_y_rad_s", "omega_B_z_rad_s"])
    wmag = np.linalg.norm(omega, axis=1)

    energy = rotational_energy_J(omega, config.spacecraft.inertia.value)
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
    if all(c in df.columns for c in ["B_B_x_T", "B_B_y_T", "B_B_z_T"]):
        bmag = mag3(df, ["B_B_x_T", "B_B_y_T", "B_B_z_T"])
        out["mean_B_body_T"] = float(bmag.mean())
        out["peak_B_body_T"] = float(bmag.max())

    out.update(independent)
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
        bsk.get("torque_cross_product_max_abs_error_Nm", float("inf")) < 1e-12,
        {
            "max_abs_error_Nm": bsk.get("torque_cross_product_max_abs_error_Nm"),
            "max_rel_error": bsk.get("torque_cross_product_max_rel_error"),
        },
        "max abs < 1e-12 Nm; recorded effector versus m_command x reconstructed B_body",
    )
    checks["telemetry_epochs_aligned"] = check(
        "telemetry_epochs_aligned", bsk.get("telemetry_timing_valid") is True,
        bsk.get("max_sensor_state_age_s"),
        "Current state/orientation/field/sensor epochs equal; held command/field epoch t-0.1 s",
    )
    # Numerical consistency tolerances for the current ideal software baseline;
    # these are not sensor accuracy, hardware calibration or flight requirements.
    for name, metric, tolerance in [
        ("B_frame_vector_aligned", "max_B_frame_vector_error_T", 1e-12),
        ("nav_rate_matches_consumed_state", "max_nav_rate_error_rad_s", 1e-12),
        ("nav_attitude_matches_consumed_state", "max_nav_mrp_error", 1e-12),
        ("effector_matches_torque_message", "expected_vs_applied_max_error_Nm", 1e-12),
        ("applied_torque_predicts_rate_step", "rigid_body_step_max_rate_error_rad_s", 1e-10),
        ("current_command_matches_m_cross_B", "current_command_cross_product_max_error_Nm", 1e-12),
        ("earth_orientation_matches_IAU_model", "max_earth_model_matrix_error", 1e-10),
    ]:
        value = bsk.get(metric)
        checks[name] = check(name, value is not None and value < tolerance, value, f"< {tolerance}")
    checks["earth_orientation_present_and_epoch_valid"] = check(
        "earth_orientation_present_and_epoch_valid", bsk.get("earth_orientation_valid") is True)
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
    if bsk.get("torque_source", "").startswith("MtbEffector."):
        # The controller torque is evaluated at the PREVIOUS acquisition state;
        # equality with a final-stage native torque would be physically wrong.
        del checks["effector_matches_torque_message"]
        checks["independent_magnetic_inputs_predict_rate_step"] = checks.pop("applied_torque_predicts_rate_step")
        for name, metric, limit in [
            ("native_output_transport", "native_output_transport_error_Nm", 1e-15),
            ("independent_magnetic_inputs_predict_attitude_step", "magnetic_step_max_mrp_error", 1e-12),
        ]:
            value = bsk.get(metric)
            checks[name] = check(name, value is not None and value < limit, value, f"< {limit}")
        for name in ("native_configuration_unchanged", "native_command_within_provisional_limits"):
            checks[name] = check(name, bsk.get(name) is True)
    if bsk.get("command_mode") == "replay":
        # An open-loop replay is an actuator experiment, not a closed-loop
        # detumble test. Controller diagnostic power is not the replay power.
        for name in ("basilisk_final_omega_less_than_initial", "basilisk_final_energy_less_than_initial",
                     "basilisk_mean_mechanical_power_negative", "peak_coil_power_within_expected_limit"):
            del checks[name]
    return {
        "passed": all(item["passed"] for item in checks.values()),
        "checks": checks,
    }


def main(bsk_csv=None, report_path=None) -> int:
    global COMPARE_OUT
    if report_path is not None:
        COMPARE_OUT = Path(report_path)
    if not REF.exists():
        maybe_remove_stale_comparison()
        raise SystemExit(f"Reference CSV not found: {REF}")
    bsk_csv = OUT_DATA / "detumble_output.csv" if bsk_csv is None else Path(bsk_csv)
    if not bsk_csv.exists():
        maybe_remove_stale_comparison()
        raise SystemExit("Detumble output not found. Run scenario_huskysat2_detumble.py first.")

    try:
        ref_df = pd.read_csv(REF)
        bsk_df = pd.read_csv(bsk_csv)
        if "simulation_config_sha256" in bsk_df:
            config_path = bsk_csv.with_name(bsk_csv.stem + "_config.json")
            bsk_df.attrs["simulation_config"] = HS2SimConfig.load(config_path).to_dict()
        ref = summarize_reference(ref_df)
        bsk = summarize_basilisk(bsk_df)
    except (ValueError, KeyError, IndexError, OSError) as exc:
        failure = {"reference_file": str(REF), "basilisk_file": str(bsk_csv),
                   "validation": {"passed": False, "error": str(exc)}}
        COMPARE_OUT.parent.mkdir(parents=True, exist_ok=True)
        COMPARE_OUT.write_text(json.dumps(failure, indent=2), encoding="utf-8")
        print(json.dumps(failure, indent=2))
        return 2
    validation = build_validation(ref, bsk)

    comparison = {
        "reference_file": str(REF),
        "basilisk_file": str(bsk_csv),
        "reference": ref,
        "basilisk": bsk,
        "historical_phase2a_window": {
            "first_crossing_0p05_s": bsk.get("detumble_time_0p05_s"),
            "old_window_s": [3000.0, 3500.0],
            "note": "Informational only: corrected Earth orientation changes the field and trajectory; this was never an HS-2 requirement.",
        },
        "validation": validation,
        "notes": [
            "This comparison is physical-metric based, not CSV schema exact.",
            "Frame, nav, torque-readback and rate-step checks use separate recorded sources with explicit epochs.",
            "Native MtbEffector readback is final-RK4-stage torque. The independent coupled MRP/Euler integration predicts that torque and the accepted rate/attitude from recorded dipole, inertial field and pre-state. Native message versus dynamics-field equality is only an output-transport check.",
            "Direct-reference mode holds body torque constant; its rate-step check uses recorded effector torque. Native/direct end-stage differences from the earlier controller torque are expected. Initialization is excluded from application checks.",
            "State, Earth orientation, WMM and sensors now share the current epoch. Commands apply over the following interval; held records validate the completed interval.",
            "The explicit low-order IAU Earth model is not measured ITRF/EOP orientation. WMM secular-variation time is rounded internally to whole seconds by Basilisk 2.10.2.",
            "Field magnitude, coil power and decreasing rate/energy are development sanity checks, not HS-2 requirements verification. The historical 3000-3500 s window is informational only.",
            "Reference summaries are informational; these checks do not establish standalone equivalence or HS-2 flight performance.",
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
    return 0 if validation["passed"] else 1


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--basilisk-csv", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    raise SystemExit(main(args.basilisk_csv, args.report))
