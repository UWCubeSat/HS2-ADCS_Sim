"""7G-2A: one live, persistent mid-burst fault. TEST-ONLY, 2026-10-01.

MEKF CLOSED-LOOP FAULT INHIBITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED.
Reuse the 7F-2C supported gyro_valid=False event at priority 588, t=0.8 s.
The existing estimator fault remains latched; no reset/reacquisition is injected.
Three seconds cover prior real actuation and two subsequent quiet/sample/burst
cycles. All tolerances are numerical diagnostics, not flight latency requirements.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np

from analyze_physical_profile_dynamics import rk_work
from attitude_mekf_adapter import ShadowOptions
from compare_reference_vs_basilisk import predict_magnetic_step
from disconnected_command_observer import ObserverOptions
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_disconnected_command_chain import LiveFixtures
from validate_mekf_closed_loop import csv_bytes, mrp, preservation, vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FAULT_NS = 800_000_000
STEP_NS = 100_000_000
DURATION_S = 3.0
SCOPE = "MEKF CLOSED-LOOP FAULT INHIBITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED"


def assess(frame, nominal, events):
    """Record-to-record live checks, never fabricate actuator evidence from math."""
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    records = frame.attrs["disconnected_command_records"]
    chain = records["telemetry"]
    owners = frame.attrs["mekf_command_owner"]
    trace = frame.attrs["disconnected_command_trace"]
    status = frame.attrs["shadow_status"]
    consumer = frame.attrs["navigation_consumer"]
    cycle = diagnostic_cycle_config()
    m = vector(frame, "mcmd", "Am2")
    held = vector(frame, "held_mcmd", "Am2")
    native = vector(frame, "native_mtbNetTorque_B", "Nm")
    applied = vector(frame, "applied_torque_B", "Nm")
    w, s = vector(frame, "omega_B", "rad_s"), mrp(frame)
    field = vector(frame, "held_B_N", "T")
    held_w, held_s = vector(frame, "held_omega_B", "rad_s"), mrp(frame, "held_sigma_BN")
    at_fault = int(np.flatnonzero(ticks == FAULT_NS)[0])
    after = ticks >= FAULT_NS
    post_interval = ticks[1:] > FAULT_NS
    burst = ticks % cycle.period_ns >= cycle.actuation_offset_ns
    sample = ticks % cycle.period_ns == cycle.sample_offset_ns
    active_before = (ticks >= 600_000_000) & (ticks < FAULT_NS)
    cfg = frame.attrs["simulation_config"]
    inertia = np.array(cfg["spacecraft"]["inertia"]["value"], dtype=float)
    dt = np.diff(ticks)*1e-9
    predicted, _, torque = predict_magnetic_step(held_s[1:], held_w[1:], held[1:], field[1:], dt, inertia)
    work, _, _, _, _ = rk_work(held_s[1:], held_w[1:], held[1:], field[1:], dt, inertia)
    energy = .5*np.sum(w*(w @ inertia.T), axis=1)
    state_error = float(np.max(np.abs(np.column_stack((s[1:], w[1:]))-predicted)))
    torque_error = float(np.max(np.linalg.norm(native[1:]-torque, axis=1)))
    work_error = float(np.max(np.abs(np.diff(energy)-work.sum(axis=1))))
    expected_rows = DURATION_S*1e9//STEP_NS+1
    boundary = owners[at_fault]
    first_health = next((r["tick"] for r in trace if r["tick"] >= FAULT_NS and not r["health"].healthy), None)
    first_zero = next((o["publication_epoch_ns"] for o in owners
                       if o["publication_epoch_ns"] >= FAULT_NS and not any(o["native_input_dipole_Am2"])), None)
    first_zero_readback = next((int(t) for t, h, tau in zip(ticks, held, native)
                               if t > FAULT_NS and not np.any(h) and not np.any(tau)), None)
    checks = {
        "complete_tick_evidence": len(frame) == len(chain) == len(owners) == len(trace) == len(status) == expected_rows
            and np.array_equal(ticks, np.arange(0, int(DURATION_S*1e9)+1, STEP_NS)),
        "one_existing_fault_no_reset": events == [(FAULT_NS, 588, "fault_to_adapter")]
            and all(not r["reset"] for r in trace) and all(r["acquisitions"] <= 1 for r in status),
        "identical_nominal_configuration_and_pre_fault_state": cfg == nominal.attrs["simulation_config"]
            and frame.attrs["magnetic_cycle"] == nominal.attrs["magnetic_cycle"]
            and frame.attrs["shadow_options"] == nominal.attrs["shadow_options"]
            and np.array_equal(w[:at_fault+1], vector(nominal, "omega_B", "rad_s")[:at_fault+1])
            and np.array_equal(s[:at_fault+1], mrp(nominal)[:at_fault+1]),
        "pre_fault_real_nonzero_actuation": bool(np.all(np.any(m[active_before], axis=1))
            and np.any(native[at_fault-1]) and np.any(native[at_fault]))
            and all(status[i]["valid"] and chain[i]["command_usable"]
                    for i in np.flatnonzero(active_before)),
        "live_fault_seen_at_first_boundary": first_health == first_zero == FAULT_NS
            and not chain[at_fault]["command_usable"] and status[at_fault]["fault"] == "7F2C_TEST_ONLY_fault"
            and "estimator_fault:" in boundary["inhibition_reason"],
        "existing_numeric_command_revoked_not_erased": chain[at_fault]["mathematical_command_valid"]
            and chain[at_fault]["command_computation_ns"] == 500_000_000
            and chain[at_fault]["sample_epoch_ns"] == 400_000_000
            and chain[at_fault]["command_id"] == chain[at_fault-1]["command_id"]
            and any(chain[at_fault]["clipped_dipole_B_Am2"]),
        "owner_and_native_zero_on_every_faulted_boundary": all(not any(o["published_dipole_Am2"])
            and not any(o["native_input_dipole_Am2"]) and not any(o["effective_dipole_B_Am2"])
            and not o["command_usable"] for o in owners[at_fault:]) and not np.any(m[after]),
        "fresh_zero_publication_and_single_owner": len({o["publisher_module_id"] for o in owners}) == 1
            and all(o["publication_epoch_ns"] == o["subscriber_epoch_ns"] == o["gate_epoch_ns"] == t
                and o["publisher_module_id"] == o["subscriber_module_id"]
                and not o["unused_cycle_message_written"] for t, o in zip(ticks, owners)),
        "independent_message_record_equals_native_readback_and_gate": all(np.array_equal(cmd, o["native_input_dipole_Am2"])
            and np.array_equal(cmd, o["published_dipole_Am2"])
            and np.array_equal(cmd, r["clipped_dipole_B_Am2"] if r["command_usable"] else np.zeros(3))
            for cmd, o, r in zip(m, owners, chain)),
        "held_input_confirms_next_interval": np.array_equal(held[1:], m[:-1])
            and np.array_equal(frame.native_input_dipole_time_ns.to_numpy()[1:], ticks[:-1]),
        "native_torque_zero_for_all_inhibited_intervals": not np.any(native[1:][post_interval])
            and not np.any(applied[1:][post_interval]) and np.array_equal(native, applied)
            and first_zero_readback == FAULT_NS+STEP_NS,
        "persistent_fault_all_bursts_inhibited": all(not r["source_healthy"] and not r["command_usable"]
            and r["lifecycle_state"] == "FAULTED" and r["inhibition_reason"].startswith("estimator_fault:")
            for r, t, is_burst in zip(chain, ticks, burst) if t >= FAULT_NS and is_burst)
            and all(not st["valid"] and st["fault"] == "7F2C_TEST_ONLY_fault" for st in status[at_fault:]),
        "no_fallback_while_simplenav_still_available": all(c["requested_source"] == "MEKF"
            and c["selected_source"] == "NONE" for c in consumer[at_fault:])
            and all(o["owner"] == "MEKF_DEVELOPMENT" for o in owners)
            and np.array_equal(frame.nav_message_time_ns.to_numpy(), ticks),
        "quiet_sample_and_faulted_actuation_zero": not np.any(m[~burst]) and not np.any(m[after & burst]),
        "valid_TAM_during_persistent_fault": frame.cycle_sample_epoch_ns.to_numpy()[sample].tolist()
            == [400_000_000, 1_400_000_000, 2_400_000_000]
            and frame.cycle_sample_valid.to_numpy(dtype=bool)[sample].all()
            and not np.any(frame.cycle_rejected_sample_count.to_numpy(dtype=int))
            and all(r["stored_tam_valid"] and r["stored_tam_quiet_valid"] for r, yes in zip(chain, sample) if yes),
        "quiet_history_from_actual_zero_command": np.array_equal(
            frame.cycle_time_since_disabled_ns.to_numpy(dtype=np.int64)[after], ticks[after]-FAULT_NS),
        "no_false_coherent_snapshot_after_failed_capture": all(r["sample_epoch_ns"] is None
            and not captured and label.startswith("NO_CURRENT_COHERENT_CONTROL_SNAPSHOT")
            for r, t, captured, label in zip(chain, ticks, frame.cycle_control_snapshot_captured,
                                            frame.cycle_navigation_source) if t >= cycle.period_ns),
        "finite_truth": np.isfinite(np.column_stack((w, s, vector(frame, "r_N", "m")))).all(),
        "continuous_independent_rigid_body_response": state_error < 1e-12
            and np.array_equal(held_w[1:], w[:-1]) and np.array_equal(held_s[1:], s[:-1]),
        "independent_native_torque": torque_error < 1e-15,
        "energy_work_balance": work_error < 1e-10,
        "continues_rotating_without_new_magnetic_torque": float(np.linalg.norm(w[-1])) > .1
            and not np.any(work[post_interval]),
        "unchanged_live_stage_order": all([e[1] for e in records["execution_order"] if e[0] == t]+[o["priority"]]
            == ([595, 587, 554, 552, 550, 548] if t % cycle.period_ns == cycle.compute_offset_ns
                else [595, 587, 554, 550, 548]) for t, o in zip(ticks, owners))}
    checks = {key: bool(value) for key, value in checks.items()}
    epoch = {"fault_event_ns": FAULT_NS, "first_health_detection_ns": first_health,
        "first_zero_publication_ns": first_zero, "first_zero_native_input_ns": boundary["subscriber_epoch_ns"],
        "first_zero_interval_start_ns": first_zero, "first_zero_interval_end_ns": first_zero_readback,
        "first_zero_native_torque_record_ns": first_zero_readback,
        "event_to_health_ns": None if first_health is None else first_health-FAULT_NS,
        "event_to_zero_publication_ns": None if first_zero is None else first_zero-FAULT_NS,
        "event_to_zero_interval_start_ns": None if first_zero is None else first_zero-FAULT_NS,
        "event_to_completed_zero_interval_record_ns": None if first_zero_readback is None else first_zero_readback-FAULT_NS}
    return {"passed": all(checks.values()), "checks": checks, "epochs": epoch,
        "faulted_publications": int(after.sum()), "faulted_actuation_boundaries": int((after & burst).sum()),
        "pre_fault_actuation_s": float(dt[np.any(held[1:], axis=1)].sum()),
        "post_fault_zero_actuation_s": float(dt[post_interval].sum()),
        "native_torque_error_Nm": torque_error, "coupled_state_step_error": state_error,
        "interval_work_balance_error_J": work_error,
        "total_magnetic_work_J": float(work.sum()), "final_rate_rad_s": float(np.linalg.norm(w[-1])),
        "pre_fault": {"owner": owners[at_fault-1], "gate": chain[at_fault-1],
                      "native_torque_Nm": native[at_fault-1].tolist()},
        "first_inhibition": {"owner": boundary, "gate": chain[at_fault],
            "preceding_interval_native_torque_Nm": native[at_fault].tolist(),
            "next_interval_native_torque_Nm": native[at_fault+1].tolist()},
        "owner_trace": owners, "chain": records, "navigation_quality": status,
        "navigation_consumer": consumer, "health_observations": [asdict(r["health"]) for r in trace]}


def validate():
    fixture = LiveFixtures("mid_burst_fault")
    options = dict(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
        cycle=diagnostic_cycle_config(), shadow=ShadowOptions(ideal_sun=True), control_source="MEKF_DEVELOPMENT")
    with contextlib.redirect_stdout(io.StringIO()):
        nominal = run(**options)
        frame = run(**options, mekf_fault_test=ObserverOptions(fixture.attach,
            "7G-2A TEST-ONLY 2026-10-01: existing mid_burst_fault at 0.8 s; latched; no reset"))
        preserved = preservation()
    evidence = assess(frame, nominal, fixture.events)
    sources = ("scenario_huskysat2_detumble.py", "validate_closed_loop_inhibition.py", "test_closed_loop_inhibition.py",
        "validate_disconnected_command_chain.py", "mekf_command_owner.py", "magnetic_control_cycle.py",
        "disconnected_command_observer.py", "command_health_gate.py", "attitude_mekf_adapter.py")
    report = {"scope": SCOPE, "passed": evidence["passed"] and preserved["passed"],
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": {n: hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        "fault": {"fixture": "LiveFixtures('mid_burst_fault')", "field": "InputBatch.gyro_valid=False",
            "adapter_reason": "7F2C_TEST_ONLY_fault", "events": fixture.events,
            "persistence": "Existing adapter/navigation/health latches; subsequent good input does not clear them",
            "priority": 588, "reset_injected": False, "flight_latency_requirement": None},
        "duration_s": DURATION_S, "configuration": frame.attrs["simulation_config"],
        "cycle": frame.attrs["magnetic_cycle"], "sensor_fixture": frame.attrs["shadow_options"],
        "evidence": evidence, "production_preservation": preserved,
        "csv_sha256": hashlib.sha256(csv_bytes(frame)).hexdigest()}
    return report, frame, nominal, fixture.events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase7g2a_inhibition.json")
    args = parser.parse_args()
    report, frame, _, _ = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.with_suffix(".csv").write_bytes(csv_bytes(frame))
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "evidence": {k: v for k, v in report["evidence"].items()
        if k not in ("owner_trace", "chain", "navigation_quality", "navigation_consumer", "health_observations")},
        "production_preservation": report["production_preservation"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
