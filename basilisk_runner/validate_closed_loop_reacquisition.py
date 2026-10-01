"""7G-2B one closed-loop reset sequence, ASSUMED / TEST-ONLY, 2026-10-01.

No runtime/gate/estimator change. Reuse the existing InputBatch validity/reset
interface and owned immutable-envelope replay hook through ObserverOptions.
Keep the 0.7 s ideal-Sun cadence: simultaneous post-reset Sun/TAM at 7.4 s,
then a strictly later TAM at 8.4 s, computation 8.5 s and application 8.6 s.
Nine seconds suffice to include the resumed burst and its completed intervals.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict, replace
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np

from analyze_physical_profile_dynamics import rk_work
from attitude_mekf_adapter import DevelopmentChannel, InputBatch, ShadowOptions
from compare_reference_vs_basilisk import predict_magnetic_step
from disconnected_command_observer import ObserverOptions, PendingCalculation, Stage
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_mekf_closed_loop import csv_bytes, mrp, preservation, vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
STEP_NS = 100_000_000
FAULT_NS, RESET_NS = 800_000_000, 1_000_000_000
REACQUIRE_NS, REPLAY_NS = 7_400_000_000, 7_700_000_000
FRESH_SAMPLE_NS, FRESH_COMPUTE_NS, RESUME_NS = 8_400_000_000, 8_500_000_000, 8_600_000_000
DURATION_S = 9.0
SCOPE = "MEKF CLOSED-LOOP RESET / REACQUISITION / DEVELOPMENT TEST / NOT FLIGHT VALIDATED"


class ResetSequenceFixture:
    """Same 7F-2C interface events, now with real MEKF actuator authority.

    Only delivery of the existing reset boolean clears the estimator fault.
    Do not call SysModel.Reset or reset counters, forge source health, change
    sample epochs, or modify the frozen old command to make its deadline newer.
    """
    def __init__(self):
        self.old_command: PendingCalculation | None = None
        self.events: list[tuple[int, int, str]] = []

    def attach(self, sim, inputs, adapter, observer):
        relay = DevelopmentChannel[InputBatch]()
        adapter.inputs = observer.health_inputs = relay

        def input_event(tick):
            batch = inputs.read()
            if tick == FAULT_NS:
                self.old_command = observer.pending
                if self.old_command is None:
                    raise ValueError("Reset test requires a real pre-fault calculation")
                batch = replace(batch, gyro_valid=False, gyro_reason="7F2C_TEST_ONLY_fault")
                self.events.append((tick, 588, "fault_to_adapter"))
            if tick == RESET_NS:
                batch = replace(batch, reset_acquisition=True)
                self.events.append((tick, 588, "reset_to_adapter_and_gate"))
            relay.write(batch, tick)

        def replay_event(tick):
            if tick == REPLAY_NS:
                if self.old_command is None:
                    raise ValueError("Missing immutable pre-reset envelope")
                observer.pending = self.old_command
                self.events.append((tick, 556, "replay_unchanged_pre_reset_envelope"))

        for priority, callback in ((588, input_event), (556, replay_event)):
            model = Stage(callback)
            model.ModelTag = f"TEST_ONLY_ResetSequence{priority}"
            observer.models.append(model)
            sim.AddModelToTask("DynamicsTask", model, ModelPriority=priority)


def assess(frame, fixture):
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    chain_records = frame.attrs["disconnected_command_records"]
    chain, trace = chain_records["telemetry"], frame.attrs["disconnected_command_trace"]
    owners, status = frame.attrs["mekf_command_owner"], frame.attrs["shadow_status"]
    consumer = frame.attrs["navigation_consumer"]
    indexes = {int(t): i for i, t in enumerate(ticks)}
    fault, reset, reacq, replay, fresh, compute, resume = (
        indexes[t] for t in (FAULT_NS, RESET_NS, REACQUIRE_NS, REPLAY_NS, FRESH_SAMPLE_NS, FRESH_COMPUTE_NS, RESUME_NS))
    m, held = vector(frame, "mcmd", "Am2"), vector(frame, "held_mcmd", "Am2")
    native, applied = vector(frame, "native_mtbNetTorque_B", "Nm"), vector(frame, "applied_torque_B", "Nm")
    w, s = vector(frame, "omega_B", "rad_s"), mrp(frame)
    held_w, held_s = vector(frame, "held_omega_B", "rad_s"), mrp(frame, "held_sigma_BN")
    field = vector(frame, "held_B_N", "T")
    dt = np.diff(ticks)*1e-9
    inertia = np.array(frame.attrs["simulation_config"]["spacecraft"]["inertia"]["value"], dtype=float)
    predicted, _, torque = predict_magnetic_step(held_s[1:], held_w[1:], held[1:], field[1:], dt, inertia)
    work, _, _, _, _ = rk_work(held_s[1:], held_w[1:], held[1:], field[1:], dt, inertia)
    energy = .5*np.sum(w*(w @ inertia.T), axis=1)
    state_error = float(np.max(np.abs(np.column_stack((s[1:], w[1:]))-predicted)))
    torque_error = float(np.max(np.linalg.norm(native[1:]-torque, axis=1)))
    work_error = float(np.max(np.abs(np.diff(energy)-work.sum(axis=1))))
    inhibited = (ticks >= FAULT_NS) & (ticks < RESUME_NS)
    zero_intervals = (ticks > FAULT_NS) & (ticks <= RESUME_NS)
    cycle = diagnostic_cycle_config()
    burst = ticks % cycle.period_ns >= cycle.actuation_offset_ns
    sample = ticks % cycle.period_ns == cycle.sample_offset_ns
    new_generation = (1, 2, 1)
    old = fixture.old_command
    old_replay = trace[replay]["application"]
    fresh_command = trace[compute]["compute"][0]
    pre = chain[fault-1]
    checks = {
        "complete_live_evidence": len(frame) == len(chain) == len(trace) == len(owners) == len(status) == 91
            and np.array_equal(ticks, np.arange(0, 9_000_000_001, STEP_NS)),
        "one_fault_reset_and_replay_only": fixture.events == [(FAULT_NS, 588, "fault_to_adapter"),
            (RESET_NS, 588, "reset_to_adapter_and_gate"), (REPLAY_NS, 556, "replay_unchanged_pre_reset_envelope")]
            and [r["tick"] for r in trace if r["reset"]] == [RESET_NS],
        "real_pre_fault_actuation": pre["command_usable"] and pre["command_generation"] == (0, 1, 0)
            and np.any(m[fault-1]) and np.any(native[fault-1]) and np.any(native[fault]),
        "fault_removes_actual_command": not chain[fault]["command_usable"] and not np.any(m[fault])
            and not any(owners[fault]["native_input_dipole_Am2"]),
        "reset_changes_generation_and_keeps_zero": chain[reset]["reset_epoch_ns"] == RESET_NS
            and chain[reset]["source_generation"] == (1, 1, 1) and not np.any(m[reset])
            and not status[reset]["valid"] and status[reset]["acquisitions"] == 1,
        "reacquiring_is_not_valid_absolute_navigation": all(r["lifecycle_state"] == "REACQUIRING"
            and not r["source_healthy"] and not r["command_usable"] for r in chain[reset:reacq])
            and all(st["state"] == "UNINITIALIZED" and not st["valid"] for st in status[reset:reacq]),
        "exact_reacquisition_and_generation": [st["sim_epoch_ns"] for st in status if st["acquisition_event"]]
            == [400_000_000, REACQUIRE_NS] and status[reacq]["reacquisition_event"]
            and chain[reacq]["source_healthy"] and chain[reacq]["source_generation"] == new_generation,
        "reacquisition_alone_not_authority": not np.any(m[reacq:resume])
            and chain[reacq+2]["generation_match"] and chain[reacq+2]["cycle_match"]
            and chain[reacq+2]["mathematical_command_valid"]
            and chain[reacq+2]["inhibition_reason"] == "fresh_post_reacquisition_snapshot_required",
        "old_envelope_rejected_by_provenance_before_deadline_check": old is not None
            and old_replay is not None and old_replay[0] is old and old_replay[3].capture_time_valid
            and old_replay[3].mathematical_command_valid and old_replay[3].current_health.healthy
            and old_replay[3].inhibition_reason == "snapshot_predates_current_acquisition"
            and not chain[replay]["generation_match"] and chain[replay]["command_generation"] == (0, 1, 0)
            and chain[replay]["source_generation"] == new_generation and not np.any(m[replay]),
        "fresh_snapshot_exact_epoch_and_generation": fresh_command is not None
            and fresh_command.generation == new_generation
            and fresh_command.snapshot.tam_acquisition_ns == fresh_command.snapshot.navigation_state_ns
            == fresh_command.snapshot.navigation_publication_ns == fresh_command.snapshot.capture_ns == FRESH_SAMPLE_NS
            and chain[fresh]["stored_tam_valid"] and chain[fresh]["stored_tam_quiet_valid"]
            and fresh_command.command.evaluation_epoch_ns == FRESH_COMPUTE_NS
            and old is not None and fresh_command.command_id > old.command_id,
        "fresh_inputs_match_independent_native_records": fresh_command is not None
            and np.array_equal(fresh_command.snapshot.omega_BN_B_rad_s,
                               vector(frame, "mekf_nav_omega_B", "rad_s")[fresh])
            and np.array_equal(fresh_command.snapshot.sigma_BN, mrp(frame, "mekf_nav_sigma_BN")[fresh])
            and np.array_equal(fresh_command.snapshot.tam_B_T,
                np.array(frame.attrs["simulation_config"]["sensors"]["magnetometer_dcm_SB"]["value"]).T
                @ vector(frame, "tam_sample_B_B", "T")[fresh])
            and frame.mekf_nav_publication_ns.to_numpy()[fresh] == FRESH_SAMPLE_NS
            and frame.tam_message_time_ns.to_numpy()[fresh] == FRESH_SAMPLE_NS,
        "resumed_command_is_only_new_generation": chain[resume]["command_usable"]
            and chain[resume]["generation_match"] and chain[resume]["cycle_match"]
            and chain[resume]["command_generation"] == new_generation
            and owners[resume]["sample_epoch_ns"] == FRESH_SAMPLE_NS
            and owners[resume]["computation_epoch_ns"] == FRESH_COMPUTE_NS
            and owners[resume]["publication_epoch_ns"] == RESUME_NS and np.any(m[resume])
            and all(r["command_generation"] == new_generation and r["sample_epoch_ns"] == FRESH_SAMPLE_NS
                    for r in chain[fault:] if r["command_usable"]),
        "zero_persists_until_fresh_authority": not np.any(m[inhibited])
            and all(not any(o["native_input_dipole_Am2"]) and not o["command_usable"]
                    for o, yes in zip(owners, inhibited) if yes),
        "native_input_equals_gate_owner_and_message": all(np.array_equal(cmd, o["published_dipole_Am2"])
            and np.array_equal(cmd, o["native_input_dipole_Am2"])
            and np.array_equal(cmd, r["clipped_dipole_B_Am2"] if r["command_usable"] else np.zeros(3))
            for cmd, o, r in zip(m, owners, chain)),
        "single_owner_and_fresh_publications": len({o["publisher_module_id"] for o in owners}) == 1
            and all(o["publisher_module_id"] == o["subscriber_module_id"] and not o["unused_cycle_message_written"]
                and o["publication_epoch_ns"] == o["subscriber_epoch_ns"] == o["gate_epoch_ns"] == t
                for o, t in zip(owners, ticks)),
        "no_automatic_fallback": all(c["requested_source"] == "MEKF" and c["selected_source"] in ("MEKF", "NONE")
            for c in consumer) and all(c["selected_source"] == "NONE" for c in consumer[fault:reacq])
            and all(o["owner"] == "MEKF_DEVELOPMENT" for o in owners)
            and np.array_equal(frame.nav_message_time_ns.to_numpy(), ticks),
        "quiet_zero_and_TAM_not_corrupted": not np.any(m[~burst])
            and frame.cycle_sample_valid.to_numpy(dtype=bool)[sample].all()
            and not np.any(frame.cycle_rejected_sample_count.to_numpy(dtype=int))
            and all(r["stored_tam_valid"] and r["stored_tam_quiet_valid"] for r, yes in zip(chain, sample) if yes),
        "native_torque_off_then_resumes_next_interval": not np.any(native[zero_intervals])
            and np.any(native[resume+1]) and np.array_equal(native, applied)
            and np.array_equal(held[1:], m[:-1])
            and np.array_equal(frame.native_input_dipole_time_ns.to_numpy()[1:], ticks[:-1]),
        "native_torque_matches_independent_physics": torque_error < 1e-15,
        "finite_continuous_plant_through_reset_and_resume": np.isfinite(np.column_stack((s, w, vector(frame, "r_N", "m")))).all()
            and state_error < 1e-12 and np.array_equal(held_w[1:], w[:-1]) and np.array_equal(held_s[1:], s[:-1]),
        "energy_work_balance": work_error < 1e-10,
        "unchanged_stage_order": all([e[1] for e in chain_records["execution_order"] if e[0] == t]+[o["priority"]]
            == ([595, 587, 554, 552, 550, 548] if t % cycle.period_ns == cycle.compute_offset_ns
                else [595, 587, 554, 550, 548]) for t, o in zip(ticks, owners))}
    checks = {k: bool(v) for k, v in checks.items()}
    return {"passed": all(checks.values()), "checks": checks,
        "epochs_ns": {"fault": FAULT_NS, "reset": RESET_NS, "reacquisition": REACQUIRE_NS,
            "replay_attempt": REPLAY_NS, "fresh_sample": FRESH_SAMPLE_NS, "fresh_compute": FRESH_COMPUTE_NS,
            "first_resumed_command": RESUME_NS, "first_completed_resumed_torque": RESUME_NS+STEP_NS},
        "old_generation": old.generation if old else None, "new_generation": new_generation,
        "old_envelope": asdict(old) if old else None,
        "fresh_envelope": asdict(fresh_command) if fresh_command else None,
        "zero_command_duration_s": (RESUME_NS-FAULT_NS)*1e-9,
        "zero_publications_before_resume": int(inhibited.sum()),
        "reacquiring_boundaries": reacq-reset,
        "native_torque_error_Nm": torque_error, "coupled_state_step_error": state_error,
        "interval_work_balance_error_J": work_error, "resumed_dipole_Am2": m[resume].tolist(),
        "first_resumed_native_torque_Nm": native[resume+1].tolist(),
        "final_rate_rad_s": float(np.linalg.norm(w[-1])), "owner_trace": owners, "chain": chain_records,
        "navigation_quality": status, "navigation_consumer": consumer,
        "health_observations": [asdict(r["health"]) for r in trace],
        "replay_decision": asdict(old_replay[3]) if old_replay else None}


def validate():
    fixture = ResetSequenceFixture()
    with contextlib.redirect_stdout(io.StringIO()):
        frame = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), shadow=ShadowOptions(ideal_sun=True), control_source="MEKF_DEVELOPMENT",
            mekf_fault_test=ObserverOptions(fixture.attach, SCOPE+"; existing reset interface, 2026-10-01"))
        preserved = preservation()
    # This dedicated harness writes the lifecycle report; the existing scenario
    # fault-test filename/manifest are not used and no nominal artifact is replaced.
    frame.attrs["control_scope"] = SCOPE
    evidence = assess(frame, fixture)
    sources = ("validate_closed_loop_reacquisition.py", "test_closed_loop_reacquisition.py",
        "scenario_huskysat2_detumble.py", "mekf_command_owner.py", "magnetic_control_cycle.py",
        "disconnected_command_observer.py", "command_health_gate.py", "attitude_mekf_adapter.py",
        "attitude_navigation_consumer.py")
    report = {"scope": SCOPE, "passed": evidence["passed"] and preserved["passed"],
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": {n: hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        "events": fixture.events, "duration_s": DURATION_S, "configuration": frame.attrs["simulation_config"],
        "cycle": frame.attrs["magnetic_cycle"], "sensor_fixture": frame.attrs["shadow_options"],
        "generation_contract": "(observer reset serial, estimator acquisition count, observer health-revocation serial)",
        "evidence": evidence, "production_preservation": preserved,
        "csv_sha256": hashlib.sha256(csv_bytes(frame)).hexdigest()}
    return report, frame, fixture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase7g2b_reacquisition.json")
    args = parser.parse_args()
    report, frame, _ = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.with_suffix(".csv").write_bytes(csv_bytes(frame))
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "evidence": {k: v for k, v in report["evidence"].items()
        if k not in ("owner_trace", "chain", "navigation_quality", "navigation_consumer", "health_observations",
                     "old_envelope", "fresh_envelope", "replay_decision")},
        "production_preservation": report["production_preservation"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
