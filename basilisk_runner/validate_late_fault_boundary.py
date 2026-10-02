"""7G-2C deterministic late source-quality fault, TEST-ONLY, 2026-10-01.

Reuse 7F-2C's late_quality mechanism: replay an actual prior quality payload
WITH its original publication epoch. First expose it at priority 549, between
the 550 gate and 548 owner. Keep it exposed after each subsequent normal MEKF
publication. No forged fault/valid bit, extra estimator call, command write,
runtime change, sensor model, or flight latency requirement is introduced.

The source-interface fault is stale mandatory quality, NOT an internal MEKF
math fault. Normal adapter history is retained separately from exposed quality.
Equal simulation epochs do not imply simultaneous callbacks: ordered witnesses
and the actual Basilisk task table establish the decision/publication ordering.
"""
from __future__ import annotations

import argparse
import contextlib
from copy import deepcopy
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
from disconnected_command_observer import ObserverOptions, Stage
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_mekf_closed_loop import csv_bytes, mrp, preservation, vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
STEP_NS, FAULT_NS = 100_000_000, 800_000_000
DURATION_S = 3.0
SCOPE = "LATE-FAULT COMMAND-VALIDITY BOUNDARY / DEVELOPMENT TIMING TEST / NOT FLIGHT VALIDATED"


class LateQualityFixture:
    """Only quality-channel fault writes; all actuator access is read-only."""
    def __init__(self):
        self.events: list[dict] = []
        self.scheduler: list[dict] = []
        self.gates: list[dict] = []
        self.publications: list[dict] = []
        self.saved_quality = None
        self.before_fault = self.after_fault = self.approved_envelope = None

    def mark(self, tick, priority, event):
        row = dict(sequence=len(self.events), tick_ns=tick, priority=priority, event=event)
        self.events.append(row)
        return row

    def attach(self, sim, inputs, adapter, observer):
        # The native effector already exists; the owner is scheduled after this
        # hook returns. Inspect the completed C++ task table at the first tick.
        native = next(m for task in sim.TaskList for m in task.TaskModels if m.ModelTag == "NativeMTB")

        def expose_prior_quality(tick):
            if self.saved_quality is None:
                raise ValueError("Late fault requires a real prior source publication")
            payload, stamp = self.saved_quality
            adapter.statusOut.write(payload, stamp)

        def normal_publication(tick):
            if not self.scheduler:
                self.scheduler = [dict(task=task.TaskPtr.TaskName, period_ns=int(task.TaskPtr.TaskPeriod),
                    tag=model.ModelPtr.ModelTag, priority=int(model.CurrentModelPriority))
                    for process in sim.TotalSim.processList for task in process.processTasks
                    for model in task.TaskPtr.TaskModels]
            event = self.mark(tick, 579, "after_normal_MEKF_publication")
            event["normal_quality_epoch_ns"] = adapter.statusOut.time_written_ns
            if tick < FAULT_NS:
                self.saved_quality = adapter.statusOut.read(), adapter.statusOut.time_written_ns

        def persistent_fault(tick):
            # Match the existing late_quality fixture's 568 insertion: the
            # 570 truth/adapter monitor checks normal publication first, then
            # the mandatory consumer-facing quality channel becomes stale.
            if tick > FAULT_NS:
                expose_prior_quality(tick)
                self.mark(tick, 568, "persistent_stale_quality_exposed")

        def after_gate(tick):
            row = deepcopy(observer.history[-1])
            event = self.mark(tick, 549, "after_gate_before_fault_or_owner")
            self.gates.append(dict(event, gate=row, quality=adapter.statusOut.read(),
                quality_header_ns=adapter.statusOut.time_written_ns,
                nav_header_ns=observer.port.snapshot().header_ns))
            if tick != FAULT_NS:
                return
            pending = observer.pending
            application = observer.trace[-1]["application"]
            if pending is None or application is None or not row["command_usable"]:
                raise ValueError("Late-fault test requires an already-approved real command")
            self.approved_envelope = asdict(pending)
            self.before_fault = dict(event, decision=asdict(application[3]), gate=row,
                source_quality=adapter.statusOut.read(), quality_header_ns=adapter.statusOut.time_written_ns)
            expose_prior_quality(tick)
            fault = self.mark(tick, 549, "late_stale_quality_fault")
            self.after_fault = dict(fault, source_quality=adapter.statusOut.read(),
                quality_header_ns=adapter.statusOut.time_written_ns,
                nav_header_ns=observer.port.snapshot().header_ns,
                gate_unchanged=observer.history[-1] == row,
                pending_unchanged=observer.pending is pending)

        def after_owner(tick):
            event = self.mark(tick, 547, "after_owner_native_readback")
            self.publications.append(dict(event, native_epoch_ns=int(native.mtbCmdInMsg.timeWritten()),
                native_module_id=int(native.mtbCmdInMsg.moduleID()),
                native_dipole_Am2=list(native.mtbCmdInMsg().mtbDipoleCmds[:3]),
                quality_header_ns=adapter.statusOut.time_written_ns,
                source_quality=adapter.statusOut.read()))

        for priority, callback in ((579, normal_publication), (568, persistent_fault),
                                   (549, after_gate), (547, after_owner)):
            model = Stage(callback)
            model.ModelTag = f"TEST_ONLY_LateQuality{priority}"
            observer.models.append(model)
            sim.AddModelToTask("DynamicsTask", model, ModelPriority=priority)


def assess(frame, fixture):
    ticks = frame.time_ns.to_numpy(dtype=np.int64)
    records = frame.attrs["disconnected_command_records"]
    chain, owners = records["telemetry"], frame.attrs["mekf_command_owner"]
    trace, consumer = frame.attrs["disconnected_command_trace"], frame.attrs["navigation_consumer"]
    m, held = vector(frame, "mcmd", "Am2"), vector(frame, "held_mcmd", "Am2")
    native, applied = vector(frame, "native_mtbNetTorque_B", "Nm"), vector(frame, "applied_torque_B", "Nm")
    w, s = vector(frame, "omega_B", "rad_s"), mrp(frame)
    held_w, held_s = vector(frame, "held_omega_B", "rad_s"), mrp(frame, "held_sigma_BN")
    inertia = np.array(frame.attrs["simulation_config"]["spacecraft"]["inertia"]["value"], dtype=float)
    dt = np.diff(ticks)*1e-9
    args = (held_s[1:], held_w[1:], held[1:], vector(frame, "held_B_N", "T")[1:], dt, inertia)
    predicted, _, torque = predict_magnetic_step(*args)
    work, _, _, _, _ = rk_work(*args)
    energy = .5*np.sum(w*(w @ inertia.T), axis=1)
    state_error = float(np.max(np.abs(np.column_stack((s[1:], w[1:]))-predicted)))
    torque_error = float(np.max(np.linalg.norm(native[1:]-torque, axis=1)))
    work_error = float(np.max(np.abs(np.diff(energy)-work.sum(axis=1))))
    fault = int(np.flatnonzero(ticks == FAULT_NS)[0])
    first_zero = next((o["subscriber_epoch_ns"] for o in owners[fault:]
        if not any(o["native_input_dipole_Am2"])), None)
    zero_record = next((int(t) for t, h, tau in zip(ticks, held, native)
        if t > FAULT_NS and not np.any(h) and not np.any(tau)), None)
    before, after = fixture.before_fault, fixture.after_fault
    if before is None or after is None or fixture.approved_envelope is None:
        raise ValueError("Missing actual late-fault evidence")
    pubs, gates = fixture.publications, fixture.gates
    cycle = diagnostic_cycle_config()
    burst = ticks % cycle.period_ns >= cycle.actuation_offset_ns
    expected_order = [("HuskySat2_detumble", 1000), ("NativeMTB", 980),
        ("IdealShadowInputs", 590), ("ShadowMEKF", 580), ("TEST_ONLY_LateQuality579", 579),
        ("ShadowNavMonitor", 570), ("TEST_ONLY_LateQuality568", 568),
        ("DummyNavConsumer", 560), ("ChainCapture", 554), ("ChainCompute", 552),
        ("ChainApplication", 550), ("TEST_ONLY_LateQuality549", 549),
        ("MEKFDevelopmentCommandOwner", 548), ("TEST_ONLY_LateQuality547", 547)]
    selected_tags = {tag for tag, _ in expected_order}
    actual_order = [(r["tag"], r["priority"]) for r in fixture.scheduler if r["tag"] in selected_tags]
    surviving = fixture.approved_envelope
    checks = {
        "complete_evidence": len(frame) == len(chain) == len(owners) == len(pubs) == len(gates) == len(trace) == 31
            and np.array_equal(ticks, np.arange(0, 3_000_000_001, STEP_NS)),
        "actual_Basilisk_order": actual_order == expected_order
            and all(r["task"] == "DynamicsTask" and r["period_ns"] == STEP_NS for r in fixture.scheduler),
        "gate_then_fault_then_owner_at_same_epoch": before["sequence"] < after["sequence"] < pubs[fault]["sequence"]
            and before["tick_ns"] == after["tick_ns"] == pubs[fault]["tick_ns"] == FAULT_NS
            and before["priority"] == after["priority"] == 549 and pubs[fault]["priority"] == 547
            and after["gate_unchanged"] and after["pending_unchanged"],
        "real_current_health_approved_before_fault": before["decision"]["command_usable"]
            and before["decision"]["current_health"]["healthy"]
            and before["decision"]["evaluation_epoch_ns"] == before["quality_header_ns"] == FAULT_NS
            and before["gate"]["source_generation"] == before["gate"]["command_generation"] == (0, 1, 0),
        "supported_fault_preserves_original_quality_epoch": after["quality_header_ns"] == FAULT_NS-STEP_NS
            and after["nav_header_ns"] == FAULT_NS and after["source_quality"] == fixture.saved_quality[0]
            and after["source_quality"]["status_epoch_ns"] == FAULT_NS-STEP_NS
            and len([e for e in fixture.events if e["event"] == "late_stale_quality_fault"]) == 1,
        "already_approved_envelope_survives_exactly_one_publication": np.any(m[fault])
            and owners[fault]["command_usable"] and owners[fault]["command_id"] == surviving["command_id"]
            and owners[fault]["generation"] == surviving["generation"]
            and surviving["snapshot"]["capture_ns"] == 400_000_000
            and surviving["command"]["evaluation_epoch_ns"] == 500_000_000
            and np.array_equal(m[fault], surviving["command"]["clipped_dipole_B_Am2"])
            and first_zero == FAULT_NS+STEP_NS,
        "native_input_independently_matches_owner_and_CSV": all(
            np.array_equal(cmd, p["native_dipole_Am2"]) and np.array_equal(cmd, o["native_input_dipole_Am2"])
            and np.array_equal(cmd, o["published_dipole_Am2"])
            and p["native_epoch_ns"] == o["subscriber_epoch_ns"] == o["publication_epoch_ns"] == t
            and p["native_module_id"] == o["subscriber_module_id"] == o["publisher_module_id"]
            for t, cmd, p, o in zip(ticks, m, pubs, owners)),
        "no_approval_after_fault": all(not g["gate"]["command_usable"] for g in gates
            if g["sequence"] > after["sequence"])
            and all(not r["compute"] or not r["compute"][3].command_usable for r in trace if r["tick"] > FAULT_NS),
        "persistent_exposed_source_fault": all(p["quality_header_ns"] == FAULT_NS-STEP_NS
            and p["source_quality"] == after["source_quality"] for p in pubs[fault:])
            and all(not r["source_healthy"] and r["lifecycle_state"] == "FAULTED" for r in chain[fault+1:])
            and all(not r["reset"] for r in trace),
        "first_eligible_boundary_and_all_later_commands_zero": not np.any(m[fault+1:])
            and all(not o["command_usable"] and not any(o["native_input_dipole_Am2"])
                and not any(o["published_dipole_Am2"]) for o in owners[fault+1:])
            and first_zero == FAULT_NS+STEP_NS,
        "one_owner_no_automatic_fallback": len({o["publisher_module_id"] for o in owners}) == 1
            and all(o["owner"] == "MEKF_DEVELOPMENT" and not o["unused_cycle_message_written"] for o in owners)
            and all(c["requested_source"] == "MEKF" and c["selected_source"] == "NONE" for c in consumer[fault+1:])
            and all(c["selected_source"] in ("MEKF", "NONE") for c in consumer)
            and np.array_equal(frame.nav_message_time_ns.to_numpy(), ticks),
        "held_inputs_and_epochs_prove_following_interval": np.array_equal(held[1:], m[:-1])
            and np.array_equal(frame.native_input_dipole_time_ns.to_numpy()[1:], ticks[:-1])
            and np.array_equal(frame.native_input_field_time_ns.to_numpy()[1:], ticks[:-1]),
        "one_surviving_torque_interval_then_exact_zero": np.any(native[fault+1])
            and not np.any(native[fault+2:]) and np.array_equal(native, applied)
            and zero_record == FAULT_NS+2*STEP_NS,
        "native_torque_matches_independent_physics": torque_error < 1e-15,
        "finite_continuous_plant": np.isfinite(np.column_stack((s, w, vector(frame, "r_N", "m")))).all()
            and state_error < 1e-12 and np.array_equal(held_w[1:], w[:-1]) and np.array_equal(held_s[1:], s[:-1]),
        "energy_work_accounts_for_last_approved_interval": work_error < 1e-10
            and np.any(work[fault]) and not np.any(work[fault+1:]),
        "quiet_and_later_bursts_remain_zero": not np.any(m[~burst])
            and not np.any(m[(ticks > FAULT_NS) & burst])
            and not np.any(frame.cycle_rejected_sample_count.to_numpy(dtype=int)),
    }
    checks = {k: bool(v) for k, v in checks.items()}
    return dict(passed=all(checks.values()), checks=checks,
        epochs_ns=dict(gate_evaluation=FAULT_NS, late_fault=FAULT_NS, owner_publication=owners[fault]["publication_epoch_ns"],
            native_input=pubs[fault]["native_epoch_ns"], first_zero_command=first_zero,
            last_approved_interval_start=FAULT_NS, last_approved_interval_end=FAULT_NS+STEP_NS,
            first_zero_interval_start=first_zero, first_zero_interval_end=zero_record,
            fault_to_zero_command=None if first_zero is None else first_zero-FAULT_NS),
        surviving_publications_after_fault=int(np.any(m[fault:], axis=1).sum()),
        subsequent_zero_publications=len(m)-fault-1,
        surviving_dipole_Am2=m[fault].tolist(), final_approved_native_torque_Nm=native[fault+1].tolist(),
        final_approved_interval_work_J=float(work[fault].sum()), native_torque_error_Nm=torque_error,
        coupled_state_step_error=state_error, interval_work_balance_error_J=work_error,
        first_inhibition_reason=chain[fault+1]["inhibition_reason"], owner_trace=owners, chain=records,
        navigation_consumer=consumer, normal_adapter_history=frame.attrs["shadow_status"],
        gate_witnesses=gates, publication_witnesses=pubs)


def validate():
    fixture = LateQualityFixture()
    with contextlib.redirect_stdout(io.StringIO()):
        frame = run(stop_time_s=DURATION_S, write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), shadow=ShadowOptions(ideal_sun=True), control_source="MEKF_DEVELOPMENT",
            mekf_fault_test=ObserverOptions(fixture.attach, SCOPE+"; existing late_quality interface, 2026-10-01"))
        preserved = preservation()
    frame.attrs["control_scope"] = SCOPE
    evidence = assess(frame, fixture)
    sources = ("validate_late_fault_boundary.py", "test_late_fault_boundary.py", "scenario_huskysat2_detumble.py",
        "mekf_command_owner.py", "magnetic_control_cycle.py", "magnetic_actuation.py",
        "disconnected_command_observer.py", "command_health_gate.py", "attitude_mekf_adapter.py",
        "attitude_navigation_consumer.py", "validate_mekf_closed_loop.py")
    report = dict(scope=SCOPE, passed=evidence["passed"] and preserved["passed"],
        base_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={n: hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in sources},
        duration_s=DURATION_S, configuration=frame.attrs["simulation_config"], cycle=frame.attrs["magnetic_cycle"],
        sensor_fixture=frame.attrs["shadow_options"], scheduler=fixture.scheduler, event_order=fixture.events,
        before_fault=fixture.before_fault, after_fault=fixture.after_fault, approved_envelope=fixture.approved_envelope,
        evidence=evidence, production_preservation=preserved, csv_sha256=hashlib.sha256(csv_bytes(frame)).hexdigest())
    return report, frame, fixture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=HERE/"output_data/phase7g2c_late_fault.json")
    args = parser.parse_args()
    report, frame, _ = validate()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.with_suffix(".csv").write_bytes(csv_bytes(frame))
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(dict(passed=report["passed"], evidence={k: v for k, v in report["evidence"].items()
        if k not in ("owner_trace", "chain", "navigation_consumer", "normal_adapter_history", "gate_witnesses",
                     "publication_witnesses")}, production_preservation=report["production_preservation"]), indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
