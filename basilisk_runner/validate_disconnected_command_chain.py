"""7F-2C short LIVE disconnected validation; ASSUMED / TEST-ONLY, 2026-09-30.

Faults affect only shadow estimator inputs or owned diagnostic views. No native
production message, physical parameter or actuator endpoint is modified.
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
from dataclasses import asdict, replace
import contextlib
import hashlib
import io
import json
from pathlib import Path
import subprocess
from types import ModuleType

from attitude_mekf_adapter import DevelopmentChannel, InputBatch, ShadowOptions
from command_health_gate import CommandHealthGate
from control_input_snapshot import capture_control_snapshot
from disconnected_command_observer import ObserverOptions, Stage, SCOPE
from disconnected_detumble_math import evaluate_snapshot
from hs2_sim_config import get_profile_config
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run


MS = 1_000_000
ROOT = Path(__file__).resolve().parents[1]
CASES = ("nominal", "previous_nav", "stale_wrong_cycle_tam", "invalid_tam", "fault_before_application",
         "fault_reset_reacquire", "reset_after_compute", "late_quality", "mid_burst_fault")


class LiveFixtures:
    """Scheduled TEST-ONLY events; observer itself contains no fault synthesis."""
    def __init__(self, name):
        self.name, self.events = name, []
        self.old_command = self.first_sample = self.previous_quality = None

    def attach(self, sim, inputs, adapter, observer):
        relay = DevelopmentChannel[InputBatch]()
        adapter.inputs = observer.health_inputs = relay

        def input_event(tick):
            batch = inputs.read()
            fault_at = {"fault_before_application": 600, "fault_reset_reacquire": 1600, "mid_burst_fault": 800}.get(self.name)
            reset_at = {"fault_reset_reacquire": 1800, "reset_after_compute": 600}.get(self.name)
            if fault_at is not None and tick == fault_at*MS:
                self.old_command = observer.pending
                batch = replace(batch, gyro_valid=False, gyro_reason="7F2C_TEST_ONLY_fault")
                self.events.append((tick, 588, "fault_to_adapter"))
            if reset_at is not None and tick == reset_at*MS:
                self.old_command = self.old_command or observer.pending
                batch = replace(batch, reset_acquisition=True)
                self.events.append((tick, 588, "reset_to_adapter_and_gate"))
            relay.write(batch, tick)

        def quality_event(tick):
            current = adapter.statusOut.read(), adapter.statusOut.time_written_ns
            if self.name == "late_quality" and tick == 600*MS and self.previous_quality is not None:
                payload, stamp = self.previous_quality
                adapter.statusOut.write(payload, stamp)
                self.events.append((tick, 568, "late_quality_publication"))
            self.previous_quality = current

        def owned_view_event(tick):
            if tick == 400*MS:
                self.first_sample = observer.sample
            if tick == 1400*MS:
                if self.name == "previous_nav":
                    observer.capture_nav_override = deepcopy(observer.early_nav)
                elif self.name == "stale_wrong_cycle_tam":
                    observer.sample = self.first_sample
                elif self.name == "invalid_tam":
                    observer.sample = replace(observer.sample, valid=False, quiet_valid=False)
                else:
                    return
                self.events.append((tick, 556, self.name))
            if self.name == "fault_reset_reacquire" and tick == 2600*MS:
                # Deliberate replay of an immutable pre-reset envelope. No epoch
                # or generation is relabeled; it must remain diagnostic only.
                observer.pending = self.old_command
                self.events.append((tick, 556, "replay_pre_reset_diagnostic"))

        for priority, callback in ((588, input_event), (568, quality_event), (556, owned_view_event)):
            model = Stage(callback)
            model.ModelTag = f"TEST_ONLY_ChainFixture{priority}"
            observer.models.append(model)
            sim.AddModelToTask("DynamicsTask", model, ModelPriority=priority)


def component_equivalence(host):
    """Replay received live event records through the original component APIs."""
    def exact(actual, expected, label):
        # Validation remains active even if Python is invoked with -O.
        if actual != expected:
            raise ValueError("live/component mismatch: "+label)

    gate = CommandHealthGate()
    cycle = diagnostic_cycle_config()
    counts = {"health": 0, "snapshots": 0, "calculations": 0, "compute_decisions": 0, "application_decisions": 0}
    for row in host.attrs["disconnected_command_trace"]:
        tick = row["tick"]
        if row["reset"]:
            gate.notify_reset(tick)
        exact(gate.observe(row["nav"], row["decision"], tick), row["health"], "current health")
        counts["health"] += 1
        if row["capture_result"] is not None:
            result = capture_control_snapshot(row["capture_nav"], row["decision"], row["sample"], row["plan"], tick,
                navigation_provenance="Live MEKF NavAtt + Phase 7E quality/consumer; 7F-2C 2026-09-30")
            exact(result, row["capture_result"], "snapshot")
            counts["snapshots"] += 1
        for event in ("compute", "application"):
            if row[event] is None:
                continue
            pending, nav, decision, result = row[event]
            if event == "compute" and pending is not None:
                # Derive the same run config without importing/reimplementing math.
                config = get_profile_config("regression_baseline").with_run_options(3.9, None)
                exact(evaluate_snapshot(pending.snapshot, tick, config), pending.command, "controller calculation")
                counts["calculations"] += 1
            checked = gate.evaluate(pending.snapshot if pending else None, pending.command if pending else None,
                nav, decision, tick, **({"application_cycle": cycle} if event == "application" else {}))
            exact(checked, result, event+" decision")
            counts[event+"_decisions"] += 1
    return {"passed": True, "exact_record_comparisons": counts}


def csv_bytes(frame):
    return frame.to_csv(index=False).encode("utf-8")


def run_case(name, baseline):
    fixture = LiveFixtures(name)
    # Explicit existing ideal-Sun option aligned to quiet samples for repeatable
    # acquisition after reset. TEST-ONLY cadence; no sensor/default change.
    options = ObserverOptions(None if name == "nominal" else fixture.attach, "7F-2C TEST-ONLY "+name)
    with contextlib.redirect_stdout(io.StringIO()):
        host = run(stop_time_s=3.9, write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
            shadow=ShadowOptions(ideal_sun=True, sun_period_ns=1_000_000_000), disconnected_commands=options)
    records = host.attrs["disconnected_command_records"]
    rows = {r["sim_epoch_ns"]//MS: r for r in records["telemetry"]}
    checks = {"production_bytes_exact": csv_bytes(host) == csv_bytes(baseline),
              "startup_inhibited": all(not rows[t]["command_usable"] for t in range(0, 600, 100)),
              "no_authority": all(r["actuator_authority"] is False for r in rows.values()),
              "sample_preserved": all(r["sample_epoch_ns"] == r["tam_acquisition_epoch_ns"] == r["snapshot_capture_ns"]
                                      for r in rows.values() if r["sample_epoch_ns"] is not None),
              "usable_only_current_health": all(r["source_healthy"] and r["generation_match"] and r["cycle_match"]
                  and r["current_mekf_state_ns"] == r["current_mekf_publication_ns"] == r["current_quality_epoch_ns"] == r["sim_epoch_ns"]
                  for r in rows.values() if r["command_usable"])}
    if name == "nominal":
        checks.update(first_capture=rows[400]["snapshot_capture_ns"] == 400*MS,
            first_computation=rows[500]["command_computation_ns"] == 500*MS,
            application_burst=all(rows[t]["command_usable"] for t in (600, 700, 800, 900)),
            no_cross_cycle_hold=rows[1000]["command_id"] is None,
            actual_prior_estimate=any(r[:4] == (1400*MS, 595, "pre_MEKF", 1300*MS) for r in records["execution_order"]))
    elif name in ("previous_nav", "stale_wrong_cycle_tam", "invalid_tam"):
        checks.update(capture_rejected=bool(rows[1400]["capture_rejection"]),
                      no_command=not rows[1600]["command_usable"] and rows[1500]["command_id"] is None)
    elif name in ("fault_before_application", "late_quality"):
        checks.update(computed_first=rows[500]["mathematical_command_valid"],
            boundary_inhibited=not rows[600]["command_usable"] and bool(rows[600]["inhibition_reason"]),
            fault_seen_before_boundary=rows[600]["lifecycle_state"] == "FAULTED",
            numerical_evidence_retained=rows[600]["clipped_dipole_B_Am2"] == rows[500]["clipped_dipole_B_Am2"])
    elif name == "fault_reset_reacquire":
        checks.update(initially_usable=rows[600]["command_usable"],
            fault_inhibits=not rows[1600]["command_usable"], reset_delivered=rows[1800]["reset_epoch_ns"] == 1800*MS,
            reacquiring=rows[1800]["lifecycle_state"] == "REACQUIRING",
            reacquired_without_permission=rows[2400]["source_healthy"] and not rows[2600]["command_usable"],
            old_generation_rejected=not rows[2600]["generation_match"] and rows[2600]["command_computation_ns"] == 1500*MS,
            fresh_restores=rows[3600]["command_usable"] and rows[3600]["sample_epoch_ns"] == 3400*MS)
    elif name == "reset_after_compute":
        checks.update(reset_inhibits=not rows[600]["command_usable"] and rows[600]["reset_epoch_ns"] == 600*MS,
            same_acquisition_sample_inhibited=not rows[1600]["command_usable"],
            fresh_restores=rows[2600]["command_usable"] and rows[2600]["sample_epoch_ns"] == 2400*MS)
    elif name == "mid_burst_fault":
        checks.update(burst_started=rows[600]["command_usable"] and rows[700]["command_usable"],
                      subsequent_intervals_inhibited=not rows[800]["command_usable"] and not rows[900]["command_usable"])
    # Actual observed stage call order, not just configured priorities.
    checks["actual_stage_order"] = all([entry[1] for entry in records["execution_order"] if entry[0] == t*MS]
        == ([595, 587, 554, 552, 550] if t % 1000 == 500 else [595, 587, 554, 550]) for t in rows)
    return {"passed": all(checks.values()), "checks": checks, "component_equivalence": component_equivalence(host),
            "injections": fixture.events, **records}


def preservation():
    source = subprocess.check_output(["git", "show", "HEAD:basilisk_runner/scenario_huskysat2_detumble.py"], cwd=ROOT, text=True, encoding="utf-8")
    previous = ModuleType("committed_phase7f2b_scenario")
    previous.__file__ = str(ROOT/"basilisk_runner/scenario_huskysat2_detumble.py")
    exec(compile(source, previous.__file__, "exec"), previous.__dict__)
    results = {}
    for profile, cycle in (("regression_baseline", None), ("hs2_candidate", None), ("regression_baseline", diagnostic_cycle_config())):
        kwargs = dict(stop_time_s=4., write_outputs=False, make_plots=False, config=get_profile_config(profile), cycle=cycle)
        with contextlib.redirect_stdout(io.StringIO()):
            old, current = previous.run(**kwargs), run(**kwargs)
        name = profile+("_cycled" if cycle else "")
        results[name] = {"bytes_equal": csv_bytes(old) == csv_bytes(current), "rows": len(current),
                         "observer_disabled": "disconnected_command_records" not in current.attrs}
    # Compare actual actuator subscription/attachment ASTs against the committed
    # scenario, alongside byte-exact enabled/disabled plant telemetry.
    def wiring(text):
        tree = ast.parse(text)
        return [ast.dump(node) for node in ast.walk(tree) if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute) and node.func.attr in ("subscribeTo", "addDynamicEffector")]
    untouched = wiring(source) == wiring(Path(previous.__file__).read_text(encoding="utf-8"))
    return {"passed": untouched and all(r["bytes_equal"] and r["observer_disabled"] for r in results.values()),
            "actuator_and_sensor_wiring_AST_identical": untouched, "committed_vs_working_4s": results}


def validate():
    with contextlib.redirect_stdout(io.StringIO()):
        baseline = run(stop_time_s=3.9, write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config())
    cases = {name: run_case(name, baseline) for name in CASES}
    preserved = preservation()
    names = ("command_health_gate.py", "disconnected_command_observer.py", "scenario_huskysat2_detumble.py",
             "validate_disconnected_command_chain.py", "test_disconnected_command_observer.py")
    return {"scope": SCOPE, "passed": preserved["passed"] and all(c["passed"] for c in cases.values()),
            "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            "source_sha256": {n: hashlib.sha256((ROOT/"basilisk_runner"/n).read_bytes()).hexdigest() for n in names},
            "fixtures": "ASSUMED / TEST-ONLY 3.9 s cases, 1 s ideal-Sun cadence; exact record equality, no numeric tolerance",
            "cases": cases, "preservation": preserved}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = validate()
    if args.report:
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "cases": {k: v["checks"] for k, v in report["cases"].items()},
                      "preservation": report["preservation"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
