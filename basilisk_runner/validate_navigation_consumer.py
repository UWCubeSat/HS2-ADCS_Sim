"""Phase 7E diagnostic tests; no actuator/control authority or flight accuracy.

ASSUMED / TEST-ONLY fixtures, 2026-09-14. Native message failures below are
injected in isolated rigs, never into production navigation or spacecraft state.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import replace
import hashlib
import io
import json
from pathlib import Path
import subprocess
from types import ModuleType

import numpy as np
import pandas as pd
from Basilisk.architecture import messaging

from attitude_mekf import VectorSample
from attitude_mekf_adapter import DevelopmentChannel, InputBatch, MEKFNavigationAdapter, ShadowOptions, causal_gyro_intervals
from attitude_navigation_consumer import ConsumerOptions, DummyNavigationConsumer, NavPort, Source
from magnetic_control_cycle import diagnostic_cycle_config
from hs2_sim_config import get_profile_config
from scenario_huskysat2_detumble import run


def publish_nav(message, quality, tick, *, stamp=None, publication=None, sigma=(0., 0., 0.),
                omega=(0., 0., 0.), source=Source.MEKF, **changes):
    """Native NavAtt/quality packet fixture, explicitly independent of truth."""
    stamp = tick if stamp is None else stamp
    publication = tick if publication is None else publication
    payload = messaging.NavAttMsgPayload()
    payload.sigma_BN, payload.omega_BN_B, payload.timeTag = list(sigma), list(omega), stamp*1e-9
    message.write(payload, publication)
    status = {"source": source.value, "sim_epoch_ns": tick, "status_epoch_ns": tick,
              "state_epoch_ns": stamp, "gyro_epoch_ns": stamp, "nav_published_epoch_ns": publication,
              "valid": True, "initialized": True, "state": "INITIALIZED", "fault": "",
              "acquisitions": 1, "acquisition_event": False, "reacquisition_event": False}
    status.update(changes)
    quality.write(status, tick)


def pair(tick, angle=0.):
    # Independent passive +X rotation: C_BN e_Y = (0,cos(theta),-sin(theta)).
    c = np.array([[1., 0., 0.], [0., np.cos(angle), np.sin(angle)], [0., -np.sin(angle), np.cos(angle)]])
    return [VectorSample(f"fixture-{sensor}-{tick}", sensor, tick, tick, c @ ref, ref,
                         "ASSUMED / TEST-ONLY analytic X rotation, 2026-09-14")
            for sensor, ref in (("magnetic", np.array([1., 0., 0.])), ("sun", np.array([0., 1., 0.])))]


class AdapterRig:
    """Actual Phase 7D adapter and native consumer, no plant or controller."""
    def __init__(self):
        self.channel = DevelopmentChannel[InputBatch]()
        self.adapter = MEKFNavigationAdapter(self.channel, ShadowOptions())
        self.port = NavPort(Source.MEKF, self.adapter.navOutMsg, self.adapter.statusOut)
        self.consumer = DummyNavigationConsumer({Source.MEKF: self.port}, ConsumerOptions(((0, Source.MEKF),)))
        self.previous = None

    def step(self, tick, deliveries=(), *, gyro=(0., 0., 0.), reset=False, fault="", corrupt_nav=False):
        intervals = causal_gyro_intervals(self.previous, tick, gyro, gyro, 10) if self.previous is not None else []
        batch = InputBatch(tick, tick, np.array(gyro), self.previous, intervals, list(deliveries), reset_acquisition=reset)
        if fault == "nonfinite_gyro":
            batch.gyro_point_B[0] = float("nan")
        elif fault == "coverage":
            batch.intervals = []
        elif fault:
            batch.gyro_valid, batch.gyro_reason = False, fault
        self.channel.write(batch, tick)
        self.adapter.UpdateState(tick)
        if corrupt_nav:
            payload = self.adapter.navOutMsg.read()
            payload.sigma_BN = [float("nan"), 0., 0.]
            self.adapter.navOutMsg.write(payload, tick)
        self.consumer.UpdateState(tick)
        self.previous = tick
        return self.consumer.history[-1]


def fault_recovery_case(kind):
    rig = AdapterRig()
    for i in range(21):
        tick = i*100_000_000
        rig.step(tick, pair(tick) if i in (4, 14) else (), reset=i == 12,
                 fault=kind if i == 8 and kind != "nonfinite_navigation" else "",
                 corrupt_nav=i == 8 and kind == "nonfinite_navigation")
    rows = rig.consumer.history
    checks = {"startup_unavailable": all(not row["accepted"] for row in rows[:4]),
              "acquires_at_400ms": rows[4]["consumer_state"] == "VALID",
              "fault_latches": all(row["consumer_state"] == "FAULTED" and not row["accepted"] for row in rows[8:12]),
              "reset_waits": all(row["consumer_state"] == "REACQUIRING" and not row["accepted"] for row in rows[12:14]),
              "fresh_reacquisition": rows[14]["consumer_state"] == "VALID" and rows[14]["reacquisition_event"],
              "fresh_epoch": rows[14]["state_epoch_ns"] == rows[14]["publication_epoch_ns"] == 1_400_000_000,
              "no_implicit_fallback": all(row["selected_source"] == "NONE" for row in rows[8:14])}
    return {"passed": all(checks.values()), "checks": checks, "telemetry": rows}


def ideal_handover():
    """Analytic constant-rate spin, actual adapter, native SimpleNav-format input.

    SimpleNav fixture deliberately uses the equivalent MRP shadow set. Neither
    source drives dynamics. This tests representation, not estimator tuning.
    """
    rig = AdapterRig()
    simple_msg, simple_quality = messaging.NavAttMsg(), DevelopmentChannel[dict]()
    ports = {Source.MEKF: rig.port, Source.SIMPLE_NAV: NavPort(Source.SIMPLE_NAV, simple_msg, simple_quality)}
    consumer = DummyNavigationConsumer(ports, ConsumerOptions(((1_000_000_000, Source.MEKF), (2_000_000_000, Source.SIMPLE_NAV))))
    maximum_frame_error = 0.
    for i in range(31):
        tick = i*100_000_000
        angle = 0.4+0.02*tick*1e-9  # TEST-ONLY rad, rad/s; analytic one-axis truth observer.
        rig.step(tick, pair(tick, angle) if i == 0 else (), gyro=(0.02, 0., 0.))
        publish_nav(simple_msg, simple_quality, tick, sigma=(-1/np.tan(angle/4), 0., 0.),
                    omega=(0.02, 0., 0.), source=Source.SIMPLE_NAV)
        consumer.UpdateState(tick)
        expected = np.array([[1., 0., 0.], [0., np.cos(angle), np.sin(angle)], [0., -np.sin(angle), np.cos(angle)]])
        maximum_frame_error = max(maximum_frame_error, float(np.max(np.abs(
            np.array(consumer.history[-1]["navigation"]["C_BN"])-expected))))
    handovers = [row for row in consumer.history if row["handover_event"] and row["previous_source"] != "NONE"]
    checked = {"both_directions": [(r["previous_source"], r["selected_source"]) for r in handovers]
               == [("SIMPLE_NAV", "MEKF"), ("MEKF", "SIMPLE_NAV")],
               "same_epoch": all(r["handover_state_epoch_jump_ns"] == 0 for r in handovers),
               "attitude_continuity": all(r["handover_attitude_jump_rad"] < 1e-12 for r in handovers),
               "rate_continuity": all(r["handover_rate_jump_rad_s"] < 1e-12 for r in handovers),
               "analytic_frame": maximum_frame_error < 1e-12,
               "always_valid": all(r["accepted"] for r in consumer.history)}
    return {"passed": all(checked.values()), "checks": checked, "max_analytic_dcm_error": maximum_frame_error,
            "tolerance": "ASSUMED / TEST-ONLY 1e-12 numerical tolerance, not flight accuracy", "handovers": handovers}


def live_cases():
    cycle = diagnostic_cycle_config()
    with contextlib.redirect_stdout(io.StringIO()):
        baseline = run(stop_time_s=4., write_outputs=False, make_plots=False, cycle=cycle)
    source_mekf = ConsumerOptions(((0, Source.MEKF),))
    cases = {"startup": (ShadowOptions(ideal_sun=True), source_mekf),
             "default_selection": (ShadowOptions(ideal_sun=True), ConsumerOptions()),
             "handover": (ShadowOptions(ideal_sun=True), ConsumerOptions(((1_000_000_000, Source.MEKF), (3_000_000_000, Source.SIMPLE_NAV)))),
             "sun_loss": (ShadowOptions(ideal_sun=True, drop_sun_after_ns=2_000_000_000), source_mekf),
             "delayed_magnetic": (ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000), source_mekf),
             "gyro_only_prior": (ShadowOptions(magnetic_enabled=False, initial_q_BN=(1., 0., 0., 0.)), source_mekf),
             "single_vector": (ShadowOptions(), source_mekf)}
    report = {}
    for name, (shadow, selection) in cases.items():
        with contextlib.redirect_stdout(io.StringIO()):
            host = run(stop_time_s=4., write_outputs=False, make_plots=False, cycle=cycle,
                       shadow=shadow, navigation_consumer=selection)
        pd.testing.assert_frame_equal(baseline, host, check_exact=True)
        rows, frozen = host.attrs["navigation_consumer"], host.attrs["navigation_frozen_consumer"]
        accepted = [row for row in rows if row["accepted"]]
        expected_first = None if name == "single_vector" else 0 if name in ("default_selection", "handover", "gyro_only_prior") else 400_000_000
        checks = {"production_exact": True,
                  "startup": (accepted[0]["consumer_epoch_ns"] if accepted else None) == expected_first,
                  "current_epoch": all(r["state_epoch_ns"] == r["publication_epoch_ns"] == r["consumer_epoch_ns"] for r in accepted),
                  "frozen_epoch": all(r["state_epoch_ns"] == r["required_epoch_ns"] and r["navigation_age_ns"] == 100_000_000
                                      for r in frozen if r["accepted"]),
                  "availability": len(accepted) == (0 if expected_first is None else 41-int(expected_first//100_000_000)),
                  "no_fault": all(r["consumer_state"] != "FAULTED" for r in rows),
                  "default_simple": name != "default_selection" or all(r["selected_source"] == "SIMPLE_NAV" for r in rows)}
        report[name] = {"passed": all(checks.values()), "checks": checks,
                        "first_valid_ns": expected_first, "accepted_rows": len(accepted),
                        "estimator_final_quality": host.attrs["shadow_status"][-1],
                        "telemetry": rows, "frozen_telemetry": frozen}
    return report


def degraded_adapter_cases():
    rig = AdapterRig()
    rig.step(0, pair(0))
    invalid = replace(pair(100_000_000)[0], valid=False, invalid_reason="TEST_ONLY_invalid_magnetic_window")
    rig.step(100_000_000, [invalid])
    # Acquisition predates the initialized replay base; reject, do not reset age.
    for i in range(2, 36):
        rig.step(i*100_000_000, pair(0) if i == 35 else ())
    quality = rig.adapter.statusOut.read()
    checks = {"quality_drives_consumer": all(r["accepted"] for r in rig.consumer.history),
              "magnetic_rejection": quality["rejected"].get("TEST_ONLY_invalid_magnetic_window") == 1,
              "history_rejection": quality["rejected"].get("outside_history") == 2,
              "no_fabricated_updates": quality["updates"] == {"magnetic": 1, "sun": 1}}
    return {"passed": all(checks.values()), "checks": checks, "final_quality": quality,
            "telemetry": rig.consumer.history}


def preservation_evidence():
    """Committed-vs-working short runs plus full saved hashes; no output rewrite."""
    root = Path(__file__).resolve().parents[1]
    committed = subprocess.check_output(["git", "show", "HEAD:docs/ATTITUDE_ESTIMATOR_ARCHITECTURE.md"], cwd=root, text=True, encoding="utf-8")
    result = {}
    for name in ("detumble_output.csv", "detumble_output_hs2_candidate.csv", "detumble_output_cycled.csv"):
        digest = hashlib.sha256((root/"basilisk_runner"/"output_data"/name).read_bytes()).hexdigest()
        result[name] = {"sha256": digest, "matches_committed_evidence": digest in committed}
    source = subprocess.check_output(["git", "show", "HEAD:basilisk_runner/scenario_huskysat2_detumble.py"],
                                     cwd=root, text=True, encoding="utf-8")
    previous = ModuleType("phase7d_committed_scenario")
    previous.__file__ = str(root/"basilisk_runner"/"scenario_huskysat2_detumble.py")
    exec(compile(source, previous.__file__, "exec"), previous.__dict__)
    short_runs = {}
    for profile, cycle in (("regression_baseline", None), ("hs2_candidate", None),
                           ("regression_baseline", diagnostic_cycle_config())):
        kwargs = dict(stop_time_s=4., write_outputs=False, make_plots=False,
                      config=get_profile_config(profile), cycle=cycle)
        with contextlib.redirect_stdout(io.StringIO()):
            old = previous.run(**kwargs)
            current = run(**kwargs)
        exact = old.to_csv(index=False).encode("utf-8") == current.to_csv(index=False).encode("utf-8")
        isolated = "navigation_consumer" not in current.attrs and "shadow_telemetry" not in current.attrs
        short_runs[profile+("_cycled" if cycle else "")] = {"bytes_equal": exact, "consumer_disabled": isolated, "rows": len(current)}
    return {"passed": all(r["matches_committed_evidence"] for r in result.values())
                      and all(r["bytes_equal"] and r["consumer_disabled"] for r in short_runs.values()),
            "files": result, "committed_vs_working_4s_runs": short_runs,
            "scope": "saved full baseline/hash integrity plus new short runs; full orbits not rerun"}


def validate():
    live = live_cases()
    faults = {kind: fault_recovery_case(kind) for kind in ("nonfinite_gyro", "coverage", "explicit_test_fault", "nonfinite_navigation")}
    handover, degraded, production = ideal_handover(), degraded_adapter_cases(), preservation_evidence()
    root = Path(__file__).resolve().parents[1]
    provenance = {"date": "2026-09-14", "status": "ASSUMED / TEST-ONLY fixtures; verified software results only",
                  "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                  "source_sha256": {name: hashlib.sha256((root/"basilisk_runner"/name).read_bytes()).hexdigest()
                      for name in ("attitude_navigation_consumer.py", "validate_navigation_consumer.py",
                                   "test_attitude_navigation_consumer.py", "scenario_huskysat2_detumble.py")}}
    return {"scope": "DUMMY NAVIGATION CONSUMER / NO CONTROL AUTHORITY / NOT FLIGHT VALIDATED", "provenance": provenance,
            "passed": all(r["passed"] for r in [*live.values(), *faults.values(), handover, degraded, production]),
            "live": live, "faults": faults, "ideal_handover": handover, "degraded": degraded, "preservation": production}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = validate()
    if args.report:
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "live": {k: v["passed"] for k, v in report["live"].items()},
        "faults": {k: v["passed"] for k, v in report["faults"].items()},
        "ideal_handover": report["ideal_handover"], "degraded": report["degraded"]["checks"],
        "preservation": report["preservation"]}, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
