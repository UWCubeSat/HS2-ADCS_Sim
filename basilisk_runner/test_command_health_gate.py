"""Phase 7F-2B inhibition/restart evidence, 2026-09-30; ASSUMED / TEST-ONLY.

Synthetic source events in exact integer ns, unchanged diagnostic cycle.
No command-math revalidation, spacecraft, effector, scenario or saved outputs.
"""
import ast
from dataclasses import FrozenInstanceError, asdict, replace
import inspect
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

import command_health_gate as module
from attitude_mekf import VectorSample
from attitude_mekf_adapter import DevelopmentChannel, InputBatch, MEKFNavigationAdapter, ShadowOptions, causal_gyro_intervals
from attitude_navigation_consumer import ConsumerOptions, DummyNavigationConsumer, NavPort, Source
from disconnected_detumble_math import CommandMathematics, evaluate_snapshot
from test_control_input_snapshot import capture, magnetic, navigation, window


MS = 1_000_000  # Exact SI conversion; no age/health threshold.


def current(ms, *, count=1, event=False, quality=None, decision=None):
    tick = ms*MS
    nav = navigation(tick)
    nav.quality.update(acquisitions=count, acquisition_event=event, reacquisition_event=event and count > 1)
    nav.quality.update(quality or {})
    consumer = DummyNavigationConsumer({}, ConsumerOptions(((0, Source.MEKF),)))
    row = consumer.consume(tick, {Source.MEKF: nav})
    row.update(decision or {})
    return nav, row


def calculation(sample_ms=400, *, count=1, nav=None):
    tick, compute = sample_ms*MS, (sample_ms+100)*MS
    nav = nav or current(sample_ms, count=count)[0]
    accepted = capture(nav, magnetic(tick), tick, window(compute))
    if accepted.snapshot is None:
        raise AssertionError(accepted.rejection_reason)
    return accepted.snapshot, evaluate_snapshot(accepted.snapshot, compute)


def ready_gate():
    gate = module.CommandHealthGate()
    result = gate.observe(*current(300, event=True), 300*MS)
    if not result.healthy:
        raise AssertionError(result.reason)
    return gate


class HealthGateTests(unittest.TestCase):
    def setUp(self):
        self.snapshot, self.command = calculation()
        self.gate = ready_gate()

    def evaluate(self, ms=500, **kwargs):
        return self.gate.evaluate(self.snapshot, self.command, *current(ms, **kwargs), ms*MS)

    def test_healthy_evaluation_preserves_three_validities_epochs_and_provenance(self):
        original = asdict(self.command)
        # Any inadvertent controller call through the production dispatcher fails.
        with patch('basilisk_adcs_adapter.controller_step', side_effect=AssertionError("math recomputation")):
            result = self.evaluate()
        self.assertTrue(result.command_usable)
        self.assertTrue(result.capture_time_valid)
        self.assertTrue(result.mathematical_command_valid)
        self.assertTrue(result.current_health.healthy)
        self.assertEqual(result.inhibition_reason, "")
        self.assertEqual((result.sample_epoch_ns, result.snapshot_capture_epoch_ns), (400*MS, 400*MS))
        self.assertEqual((result.command_computation_epoch_ns, result.evaluation_epoch_ns,
                          result.current_health.quality_epoch_ns), (500*MS, 500*MS, 500*MS))
        self.assertIs(result.command, self.command)
        self.assertEqual(asdict(self.command), original)

    def test_startup_requires_witnessed_acquisition_and_allows_initial_sample(self):
        gate = module.CommandHealthGate()
        missing = gate.observe(*current(300), 300*MS)
        self.assertFalse(missing.healthy)
        self.assertEqual(missing.reason, "fresh_acquisition_event_required")
        gate.observe(*current(400, event=True), 400*MS)
        result = gate.evaluate(self.snapshot, self.command, *current(500), 500*MS)
        self.assertTrue(result.command_usable)

    def test_fault_after_capture_before_computation_revokes(self):
        snapshot, _ = calculation()
        self.gate.observe(*current(450, quality={"state": "FAULT", "fault": "gyro_gap", "valid": False}), 450*MS)
        command = evaluate_snapshot(snapshot, 500*MS)
        self.assertTrue(command.command_valid)
        result = self.gate.evaluate(snapshot, command, *current(500), 500*MS)
        self.assertFalse(result.command_usable)
        self.assertEqual(result.inhibition_reason, "explicit_reset_required")

    def test_fault_after_computation_at_same_tick_revokes_without_mutation(self):
        # Command has already been computed at 0.5 s; the fault is the next event
        # at 0.5 s, before USE evaluation. No fictitious sub-tick time is invented.
        original = asdict(self.command)
        result = self.evaluate(quality={"state": "FAULT", "fault": "missing_gyro_coverage", "valid": False})
        self.assertFalse(result.command_usable)
        self.assertEqual(result.inhibition_reason, "estimator_fault:missing_gyro_coverage")
        self.assertTrue(result.capture_time_valid)
        self.assertTrue(result.mathematical_command_valid)
        self.assertFalse(result.current_health.healthy)
        self.assertEqual(asdict(result.command), original)
        with self.assertRaises(FrozenInstanceError):
            setattr(result, "command_usable", True)

    def test_all_supported_invalid_health_classes_inhibit_and_latch(self):
        cases = [({"state": "FAULT", "fault": "gyro_gap"}, {}),
                 ({"valid": False}, {}), ({"initialized": False, "state": "UNINITIALIZED", "valid": False}, {}),
                 ({}, {"fault_latched": True}), ({"state": "REACQUIRING", "valid": False}, {}),
                 ({"gyro_epoch_ns": float("nan")}, {}), ({"state_epoch_ns": float("inf")}, {}),
                 ({"sample_ages_ns": {"magnetic": float("nan")}}, {}),
                 ({"fault": float("nan")}, {}), ({"acquisitions": float("nan")}, {}),
                 ({"acquisition_event": 1}, {}), ({"state": "INCONSISTENT"}, {}),
                 ({}, {"accepted": False})]
        for quality, decision in cases:
            with self.subTest(quality=quality, decision=decision):
                gate = ready_gate()
                result = gate.evaluate(self.snapshot, self.command, *current(500, quality=quality, decision=decision), 500*MS)
                self.assertFalse(result.command_usable)
                self.assertTrue(result.inhibition_reason)
                recovered_bit = gate.evaluate(self.snapshot, self.command, *current(500), 500*MS)
                self.assertFalse(recovered_bit.command_usable)
                self.assertEqual(recovered_bit.inhibition_reason, "explicit_reset_required")

    def test_nonfinite_navigation_inhibits(self):
        for field, value in (("omega", [0., float("nan"), 0.]), ("sigma", [0., 0., float("inf")])):
            with self.subTest(field=field):
                gate = ready_gate()
                nav, row = current(500)
                setattr(nav, field, value)
                result = gate.evaluate(self.snapshot, self.command, nav, row, 500*MS)
                self.assertFalse(result.command_usable)
                self.assertIn("invalid_navigation_contract", result.inhibition_reason)

    def test_repeated_fault_and_cleared_valid_bits_never_retain_command(self):
        self.assertTrue(self.evaluate().command_usable)
        for ms in (500, 600, 700):
            result = self.evaluate(ms, quality={"state": "FAULT", "fault": "gyro_gap", "valid": False})
            self.assertFalse(result.command_usable)
            self.assertIs(result.command, self.command)
        self.assertFalse(self.evaluate(800).command_usable)
        self.assertFalse(any(isinstance(value, CommandMathematics) for value in vars(self.gate).values()))

    def test_reset_reacquisition_without_new_snapshot_cannot_revive_old_command(self):
        self.assertTrue(self.evaluate().command_usable)
        self.assertFalse(self.evaluate(600, quality={"state": "FAULT", "fault": "gyro_gap"}).command_usable)
        self.gate.notify_reset(700*MS)
        uninit = {"state": "UNINITIALIZED", "initialized": False, "valid": False}
        reset = self.evaluate(700, quality=uninit)
        self.assertFalse(reset.command_usable)
        self.assertEqual(reset.current_health.lifecycle_state, "REACQUIRING")
        acquired = self.evaluate(800, count=2, event=True)
        self.assertTrue(acquired.current_health.healthy)
        self.assertFalse(acquired.command_usable)
        self.assertEqual(acquired.inhibition_reason, "snapshot_predates_current_acquisition")
        missing = self.gate.evaluate(None, None, *current(900, count=2), 900*MS)
        self.assertFalse(missing.command_usable)
        self.assertEqual(missing.inhibition_reason, "fresh_snapshot_and_command_required")
        fresh, command = calculation(1400, count=2)
        restored = self.gate.evaluate(fresh, command, *current(1500, count=2), 1500*MS)
        self.assertTrue(restored.command_usable)
        self.assertEqual(restored.current_health.acquisition_count, 2)
        self.assertFalse(self.evaluate(1500, count=2).command_usable)

    def test_pre_fault_command_rejected_even_at_its_original_deadline_after_reacquisition(self):
        self.gate.observe(*current(450, quality={"state": "FAULT", "fault": "gyro_gap"}), 450*MS)
        self.gate.notify_reset(460*MS)
        self.gate.observe(*current(470, count=2, event=True), 470*MS)
        result = self.evaluate(count=2)
        self.assertEqual(result.evaluation_epoch_ns, self.command.evaluation_epoch_ns)
        self.assertFalse(result.command_usable)
        self.assertEqual(result.inhibition_reason, "snapshot_predates_current_acquisition")

    def test_equal_timestamp_reacquisition_cannot_relabel_pre_reset_snapshot(self):
        gate = ready_gate()
        # The previously captured snapshot is also stamped 0.4 s; event ordering
        # cannot be proved by that timestamp after a reset at the same stamp.
        gate.notify_reset(400*MS)
        gate.observe(*current(400, count=2, event=True), 400*MS)
        result = gate.evaluate(self.snapshot, self.command, *current(500, count=2), 500*MS)
        self.assertFalse(result.command_usable)
        self.assertEqual(result.inhibition_reason, "fresh_post_reacquisition_snapshot_required")

    def test_reset_requires_new_acquisition_event_and_count(self):
        self.gate.notify_reset(450*MS)
        for count, event in ((1, False), (1, True), (2, False)):
            with self.subTest(count=count, event=event):
                result = self.evaluate(count=count, event=event)
                self.assertFalse(result.command_usable)
                self.assertEqual(result.inhibition_reason, "fresh_acquisition_event_required")
        gate = ready_gate()
        result = gate.evaluate(self.snapshot, self.command, *current(500, count=2, event=True), 500*MS)
        self.assertFalse(result.command_usable)
        self.assertEqual(result.inhibition_reason, "acquisition_sequence_changed_without_reset")

    def test_stale_future_inconsistent_and_missing_status_are_rejected_exactly(self):
        cases = ["stale", "future", "quality_header", "sim_epoch", "decision_epoch", "publication", "missing", "malformed"]
        for case in cases:
            with self.subTest(case=case):
                nav, row = current(500)
                if case in ("stale", "future"):
                    nav, row = current(499 if case == "stale" else 501)
                elif case == "quality_header":
                    nav.quality_header_ns -= 1
                elif case == "sim_epoch":
                    nav.quality["sim_epoch_ns"] -= 1
                elif case == "decision_epoch":
                    row["consumer_epoch_ns"] -= 1
                elif case == "publication":
                    nav.quality["nav_published_epoch_ns"] -= 1
                elif case == "malformed":
                    setattr(nav, "quality", None)
                else:
                    nav.quality = {}
                result = ready_gate().evaluate(self.snapshot, self.command, nav, row, 500*MS)
                self.assertFalse(result.command_usable)
                self.assertTrue(result.inhibition_reason)

    def test_missing_invalid_and_mismatched_command_provenance_inhibits(self):
        bad_snapshot = replace(self.snapshot, magnetic_provenance="")
        cases = [(None, None), (bad_snapshot, replace(self.command, snapshot=bad_snapshot)),
                 (self.snapshot, replace(self.command, snapshot=replace(self.snapshot, navigation_provenance="different"))),
                 (self.snapshot, replace(self.command, configuration_fingerprint="wrong")),
                 (self.snapshot, replace(self.command, command_valid=False)),
                 (self.snapshot, replace(self.command, predicted_torque_B_Nm=(float("nan"), 0., 0.)))]
        for snapshot, command in cases:
            result = self.gate.evaluate(snapshot, command, *current(500), 500*MS)
            self.assertFalse(result.command_usable)
            self.assertTrue(result.inhibition_reason)

    def test_deadline_no_fallback_and_no_reusable_historical_decision(self):
        self.assertTrue(self.evaluate().command_usable)
        self.assertEqual(self.evaluate(600).inhibition_reason, "wrong_evaluation_epoch")
        # Backward evaluation cannot resurrect a former true result.
        self.assertEqual(self.evaluate(500).inhibition_reason, "nonmonotonic_health_epoch")
        nav, row = current(500)
        nav.source = Source.SIMPLE_NAV
        self.assertEqual(ready_gate().evaluate(self.snapshot, self.command, nav, row, 500*MS).inhibition_reason,
                         "wrong_source_no_fallback")

    def test_structural_isolation_and_immutable_diagnostic_result(self):
        tree = ast.parse(inspect.getsource(module))
        attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertTrue({"write", "subscribeTo", "AddModelToTask", "addDynamicEffector", "controller_step",
                         "evaluate_snapshot", "mtbCmdOutMsg", "cmdTorqueOutMsg"}.isdisjoint(attributes))
        result = self.evaluate()
        self.assertFalse(result.actuator_authority)
        self.assertFalse(hasattr(self.gate, "mtbCmdOutMsg"))
        with self.assertRaises(FrozenInstanceError):
            setattr(result.command, "command_valid", False)
        runner = Path(__file__).resolve().parent
        for name in ("scenario_huskysat2_detumble.py", "magnetic_control_cycle.py", "basilisk_adcs_adapter.py",
                     "attitude_mekf_adapter.py", "disconnected_detumble_math.py"):
            self.assertNotIn("command_health_gate", (runner/name).read_text(encoding="utf-8"))

    def test_actual_adapter_consumer_fault_reset_and_fresh_command_lifecycle(self):
        # Real existing MEKF adapter/status and persistent Phase 7E fault latch.
        # Synthetic gyro/two-vector inputs only; no plant/actuator/task exists.
        inputs = DevelopmentChannel[InputBatch]()
        adapter = MEKFNavigationAdapter(inputs, ShadowOptions())
        port = NavPort(Source.MEKF, adapter.navOutMsg, adapter.statusOut)
        consumer = DummyNavigationConsumer({Source.MEKF: port}, ConsumerOptions(((0, Source.MEKF),)))
        gate = module.CommandHealthGate()

        def send(ms, start_ms=None, *, pair=False, bad_epoch=False, reset=False):
            tick = ms*MS
            gyro = np.array([0.2, -0.1, 0.05])
            intervals = causal_gyro_intervals(start_ms*MS, tick, gyro, gyro, 10) if start_ms is not None else []
            vectors = [VectorSample(f"{name}-{tick}", name, tick, tick, ref, ref, "7F-2B ASSUMED / TEST-ONLY")
                       for name, ref in (("magnetic", np.array([1., 0., 0.])), ("sun", np.array([0., 1., 0.])))] if pair else []
            inputs.write(InputBatch(tick, tick-100*MS if bad_epoch else tick, gyro,
                start_ms*MS if start_ms is not None else None, intervals, vectors, reset_acquisition=reset), tick)
            if reset:
                gate.notify_reset(tick)
            adapter.UpdateState(tick)
            return port.snapshot(), consumer.consume(tick)

        gate.observe(*send(300, pair=True), 300*MS)
        nav, row = send(400, 300)
        gate.observe(nav, row, 400*MS)
        snapshot, command = calculation(nav=nav)
        fault = gate.evaluate(snapshot, command, *send(500, 400, bad_epoch=True), 500*MS)
        self.assertFalse(fault.command_usable)
        self.assertIn("estimator_fault:gyro_or_batch_epoch_mismatch", fault.inhibition_reason)
        self.assertTrue(command.command_valid)
        reset = gate.evaluate(snapshot, command, *send(600, reset=True), 600*MS)
        self.assertFalse(reset.command_usable)
        self.assertEqual(reset.current_health.lifecycle_state, "REACQUIRING")
        acquired = gate.evaluate(snapshot, command, *send(700, pair=True), 700*MS)
        self.assertTrue(acquired.current_health.healthy)
        self.assertFalse(acquired.command_usable)
        nav, row = send(1400, 700)
        gate.observe(nav, row, 1400*MS)
        fresh, new_command = calculation(1400, count=2, nav=nav)
        restored = gate.evaluate(fresh, new_command, *send(1500, 1400), 1500*MS)
        self.assertTrue(restored.command_usable)
        self.assertEqual(restored.current_health.acquisition_count, 2)
        self.assertEqual(restored.sample_epoch_ns, 1400*MS)


if __name__ == "__main__":
    unittest.main()
