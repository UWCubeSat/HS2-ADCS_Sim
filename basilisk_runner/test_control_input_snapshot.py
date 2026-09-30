"""Phase 7F-1 snapshot/ordering tests, 2026-09-30; no controller is executed.

Fixtures are ASSUMED / TEST-ONLY. Timing comes from the unchanged Phase 6A
diagnostic config. The isolated Basilisk task has no spacecraft or effector.
"""
import ast
from dataclasses import FrozenInstanceError, replace
import inspect
from pathlib import Path
import unittest

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import SimulationBaseClass

import control_input_snapshot as module
from control_input_snapshot import CycleWindow, MagneticSample, capture_control_snapshot
from attitude_mekf import VectorSample
from attitude_mekf_adapter import DevelopmentChannel, InputBatch, MEKFNavigationAdapter, ShadowOptions, causal_gyro_intervals
from attitude_navigation_consumer import ConsumerOptions, DummyNavigationConsumer, NavPort, Snapshot, Source
from magnetic_control_cycle import diagnostic_cycle_config


def window(evaluation_ns=500_000_000):
    cycle = diagnostic_cycle_config()
    return CycleWindow(cycle.period_ns, cycle.sample_offset_ns, cycle.compute_offset_ns, evaluation_ns,
                       "Phase 6A diagnostic config, 2026-09-06; ASSUMED / TEST-ONLY; "+cycle.fingerprint())


def navigation(stamp=400_000_000, publication=None):
    publication = stamp if publication is None else publication
    quality = {"source": "MEKF", "state_epoch_ns": stamp, "nav_published_epoch_ns": publication,
               "status_epoch_ns": publication, "sim_epoch_ns": publication, "gyro_epoch_ns": stamp,
               "valid": True, "initialized": True, "state": "INITIALIZED", "fault": "", "acquisitions": 1}
    return Snapshot(Source.MEKF, publication, stamp*1e-9, [0.1, -0.05, 0.02], [0.2, -0.1, 0.05], publication, quality)


def magnetic(stamp=400_000_000):
    return MagneticSample((2e-5, 3e-5, -1e-5), ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
                          stamp, stamp, True, True, "SAMPLE", True,
                          "TEST-ONLY stored native TAM acquisition evidence, 2026-09-30",
                          (1e-5, 2e-5, 3e-5), stamp)


def capture(nav=None, tam=None, capture_ns=400_000_000, plan=None, decision=None):
    nav, tam, plan = nav or navigation(), tam or magnetic(), plan or window()
    if decision is None:
        consumer = DummyNavigationConsumer({}, ConsumerOptions(((0, Source.MEKF),)), max(0, capture_ns-plan.sample_ns))
        decision = consumer.consume(capture_ns, {Source.MEKF: nav}, plan.sample_ns)
    return capture_control_snapshot(nav, decision, tam, plan, capture_ns,
                                    navigation_provenance="Phase 7E native NavAtt/quality fixture; TEST-ONLY")


class SnapshotContractTests(unittest.TestCase):
    def test_matching_pair_freezes_distinct_sample_capture_evaluation_epochs(self):
        result = capture()
        self.assertTrue(result.accepted, result.rejection_reason)
        snap = result.snapshot
        self.assertEqual((snap.navigation_state_ns, snap.tam_acquisition_ns, snap.capture_ns, snap.window.evaluation_ns),
                         (400_000_000, 400_000_000, 400_000_000, 500_000_000))
        self.assertEqual(snap.evaluation_rejection(500_000_000), "")
        self.assertEqual(snap.evaluation_rejection(600_000_000), "wrong_evaluation_epoch")

    def test_previous_navigation_cannot_pair_with_current_tam(self):
        self.assertFalse(capture(nav=navigation(300_000_000)).accepted)
        # Even refreshing the publication leaves the state epoch old.
        self.assertFalse(capture(nav=navigation(300_000_000, 400_000_000)).accepted)

    def test_current_navigation_cannot_pair_with_stale_tam_or_stale_pair(self):
        self.assertEqual(capture(tam=magnetic(300_000_000)).rejection_reason, "tam_does_not_match_selected_sample_epoch")
        self.assertFalse(capture(capture_ns=1_400_000_000, plan=window(1_500_000_000)).accepted)
        with self.assertRaises(ValueError):
            window(600_000_000)  # Cannot disguise a stale pair with arbitrary evaluation time.

    def test_invalid_uninitialized_faulted_and_latched_navigation_reject(self):
        for changes in ({"valid": False}, {"initialized": False}, {"fault": "gyro_gap"}, {"state": "FAULT"}):
            nav = navigation()
            nav.quality.update(changes)
            self.assertFalse(capture(nav=nav).accepted, changes)
        nav = navigation()
        gate = DummyNavigationConsumer({}, ConsumerOptions(((0, Source.MEKF),)))
        decision = gate.consume(400_000_000, {Source.MEKF: nav})
        decision["fault_latched"] = True
        self.assertEqual(capture(nav=nav, decision=decision).rejection_reason, "navigation_consumer_not_accepting")

    def test_invalid_quiet_or_actuation_period_magnetic_evidence_rejects(self):
        for changes in ({"valid": False}, {"quiet_valid": False}, {"acquisition_phase": "ACTUATE"},
                        {"acquisition_event": False}):
            self.assertFalse(capture(tam=replace(magnetic(), **changes)).accepted, changes)

    def test_future_navigation_tam_and_impossible_publication_reject(self):
        self.assertFalse(capture(nav=navigation(500_000_000)).accepted)
        self.assertFalse(capture(nav=navigation(400_000_000, 500_000_000)).accepted)
        for changes in ({"acquisition_ns": 500_000_000}, {"publication_ns": 500_000_000}, {"publication_ns": 300_000_000}):
            self.assertFalse(capture(tam=replace(magnetic(), **changes)).accepted, changes)

    def test_nonfinite_attitude_rate_tam_reference_and_epochs_reject(self):
        for attr in ("sigma", "omega"):
            nav = navigation()
            setattr(nav, attr, [float("nan"), 0., 0.])
            self.assertFalse(capture(nav=nav).accepted)
        for changes in ({"tam_S_T": (np.inf, 0., 0.)}, {"reference_N_T": (0., np.nan, 0.)},
                        {"acquisition_ns": float("nan")}):
            self.assertFalse(capture(tam=replace(magnetic(), **changes)).accepted)

    def test_later_publication_can_represent_sample_but_cannot_relabel_current_state(self):
        # Explicit historical-state fixture. The live 7D adapter does not expose
        # this interface: its published state always advances to the current tick.
        self.assertTrue(capture(nav=navigation(400_000_000, 500_000_000), capture_ns=500_000_000).accepted)
        self.assertFalse(capture(nav=navigation(500_000_000), capture_ns=500_000_000).accepted)
        nav = navigation(500_000_000)
        nav.quality["state_epoch_ns"] = 400_000_000  # Header/quality relabel is insufficient.
        self.assertFalse(capture(nav=nav, capture_ns=500_000_000).accepted)
        self.assertFalse(capture(capture_ns=500_000_001).accepted)  # No grace interval.

    def test_reference_is_optional_but_if_present_must_belong_to_acquisition(self):
        self.assertTrue(capture(tam=replace(magnetic(), reference_N_T=None, reference_epoch_ns=None)).accepted)
        self.assertEqual(capture(tam=replace(magnetic(), reference_epoch_ns=500_000_000)).rejection_reason,
                         "reference_epoch_mismatch")

    def test_source_or_decision_mismatch_rejects(self):
        nav = navigation()
        nav.quality["source"] = "SIMPLE_NAV"
        self.assertFalse(capture(nav=nav).accepted)
        nav = navigation()
        gate = DummyNavigationConsumer({}, ConsumerOptions(((0, Source.MEKF),)))
        decision = gate.consume(400_000_000, {Source.MEKF: nav})
        decision["received_omega_BN_B_rad_s"] = [1., 2., 3.]
        self.assertEqual(capture(nav=nav, decision=decision).rejection_reason,
                         "navigation_decision_does_not_describe_this_input")

    def test_sensor_to_body_transform_and_immutable_ownership(self):
        nav, raw = navigation(), [2e-5, 3e-5, -1e-5]
        dcm = [[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]]  # Explicit passive C_SB.
        tam = replace(magnetic(), tam_S_T=raw, dcm_SB=dcm)
        snap = capture(nav, tam).snapshot
        np.testing.assert_array_equal(snap.tam_B_T, [-3e-5, 2e-5, -1e-5])
        raw[0], dcm[0][0], nav.omega[0] = 9., 9., 9.
        nav.quality["valid"] = False
        self.assertEqual(snap.tam_S_T[0], 2e-5)
        self.assertEqual(snap.omega_BN_B_rad_s[0], 0.2)
        with self.assertRaises(FrozenInstanceError):
            snap.capture_ns = 0
        with self.assertRaises(TypeError):
            snap.dcm_SB[0][0] = 0.
        self.assertFalse(capture(tam=replace(magnetic(), dcm_SB=((1., 0., 0.), (0., 1., 0.), (0., 0., -1.)))).accepted)
        self.assertFalse(capture(tam=replace(magnetic(), provenance={"mutable": "source"})).accepted)
        with self.assertRaises(ValueError):
            replace(window(), provenance={"mutable": "timing"})

    def test_no_runtime_import_or_command_authority(self):
        source = inspect.getsource(module)
        tree = ast.parse(source)
        imports = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        self.assertEqual(imports, {"__future__", "dataclasses", "typing", "attitude_navigation_consumer"})
        self.assertNotIn("messaging", source)
        runner = Path(__file__).resolve().parent
        for name in ("scenario_huskysat2_detumble.py", "magnetic_control_cycle.py", "basilisk_adcs_adapter.py", "attitude_mekf_adapter.py"):
            self.assertNotIn("control_input_snapshot", (runner/name).read_text(encoding="utf-8"))


class Callback(sysModel.SysModel):
    def __init__(self, callback):
        super().__init__()
        self.callback = callback

    def UpdateState(self, tick):
        self.callback(int(tick))


def isolated_timing_trace():
    """Native messages + actual MEKF in Basilisk; only synthetic sensor packets.

    900 reference, 700 TAM, 600 hypothetical early read, 590 input delivery,
    580 MEKF, 560 quality consumer, 555 snapshot, 550 evaluation-epoch observer.
    The early read reproduces driver priority without instantiating a controller.
    """
    inputs = DevelopmentChannel[InputBatch]()
    adapter = MEKFNavigationAdapter(inputs, ShadowOptions())
    port = NavPort(Source.MEKF, adapter.navOutMsg, adapter.statusOut)
    gate = DummyNavigationConsumer({Source.MEKF: port}, ConsumerOptions(((0, Source.MEKF),)))
    tam_message, field_message = messaging.TAMSensorMsg(), messaging.MagneticFieldMsg()
    tam_reader, field_reader = messaging.TAMSensorMsgReader(), messaging.MagneticFieldMsgReader()
    tam_reader.subscribeTo(tam_message)
    field_reader.subscribeTo(field_message)
    acquisitions, before, after, evaluated = {}, {}, {}, {}
    step = 100_000_000

    def reference(tick):
        payload = messaging.MagneticFieldMsgPayload()
        payload.magField_N = [2e-5+tick*1e-15, 3e-5, -1e-5]
        field_message.write(payload, tick)

    def acquire(tick):
        if tick % 1_000_000_000 != 400_000_000:
            return
        payload = messaging.TAMSensorMsgPayload()
        payload.tam_S = list(field_reader().magField_N)  # TEST-ONLY identity fixture, not spacecraft truth.
        tam_message.write(payload, tick)
        acquisitions[tick] = MagneticSample(tuple(tam_reader().tam_S), magnetic().dcm_SB,
            tick, int(tam_reader.timeWritten()), True, True, "SAMPLE", True,
            "TEST-ONLY native TAM acquisition; no coil model", tuple(field_reader().magField_N), int(field_reader.timeWritten()))

    def early(tick):
        if tick in acquisitions:
            nav = port.snapshot()
            before[tick] = {"navigation_epoch_ns": nav.quality.get("state_epoch_ns"),
                            "publication_ns": nav.header_ns,
                            "result": capture(nav, acquisitions[tick], tick, window(tick+step))}

    def deliver(tick):
        intervals = causal_gyro_intervals(tick-step, tick, [0., 0., 0.], [0., 0., 0.], 10) if tick else []
        events = []
        if tick in acquisitions:
            measured = np.array(acquisitions[tick].tam_S_T)
            events.append(VectorSample(f"mag-{tick}", "magnetic", tick, tick, measured, measured, "TEST-ONLY native TAM fixture"))
        if tick == 400_000_000:
            events.append(VectorSample("sun-initial", "sun", tick, tick, [0., 1., 0.], [0., 1., 0.], "TEST-ONLY Sun"))
        if tick == 1_400_000_000:
            events.append(VectorSample("sun-delayed", "sun", 900_000_000, 900_000_000,
                                       [0., 1., 0.], [0., 1., 0.], "TEST-ONLY delayed Sun"))
        inputs.write(InputBatch(tick, tick, np.zeros(3), tick-step if tick else None, intervals, events), tick)

    def freeze(tick):
        if tick in acquisitions:
            after[tick] = capture_control_snapshot(port.snapshot(), gate.history[-1], acquisitions[tick],
                window(tick+step), tick, navigation_provenance="Actual Phase 7D MEKF NavAtt/quality in isolated fixture")

    def evaluate(tick):
        if tick-step in after:
            snap = after[tick-step].snapshot
            evaluated[tick] = (snap, snap.evaluation_rejection(tick), list(tam_reader().tam_S))

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("SnapshotTestProcess")
    process.addTask(sim.CreateNewTask("SnapshotTestTask", step))
    models = [(Callback(reference), 900), (Callback(acquire), 700), (Callback(early), 600),
              (Callback(deliver), 590), (adapter, 580), (gate, 560), (Callback(freeze), 555), (Callback(evaluate), 550)]
    for model, priority in models:
        sim.AddModelToTask("SnapshotTestTask", model, ModelPriority=priority)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(1_500_000_000)
    sim.ExecuteSimulation()
    return before, after, evaluated, adapter.statusOut.read()


class NativeTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.before, cls.after, cls.evaluated, cls.quality = isolated_timing_trace()

    def test_actual_priority_order_reads_previous_estimate_before_publication(self):
        old = self.before[1_400_000_000]
        self.assertEqual((old["navigation_epoch_ns"], old["publication_ns"]), (1_300_000_000, 1_300_000_000))
        self.assertFalse(old["result"].accepted)
        self.assertFalse(self.before[400_000_000]["result"].accepted)  # Not yet acquired.

    def test_post_publication_freeze_and_exact_later_evaluation(self):
        for stamp, result in self.after.items():
            self.assertTrue(result.accepted, result.rejection_reason)
            snap, reason, payload = self.evaluated[stamp+100_000_000]
            self.assertEqual((snap.navigation_state_ns, snap.navigation_publication_ns, snap.tam_acquisition_ns), (stamp, stamp, stamp))
            self.assertEqual(snap, result.snapshot)
            self.assertEqual(reason, "")
            np.testing.assert_array_equal(snap.tam_S_T, payload)

    def test_replayed_measurement_is_acceptable_only_with_matching_current_sample(self):
        self.assertGreater(self.quality["replays"], 0)
        snap = self.after[1_400_000_000].snapshot
        self.assertEqual(snap.navigation_state_ns, 1_400_000_000)
        self.assertEqual(snap.tam_acquisition_ns, 1_400_000_000)
        self.assertEqual(self.quality["sensor_epochs"]["sun"], 900_000_000)
        # Sun measurement epoch does not become the replayed navigation epoch.


if __name__ == "__main__":
    unittest.main()
