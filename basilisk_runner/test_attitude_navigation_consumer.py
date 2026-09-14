"""Phase 7E consumer boundaries; estimator mathematics remain covered by 7C/7D."""
import ast
import inspect
import json
import unittest

import numpy as np
from Basilisk.architecture import messaging

import attitude_navigation_consumer as consumer_module
from attitude_mekf_adapter import DevelopmentChannel
from attitude_navigation_consumer import ConsumerOptions, DummyNavigationConsumer, NavPort, Source, representation
from validate_navigation_consumer import publish_nav, fault_recovery_case, ideal_handover, live_cases, degraded_adapter_cases


class ConsumerContractTests(unittest.TestCase):
    def fixture(self, *, max_age_ns=0, selections=((0, Source.MEKF),), include_mekf=True):
        self.messages = {source: messaging.NavAttMsg() for source in (Source.SIMPLE_NAV, Source.MEKF)}
        self.qualities = {source: DevelopmentChannel[dict]() for source in self.messages}
        ports = {source: NavPort(source, message, self.qualities[source]) for source, message in self.messages.items()
                 if source != Source.MEKF or include_mekf}
        return DummyNavigationConsumer(ports, ConsumerOptions(selections), max_age_ns)

    def publish(self, tick, source=Source.MEKF, **changes):
        publish_nav(self.messages[source], self.qualities[source], tick, source=source, **changes)

    def test_numeric_zero_is_valid_only_with_explicit_quality(self):
        consumer = self.fixture()
        self.publish(0, initialized=False, valid=False, state="UNINITIALIZED")
        row = consumer.consume(0)
        self.assertEqual(row["consumer_state"], "UNINITIALIZED")
        self.assertIsNone(row["navigation"])
        self.publish(100_000_000)
        row = consumer.consume(100_000_000)
        self.assertEqual(row["consumer_state"], "VALID")
        self.assertEqual(row["navigation"]["omega_BN_B_rad_s"], [0., 0., 0.])

    def test_unwritten_quality_never_authorizes_native_nav(self):
        consumer = self.fixture()
        self.messages[Source.MEKF].write(messaging.NavAttMsgPayload(), 0)
        row = consumer.consume(0)
        self.assertEqual(row["consumer_state"], "DEGRADED")
        self.assertEqual(row["selected_source"], "NONE")

    def test_stale_quality_and_fresh_recovery(self):
        consumer = self.fixture()
        self.publish(0)
        self.assertTrue(consumer.consume(0)["accepted"])
        stale = consumer.consume(100_000_000)
        self.assertEqual(stale["consumer_state"], "STALE")
        self.assertEqual(stale["publication_epoch_ns"], 0)
        self.assertEqual(stale["navigation_age_ns"], 100_000_000)
        self.publish(200_000_000)
        self.assertTrue(consumer.consume(200_000_000)["accepted"])

    def test_republication_does_not_refresh_state_age(self):
        consumer = self.fixture()
        self.publish(0)
        consumer.consume(0)
        self.publish(100_000_000, stamp=0)
        row = consumer.consume(100_000_000)
        self.assertEqual(row["rejection_reason"], "stale_state")
        self.assertEqual(row["navigation_age_ns"], 100_000_000)
        self.assertFalse(row["state_advanced"])
        self.assertTrue(row["publication_advanced"])

    def test_test_only_age_boundary_is_inclusive_and_state_based(self):
        consumer = self.fixture(max_age_ns=100_000_000)
        self.publish(100_000_000, stamp=0)
        self.assertTrue(consumer.consume(100_000_000)["accepted"])
        self.publish(100_000_001, stamp=0)
        row = consumer.consume(100_000_001)
        self.assertEqual(row["rejection_reason"], "stale_state")
        self.assertEqual(row["navigation_age_ns"], 100_000_001)

    def test_frozen_consumer_requires_exact_sample_not_merely_recent_nav(self):
        consumer = self.fixture(max_age_ns=100_000_000)
        self.publish(400_000_000)
        saved = {Source.MEKF: consumer.ports[Source.MEKF].snapshot()}
        self.publish(500_000_000)  # A current point is NOT the frozen sample.
        self.assertEqual(consumer.consume(500_000_000, required_epoch_ns=400_000_000)["rejection_reason"], "wrong_required_sample_epoch")
        row = consumer.consume(500_000_000, saved, 400_000_000)
        self.assertTrue(row["accepted"])
        self.assertEqual((row["state_epoch_ns"], row["publication_epoch_ns"], row["consumer_epoch_ns"]),
                         (400_000_000, 400_000_000, 500_000_000))

    def test_future_epoch_and_inconsistent_headers_fail(self):
        for changes in ({"stamp": 200_000_000, "publication": 200_000_000},
                        {"publication": 200_000_000}, {"nav_published_epoch_ns": 0},
                        {"gyro_epoch_ns": 0}, {"state_epoch_ns": float("nan")}, {"acquisitions": float("nan")}):
            with self.subTest(changes=changes):
                consumer = self.fixture()
                self.publish(100_000_000, **changes)
                row = consumer.consume(100_000_000)
                self.assertEqual(row["consumer_state"], "FAULTED")
                self.assertIsNone(row["navigation"])
                json.dumps(row, allow_nan=False)

    def test_future_quality_and_payload_time_mismatch_fail(self):
        consumer = self.fixture()
        self.publish(200_000_000)
        self.assertEqual(consumer.consume(100_000_000)["rejection_reason"], "future_quality")
        consumer = self.fixture()
        self.publish(100_000_000)
        payload = self.messages[Source.MEKF].read()
        payload.timeTag = 0.
        self.messages[Source.MEKF].write(payload, 100_000_000)
        self.assertEqual(consumer.consume(100_000_000)["rejection_reason"], "payload_state_epoch_mismatch")

    def test_nonfinite_attitude_and_rate_latch(self):
        for changes in ({"sigma": [np.nan, 0., 0.]}, {"omega": [0., np.inf, 0.]}):
            consumer = self.fixture()
            self.publish(0, **changes)
            self.assertEqual(consumer.consume(0)["consumer_state"], "FAULTED")
            self.publish(100_000_000)
            row = consumer.consume(100_000_000)
            self.assertEqual(row["consumer_state"], "FAULTED")
            self.assertTrue(row["fault_latched"])

    def test_source_selection_never_implicitly_falls_back(self):
        for state, valid, initialized, fault in (("UNINITIALIZED", False, False, ""),
                                                ("FAULT", False, True, "fixture_fault"),
                                                ("INITIALIZED", False, True, "")):
            consumer = self.fixture()
            self.publish(0, source=Source.SIMPLE_NAV)
            self.publish(0, state=state, valid=valid, initialized=initialized, fault=fault)
            self.assertEqual(consumer.consume(0)["selected_source"], "NONE")
        consumer = self.fixture(include_mekf=False)
        self.publish(0, source=Source.SIMPLE_NAV)
        self.assertEqual(consumer.consume(0)["rejection_reason"], "source_unavailable")
        for requested in (Source.NONE, Source.SIMPLE_NAV):
            consumer = self.fixture(selections=((0, requested),))
            self.publish(0, source=Source.SIMPLE_NAV)
            self.assertEqual(consumer.consume(0)["selected_source"], requested.value)
        self.assertEqual(ConsumerOptions().requested(0), Source.SIMPLE_NAV)

    def test_source_identity_and_selection_command_order(self):
        consumer = self.fixture()
        self.publish(0)
        sample = consumer.ports[Source.MEKF].snapshot()
        sample.quality["source"] = "SIMPLE_NAV"
        self.assertEqual(consumer.consume(0, {Source.MEKF: sample})["rejection_reason"], "source_identity_mismatch")
        with self.assertRaises(ValueError):
            ConsumerOptions(((1, Source.MEKF), (0, Source.NONE))).requested(1)

    def test_principal_shadow_quaternion_and_rate_frame(self):
        sigma, rate = np.array([0.2, -0.1, 0.05]), [0.02, -0.03, 0.04]
        a, b = representation(sigma, rate), representation(-sigma/(sigma @ sigma), rate)
        np.testing.assert_allclose(a["C_BN"], b["C_BN"], atol=1e-15)
        np.testing.assert_allclose(a["q_BN"], b["q_BN"], atol=1e-15)
        np.testing.assert_array_equal(a["omega_BN_B_rad_s"], rate)
        x = representation([np.tan(np.pi/8), 0., 0.], rate)
        np.testing.assert_allclose(np.array(x["C_BN"]) @ [0., 1., 0.], [0., 0., -1.], atol=1e-15)

    def test_unequal_epoch_handover_is_reported_without_false_angle_comparison(self):
        consumer = self.fixture(max_age_ns=100_000_000,
                                selections=((100_000_000, Source.MEKF),))
        self.publish(0, source=Source.SIMPLE_NAV)
        consumer.consume(0)
        self.publish(100_000_000, source=Source.SIMPLE_NAV, stamp=0)
        self.publish(100_000_000)
        row = consumer.consume(100_000_000)
        self.assertEqual(row["handover_state_epoch_jump_ns"], 100_000_000)
        self.assertEqual(row["handover_comparison"], "different_state_epochs")
        self.assertIsNone(row["handover_attitude_jump_rad"])

    def test_consumer_has_no_truth_sensor_or_actuator_dependency(self):
        tree = ast.parse(inspect.getsource(consumer_module))
        message_apis = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
                        and isinstance(node.value, ast.Name) and node.value.id == "messaging"}
        self.assertEqual(message_apis, {"NavAttMsgReader"})
        # Basilisk wraps both the class and __init__ with *args/**kwargs.
        # Inspect the first-party AST to audit the declared input surface.
        definition = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "DummyNavigationConsumer")
        initializer = next(node for node in definition.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
        self.assertEqual({arg.arg for arg in initializer.args.args}, {"self", "ports", "options", "max_age_ns"})
        self.assertEqual(set(inspect.signature(consumer_module.attach_consumers).parameters),
                         {"sim", "simple_message", "mekf_message", "mekf_quality", "cycle", "options"})
        consumer = self.fixture()
        self.publish(0)
        prior = consumer.ports[Source.MEKF].snapshot()
        consumer.consume(0)
        after = consumer.ports[Source.MEKF].snapshot()
        self.assertEqual(prior.quality, after.quality)
        self.assertEqual(prior.sigma, after.sigma)
        self.assertEqual(prior.header_ns, after.header_ns)


class ConsumerIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.live = live_cases()

    def test_live_startup_and_exact_frozen_epochs(self):
        self.assertTrue(self.live["startup"]["passed"])
        frozen = self.live["startup"]["frozen_telemetry"][0]
        self.assertEqual((frozen["consumer_epoch_ns"], frozen["state_epoch_ns"]), (500_000_000, 400_000_000))

    def test_live_source_handover_and_default_isolation(self):
        self.assertTrue(self.live["handover"]["passed"])
        self.assertTrue(self.live["default_selection"]["passed"])
        rows = self.live["handover"]["telemetry"]
        self.assertEqual(rows[10]["selected_source"], "MEKF")
        self.assertEqual(rows[30]["selected_source"], "SIMPLE_NAV")
        self.assertEqual(rows[10]["handover_state_epoch_jump_ns"], 0)

    def test_loss_delay_prior_and_single_vector_follow_estimator_quality(self):
        for name in ("sun_loss", "delayed_magnetic", "gyro_only_prior", "single_vector"):
            self.assertTrue(self.live[name]["passed"], name)
        self.assertGreater(self.live["delayed_magnetic"]["estimator_final_quality"]["replays"], 0)
        self.assertGreater(sum(self.live["sun_loss"]["estimator_final_quality"]["rejected"].values()), 0)

    def test_supported_faults_latch_until_explicit_reset_and_reacquisition(self):
        for kind in ("nonfinite_gyro", "coverage", "explicit_test_fault", "nonfinite_navigation"):
            result = fault_recovery_case(kind)
            self.assertTrue(result["passed"], (kind, result["checks"]))

    def test_history_and_invalid_windows_degrade_measurements_not_nav_validity(self):
        checked = degraded_adapter_cases()
        self.assertTrue(checked["passed"], checked["checks"])

    def test_analytic_handover_both_directions(self):
        result = ideal_handover()
        self.assertTrue(result["passed"], result["checks"])

    def test_scenario_default_remains_opt_out(self):
        from scenario_huskysat2_detumble import run
        self.assertIsNone(inspect.signature(run).parameters["navigation_consumer"].default)
        with self.assertRaisesRegex(ValueError, "requires explicit shadow"):
            run(write_outputs=False, make_plots=False, navigation_consumer=ConsumerOptions())


if __name__ == "__main__":
    unittest.main()
