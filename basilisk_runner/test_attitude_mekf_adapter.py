"""Phase 7D interface tests; no repeated estimator-law tests or flight inputs."""
from dataclasses import replace
import contextlib
import io
import unittest

import numpy as np
import pandas as pd
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf import MEKF, VectorSample
from attitude_mekf_adapter import (DevelopmentChannel, InputBatch, MEKFNavigationAdapter,
                                   ShadowOptions, causal_gyro_intervals, IdealLiveBridge)
from attitude_mekf_prototype import load_test_policy
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_mekf_shadow import equivalence


def ideal_pair(tick, sigma=(0., 0., 0.)):
    c = np.asarray(rbk.MRP2C(sigma))
    return [VectorSample(f"{sensor}-{tick}", sensor, tick, tick, c @ ref, ref, "TEST-ONLY input")
            for sensor, ref in (("magnetic", np.array([1., 0., 0.])), ("sun", np.array([0., 1., 0.])))]


class AdapterTests(unittest.TestCase):
    def fixture(self, options=None):
        channel = DevelopmentChannel[InputBatch]()
        adapter = MEKFNavigationAdapter(channel, options or ShadowOptions())
        reader = messaging.NavAttMsgReader()
        reader.subscribeTo(adapter.navOutMsg)
        return channel, adapter, reader

    def send(self, channel, adapter, tick, deliveries=(), gyro=(0., 0., 0.), start=None, **kwargs):
        intervals = causal_gyro_intervals(start, tick, gyro, gyro, 10) if start is not None else []
        batch = InputBatch(tick, tick, np.array(gyro), start, intervals, list(deliveries), **kwargs)
        channel.write(batch, tick)
        adapter.UpdateState(tick)
        return adapter.statusOut.read()

    def test_initialization_gates_and_no_uninitialized_gyro_propagation(self):
        for selected in ([], ideal_pair(0)[:1], ideal_pair(0)[1:]):
            channel, adapter, reader = self.fixture()
            self.send(channel, adapter, 0, selected)
            status = self.send(channel, adapter, 100_000_000, start=0)
            self.assertFalse(status["initialized"])
            self.assertIsNone(adapter.engine)
            self.assertFalse(reader.isWritten())
        channel, adapter, reader = self.fixture()
        self.assertTrue(self.send(channel, adapter, 0, ideal_pair(0))["valid"])
        self.assertTrue(reader.isWritten())
        self.assertEqual(reader.timeWritten(), 0)

    def test_navatt_frame_epoch_and_current_point_rate_after_correction(self):
        channel, adapter, reader = self.fixture()
        sigma = [0.1, -0.2, 0.05]
        self.send(channel, adapter, 0, ideal_pair(0, sigma), gyro=[0.2, -0.3, 0.1])
        payload = reader()
        np.testing.assert_allclose(rbk.MRP2C(payload.sigma_BN), rbk.MRP2C(sigma), atol=1e-15)
        np.testing.assert_array_equal(payload.omega_BN_B, [0.2, -0.3, 0.1])
        self.assertEqual(payload.timeTag, 0.)
        status = self.send(channel, adapter, 100_000_000, start=0, gyro=[0.2, -0.3, 0.1])
        self.assertTrue(status["valid"])
        self.assertEqual(reader.timeWritten(), 100_000_000)
        self.assertEqual(reader().timeTag, 0.1)

    def test_core_current_rate_accessor_is_nonmutating_and_uses_posterior_bias(self):
        policy, _ = load_test_policy()
        core = MEKF(policy, prior_q=[1., 0., 0., 0.], prior_bias=[0.01, -0.02, 0.03])
        p, q, b = core.p.copy(), core.q.copy(), core.bias.copy()
        np.testing.assert_allclose(core.point_rate([0.2, 0.3, -0.1], 0), [0.19, 0.32, -0.13], atol=1e-16)
        for old, new in ((p, core.p), (q, core.q), (b, core.bias)):
            np.testing.assert_array_equal(old, new)
        with self.assertRaises(ValueError):
            core.point_rate([0., 0., 0.], 1)

    def test_explicit_sensor_mount_and_sign_transfer(self):
        c_sb = np.asarray(rbk.PRV2C([0.4, -0.2, 0.1]))
        pair = ideal_pair(0, [0.1, 0.2, -0.1])
        mounted = [replace(item, measured=c_sb @ item.measured, frame="S", c_sb=c_sb) for item in pair]
        channel, adapter, reader = self.fixture()
        self.assertTrue(self.send(channel, adapter, 0, mounted)["valid"])
        np.testing.assert_allclose(rbk.MRP2C(reader().sigma_BN), rbk.MRP2C([0.1, 0.2, -0.1]), atol=1e-15)

    def test_gyro_fault_latches_and_old_nav_is_not_republished(self):
        channel, adapter, reader = self.fixture()
        self.send(channel, adapter, 0, ideal_pair(0))
        bad = InputBatch(100_000_000, 0, np.zeros(3), 0)
        channel.write(bad, 100_000_000)
        adapter.UpdateState(100_000_000)
        self.assertFalse(adapter.statusOut.read()["valid"])
        self.assertEqual(reader.timeWritten(), 0)
        self.send(channel, adapter, 200_000_000, start=100_000_000)
        self.assertFalse(adapter.statusOut.read()["valid"])
        status = self.send(channel, adapter, 300_000_000, ideal_pair(300_000_000), reset_acquisition=True)
        self.assertTrue(status["valid"])
        self.assertEqual(reader.timeWritten(), 300_000_000)
        self.assertEqual(status["acquisitions"], 2)
        self.assertTrue(status["reacquisition_event"])

    def test_nonfinite_epoch_and_missing_batch_fail_explicitly(self):
        channel, adapter, reader = self.fixture()
        channel.write(InputBatch(0, float("nan"), np.zeros(3), None), 0)
        adapter.UpdateState(0)
        self.assertEqual(adapter.statusOut.read()["state"], "FAULT")
        self.assertFalse(reader.isWritten())
        channel, adapter, reader = self.fixture()
        adapter.UpdateState(0)
        self.assertIn("stale_or_missing_input_batch", adapter.statusOut.read()["fault"])

    def test_invalid_vector_degrades_without_fabricated_measurement(self):
        channel, adapter, reader = self.fixture()
        self.send(channel, adapter, 0, ideal_pair(0))
        bad = replace(ideal_pair(100_000_000)[0], valid=False, invalid_reason="magnetic_invalid")
        status = self.send(channel, adapter, 100_000_000, [bad], start=0)
        self.assertTrue(status["valid"])
        self.assertEqual(status["updates"], {"magnetic": 1, "sun": 1})
        self.assertEqual(status["rejected"]["magnetic_invalid"], 1)
        self.assertEqual(status["sensor_epochs"]["magnetic"], 100_000_000)
        self.assertEqual(status["accepted_sensor_epochs"]["magnetic"], 0)
        self.assertEqual(status["accepted_sample_ages_ns"]["magnetic"], 100_000_000)
        nonfinite = replace(ideal_pair(200_000_000)[1], measured=[np.nan, 1., 0.])
        status = self.send(channel, adapter, 200_000_000, [nonfinite], start=100_000_000)
        self.assertTrue(status["valid"])
        self.assertEqual(sum(status["rejected"].values()), 2)

    def test_degenerate_pair_and_delayed_startup_cannot_initialize(self):
        channel, adapter, reader = self.fixture()
        first, second = ideal_pair(0)
        second = replace(second, measured=first.measured, reference_n=first.reference_n)
        status = self.send(channel, adapter, 0, [first, second])
        self.assertFalse(status["initialized"])
        self.assertEqual(status["rejected"]["acquisition_geometry"], 1)
        status = self.send(channel, adapter, 100_000_000, ideal_pair(0), start=0)
        self.assertFalse(status["initialized"])
        self.assertEqual(status["rejected"]["initial_pair_not_current"], 2)

    def test_delayed_out_of_order_and_history_limit(self):
        channel, adapter, reader = self.fixture()
        self.send(channel, adapter, 0, ideal_pair(0))
        for i in range(1, 36):
            tick = i*100_000_000
            events = []
            if i == 10:
                events = ideal_pair(800_000_000)
            if i == 35:
                events = ideal_pair(100_000_000)
            status = self.send(channel, adapter, tick, events, start=tick-100_000_000)
        self.assertTrue(status["valid"])
        self.assertEqual(status["replays"], 2)
        self.assertEqual(status["rejected"]["outside_history"], 2)


class LiveIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cycle = diagnostic_cycle_config()
        cls.options = ShadowOptions(ideal_sun=True, magnetic_delay_ns=200_000_000)
        with contextlib.redirect_stdout(io.StringIO()):
            cls.baseline = run(stop_time_s=4., cycle=cls.cycle, write_outputs=False, make_plots=False)
            cls.live = run(stop_time_s=4., cycle=cls.cycle, shadow=cls.options, write_outputs=False, make_plots=False)

    def test_shadow_host_and_controller_telemetry_are_exactly_unchanged(self):
        pd.testing.assert_frame_equal(self.baseline, self.live, check_exact=True)
        self.assertNotIn("shadow_telemetry", self.baseline.attrs)

    def test_actual_basilisk_message_schedule_and_delayed_sample_ages(self):
        telemetry = self.live.attrs["shadow_telemetry"]
        valid = telemetry.loc[telemetry.valid]
        self.assertEqual(int(valid.time_ns.iloc[0]), 400_000_000)
        for field in ("nav_written_ns", "estimator_state_epoch_ns", "gyro_epoch_ns", "status_epoch_ns",
                      "truth_epoch_ns", "simple_nav_epoch_ns"):
            np.testing.assert_array_equal(valid[field].to_numpy(), valid.time_ns.to_numpy())
        delayed = telemetry.loc[telemetry.time_ns == 1_600_000_000].iloc[0]
        self.assertEqual(int(delayed.magnetic_epoch_ns), 1_400_000_000)
        self.assertEqual(int(delayed.magnetic_age_ns), 200_000_000)
        self.assertGreater(int(telemetry.replays.iloc[-1]), 0)

    def test_live_offline_same_events_match_q_bias_covariance_counts_epochs(self):
        checked = equivalence(self.live, self.options)
        self.assertTrue(checked["passed"], checked)

    def test_live_magnetic_values_are_acquired_tam_and_never_actuation_fields(self):
        by_tick = self.live.set_index("time_ns")
        for batch in self.live.attrs["shadow_input_trace"]:
            for measurement in batch.deliveries:
                if measurement.sensor == "magnetic":
                    row = by_tick.loc[measurement.epoch_ns]
                    self.assertEqual(row.cycle_phase, "SAMPLE")
                    np.testing.assert_array_equal(measurement.measured,
                        [row[f"tam_sample_B_B_{axis}_T"] for axis in "xyz"])
                    self.assertTrue(measurement.valid)
                    self.assertEqual(measurement.epoch_ns, measurement.reference_epoch_ns)

    def test_missing_sources_and_loss_after_acquisition(self):
        cases = [(ShadowOptions(), False), (ShadowOptions(ideal_sun=True, magnetic_enabled=False), False),
                 (ShadowOptions(magnetic_enabled=False), False),
                 (ShadowOptions(magnetic_enabled=False, initial_q_BN=(1., 0., 0., 0.)), True),
                 (ShadowOptions(ideal_sun=True, drop_sun_after_ns=1_000_000_000), True)]
        for options, initialized in cases:
            with contextlib.redirect_stdout(io.StringIO()):
                host = run(stop_time_s=2., cycle=self.cycle, shadow=options, write_outputs=False, make_plots=False)
            status = host.attrs["shadow_status"][-1]
            self.assertEqual(status["initialized"], initialized)
            self.assertFalse(status["fault"])

    def test_cli_default_and_explicit_sun_selection(self):
        from unittest.mock import patch
        from scenario_huskysat2_detumble import main
        with patch("scenario_huskysat2_detumble.run") as call:
            main(["--no-plots"])
            self.assertIsNone(call.call_args.kwargs["shadow"])
            main(["--navigation", "shadow-mekf", "--shadow-ideal-sun", "--no-plots"])
            self.assertTrue(call.call_args.kwargs["shadow"].ideal_sun)

    def test_bridge_rejects_missing_stale_and_nonfinite_native_tam(self):
        from types import SimpleNamespace
        state, tam, magnetic = messaging.SCStatesMsg(), messaging.TAMSensorMsg(), messaging.MagneticFieldMsg()
        tick = 400_000_000
        state.write(messaging.SCStatesMsgPayload(), tick)
        magnetic.write(messaging.MagneticFieldMsgPayload(), tick)
        driver = SimpleNamespace(cycle=self.cycle, history=[self.baseline.iloc[4].to_dict()],
                                 effector=SimpleNamespace(torqueExternalPntB_B=[0., 0., 0.]))
        bridge = IdealLiveBridge(state, tam, magnetic, driver, np.eye(3), ShadowOptions(), 100_000_000)
        self.assertFalse(bridge.magnetic_acquisition(tick).valid)
        payload = messaging.TAMSensorMsgPayload()
        payload.tam_S = [1e-5, 2e-5, 3e-5]
        tam.write(payload, 300_000_000)
        self.assertFalse(bridge.magnetic_acquisition(tick).valid)
        payload.tam_S = [float("nan"), 2e-5, 3e-5]
        tam.write(payload, tick)
        bridge.UpdateState(tick)
        batch = bridge.out.read()
        self.assertTrue(batch.gyro_valid)
        self.assertFalse(batch.deliveries[0].valid)
        state.write(messaging.SCStatesMsgPayload(), 300_000_000)
        bridge.UpdateState(tick)
        batch = bridge.out.read()
        self.assertFalse(batch.gyro_valid)
        self.assertEqual(batch.gyro_epoch_ns, 300_000_000)  # Never relabeled as now.

    def test_continuous_shadow_does_not_claim_a_quiet_magnetic_sample(self):
        with contextlib.redirect_stdout(io.StringIO()):
            host = run(stop_time_s=1., shadow=ShadowOptions(ideal_sun=True), write_outputs=False, make_plots=False)
        status = host.attrs["shadow_status"][-1]
        self.assertFalse(status["initialized"])
        self.assertEqual(status["source_status"]["magnetic"], "no_quiet_contract")
        self.assertEqual(status["updates"].get("magnetic", 0), 0)


if __name__ == "__main__":
    unittest.main()
