"""Opt-in gyro source isolation and exact live ideal equivalence."""
from dataclasses import replace
import unittest

import numpy as np
from Basilisk.architecture import messaging

from attitude_mekf_adapter import DevelopmentChannel, InputBatch, MEKFNavigationAdapter, ShadowOptions
from gyro_sensor_model import GyroModel, fixture, profile_config
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_gyro_sensor_model import validate


class GyroShadowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.frame = validate()

    def test_complete_live_ideal_gate(self):
        for name, passed in self.report["checks"].items():
            with self.subTest(check=name):
                self.assertTrue(passed)
        self.assertEqual(self.report["live_samples"], 31)
        self.assertTrue(self.report["passed"])

    def test_production_preservation(self):
        self.assertTrue(self.report["production_preservation"]["passed"])

    def test_live_rejects_latency_different_cadence_and_sample_offset(self):
        configs = (profile_config("TEST_DELAYED_SAMPLE"),
            replace(profile_config(), sample_period_ns=fixture(200_000_000, "ns", "simulation clock", "synthetic slower cadence")),
            replace(profile_config(), sample_offset_ns=fixture(100_000_000, "ns", "simulation clock", "synthetic offset")))
        for config in configs:
            with self.subTest(profile=config.profile), self.assertRaisesRegex(ValueError, "isolated-only"):
                ShadowOptions(gyro_model=config).validate(100_000_000)

    def test_delayed_gyro_cannot_be_relabelled_as_current_MEKF_input(self):
        sample = GyroModel(profile_config("TEST_DELAYED_SAMPLE")).acquire([.1, -.2, .3], 0)
        channel = DevelopmentChannel[InputBatch]()
        adapter = MEKFNavigationAdapter(channel, ShadowOptions())
        reader = messaging.NavAttMsgReader()
        reader.subscribeTo(adapter.navOutMsg)
        channel.write(InputBatch(sample.publication_ns, sample.acquisition_ns,
            np.array(sample.measurement_B_rad_s), None, gyro_sample=sample), sample.publication_ns)
        adapter.UpdateState(sample.publication_ns)
        self.assertEqual(adapter.statusOut.read()["fault"], "gyro_or_batch_epoch_mismatch")
        self.assertFalse(reader.isWritten())
        self.assertEqual(sample.age_ns(sample.publication_ns), 200_000_000)

    def test_existing_control_guard_rejects_all_modeled_gyro_sources(self):
        for name in ("IDEAL_REGRESSION", "TEST_BIAS_ONLY", "TEST_NOISE_ONLY"):
            with self.subTest(profile=name), self.assertRaisesRegex(ValueError, "MEKF_DEVELOPMENT requires"):
                run(stop_time_s=1., write_outputs=False, make_plots=False, cycle=diagnostic_cycle_config(),
                    shadow=ShadowOptions(ideal_sun=True, gyro_model=profile_config(name)), control_source="MEKF_DEVELOPMENT")


if __name__ == "__main__":
    unittest.main()
