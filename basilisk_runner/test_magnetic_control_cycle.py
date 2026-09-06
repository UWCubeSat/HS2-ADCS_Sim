"""Independent cycle event expectations and deliberately invalid native inputs."""
import contextlib
from dataclasses import replace
import io
import json
from typing import Iterable, cast
import unittest

import numpy as np
import pandas as pd
from Basilisk.architecture import messaging
from Basilisk.simulation import magnetometer, MtbEffector

from basilisk_adcs_adapter import ADCSConfig
from hs2_sim_config import DEFAULT_CONFIG
from magnetic_control_cycle import MagneticCycleDriver, diagnostic_cycle_config, test_duration
from scenario_huskysat2_detumble import run, configure_mtb_config_message
from validate_magnetic_cycle import cycle_checks, analyze
from compare_reference_vs_basilisk import summarize_basilisk, build_validation


class MagneticCycleTests(unittest.TestCase):
    def physics_checks(self, frame):
        checks = build_validation({}, summarize_basilisk(frame))["checks"]
        del checks["reference_numeric_finite"]  # No standalone CSV is an input to this fixture.
        return checks

    @classmethod
    def setUpClass(cls):
        cls.cycle = diagnostic_cycle_config()
        cls.config = DEFAULT_CONFIG.with_run_options(3.)
        with contextlib.redirect_stdout(io.StringIO()):
            cls.frame = run(config=cls.config, cycle=cls.cycle, write_outputs=False, make_plots=False)
            cls.continuous = run(config=cls.config, write_outputs=False, make_plots=False)

    def driver_fixture(self, strict=True):
        nav, state, field = messaging.NavAttMsg(), messaging.SCStatesMsg(), messaging.MagneticFieldMsg()
        tam = magnetometer.Magnetometer()
        tam.stateInMsg.subscribeTo(state)
        tam.magInMsg.subscribeTo(field)
        driver = MagneticCycleDriver(self.cycle, 100_000_000, tam, nav, state, field, ADCSConfig(), strict)
        native = MtbEffector.MtbEffector()
        native.mtbCmdInMsg.subscribeTo(driver.mtbCmdOutMsg)
        native.magInMsg.subscribeTo(field)
        params = configure_mtb_config_message(ADCSConfig())
        native.mtbParamsInMsg.subscribeTo(params)
        driver.effector = native
        driver.Reset(0)
        n, b = messaging.NavAttMsgPayload(), messaging.MagneticFieldMsgPayload()
        n.omega_BN_B = [0.8, -0.2, 0.3]
        b.magField_N = [1e-5, -2e-5, 3e-5]
        nav.write(n, 400_000_000)
        state.write(messaging.SCStatesMsgPayload(), 400_000_000)
        field.write(b, 400_000_000)
        # Retain SWIG message ownership during the fixture.
        return driver, params

    def test_literal_phase_progression_and_repeatability(self):
        expected = ["COIL_OFF", "QUIET", "SETTLING", "SETTLING", "SAMPLE", "COMPUTE"] + ["ACTUATE"]*4
        self.assertEqual(self.frame.cycle_phase.tolist(), expected*3+["COIL_OFF"])
        self.assertEqual(self.frame.loc[self.frame.cycle_sample_event, "time_ns"].tolist(), [400_000_000, 1_400_000_000, 2_400_000_000])
        self.assertEqual(self.frame.loc[self.frame.cycle_controller_consumed_sample, "time_ns"].tolist(), [500_000_000, 1_500_000_000, 2_500_000_000])
        self.assertTrue(all(v["passed"] for v in cycle_checks(self.frame, self.config).values()))
        self.assertTrue(all(v["passed"] for v in self.physics_checks(self.frame).values()))
        report = analyze(self.frame, self.config, self.continuous)
        self.assertTrue(report["passed"])
        json.dumps(report, allow_nan=False)  # Full energy report must be a writable JSON artifact.

    def test_zero_native_torque_uses_completed_interval_not_new_phase_label(self):
        at = self.frame.set_index("time_ns")
        # Unique tick/column lookups select real numeric telemetry, not arbitrary Scalar.
        self.assertEqual(float(cast(float, at.loc[600_000_000, "applied_torque_B_mag_Nm"])), 0.)
        self.assertGreater(float(cast(float, at.loc[700_000_000, "applied_torque_B_mag_Nm"])), 0.)
        self.assertGreater(float(cast(float, at.loc[1_000_000_000, "applied_torque_B_mag_Nm"])), 0.)
        self.assertEqual(float(cast(float, at.loc[1_100_000_000, "applied_torque_B_mag_Nm"])), 0.)
        self.assertEqual(float(cast(float, at.loc[1_000_000_000, "mcmd_x_Am2"])), 0.)

    def test_actual_nonzero_dipole_rejects_acquisition_before_tam_executes(self):
        driver, params = self.driver_fixture()
        p = messaging.MTBCmdMsgPayload()
        p.mtbDipoleCmds = [0.1] + [0.]*35
        driver.mtbCmdOutMsg.write(p, 300_000_000)
        with self.assertRaisesRegex(ValueError, "INVALID magnetic acquisition"):
            driver.acquire(400_000_000)
        self.assertEqual(driver.rejected_samples, 1)
        self.assertEqual(driver.controller.tamSensorInMsg.timeWritten(), 0)
        self.assertFalse(driver.valid)

    def test_invalid_and_previous_cycle_samples_cannot_drive_controller(self):
        driver, params = self.driver_fixture()
        with self.assertRaisesRegex(ValueError, "INVALID/stale"):
            driver.compute(500_000_000)
        driver.acquire(400_000_000)
        with self.assertRaisesRegex(ValueError, "INVALID/stale"):
            driver.compute(1_500_000_000)
        driver.compute(500_000_000)
        with self.assertRaisesRegex(ValueError, "INVALID/stale"):
            driver.compute(500_000_000)

    def test_insufficient_settling_is_rejected_and_never_consumed(self):
        driver, params = self.driver_fixture(strict=False)
        driver.disabled_since = 200_000_000
        driver.acquire(400_000_000)
        self.assertEqual(driver.valid_samples, 0)
        self.assertEqual(driver.rejected_samples, 1)
        with self.assertRaisesRegex(ValueError, "INVALID/stale"):
            driver.compute(500_000_000)
        # An actual foreign command during quiet must restart the settling clock.
        driver, params = self.driver_fixture()
        payload = messaging.MTBCmdMsgPayload()
        payload.mtbDipoleCmds = [0.1]+[0.]*35
        driver.mtbCmdOutMsg.write(payload, 200_000_000)
        driver.UpdateState(300_000_000)
        self.assertEqual(driver.disabled_since, 300_000_000)
        with self.assertRaisesRegex(ValueError, "INVALID magnetic acquisition"):
            driver.acquire(400_000_000)

    def test_corrupted_telemetry_fails_independent_timing_and_native_checks(self):
        cases = [
            (4, "mcmd_x_Am2", 0.1, "cycle_sample_coils_quiet"),
            (5, "cycle_sample_valid", False, "cycle_sample_validity"),
            (5, "mcmd_x_Am2", 0.1, "cycle_command_epoch_and_hold"),
            (11, "mcmd_x_Am2", 0.1, "cycle_off_command_zero"),
            (15, "cycle_sample_epoch_ns", 400_000_000, "cycle_sample_epochs"),
            (6, "cycle_actuation_start_ns", 500_000_000, "cycle_phase_epochs"),
            (11, "applied_torque_B_x_Nm", 1e-6, "cycle_native_off_torque_zero"),
            (5, "cycle_sample_B_B_x", 1e-3, "cycle_cycle_sample_B_B_held"),
        ]
        for row, column, value, failed in cases:
            with self.subTest(column=column):
                frame = self.frame.copy(deep=True)
                frame.loc[row, column] = value
                self.assertFalse(cycle_checks(frame, self.config)[failed]["passed"])
                self.assertFalse(all(v["passed"] for v in self.physics_checks(frame).values()))

    def test_durations_require_provenance_and_grid_optional_settling_is_real(self):
        with self.assertRaisesRegex(ValueError, "task clock"):
            replace(self.cycle, quiet=test_duration(0.15, "off grid")).validate(100_000_000)
        with self.assertRaisesRegex(ValueError, "at least one"):
            replace(self.cycle, quiet=test_duration(0., "no quiet")).validate(100_000_000)
        alternate = replace(self.cycle, settling=test_duration(0., "optional settling absent"),
                            sample_to_compute=test_duration(0.2, "alternate test delay"))
        config = DEFAULT_CONFIG.with_run_options(2.7)
        with contextlib.redirect_stdout(io.StringIO()):
            frame = run(config=config, cycle=alternate, write_outputs=False, make_plots=False)
        self.assertNotIn("SETTLING", frame.cycle_phase.tolist())
        self.assertEqual(frame.loc[frame.cycle_sample_event, "time_ns"].tolist(), [200_000_000, 1_100_000_000, 2_000_000_000])
        self.assertTrue(all(v["passed"] for v in cycle_checks(frame, config).values()))

    def test_continuous_default_and_explicit_mode_are_identical(self):
        with contextlib.redirect_stdout(io.StringIO()):
            a = run(stop_time_s=3., write_outputs=False, make_plots=False)
            b = run(stop_time_s=3., cycle=None, write_outputs=False, make_plots=False)
        pd.testing.assert_frame_equal(a, b, check_exact=True)
        self.assertEqual(int(a.telemetry_schema_version.iloc[0]), 4)
        # The scenario constructs string telemetry labels.
        self.assertFalse(any(col.startswith("cycle_") for col in cast(Iterable[str], a)))
        self.assertEqual(DEFAULT_CONFIG.fingerprint(), "99de374af2f512e27d34e40fdc3327356662ef48804f7ba7945b7a703a68ad8f")


if __name__ == "__main__":
    unittest.main()
