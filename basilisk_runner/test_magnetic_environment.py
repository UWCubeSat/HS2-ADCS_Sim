"""Focused Phase 2B contract regressions; no network, kernels or saved outputs."""
import contextlib
import io
import unittest
from typing import TypedDict, cast

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import magneticFieldWMM

from magnetic_environment import EarthOrientation, WMMInputGuard, earth_dcm, tdb_seconds
from scenario_huskysat2_detumble import run, find_wmm2025_path
from test_detumble_telemetry import analytic_telemetry, validate
import compare_reference_vs_basilisk as comparison


class InputOptions(TypedDict, total=False):
    state_tick: int
    earth_tick: int
    link_earth: bool


class MagneticEnvironmentTests(unittest.TestCase):
    def make_inputs(self, tick: int = 100_000_000, state_tick: int | None = None,
                    earth_tick: int | None = None, link_earth: bool = True):
        wmm = magneticFieldWMM.MagneticFieldWMM()
        state = messaging.SCStatesMsgPayload()
        state.r_BN_N = [6971000.0, 0.0, 0.0]  # synthetic test position, no model change
        state_msg = messaging.SCStatesMsg().write(state, tick if state_tick is None else state_tick)
        wmm.addSpacecraftToModel(state_msg)
        earth = EarthOrientation()
        earth.UpdateState(tick if earth_tick is None else earth_tick)
        if link_earth:
            wmm.planetPosInMsg.subscribeTo(earth.planetOutMsg)
        return wmm, state_msg, earth, WMMInputGuard(wmm)

    def test_guard_accepts_current_and_rejects_stale_or_missing_inputs(self):
        objects = self.make_inputs()
        objects[-1].UpdateState(100_000_000)
        cases = (InputOptions(state_tick=0), InputOptions(earth_tick=0), InputOptions(link_earth=False))
        for options in cases:
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(ValueError):
                objects = self.make_inputs(**options)
                objects[-1].UpdateState(100_000_000)

    def test_guard_rejects_fresh_header_with_stale_absolute_orientation_time(self):
        wmm, state, earth, guard = self.make_inputs(earth_tick=0)
        earth.planetOutMsg.write(earth.planetOutMsg.read(), 100_000_000)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(ValueError):
            guard.UpdateState(100_000_000)

    def test_earth_rotation_anchor_and_direction(self):
        # At J2000 TDB, RA=0, DEC=90 and W=190.147 degrees from pck00011.
        # A passive z rotation of W+90 has these analytically known rows.
        angle = np.radians(280.147)
        expected = [[np.cos(angle), np.sin(angle), 0],
                    [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
        matrix, derivative = earth_dcm(0.0)
        np.testing.assert_allclose(matrix, expected, rtol=0, atol=1e-15)
        # Positive Earth rotation makes a fixed inertial vector move west in P.
        future, _ = earth_dcm(1.0)
        relative = future @ matrix.T
        self.assertLess(relative[1, 0], 0.0)
        np.testing.assert_allclose(matrix @ matrix.T, np.eye(3), atol=1e-15, rtol=0)
        h = 0.1
        finite_difference = (earth_dcm(h)[0] - earth_dcm(-h)[0]) / (2 * h)
        np.testing.assert_allclose(derivative, finite_difference, rtol=0, atol=1e-13)
        self.assertLess(abs(float(tdb_seconds(0)) - (9496.5 * 86400 + 69.184)), 0.0017)

    def test_native_wmm_vector_covariance_not_just_norm(self):
        # Evaluate the same Earth-fixed location under a nontrivial known
        # orientation. This exercises installed 2.10.2's actual input/output
        # transforms without treating the defective standalone code as truth.
        c = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        r_n = np.array([6971000.0, 120000.0, 340000.0])

        def field(position, orientation):
            model = magneticFieldWMM.MagneticFieldWMM()
            model.wmmDataFullPath = find_wmm2025_path()
            model.epochDateFractionalYear = 2026.0
            state = messaging.SCStatesMsgPayload()
            state.r_BN_N = position.tolist()
            state_msg = messaging.SCStatesMsg().write(state)
            model.addSpacecraftToModel(state_msg)
            planet = messaging.SpicePlanetStateMsgPayload()
            planet.J20002Pfix = orientation.tolist()
            planet.computeOrient = True
            planet_msg = messaging.SpicePlanetStateMsg().write(planet)
            model.planetPosInMsg.subscribeTo(planet_msg)
            model.Reset(0)
            model.UpdateState(0)
            return np.array(model.envOutMsgs[0].read().magField_N)

        fixed_field = field(c @ r_n, np.eye(3))
        actual = field(r_n, c)
        np.testing.assert_allclose(actual, c.T @ fixed_field, rtol=0, atol=1e-18)
        self.assertGreater(np.linalg.norm(actual - c @ fixed_field), 1e-7)

    def test_epoch_and_direction_corruption_are_rejected(self):
        for column in ("wmm_state_time_ns", "earth_orientation_time_ns", "field_evaluation_time_ns"):
            with self.subTest(column=column):
                df = analytic_telemetry()
                # Known numeric fixture columns; casts leave their runtime values intact.
                df.loc[1, column] = cast(int, df.loc[1, column]) - 100_000_000
                self.assertFalse(validate(df)["checks"]["telemetry_epochs_aligned"]["passed"])
        for corruption in ("transpose", "stale_matrix", "disabled", "stale_tdb"):
            with self.subTest(corruption=corruption):
                df = analytic_telemetry()
                columns = [f"earth_C_PN_{i}{j}" for i in (1, 2, 3) for j in (1, 2, 3)]
                if corruption == "transpose":
                    df.loc[1, columns] = df.loc[1, columns].to_numpy().reshape(3, 3).T.ravel()
                elif corruption == "stale_matrix":
                    df.loc[1, columns] = earth_dcm(float(tdb_seconds(3199.9)))[0].ravel()
                elif corruption == "disabled":
                    df.loc[1, "earth_orientation_enabled"] = 0
                else:
                    df.loc[1, "earth_orientation_tdb_s"] = cast(float, df.loc[1, "earth_orientation_tdb_s"]) - 0.1
                self.assertFalse(validate(df)["passed"])
        df = analytic_telemetry()
        # Inverse body rotation preserves |B| but sends +Z the wrong direction.
        df["sensor_state_sigma_BN_2"] *= -1.0
        checks = validate(df)["checks"]
        self.assertTrue(checks["B_frame_norm_preserved"]["passed"])
        self.assertFalse(checks["B_frame_vector_aligned"]["passed"])

    def test_actual_scenario_schedule_and_following_interval(self):
        with contextlib.redirect_stdout(io.StringIO()):
            df = run(stop_time_s=2.0, write_outputs=False)
        metrics = comparison.summarize_basilisk(df)
        self.assertTrue(metrics["telemetry_timing_valid"])
        self.assertEqual(metrics["max_sensor_state_age_s"], 0.0)
        self.assertLess(metrics["max_B_frame_vector_error_T"], 1e-12)
        self.assertLess(metrics["torque_cross_product_max_abs_error_Nm"], 1e-12)
        self.assertLess(metrics["rigid_body_step_max_rate_error_rad_s"], 1e-10)
        self.assertEqual(df.loc[1, "held_dipole_time_ns"], 900_000_000)
        self.assertEqual(df.loc[1, "dipole_command_time_ns"], 1_000_000_000)
        self.assertGreater(np.linalg.norm(df.loc[1, ["control_torque_B_x_Nm", "control_torque_B_y_Nm", "control_torque_B_z_Nm"]]
                                         .to_numpy(dtype=float) - df.loc[1, ["applied_torque_B_x_Nm", "applied_torque_B_y_Nm", "applied_torque_B_z_Nm"]]
                                         .to_numpy(dtype=float)), 1e-10)


if __name__ == "__main__":
    unittest.main()
