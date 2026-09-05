r"""Phase 2A regressions: analytic fixtures are test inputs, not HS-2 parameters.

Run from the repository root:
    .\.venv\Scripts\python.exe -m unittest discover -s basilisk_runner -p test_detumble_telemetry.py -v
"""

from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd
from Basilisk.architecture import messaging
from Basilisk.simulation import extForceTorque, simpleNav, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros

import compare_reference_vs_basilisk as comparison
from scenario_huskysat2_detumble import sample_at_ticks
from basilisk_adcs_adapter import ADCSConfig, PythonBdotMTQController, MAX_EFF_CNT
from magnetic_environment import earth_dcm, tdb_seconds, MODEL_NAME


def analytic_telemetry():
    """Principal-axis spin under constant torque: closed-form attitude and rate.

    Construct inertial field components with sine/cosine, independently of
    the validator's quaternion rotation and numerical rigid-body integration.
    """
    t = np.array([0.0, 3200.0, 5792.0])
    ticks = (t * 1e9).astype(np.int64)
    source_ticks = np.maximum(ticks - 100_000_000, 0)
    pre_t = source_ticks * 1e-9
    alpha = -1.0 / 60000.0
    torque_y = (2.6 / 12.0) * (0.1**2 + 0.2**2) * alpha
    pre_angle = 0.1 * pre_t + 0.5 * alpha * pre_t**2
    post_angle = 0.1 * t + 0.5 * alpha * t**2
    df = pd.DataFrame({
        "telemetry_schema_version": 3, "time_s": t, "time_ns": ticks,
        "control_step_ns": 100_000_000, "sensor_state_time_ns": ticks,
        "application_interval_valid": ticks > 0,
        "nav_time_tag_s": t, "pcoil_total_W": 0.1,
        "applied_torque_source": "ExtForceTorque.torqueExternalPntB_B",
        "earth_orientation_model": MODEL_NAME, "earth_orientation_enabled": 1,
        "earth_orientation_tdb_s": tdb_seconds(t), "wmm_coefficient_time_ns": ticks,
    })
    for col in comparison.PUBLICATION_TIME_COLUMNS:
        df[col] = ticks
    for col in comparison.HELD_TIME_COLUMNS:
        df[col] = source_ticks
    matrices = np.array([earth_dcm(et)[0] for et in tdb_seconds(t)])
    for i in range(3):
        for j in range(3):
            df[f"earth_C_PN_{i+1}{j+1}"] = matrices[:, i, j]
    vectors = {
        "omega_B": ([np.zeros(3), 0.1 + alpha * t, np.zeros(3)], "rad_s"),
        "sensor_state_omega_B": ([np.zeros(3), 0.1 + alpha * t, np.zeros(3)], "rad_s"),
        "nav_omega_B": ([np.zeros(3), 0.1 + alpha * t, np.zeros(3)], "rad_s"),
        "held_omega_B": ([np.zeros(3), 0.1 + alpha * pre_t, np.zeros(3)], "rad_s"),
        "B_N": ([4e-5 * np.sin(post_angle), np.zeros(3), 4e-5 * np.cos(post_angle)], "T"),
        "held_B_N": ([4e-5 * np.sin(pre_angle), np.zeros(3), 4e-5 * np.cos(pre_angle)], "T"),
        "B_B": ([0.0, 0.0, 4e-5], "T"),
        "mcmd": ([-torque_y / 4e-5, 0.0, 0.0], "Am2"),
        "held_mcmd": ([-torque_y / 4e-5, 0.0, 0.0], "Am2"),
        "control_torque_B": ([0.0, torque_y, 0.0], "Nm"),
        "held_control_torque_B": ([0.0, torque_y, 0.0], "Nm"),
        "applied_torque_B": ([0.0, torque_y, 0.0], "Nm"),
    }
    for prefix, (values, suffix) in vectors.items():
        for axis, value in zip("xyz", values):
            df[f"{prefix}_{axis}_{suffix}"] = value
    for prefix, angle in [("sigma_BN", post_angle), ("sensor_state_sigma_BN", post_angle),
                          ("nav_sigma_BN", post_angle), ("held_sigma_BN", pre_angle)]:
        df[f"{prefix}_1"] = 0.0
        df[f"{prefix}_2"] = np.tan(angle / 4.0)
        df[f"{prefix}_3"] = 0.0
    return df


def validate(df):
    return comparison.build_validation({"finite_numeric": True}, comparison.summarize_basilisk(df))


class TelemetryRegressionTests(unittest.TestCase):
    def test_controller_publishes_nonzero_dipole_array(self):
        # omega along +X and B along +Z request -Y dipole, limited to -0.2 A m^2
        # by the unchanged development configuration. Check the actual message.
        ctrl = PythonBdotMTQController(ADCSConfig(use_cpp_core_if_available=False))
        nav = messaging.NavAttMsgPayload()
        nav.omega_BN_B = [1.0, 0.0, 0.0]
        nav_msg = messaging.NavAttMsg().write(nav)
        tam = messaging.TAMSensorMsgPayload()
        tam.tam_S = [0.0, 0.0, 3e-5]
        tam_msg = messaging.TAMSensorMsg().write(tam)
        ctrl.navAttInMsg.subscribeTo(nav_msg)
        ctrl.tamSensorInMsg.subscribeTo(tam_msg)
        ctrl.Reset(0)
        ctrl.UpdateState(100_000_000)
        dipole = ctrl.mtbCmdOutMsg.read().mtbDipoleCmds
        np.testing.assert_allclose(dipole[:3], [0.0, -0.2, 0.0], rtol=0, atol=1e-15)
        np.testing.assert_array_equal(dipole[3:], np.zeros(MAX_EFF_CNT - 3))
        np.testing.assert_allclose(ctrl.cmdTorqueOutMsg.read().torqueRequestBody,
                                   [-6e-6, 0.0, 0.0], rtol=0, atol=1e-18)

    def test_analytic_principal_axis_case_passes(self):
        result = validate(analytic_telemetry())
        self.assertTrue(result["passed"], result)

    def test_exact_tick_selection_rejects_missing_or_invalid_evidence(self):
        np.testing.assert_array_equal(sample_at_ticks([0, 20], [0, 10, 20], [3, 4, 5], "test"), [3, 5])
        for times, values in [([0, 19], [3, 5]), ([0, 0, 20], [3, 4, 5]),
                              ([20, 0], [5, 3]), ([], []), ([0, 20], [3])]:
            with self.subTest(times=times), self.assertRaises(ValueError):
                sample_at_ticks([0, 20], times, values, "test")

    def test_one_step_attitude_pairing_fails_direction_check(self):
        df = analytic_telemetry()
        df["sensor_state_sigma_BN_2"] = df["held_sigma_BN_2"]
        checks = validate(df)["checks"]
        self.assertTrue(checks["B_frame_norm_preserved"]["passed"])
        self.assertFalse(checks["B_frame_vector_aligned"]["passed"])

    def test_same_norm_wrong_sensor_direction_is_rejected(self):
        df = analytic_telemetry()
        df["B_B_x_T"], df["B_B_z_T"] = 4e-5, 0.0
        checks = validate(df)["checks"]
        self.assertTrue(checks["B_frame_norm_preserved"]["passed"])
        self.assertFalse(checks["B_frame_vector_aligned"]["passed"])

    def test_one_nanosecond_timestamp_shift_is_rejected(self):
        for col in ["sensor_state_time_ns", "tam_message_time_ns", "applied_torque_time_ns"]:
            with self.subTest(column=col):
                df = analytic_telemetry()
                df.loc[1, col] += 1
                self.assertFalse(validate(df)["checks"]["telemetry_epochs_aligned"]["passed"])

    def test_zero_applied_torque_cannot_pass_using_expected_torque(self):
        df = analytic_telemetry()
        df["applied_torque_B_y_Nm"] = 0.0
        checks = validate(df)["checks"]
        for name in ["torque_matches_m_cross_B", "effector_matches_torque_message",
                     "applied_torque_predicts_rate_step"]:
            self.assertFalse(checks[name]["passed"], name)

    def test_consistent_command_and_readback_corruption_fails_plant_check(self):
        df = analytic_telemetry()
        for col in ["mcmd_x_Am2", "control_torque_B_y_Nm", "applied_torque_B_y_Nm",
                    "held_mcmd_x_Am2", "held_control_torque_B_y_Nm"]:
            df[col] *= 2.0
        checks = validate(df)["checks"]
        self.assertTrue(checks["torque_matches_m_cross_B"]["passed"])
        self.assertTrue(checks["effector_matches_torque_message"]["passed"])
        self.assertFalse(checks["applied_torque_predicts_rate_step"]["passed"])

    def test_missing_and_nonfinite_data_are_rejected(self):
        df = analytic_telemetry()
        with self.assertRaises(ValueError):
            comparison.summarize_basilisk(df.drop(columns="sensor_state_time_ns"))
        df.loc[1, "applied_torque_B_y_Nm"] = np.nan
        with self.assertRaises(ValueError):
            comparison.summarize_basilisk(df)

    def test_cli_exit_codes_and_failure_report(self):
        with tempfile.TemporaryDirectory(prefix="hs2_phase2a_test_") as directory:
            root = Path(directory)
            reference = root / "reference.csv"
            pd.DataFrame({"t": [0.0, 1.0], "p": [0.1, 0.09], "q": [0.0, 0.0],
                          "r": [0.0, 0.0]}).to_csv(reference, index=False)
            code = ("import sys; from pathlib import Path; "
                    "sys.path.insert(0, sys.argv[1]); import compare_reference_vs_basilisk as c; "
                    "c.OUT_DATA=Path(sys.argv[2]); c.REF=c.OUT_DATA/'reference.csv'; "
                    "c.COMPARE_OUT=c.OUT_DATA/'metrics.json'; raise SystemExit(c.main())")
            for expected_code in (0, 1, 2):
                df = analytic_telemetry()
                if expected_code == 1:
                    df["applied_torque_B_y_Nm"] = 0.0
                elif expected_code == 2:
                    df = df.drop(columns="sensor_state_time_ns")
                df.to_csv(root / "detumble_output.csv", index=False)
                result = subprocess.run([sys.executable, "-c", code, str(Path(__file__).parent), str(root)],
                                        capture_output=True, text=True, check=False)
                with self.subTest(expected_code=expected_code):
                    self.assertEqual(result.returncode, expected_code, result.stdout + result.stderr)
                    report = json.loads((root / "metrics.json").read_text())
                    self.assertEqual(report["validation"]["passed"], expected_code == 0)

    def test_basilisk_effector_and_acquisition_state_semantics(self):
        # Deliberately different static and message inputs distinguish the
        # combined dynamics field from both command and static-input aliases.
        sim = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess("testProcess")
        process.addTask(sim.CreateNewTask("testTask", macros.sec2nano(0.1)))
        sc = spacecraft.Spacecraft()
        sc.hub.mHub = 1.0
        sc.hub.IHubPntBc_B = np.eye(3).tolist()
        effector = extForceTorque.ExtForceTorque()
        effector.extTorquePntB_B = [0.01, 0.02, 0.03]
        payload = messaging.CmdTorqueBodyMsgPayload()
        payload.torqueRequestBody = [0.1, 0.2, 0.3]
        command = messaging.CmdTorqueBodyMsg().write(payload)
        effector.cmdTorqueInMsg.subscribeTo(command)
        sc.addDynamicEffector(effector)
        nav = simpleNav.SimpleNav()
        nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
        before = sc.scStateOutMsg.recorder()
        after = sc.scStateOutMsg.recorder()
        nav_log = nav.attOutMsg.recorder()
        torque_log = effector.logger(["torqueExternalPntB_B"])
        for model, priority in [(before, 1000), (nav, 800), (effector, 500), (sc, 100),
                                (after, 0), (nav_log, 0), (torque_log, 0)]:
            sim.AddModelToTask("testTask", model, ModelPriority=priority)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(0.3))
        sim.ExecuteSimulation()
        np.testing.assert_array_equal(before.timesWritten(), [0, 0, 100_000_000, 200_000_000])
        np.testing.assert_array_equal(after.timesWritten(), after.times())
        np.testing.assert_allclose(nav_log.omega_BN_B, before.omega_BN_B, atol=1e-15, rtol=0)
        np.testing.assert_allclose(torque_log.torqueExternalPntB_B,
                                   np.tile([0.11, 0.22, 0.33], (4, 1)), atol=1e-15, rtol=0)
        np.testing.assert_allclose(after.omega_BN_B[-1], [0.033, 0.066, 0.099], atol=1e-15, rtol=0)


if __name__ == "__main__":
    unittest.main()
