"""Installed Basilisk 2.10.2 magnetic actuation, independent physics and A/B tests.

Synthetic unit-test fields, attitudes and commands are not HS-2 flight inputs.
All existing Phase 2A/2B regressions remain in their original test modules.
"""
import contextlib
import io
import unittest
from typing import Callable, Sequence, TypedDict, cast

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import MtbEffector, extForceTorque
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.simulation import svIntegrators

from basilisk_adcs_adapter import ADCSConfig, MAX_EFF_CNT
from scenario_huskysat2_detumble import configure_spacecraft, configure_mtb_config_message, run
from magnetic_actuation import MagneticInputGuard
import compare_reference_vs_basilisk as comparison


def dipole_message(values):
    payload = messaging.MTBCmdMsgPayload()
    payload.mtbDipoleCmds = list(values) + [0.0] * (MAX_EFF_CNT - 3)
    return messaging.MTBCmdMsg().write(payload)


def native_fixture(command, field, sigma=(0, 0, 0), omega=(0, 0, 0), axes=None):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("probeProcess")
    proc.addTask(sim.CreateNewTask("probeTask", macros.sec2nano(0.1)))
    sc = configure_spacecraft()
    sc.hub.sigma_BNInit = list(sigma)
    sc.hub.omega_BN_BInit = list(omega)
    native = MtbEffector.MtbEffector()
    config = configure_mtb_config_message(ADCSConfig())
    if axes is not None:
        payload = config.read()
        payload.GtMatrix_B = np.asarray(axes).ravel().tolist() + [0.0] * (3*MAX_EFF_CNT - 9)
        config.write(payload)
    command_msg = dipole_message(command)
    payload = messaging.MagneticFieldMsgPayload()
    payload.magField_N = list(field)
    field_msg = messaging.MagneticFieldMsg().write(payload)
    native.mtbCmdInMsg.subscribeTo(command_msg)
    native.magInMsg.subscribeTo(field_msg)
    native.mtbParamsInMsg.subscribeTo(config)
    sc.addDynamicEffector(native)
    sim.AddModelToTask("probeTask", sc, ModelPriority=1000)
    sim.AddModelToTask("probeTask", native, ModelPriority=980)
    state_log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("probeTask", state_log)
    sim.InitializeSimulation()
    # Keep all messages/dynamic objects alive while C++ holds subscribers.
    return sim, sc, native, command_msg, field_msg, config, state_log


class BenchInputs(sysModel.SysModel):
    def __init__(self, state, field, command_function):
        super().__init__()
        self.state = state
        self.field = np.asarray(field)
        self.command_function = command_function
        self.command = dipole_message([0, 0, 0])
        payload = messaging.MagneticFieldMsgPayload()
        payload.magField_N = self.field.tolist()
        self.field_msg = messaging.MagneticFieldMsg().write(payload)
        self.torque = messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())

    def UpdateState(self, tick):
        # Both paths receive IDENTICAL dipoles on the fixed 0.1 s command clock.
        if tick % 100_000_000 == 0:
            payload = self.command.read()
            payload.mtbDipoleCmds = list(self.command_function(tick * 1e-9)) + [0.0]*(MAX_EFF_CNT - 3)
            self.command.write(payload, tick)
        # Refine ONLY direct torque evaluation for the convergence experiment.
        sigma = np.array([self.state.read().sigma_BN])
        body_field = comparison.rotate_inertial_to_body(sigma, self.field[None, :])[0]
        payload = messaging.CmdTorqueBodyMsgPayload()
        payload.torqueRequestBody = np.cross(self.command.read().mtbDipoleCmds[:3], body_field).tolist()
        self.torque.write(payload, tick)


class BenchOptions(TypedDict, total=False):
    omega: Sequence[float]
    field: Sequence[float]
    command_function: Callable[[float], Sequence[float]]
    duration: float


def bench_trajectory(mode, dt=0.1, duration=2.0, omega: Sequence[float] = (0.8, -0.2, 0.3),
                     field: Sequence[float] = (1e-5, -2e-5, 3e-5),
                     command_function: Callable[[float], Sequence[float]] | None = None, integrator="rk4"):
    """Controlled ExtForceTorque/native A/B; no WMM/controller feedback changes.

    At dt=0.1 direct mode has the legacy hold semantics. Smaller dt evaluates
    direct m x B more often while retaining the SAME dipole and inertial field
    history. This isolates the expected body-hold discretization error.
    """
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("benchProcess")
    proc.addTask(sim.CreateNewTask("benchTask", macros.sec2nano(dt)))
    sc = configure_spacecraft()
    sc.hub.omega_BN_BInit = list(omega)
    if integrator == "euler":
        # Test only: one force evaluation at the same pre-state in both paths.
        # This isolates actuator equivalence from their different RK4 hold laws.
        stepper = svIntegrators.svIntegratorEuler(sc)
        sc.setIntegrator(stepper)
    source = BenchInputs(sc.scStateOutMsg, field,
                         command_function or (lambda t: [0.15 if t < 1 else -0.15, -0.1, 0.7]))
    config = configure_mtb_config_message(ADCSConfig())
    if mode == "native":
        effector = MtbEffector.MtbEffector()
        effector.mtbCmdInMsg.subscribeTo(source.command)
        effector.magInMsg.subscribeTo(source.field_msg)
        effector.mtbParamsInMsg.subscribeTo(config)
    else:
        effector = extForceTorque.ExtForceTorque()
        effector.cmdTorqueInMsg.subscribeTo(source.torque)
    sc.addDynamicEffector(effector)
    state_log = sc.scStateOutMsg.recorder(macros.sec2nano(0.1))
    command_log = source.command.recorder(macros.sec2nano(0.1))
    torque_log = effector.logger(["torqueExternalPntB_B"])
    for model, priority in [(sc, 1000), (torque_log, 975), (source, 600),
                            (effector, 980 if mode == "native" else 500),
                            (state_log, 0), (command_log, 0)]:
        sim.AddModelToTask("benchTask", model, ModelPriority=priority)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    return {"omega": np.array(state_log.omega_BN_B), "sigma": np.array(state_log.sigma_BN),
            "commands": np.array(command_log.mtbDipoleCmds)[:, :3],
            "torque": np.array(torque_log.torqueExternalPntB_B)}


class NativeMagneticActuationTests(unittest.TestCase):
    def torque(self, command, field=(1e-5, -2e-5, 3e-5), sigma=(0, 0, 0), axes=None):
        objects = native_fixture(command, field, sigma=sigma, axes=axes)
        native = objects[2]
        native.computeForceTorque(0.0, 0.1)
        native.UpdateState(0)
        actual = np.array(native.torqueExternalPntB_B).reshape(3)
        np.testing.assert_array_equal(actual, native.mtbOutMsg.read().mtbNetTorque_B)
        return actual

    def test_zero_and_parallel_dipoles(self):
        np.testing.assert_array_equal(self.torque([0, 0, 0]), [0, 0, 0])
        np.testing.assert_allclose(self.torque([0.1, -0.2, 0.3]), [0, 0, 0], atol=1e-20, rtol=0)

    def test_reversal_and_axes(self):
        field = [0, 0, 4e-5]
        # Hand-calculated right-handed x cross z = -y; y cross z = +x.
        np.testing.assert_allclose(self.torque([0.1, 0, 0], field), [0, -4e-6, 0], atol=1e-20, rtol=0)
        np.testing.assert_allclose(self.torque([0, 0.1, 0], field), [4e-6, 0, 0], atol=1e-20, rtol=0)
        for axis in range(3):
            command = np.eye(3)[axis] * 0.1
            np.testing.assert_allclose(self.torque(-command), -self.torque(command), atol=1e-20, rtol=0)
        # Nonidentity layout distinguishes contiguous 3x3 from MAX_EFF_CNT stride.
        axes = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
        np.testing.assert_allclose(self.torque([0.1, 0, 0], field, axes=axes), [4e-6, 0, 0], atol=1e-20, rtol=0)
        np.testing.assert_allclose(self.torque([0, 0, 0.1], field, axes=axes), [0, 4e-6, 0], atol=1e-20, rtol=0)
        np.testing.assert_allclose(self.torque([0, 0.1, 0], [4e-5, 0, 0], axes=axes), [0, -4e-6, 0], atol=1e-20, rtol=0)

    def test_saturation_is_per_bar_before_axis_mapping(self):
        limits = np.array([0.2, 0.2, 0.85])  # ASSUMED repository limits, not flight hardware.
        for sign in (-1, 1):
            np.testing.assert_allclose(self.torque(sign * limits * 3), self.torque(sign * limits), atol=1e-20, rtol=0)
        axes = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
        expected = np.cross(np.array([-0.85, 0.2, 0.2]), [1e-5, -2e-5, 3e-5])
        np.testing.assert_allclose(self.torque([0.8, -0.8, 2.0], axes=axes), expected, atol=1e-20, rtol=0)

    def test_nontrivial_attitude_uses_inertial_field_and_correct_direction(self):
        # +90deg body attitude about z: inertial +x has body components -y.
        sigma = [0, 0, np.tan(np.pi/8)]
        np.testing.assert_allclose(self.torque([0, 0, 0.5], [4e-5, 0, 0], sigma),
                                   [2e-5, 0, 0], atol=1e-20, rtol=0)

    def test_recorded_native_stage_torques_predict_rigid_body_step(self):
        s0, w0 = [0.13, -0.08, 0.21], [0.8, -0.2, 0.3]
        args = ([0.2, -0.2, 0.8], [1e-5, -2e-5, 3e-5])
        probe = native_fixture(*args, sigma=s0, omega=w0)
        observed_torques = []

        def derivative(y):
            s, w = y[:3], y[3:]
            probe[2].hubSigma.setState(s.reshape(3, 1))
            probe[2].computeForceTorque(0.0, 0.1)
            tau = np.array(probe[2].torqueExternalPntB_B).reshape(3)
            observed_torques.append(tau)
            sdot = ((1-s@s)*w + 2*np.cross(s, w) + 2*s*(s@w))/4
            inertia = comparison.INERTIA_DIAG_KG_M2
            return np.r_[sdot, (tau - np.cross(w, inertia*w))/inertia]

        y0 = np.r_[s0, w0]
        k1 = derivative(y0)
        k2 = derivative(y0 + 0.05*k1)
        k3 = derivative(y0 + 0.05*k2)
        k4 = derivative(y0 + 0.1*k3)
        predicted = y0 + 0.1*(k1+2*k2+2*k3+k4)/6
        actual = native_fixture(*args, sigma=s0, omega=w0)
        actual[0].ConfigureStopTime(macros.sec2nano(0.1))
        actual[0].ExecuteSimulation()
        np.testing.assert_allclose(actual[-1].omega_BN_B[-1], predicted[3:], atol=2e-15, rtol=0)
        np.testing.assert_allclose(actual[-1].sigma_BN[-1], predicted[:3], atol=2e-15, rtol=0)
        np.testing.assert_allclose(np.array(actual[2].torqueExternalPntB_B).reshape(3), observed_torques[-1], atol=1e-19, rtol=0)

    def test_identical_command_trajectory_and_hold_error_convergence(self):
        native = bench_trajectory("native", dt=0.0015625)
        errors = []
        for dt in (0.1, 0.025, 0.00625, 0.0015625):
            direct = bench_trajectory("direct", dt=dt)
            np.testing.assert_array_equal(direct["commands"], native["commands"])
            errors.append(np.linalg.norm(direct["omega"][-1] - native["omega"][-1]))
        self.assertGreater(errors[0], 1e-7)  # Do not pretend the hold laws are identical.
        self.assertLess(errors[1], errors[0]*0.3)
        self.assertLess(errors[2], errors[1]*0.3)
        self.assertLess(errors[3], errors[2]*0.3)
        self.assertLess(errors[3], 3e-6)
        # When B_B remains fixed, both actuator laws must give the same trajectory.
        options = BenchOptions(omega=(0, 0, 0), field=(0, 0, 4e-5), command_function=lambda t: [0, 0, 0.5])
        np.testing.assert_allclose(bench_trajectory("native", **options)["omega"],
                                   bench_trajectory("direct", **options)["omega"], atol=1e-15, rtol=0)
        # Nonzero first step from rest: tiny rotation makes the hold error higher
        # order, giving a separate trajectory equivalence anchor with actual torque.
        options.update(duration=0.1, command_function=lambda t: [0.1, 0, 0])
        active = bench_trajectory("native", **options)
        self.assertGreater(np.linalg.norm(active["omega"][-1]), 1e-5)
        np.testing.assert_allclose(active["omega"], bench_trajectory("direct", **options)["omega"], atol=1e-13, rtol=0)

    def test_command_applies_on_following_interval(self):
        trajectory = bench_trajectory("native", duration=0.3, omega=(0, 0, 0), field=(0, 0, 4e-5),
                                      command_function=lambda t: [0.1 if t < 0.1 else (-0.1 if t < 0.2 else 0), 0, 0])
        self.assertEqual(np.linalg.norm(trajectory["torque"][0]), 0)
        self.assertLess(trajectory["torque"][1, 1], 0)
        self.assertGreater(trajectory["torque"][2, 1], 0)
        np.testing.assert_array_equal(trajectory["torque"][3], [0, 0, 0])
        self.assertLess(trajectory["omega"][1, 1], 0)
        self.assertGreater(trajectory["omega"][2, 1], trajectory["omega"][1, 1])

    def test_native_direct_trajectory_equivalence_with_matched_evaluation_times(self):
        native = bench_trajectory("native", integrator="euler")
        direct = bench_trajectory("direct", integrator="euler")
        np.testing.assert_array_equal(native["commands"], direct["commands"])
        self.assertGreater(np.linalg.norm(native["torque"]), 1e-6)
        np.testing.assert_allclose(native["torque"], direct["torque"], atol=1e-18, rtol=0)
        np.testing.assert_allclose(native["omega"], direct["omega"], atol=1e-14, rtol=0)
        np.testing.assert_allclose(native["sigma"], direct["sigma"], atol=1e-14, rtol=0)

    def test_input_guard_rejects_stale_and_future_inputs(self):
        for which in (3, 4):
            for time in (0, 200_000_000):
                objects = native_fixture([0.1, 0, 0], [0, 0, 4e-5])
                # Index only the message pair, not the heterogeneous keepalive tuple.
                messages = objects[3:5]
                for msg in messages:
                    msg.write(msg.read(), 100_000_000)
                selected_msg = messages[which - 3]
                selected_msg.write(selected_msg.read(), time)
                guard = MagneticInputGuard(objects[2], 100_000_000)
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(ValueError):
                    guard.UpdateState(200_000_000)

    def test_actual_native_scenario_and_independent_failure_detection(self):
        with contextlib.redirect_stdout(io.StringIO()):
            df = run(2.0, False, capture_commands=True)
        metrics = comparison.summarize_basilisk(df)
        self.assertEqual(metrics["torque_source"], "MtbEffector.torqueExternalPntB_B")
        self.assertTrue(comparison.build_validation({"finite_numeric": True}, metrics)["passed"])
        self.assertGreater(np.linalg.norm(df[[f"mcmd_{a}_Am2" for a in "xyz"]].to_numpy()), 0)
        self.assertLess(metrics["torque_cross_product_max_abs_error_Nm"], 1e-18)
        self.assertLess(metrics["rigid_body_step_max_rate_error_rad_s"], 1e-14)
        for corruption in ("torque", "rate", "epoch", "field_direction", "evaluation"):
            bad = df.copy()
            if corruption == "torque":
                # Corrupt BOTH native exports: transport still passes, physics must fail.
                bad[[f"{p}_{a}_Nm" for p in ("applied_torque_B", "native_mtbNetTorque_B") for a in "xyz"]] = 0
            elif corruption == "rate":
                # Scalar casts describe the known numeric telemetry columns; they
                # do not coerce the values used by these failure-detection tests.
                bad.loc[1, "omega_B_x_rad_s"] = cast(float, bad.loc[1, "omega_B_x_rad_s"]) + 1e-5
            elif corruption == "epoch":
                bad.loc[1, "native_input_field_time_ns"] = cast(int, bad.loc[1, "native_input_field_time_ns"]) + 100_000_000
            elif corruption == "field_direction":
                bad.loc[1, "held_B_N_x_T"] = cast(float, bad.loc[1, "held_B_N_x_T"]) * -1
            else:
                bad["applied_torque_evaluation"] = "constant_body_hold"
            with self.subTest(corruption=corruption):
                try:
                    checks = comparison.build_validation({"finite_numeric": True}, comparison.summarize_basilisk(bad))
                except ValueError:
                    self.assertEqual(corruption, "evaluation")
                else:
                    self.assertFalse(checks["passed"])
        with contextlib.redirect_stdout(io.StringIO()):
            replay = run(2.0, False, actuator="direct", replay_commands=df.attrs["command_history"], capture_commands=True)
        np.testing.assert_array_equal(replay.attrs["command_history"], df.attrs["command_history"])
        self.assertTrue(comparison.summarize_basilisk(replay)["telemetry_timing_valid"])


if __name__ == "__main__":
    unittest.main()
