"""Independent Phase 7C algorithm fixtures; every number is TEST-ONLY.

Truth rotations/metrics use Basilisk utilities and Phase 7B analytic anchors,
never the estimator's attitude, covariance or forward model as the truth oracle.
"""
from dataclasses import replace
import contextlib
import io
import unittest

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf import (MEKF, LossEvent, ReplayEstimator, VectorSample, covariance,
                           right_jacobian, transition, triad)
from attitude_mekf_prototype import independent_error, interpolated_gyro, load_test_policy, magnetic_cycle_sample
from test_attitude_estimator_architecture import passive_rodrigues, exp_q, product, inverse, log_q


def sample(sensor, epoch, c, reference, identity=None, **kwargs):
    return VectorSample(identity or f"{sensor}-{epoch}", sensor, epoch, epoch,
                        c @ np.asarray(reference), reference, "independent ideal test fixture", **kwargs)


class MEKFTests(unittest.TestCase):
    def setUp(self):
        self.policy, self.fixtures = load_test_policy()

    def attitude_error(self, state, c):
        return independent_error(state.q, rbk.C2MRP(c))[0]

    def test_zero_and_xyz_arbitrary_rate_propagation(self):
        initial = np.array([0.2, -0.3, 0.1])
        for omega in (np.zeros(3), np.array([0.4, 0., 0.]), np.array([0., 0.4, 0.]),
                      np.array([0., 0., 0.4]), np.array([0.23, -0.42, 0.17])):
            with self.subTest(omega=omega):
                core = MEKF(self.policy, prior_q=exp_q(initial))
                for k in range(1, 11):
                    core.propagate(k*100_000_000, omega)
                expected = passive_rodrigues(omega) @ passive_rodrigues(initial)
                self.assertLess(self.attitude_error(core, expected), 2e-15)
                self.assertAlmostEqual(float(core.q @ core.q), 1., places=14)
                np.testing.assert_allclose(core.last_rate, omega, atol=0)
                self.assertEqual(core.output()["epoch_ns"], 1_000_000_000)

    def test_bias_subtraction_and_quaternion_sign(self):
        rate, bias = np.array([0.2, -0.1, 0.3]), np.array([0.01, -0.02, 0.03])
        a = MEKF(self.policy, prior_q=exp_q(rate), prior_bias=bias)
        b = MEKF(self.policy, prior_q=-exp_q(rate), prior_bias=bias)
        for core in (a, b):
            core.propagate(200_000_000, rate+bias)
            self.assertLess(self.attitude_error(core, passive_rodrigues(1.2*rate)), 1e-15)
        np.testing.assert_allclose(a.output()["sigma_BN"], b.output()["sigma_BN"], atol=0)

    def test_triad_exact_at_large_angles_and_near_collinearity(self):
        r1, r2 = np.array([1., 2., -1.]), np.array([2., -1., 3.])
        for angle in (np.array([0.3, -0.9, 2.8]), np.array([np.pi, 0., 0.])):
            c = passive_rodrigues(angle)
            q, sine = triad(c @ r1, c @ r2, r1, r2, self.policy.minimum_acquisition_sine)
            np.testing.assert_allclose(rbk.EP2C(q), c, atol=1e-15)
            self.assertGreaterEqual(q[0], 0)
            self.assertGreater(sine, 0.1)
        for second in ([2., 0., 0.], [1., 1e-8, 0.], [0., 0., 0.]):
            with self.assertRaises(ValueError):
                triad([1., 0., 0.], second, [1., 0., 0.], second, self.policy.minimum_acquisition_sine)
        # A small transverse disturbance is strongly amplified near collinearity.
        _, wide = triad([1., 0., 0.], [0., 1., 0.], [1., 0., 0.], [0., 1., 0.], 1e-6)
        q, narrow = triad([1., 0., 0.], [1., 1e-4, 1e-5], [1., 0., 0.], [1., 1e-4, 0.], 1e-6)
        self.assertLess(narrow, wide/1000)
        self.assertGreater(independent_error(q, [0., 0., 0.])[0], 0.09)

    def test_acquisition_requires_two_valid_common_epoch_vectors(self):
        c = passive_rodrigues(np.array([0.5, 1.2, -0.3]))
        core = MEKF(self.policy)
        self.assertEqual(core.observe(sample("magnetic", 0, c, [1., 0., 0.])), "awaiting_noncollinear_pair")
        self.assertFalse(core.output()["initialized"])
        core.propagate(100_000_000, [0., 0., 0.])
        core.observe(sample("sun", 100_000_000, c, [0., 1., 0.]))
        self.assertFalse(core.initialized)
        core.observe(sample("magnetic", 100_000_000, c, [1., 0., 0.]))
        self.assertTrue(core.initialized)
        self.assertLess(self.attitude_error(core, c), 1e-15)
        self.assertEqual(core.acquisitions, 1)
        self.assertEqual(core.instantaneous_attitude_rank, 3)

    def test_magnetic_and_sun_correction_direction_and_scaling(self):
        c = passive_rodrigues(np.array([0.3, 0.2, -0.1]))
        for sensor in ("magnetic", "sun"):
            estimates = []
            for scale in (1., 1e-5, 1e5):
                prior_c = passive_rodrigues(np.array([0.07, -0.04, 0.02])) @ c
                core = MEKF(self.policy, prior_q=rbk.C2EP(prior_c))
                before = self.attitude_error(core, c)
                self.assertEqual(core.observe(sample(sensor, 0, c, scale*np.array([1., 2., 3.]))), "updated")
                self.assertLess(self.attitude_error(core, c), before/2)
                self.assertEqual(core.instantaneous_attitude_rank, 2)
                estimates.append(core.q)
            for q in estimates[1:]:
                np.testing.assert_allclose(q, estimates[0], atol=3e-16)

    def test_inconsistent_acquisition_pair_does_not_initialize(self):
        core = MEKF(self.policy)
        core.observe(sample("magnetic", 0, np.eye(3), [1., 0., 0.]))
        corrupted = VectorSample("bad-sun", "sun", 0, 0, [1., 1., 0.], [0., 1., 0.], "test")
        self.assertEqual(core.observe(corrupted), "inconsistent_acquisition_pair")
        self.assertFalse(core.initialized)

    def test_local_large_error_after_acquisition(self):
        core = MEKF(self.policy)
        c = np.eye(3)
        core.observe(sample("magnetic", 0, c, [1., 0., 0.]))
        core.observe(sample("sun", 0, c, [0., 1., 0.]))
        core.q = product(core.q, exp_q(np.array([0.35, -0.3, 0.2])))
        before = self.attitude_error(core, c)
        for k in range(1, 21):
            core.propagate(k*100_000_000, [0., 0., 0.])
            for sensor, reference in (("magnetic", [1., 0., 0.]), ("sun", [0., 1., 0.])):
                self.assertEqual(core.observe(sample(sensor, core.epoch_ns, c, reference)), "updated")
        self.assertLess(self.attitude_error(core, c), before/10)

    def test_antipodal_and_wrong_transform_are_directional_failures(self):
        c = passive_rodrigues(np.array([0., 0., 0.9]))
        for sensor in ("magnetic", "sun"):
            core = MEKF(self.policy, prior_q=rbk.C2EP(c))
            q, p = core.q.copy(), core.p.copy()
            wrong = sample(sensor, 0, c.T, [1., 0., 0.])
            self.assertEqual(core.observe(wrong), "outside_local_gate")
            reversed_sample = sample(sensor, 0, -c, [1., 0., 0.])
            self.assertEqual(core.observe(reversed_sample), "outside_local_gate")
            np.testing.assert_array_equal(core.q, q)
            np.testing.assert_array_equal(core.p, p)

    def test_mounts_malformed_inputs_and_epoch_guards(self):
        c, mount = passive_rodrigues(np.array([0.2, -0.3, 0.1])), passive_rodrigues(np.array([0.4, 0.1, -0.2]))
        core = MEKF(self.policy, prior_q=rbk.C2EP(c))
        ref = np.array([1., 2., 3.])
        valid = sample("sun", 0, mount @ c, ref, frame="S", c_sb=mount)
        self.assertEqual(core.observe(valid), "updated")
        self.assertLess(self.attitude_error(core, c), 1e-15)
        for event in (replace(valid, c_sb=np.diag([1., 1., -1.])), replace(valid, c_sb=None),
                      replace(valid, measured=[np.nan, 0., 1.]), replace(valid, measured=[0., 0., 0.]),
                      replace(valid, reference_epoch_ns=1), replace(valid, valid=False)):
            self.assertNotEqual(core.observe(event), "updated")
        with self.assertRaisesRegex(ValueError, "epoch"):
            core.observe(replace(valid, epoch_ns=1, reference_epoch_ns=1))
        with self.assertRaises(ValueError):
            MEKF(self.policy, prior_q=[0., 0., 0., 0.])

    def test_single_vector_null_attitude_and_bias_directions(self):
        for sensor in ("magnetic", "sun"):
            core = MEKF(self.policy, prior_q=exp_q(np.array([0., 0., 0.5])))
            for i in range(10):
                core.propagate((i+1)*100_000_000, [0., 0., 0.])
                before = core.p.copy()
                core.observe(sample(sensor, core.epoch_ns, np.eye(3), [0., 0., 1.]))
                self.assertEqual(core.instantaneous_attitude_rank, 2)
                self.assertAlmostEqual(core.p[2, 2], before[2, 2], places=14)
                self.assertAlmostEqual(core.p[5, 5], before[5, 5], places=14)
            self.assertAlmostEqual(self.attitude_error(core, np.eye(3)), 0.5, places=14)
            self.assertGreater(core.p[2, 2], self.policy.p0[2, 2])

    def test_zero_one_axis_and_three_axis_bias_convergence(self):
        for key in ("basic_true_bias", "synthetic_one_axis_bias", "synthetic_three_axis_bias"):
            imposed = np.array(self.fixtures[key]["value"])
            core = MEKF(self.policy, prior_q=exp_q(np.array([0.03, -0.02, 0.01])))
            for i in range(1, 601):
                core.propagate(i*100_000_000, imposed)
                if i % 5 == 0:
                    for sensor, ref in (("magnetic", [1., 0., 0.]), ("sun", [0., 1., 0.])):
                        core.observe(sample(sensor, core.epoch_ns, np.eye(3), ref))
            self.assertLess(float(np.linalg.norm(core.bias-imposed)), 2e-6, key)
            self.assertLess(self.attitude_error(core, np.eye(3)), 1e-4, key)

    def test_rotating_body_preserves_single_sun_gauge_ambiguity(self):
        # The ambiguous rotation is about the inertial Sun line; its components
        # rotate in B. Checking only one fixed covariance coordinate would miss it.
        omega = np.array([0.2, -0.1, 0.3])
        core = MEKF(self.policy, prior_q=exp_q(np.array([0., 0., 0.5])))
        for i in range(1, 201):
            tick = i*100_000_000
            truth = passive_rodrigues(omega*tick*1e-9)
            core.propagate(tick, omega)
            core.observe(sample("sun", tick, truth, [0., 0., 1.]))
            null_body = truth[:, 2]
            self.assertGreaterEqual(float(null_body @ core.p[:3, :3] @ null_body), self.policy.p0[2, 2]-1e-12)
            self.assertAlmostEqual(self.attitude_error(core, truth), 0.5, places=12)

    def test_no_vectors_leave_bias_unobservable_and_attitude_drifting(self):
        imposed = np.array(self.fixtures["synthetic_three_axis_bias"]["value"])
        core = MEKF(self.policy, prior_q=[1., 0., 0., 0.])
        for i in range(1, 601):
            core.propagate(i*100_000_000, imposed)
        np.testing.assert_array_equal(core.bias, np.zeros(3))
        self.assertAlmostEqual(self.attitude_error(core, np.eye(3)), float(np.linalg.norm(imposed))*60, places=12)
        self.assertEqual(core.instantaneous_attitude_rank, 0)

    def test_constant_rate_covariance_transition_and_quadrature(self):
        omega = np.array([0.3, -0.2, 0.1])
        h = 0.4
        # Independent time integration of exp(-[omega]x t) for Phi's bias block.
        nodes, weights = np.polynomial.legendre.leggauss(32)
        integrated = sum(weight*h/2*passive_rodrigues(omega*h*(node+1)/2) for node, weight in zip(nodes, weights))
        np.testing.assert_allclose(transition(omega, h)[:3, 3:], -integrated, atol=3e-15)
        policy = replace(self.policy, qc=np.diag([1., 2., 3., 0.1, 0.2, 0.3]))
        a, b = MEKF(policy, prior_q=[1., 0., 0., 0.]), MEKF(policy, prior_q=[1., 0., 0., 0.])
        a.propagate(400_000_000, omega)
        for i in range(1, 9):
            b.propagate(i*50_000_000, omega)
        np.testing.assert_allclose(a.p, b.p, atol=2e-15)

    def test_synthetic_gyro_interpolation_and_refinement_independent_of_attitude(self):
        times = np.arange(8, dtype=float)
        # Polynomial angular rate around Z: integral is known independently.
        rates = np.column_stack((np.zeros(8), np.zeros(8), 0.1+0.02*times**2))
        for i, fraction in ((0, 0.3), (3, 0.6), (6, 0.8)):
            np.testing.assert_allclose(interpolated_gyro(rates, i, fraction),
                                       [0., 0., 0.1+0.02*(i+fraction)**2], atol=2e-16)
        errors = []
        for parts in (1, 10):
            core = MEKF(self.policy, prior_q=[1., 0., 0., 0.])
            for part in range(parts):
                core.propagate((part+1)*1_000_000_000//parts,
                               interpolated_gyro(rates, 0, (part+0.5)/parts))
            errors.append(self.attitude_error(core, passive_rodrigues(np.array([0., 0., 0.1+0.02/3]))))
        self.assertAlmostEqual(errors[0]/errors[1], 100., places=8)

    def test_reset_jacobian_and_actual_covariance_reset(self):
        a = np.array([0.35, -0.2, 0.1])
        eps = 1e-6
        numerical = np.column_stack([(log_q(product(inverse(exp_q(a)), exp_q(a+eps*axis)))
                                     - log_q(product(inverse(exp_q(a)), exp_q(a-eps*axis))))/(2*eps) for axis in np.eye(3)])
        np.testing.assert_allclose(right_jacobian(a), numerical, atol=1e-10)
        core = MEKF(self.policy, prior_q=exp_q(a))
        core.propagate(100_000_000, [0., 0., 0.])
        old_p, old_q = core.p.copy(), core.q.copy()
        predicted = np.asarray(rbk.EP2C(old_q)) @ np.array([0., 0., 1.])
        # Full three-component residual with radial variance only in algebra
        # yields the same two informative tangent directions independently.
        h = np.zeros((3, 6))
        for j, axis in enumerate(np.eye(3)):
            h[:, j] = (np.asarray(rbk.EP2C(product(old_q, exp_q(eps*axis))))[:, 2]
                       - np.asarray(rbk.EP2C(product(old_q, exp_q(-eps*axis))))[:, 2])/(2*eps)
        r = self.policy.vector_weight*np.eye(3)
        k = np.linalg.solve(h @ old_p @ h.T+r, h @ old_p).T
        pe = old_p-k @ h @ old_p
        core.observe(sample("sun", 100_000_000, np.eye(3), [0., 0., 1.]))
        correction = log_q(product(inverse(old_q), core.q))
        numerical = np.column_stack([(log_q(product(inverse(exp_q(correction)), exp_q(correction+eps*axis)))
                                     - log_q(product(inverse(exp_q(correction)), exp_q(correction-eps*axis))))/(2*eps) for axis in np.eye(3)])
        gamma = np.eye(6)
        gamma[:3, :3] = numerical
        np.testing.assert_allclose(core.p, gamma @ pe @ gamma.T, atol=2e-11)
        self.assertGreater(float(np.linalg.norm(core.p-pe)), 1e-4)

    def test_covariance_health_and_invalid_models(self):
        for invalid in (np.diag([-1., 1., 1., 1., 1., 1.]), np.full((6, 6), np.nan), np.triu(np.ones((6, 6)))):
            with self.assertRaises(ValueError):
                covariance(invalid, 6)
        with self.assertRaises(ValueError):
            replace(self.policy, vector_weight=0.)
        core = MEKF(self.policy, prior_q=[1., 0., 0., 0.])
        for i in range(100):
            core.propagate((i+1)*100_000_000, [0., 0., 0.])
            core.observe(sample("sun", core.epoch_ns, np.eye(3), [1., 1., 1.]))
            self.assertTrue(np.isfinite(core.p).all())
            np.testing.assert_allclose(core.p, core.p.T, atol=1e-16)
            self.assertGreater(float(np.linalg.eigvalsh(core.p).min()), 0.)


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.policy, _ = load_test_policy()
        self.omega = np.array([0.2, -0.3, 0.1])

    def initial(self, policy=None):
        return ReplayEstimator(MEKF(policy or self.policy, prior_q=exp_q(np.array([0.04, -0.02, 0.03]))))

    def event(self, tick, sensor):
        ref = np.array([1., 0.4, 0.2]) if sensor == "magnetic" else np.array([-0.1, 1., 0.3])
        ref = passive_rodrigues(np.array([0., 0.01*tick*1e-9, 0.])) @ ref
        return sample(sensor, tick, passive_rodrigues(self.omega*tick*1e-9), ref)

    def test_async_interior_epochs_and_replay_all_later_measurements(self):
        chronological, delayed = self.initial(), self.initial()
        first = self.event(250_000_000, "magnetic")
        events = [first, self.event(550_000_000, "sun"), self.event(800_000_000, "magnetic"), self.event(900_000_000, "sun")]
        previous = 0
        for event in events:
            chronological.advance(previous, event.epoch_ns, self.omega)
            chronological.submit(event, event.epoch_ns)
            previous = event.epoch_ns
        chronological.advance(previous, 1_000_000_000, self.omega)
        delayed.advance(0, 1_000_000_000, self.omega)
        for event in events[1:]:
            delayed.submit(event, 1_000_000_000)
        before = delayed.core.q.copy()
        delayed.submit(first, 1_000_000_000)
        self.assertGreater(float(np.linalg.norm(delayed.core.q-before)), 1e-6)
        for attr in ("q", "bias", "p"):
            np.testing.assert_allclose(getattr(delayed.core, attr), getattr(chronological.core, attr), atol=2e-16)
        self.assertEqual(delayed.core.updates, {"magnetic": 2, "sun": 2})
        self.assertEqual(delayed.core.last_update["epoch_ns"], 900_000_000)
        self.assertEqual(delayed.core.epoch_ns, 1_000_000_000)

    def test_rejections_coverage_duplicate_history_and_capacity(self):
        policy = replace(self.policy, history_ns=300_000_000, max_events=8)
        estimator = self.initial(policy)
        with self.assertRaisesRegex(ValueError, "coverage"):
            estimator.advance(1, 100_000_000, self.omega)
        for i in range(10):
            estimator.advance(i*100_000_000, (i+1)*100_000_000, self.omega)
        self.assertEqual(estimator.submit(self.event(200_000_000, "sun"), 1_000_000_000), "outside_history")
        self.assertEqual(estimator.submit(self.event(1_100_000_000, "sun"), 1_200_000_000), "future_or_missing_gyro_coverage")
        event = self.event(900_000_000, "sun")
        self.assertEqual(estimator.submit(event, 1_000_000_000), "updated")
        self.assertEqual(estimator.submit(event, 1_000_000_000), "duplicate_event")
        self.assertEqual(estimator.submit(self.event(950_000_000, "sun"), 900_000_000), "receipt_before_acquisition_or_output")
        self.assertLessEqual(len(estimator.gyros)+len(estimator.events), 8)

    def test_epoch_type_and_finiteness_are_not_silently_coerced(self):
        estimator = self.initial()
        for invalid in (True, -1, 0.1, float("nan")):
            with self.assertRaisesRegex(ValueError, "epoch"):
                estimator.advance(0, invalid, self.omega)
            with self.assertRaisesRegex(ValueError, "epoch"):
                estimator.submit(self.event(0, "sun"), invalid)

    def test_same_epoch_order_input_ownership_and_reacquisition(self):
        a, b = self.initial(), self.initial()
        events = [self.event(0, "sun"), self.event(0, "magnetic")]
        for event in events:
            a.submit(event, 0)
        for event in reversed(events):
            b.submit(event, 0)
        np.testing.assert_array_equal(a.core.q, b.core.q)
        measured = np.array([1., 0., 0.])
        event = VectorSample("mutable", "sun", 0, 0, measured, [1., 0., 0.], "test")
        a.submit(event, 0)
        measured[:] = 0
        a.advance(0, 100_000_000, self.omega)
        a.submit(LossEvent("loss", 100_000_000), 100_000_000)
        self.assertFalse(a.core.initialized)
        for sensor in ("magnetic", "sun"):
            a.submit(self.event(100_000_000, sensor), 100_000_000)
        self.assertTrue(a.core.initialized)
        self.assertEqual(a.core.acquisitions, 1)

    def test_pruned_checkpoint_matches_full_chronological_history(self):
        full = self.initial()
        pruned = self.initial(replace(self.policy, history_ns=300_000_000))
        delayed = None
        for i in range(1, 21):
            tick = i*100_000_000
            for estimator in (full, pruned):
                estimator.advance(tick-100_000_000, tick, self.omega)
            if i % 2 == 0:
                event = self.event(tick, "magnetic" if i % 4 else "sun")
                full.submit(event, tick)
                if i == 18:
                    delayed = event
                else:
                    pruned.submit(event, tick)
        pruned.submit(delayed, 2_000_000_000)
        np.testing.assert_allclose(pruned.core.q, full.core.q, atol=3e-16)
        np.testing.assert_allclose(pruned.core.p, full.core.p, atol=3e-16)
        self.assertEqual(pruned.core.updates, full.core.updates)


class CycleBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from magnetic_control_cycle import diagnostic_cycle_config
        from scenario_huskysat2_detumble import run
        cls.cycle = diagnostic_cycle_config()
        with contextlib.redirect_stdout(io.StringIO()):
            cls.frame = run(stop_time_s=2., cycle=cls.cycle, write_outputs=False, make_plots=False)

    def test_quiet_sample_acceptance_and_actual_tam_source(self):
        row = self.frame.iloc[4].to_dict()
        original = magnetic_cycle_sample(row, self.cycle)
        self.assertTrue(original.valid)
        for axis in "xyz":
            row[f"B_B_{axis}_T"] = 99.  # Current truth must have no influence.
        unchanged = magnetic_cycle_sample(row, self.cycle)
        np.testing.assert_array_equal(original.measured, unchanged.measured)
        self.assertTrue(unchanged.valid)
        policy, _ = load_test_policy()
        estimator = ReplayEstimator(MEKF(policy, prior_q=[1., 0., 0., 0.]))
        estimator.advance(0, 500_000_000, [0., 0., 0.])
        self.assertEqual(estimator.submit(original, 500_000_000), "updated")
        self.assertEqual(estimator.core.last_update["epoch_ns"], 400_000_000)
        self.assertEqual(estimator.core.epoch_ns, 500_000_000)

    def test_actuation_stale_stamp_and_inadequate_settling_reject(self):
        self.assertFalse(magnetic_cycle_sample(self.frame.iloc[6].to_dict(), self.cycle).valid)
        for key, value in (("cycle_sample_valid", False), ("cycle_time_since_disabled_ns", 0),
                           ("cycle_pre_native_command_x", 0.1), ("cycle_pre_native_effective_dipole_y", 0.1),
                           ("applied_torque_B_z_Nm", 1e-5), ("tam_message_time_ns", 0),
                           ("field_evaluation_time_ns", 0), ("tam_sample_B_B_x_T", 9.)):
            row = self.frame.iloc[4].to_dict()
            row[key] = value
            self.assertFalse(magnetic_cycle_sample(row, self.cycle).valid, key)


if __name__ == "__main__":
    unittest.main()
