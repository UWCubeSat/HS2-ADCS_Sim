"""Independent sensor algebra/statistics and timestamp tests, not flight accuracy."""
from dataclasses import replace
import unittest

import numpy as np

from attitude_mekf import MEKF
from attitude_mekf_prototype import load_test_policy
from gyro_sensor_model import GyroConfig, GyroModel, fixture, profile_config


class GyroModelTests(unittest.TestCase):
    def test_ideal_zero_axes_and_arbitrary_are_bit_exact(self):
        model = GyroModel(profile_config())
        for i, rate in enumerate(([0., -0., 0.], [1., 0., 0.], [0., -2., 0.], [0., 0., 3.], [.123, -.987, .456])):
            actual = model.acquire(rate, i*100_000_000)
            self.assertTrue(actual.valid)
            self.assertEqual(np.array(actual.measurement_B_rad_s).tobytes(), np.array(rate).tobytes())
            self.assertEqual(actual.acquisition_ns, actual.publication_ns)
        self.assertIsNone(model.rng)

    def test_nonidentity_direction_sign_and_inverse(self):
        # Define sensor axes directly in B: Sx=By, Sy=-Bx, Sz=Bz.
        c = ((0., 1., 0.), (-1., 0., 0.), (0., 0., 1.))
        config = replace(GyroConfig(), mounting_C_SB=fixture(c, "1", "B to S", "synthetic quarter turn"))
        sample = GyroModel(config).acquire([2., -3., 4.], 0)
        np.testing.assert_array_equal(sample.measurement_S_rad_s, [-3., -2., 4.])
        np.testing.assert_array_equal(sample.measurement_B_rad_s, [2., -3., 4.])

    def test_arbitrary_orientation_uses_independent_axis_projections(self):
        x = np.array([1., 2., 3.])/np.sqrt(14.)
        y = np.array([2., -1., 0.])/np.sqrt(5.)
        z = np.cross(x, y)
        c = tuple(tuple(row) for row in (x, y, z))
        rate = np.array([.7, -.2, 1.1])
        model = GyroModel(replace(GyroConfig(), mounting_C_SB=fixture(c, "1", "B to S", "synthetic basis")))
        sample = model.acquire(rate, 0)
        expected_s = [np.dot(axis, rate) for axis in (x, y, z)]
        np.testing.assert_allclose(sample.measurement_S_rad_s, expected_s, rtol=0, atol=3e-16)
        np.testing.assert_allclose(sample.measurement_B_rad_s, rate, rtol=0, atol=8e-16)

    def test_bias_positive_negative_three_axes_and_MEKF_subtraction(self):
        policy, _ = load_test_policy()
        truth = np.array([.1, -.4, .7])
        for bias in ((.003, 0., 0.), (0., -.002, 0.), (.003, -.002, .001)):
            config = replace(GyroConfig(), bias_S=fixture(bias, "rad/s", "S", "synthetic sign test"))
            sample = GyroModel(config).acquire(truth, 0)
            np.testing.assert_array_equal(sample.measurement_B_rad_s, truth+np.array(bias))
            core = MEKF(policy, prior_q=[1., 0., 0., 0.], prior_bias=bias)
            np.testing.assert_allclose(core.point_rate(sample.measurement_B_rad_s, 0), truth, rtol=0, atol=1e-16)
            np.testing.assert_array_equal(core.bias, bias)  # No model-side subtraction or estimator mutation.

    def test_sensor_bias_is_rotated_into_B_before_estimator(self):
        c = ((0., 1., 0.), (-1., 0., 0.), (0., 0., 1.))
        config = replace(profile_config("TEST_BIAS_ONLY"), mounting_C_SB=fixture(c, "1", "B to S", "synthetic"))
        sample = GyroModel(config).acquire([0., 0., 0.], 0)
        np.testing.assert_array_equal(sample.measurement_B_rad_s, [.002, .003, .001])

    def test_scale_and_cross_axis_signs_and_zero_input(self):
        config = profile_config("TEST_SCALE_ONLY")
        sample = GyroModel(config).acquire([2., -3., 4.], 0)
        np.testing.assert_allclose(sample.measurement_B_rad_s, [2.02, -2.94, 4.12], rtol=0, atol=5e-16)
        cross = ((0., .01, 0.), (0., 0., -.02), (.03, 0., 0.))
        coupled = replace(GyroConfig(), cross_axis=fixture(cross, "1", "S", "synthetic off-diagonal leakage"))
        np.testing.assert_array_equal(GyroModel(coupled).acquire([0., 0., 0.], 0).measurement_B_rad_s, [0., 0., 0.])
        np.testing.assert_allclose(GyroModel(coupled).acquire([2., -3., 4.], 0).measurement_B_rad_s,
                                   [1.97, -3.08, 4.06], rtol=0, atol=5e-16)

    def test_noise_seed_statistics_and_zero_noise_no_draws(self):
        config = profile_config("TEST_NOISE_ONLY")
        model, repeat = GyroModel(config), GyroModel(config)
        count = 20_000
        samples = np.array([model.acquire([0., 0., 0.], i*100_000_000).measurement_S_rad_s for i in range(count)])
        for i in range(10):
            np.testing.assert_array_equal(samples[i], repeat.acquire([0., 0., 0.], i*100_000_000).measurement_S_rad_s)
        sigma = np.array(config.noise_sigma_S.value)
        self.assertTrue(np.all(np.abs(samples.mean(axis=0)) < 6*sigma/np.sqrt(count)))
        self.assertTrue(np.all(np.abs(samples.var(axis=0, ddof=1)/sigma**2-1) < 6*np.sqrt(2/(count-1))))
        correlation = np.corrcoef(samples.T)-np.eye(3)
        self.assertLess(float(np.max(np.abs(correlation))), 6/np.sqrt(count))
        zero = GyroModel(replace(config, noise_sigma_S=fixture((0.,)*3, "rad/s", "S", "zero noise test")))
        self.assertIsNone(zero.rng)
        np.testing.assert_array_equal(zero.acquire([1., 2., 3.], 0).measurement_B_rad_s, [1., 2., 3.])

    def test_off_grid_calls_do_not_draw_noise_and_seed_changes_stream(self):
        config = profile_config("TEST_NOISE_ONLY")
        a, b = GyroModel(config), GyroModel(config)
        self.assertEqual(a.acquire([0., 0., 0.], 0), b.acquire([0., 0., 0.], 0))
        self.assertIsNone(a.acquire([0., 0., 0.], 50_000_000))
        self.assertEqual(a.acquire([0., 0., 0.], 100_000_000), b.acquire([0., 0., 0.], 100_000_000))
        other = GyroModel(replace(config, seed=fixture(8102, "1", "software RNG", "second synthetic seed")))
        self.assertNotEqual(other.acquire([0., 0., 0.], 0).noise_S_rad_s,
                            GyroModel(config).acquire([0., 0., 0.], 0).noise_S_rad_s)

    def test_cadence_publication_and_processing_age(self):
        model = GyroModel(profile_config("TEST_DELAYED_SAMPLE"))
        sample = model.acquire([1., 2., 3.], 0)
        self.assertEqual(sample.acquisition_ns, 0)
        self.assertEqual(sample.publication_ns, 200_000_000)
        self.assertEqual(sample.age_ns(250_000_000), 250_000_000)
        with self.assertRaises(ValueError):
            sample.age_ns(199_999_999)
        self.assertIsNone(model.acquire([1., 2., 3.], 50_000_000))
        self.assertEqual(model.acquire([1., 2., 3.], 100_000_000).publication_ns, 300_000_000)
        for tick in (100_000_000, 0, -1, True, 1.5):
            with self.assertRaises(ValueError):
                model.acquire([0., 0., 0.], tick)
        with self.assertRaisesRegex(ValueError, "delayed gyro is isolated-only"):
            model.config.validate_live(100_000_000)

    def test_range_both_signs_and_invalid_saturation_flag(self):
        config = replace(GyroConfig(), range_S=fixture((1., 2., 3.), "rad/s", "S", "synthetic hard range"))
        sample = GyroModel(config).acquire([2., -3., 3.], 0)
        np.testing.assert_array_equal(sample.measurement_S_rad_s, [1., -2., 3.])
        self.assertEqual(sample.saturated_axes, (True, True, False))
        self.assertFalse(sample.valid)
        self.assertEqual(sample.reason, "gyro_saturated")
        self.assertTrue(GyroModel(config).acquire([-1., 2., -3.], 0).valid)

    def test_saturation_happens_in_sensor_frame_before_inverse_mount(self):
        config = replace(GyroConfig(),
            mounting_C_SB=fixture(((0.,1.,0.),(-1.,0.,0.),(0.,0.,1.)), "1", "B to S", "synthetic quarter turn"),
            range_S=fixture((1., 2., 3.), "rad/s", "S", "synthetic axis-dependent range"))
        sample = GyroModel(config).acquire([3., 2., -4.], 0)
        np.testing.assert_array_equal(sample.measurement_S_rad_s, [1., -2., -3.])
        np.testing.assert_array_equal(sample.measurement_B_rad_s, [2., 1., -3.])
        self.assertFalse(sample.valid)

    def test_reject_invalid_configuration_and_nonfinite_truth(self):
        changes = [dict(runtime_usable_for_flight=True), dict(profile=""),
            dict(sample_period_ns=fixture(0, "ns", "clock", "bad")),
            dict(latency_ns=fixture(-1, "ns", "clock", "bad")),
            dict(scale=fixture((1., 0., 1.), "1", "S", "bad")),
            dict(bias_S=fixture((1., 2.), "rad/s", "S", "bad")),
            dict(range_S=fixture((1., -2., 3.), "rad/s", "S", "bad")),
            dict(noise_sigma_S=fixture((.1,)*3, "rad/s", "S", "missing seed")),
            dict(bias_S=fixture((0.,)*3, "deg/s", "S", "wrong units")),
            dict(bias_S=fixture((0.,)*3, "rad/s", "B", "wrong frame")),
            dict(cross_axis=fixture(((1.,0.,0.),(0.,0.,0.),(0.,0.,0.)), "1", "S", "bad diagonal")),
            dict(mounting_C_SB=fixture(((1.,0.,0.),(0.,1.,0.),(0.,0.,-1.)), "1", "B to S", "reflection"))]
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(GyroConfig(), **change)
        with self.assertRaises(ValueError):
            fixture((float('nan'), 0., 0.), "rad/s", "S", "bad")
        sample = GyroModel(GyroConfig()).acquire([float('inf'), 0., 0.], 0)
        self.assertFalse(sample.valid)
        self.assertIsNone(sample.measurement_B_rad_s)

    def test_provenance_and_fingerprint_follow_configuration(self):
        a, b = profile_config(), profile_config("TEST_BIAS_ONLY")
        self.assertNotEqual(a.fingerprint(), b.fingerprint())
        self.assertFalse(a.to_dict()['runtime_usable_for_flight'])
        for item in a.to_dict().values():
            if isinstance(item, dict):
                self.assertTrue(all(k in item for k in ('value','units','status','source','revision','frame','treatment')))


if __name__ == "__main__":
    unittest.main()
