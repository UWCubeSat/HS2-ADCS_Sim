"""8B-1 isolated value/frame/unit/eligibility tests, no estimator performance."""
from dataclasses import replace
import unittest

import numpy as np

from tam_sensor_model import (CleanlinessContext, TAMConfig, TAMModel, fixture, profile_config,
    microtesla_to_tesla, tesla_to_microtesla)


def quiet(tick=400_000_000, **changes):
    return replace(CleanlinessContext(tick, "SAMPLE", 0, (0.,0.,0.), (0.,0.,0.),
        400_000_000, 400_000_000, True, "TEST_ONLY_SCHEDULER", "isolated software fixture, not measured settling", "test-cycle"), **changes)


def sample(config=TAMConfig(), field=(20e-6,-30e-6,40e-6), context=None):
    return TAMModel(config).acquire(field, truth_epoch_ns=400_000_000, acquisition_ns=400_000_000,
        context=quiet() if context is None else context, field_source="synthetic unit-test local field")


class TAMModelTests(unittest.TestCase):
    def test_ideal_axes_arbitrary_field_and_no_rng(self):
        for field in ([1e-5,0.,0.], [0.,-2e-5,0.], [0.,0.,3e-5], [2.1e-5,-3.2e-5,4.3e-5]):
            result = sample(field=field)
            self.assertTrue(result.valid)
            np.testing.assert_array_equal(result.measurement_S_T, field)
            np.testing.assert_array_equal(result.reconstructed_B_T, field)
        self.assertIsNone(TAMModel(TAMConfig()).rng)

    def test_explicit_tesla_microtesla_and_config_units(self):
        np.testing.assert_allclose(microtesla_to_tesla([20.,-30.,40.]), [20e-6,-30e-6,40e-6], rtol=0, atol=1e-20)
        np.testing.assert_allclose(tesla_to_microtesla([20e-6,-30e-6,40e-6]), [20.,-30.,40.], rtol=0, atol=1e-14)
        with self.assertRaises(ValueError):
            replace(TAMConfig(), bias_S=fixture((1.,2.,3.), "uT", "S", "wrong units must not be implicit"))
        np.testing.assert_allclose(np.array(sample(profile_config("TEST_BIAS_ONLY")).measurement_S_T)-sample().measurement_S_T,
            [1e-6,-2e-6,3e-6], rtol=0, atol=1e-20)

    def test_positive_negative_axes_and_mounting_inverse(self):
        c = ((0.,1.,0.),(-1.,0.,0.),(0.,0.,1.))
        config = replace(TAMConfig(), mounting_C_SB=fixture(c, "1", "B to S", "synthetic +90-degree passive Z rotation"))
        for sign in (-1.,1.):
            for field, expected in (([1e-5,0,0], [0,-1e-5,0]), ([0,1e-5,0],[1e-5,0,0]), ([0,0,1e-5],[0,0,1e-5])):
                result = sample(config, sign*np.array(field))
                np.testing.assert_array_equal(result.measurement_S_T, sign*np.array(expected))
                np.testing.assert_array_equal(result.reconstructed_B_T, sign*np.array(field))

    def test_arbitrary_rotation_from_sensor_basis_dot_products(self):
        # Rows are orthonormal S-axis directions expressed in B, independently
        # project by scalar dot products; no MEKF/Basilisk frame helper reused.
        c = np.array([[1,2,2],[2,1,-2],[-2,2,-1]], dtype=float)/3
        field = np.array([1.7e-5,-2.3e-5,4.1e-5])
        cfg = replace(TAMConfig(), mounting_C_SB=fixture(tuple(tuple(float(x) for x in row) for row in c), "1", "B to S", "synthetic surveyed axes"))
        result = sample(cfg, field)
        expected = [sum(axis[j]*field[j] for j in range(3)) for axis in c]
        np.testing.assert_allclose(result.measurement_S_T, expected, rtol=0, atol=1e-20)
        np.testing.assert_allclose(result.reconstructed_B_T, field, rtol=0, atol=1e-20)
        with self.assertRaises(ValueError):
            replace(cfg, mounting_C_SB=fixture(((1,0,0),(0,1,0),(0,0,-1)), "1", "B to S", "invalid reflection"))

    def test_scale_cross_axis_and_sensor_frame_bias_once(self):
        field = np.array([20e-6,-30e-6,40e-6])
        np.testing.assert_array_equal(sample(profile_config("TEST_SCALE_ONLY"), field).measurement_S_T, field*[1.01,.98,1.03])
        cfg = replace(TAMConfig(), cross_axis=fixture(((0,.1,0),(0,0,-.2),(.3,0,0)), "1", "S", "synthetic cross-axis errors"))
        np.testing.assert_allclose(sample(cfg,field).measurement_S_T, [17e-6,-38e-6,46e-6], rtol=0, atol=1e-20)
        cfg = replace(profile_config("TEST_BIAS_ONLY"), mounting_C_SB=fixture(((0,1,0),(-1,0,0),(0,0,1)), "1", "B to S", "synthetic mount"))
        result = sample(cfg, field)
        np.testing.assert_allclose(result.measurement_S_T, [-29e-6,-22e-6,43e-6], rtol=0, atol=1e-20)
        np.testing.assert_allclose(result.reconstructed_B_T, [22e-6,-29e-6,43e-6], rtol=0, atol=1e-20)

    def test_exact_seeded_noise_once_and_repeatability(self):
        cfg = profile_config("TEST_NOISE_ONLY")
        a,b = TAMModel(cfg), TAMModel(cfg)
        expected = np.random.Generator(np.random.PCG64(8201)).standard_normal((4,3))*[1e-7,2e-7,3e-7]
        field = np.array([20e-6,-30e-6,40e-6])
        for i,noise in enumerate(expected):
            args = dict(truth_epoch_ns=i, acquisition_ns=i, context=quiet(i), field_source="synthetic test")
            x,y = a.acquire(field, **args), b.acquire(field, **args)
            self.assertEqual(x,y)
            np.testing.assert_array_equal(x.noise_S_T, noise)
            np.testing.assert_array_equal(x.measurement_S_T, field+noise)
        with self.assertRaises(ValueError):
            a.acquire(field, **args)

    def test_coils_quiet_and_missing_evidence_preserve_finite_value(self):
        conditions = [quiet(commanded_axis_dipoles_Am2=(.01,0,0)), quiet(effective_dipole_B_Am2=(0,.01,0)),
            quiet(coil_current_A=(0,0,.001)), quiet(phase="ACTUATE"), quiet(quiet_elapsed_ns=399_999_999),
            quiet(required_quiet_ns=None), quiet(evidence_scope="UNESTABLISHED"), quiet(source=""), quiet(epoch_ns=0)]
        for context in conditions:
            with self.subTest(context=context):
                result = sample(context=context)
                self.assertFalse(result.valid)
                self.assertTrue(result.rejection_reasons)
                np.testing.assert_array_equal(result.measurement_S_T, [20e-6,-30e-6,40e-6])
        self.assertTrue(sample(context=quiet(coil_current_A=(0.,0.,0.))).valid)
        result = TAMModel(TAMConfig()).acquire([1e-5,0,0], truth_epoch_ns=0, acquisition_ns=0, context=None, field_source="test")
        self.assertIn("missing_cleanliness_context", result.rejection_reasons)

    def test_saturation_and_nonfinite_are_not_fake_zero_samples(self):
        cfg = replace(TAMConfig(), range_S=fixture((25e-6,25e-6,25e-6), "T", "S", "synthetic clipping boundary"))
        result = sample(cfg)
        self.assertFalse(result.valid)
        self.assertEqual(result.saturated_axes, (False,True,True))
        np.testing.assert_array_equal(result.measurement_S_T, [20e-6,-25e-6,25e-6])
        for value in (np.nan,np.inf):
            result = sample(field=[value,1e-5,2e-5])
            self.assertFalse(result.valid)
            self.assertIsNone(result.measurement_S_T)
            self.assertIn("invalid_true_field", result.rejection_reasons)
        overflow = replace(TAMConfig(), scale=fixture((1e300,1.,1.), "1", "S", "numeric overflow guard fixture only"))
        result = sample(overflow, field=[1e200,0.,0.])
        self.assertFalse(result.valid)
        self.assertIsNone(result.measurement_S_T)
        self.assertIn("nonfinite_measurement", result.rejection_reasons)

    def test_epochs_availability_age_and_provenance_are_separate(self):
        cfg = replace(TAMConfig(), latency_ns=fixture(200_000_000, "ns", "simulation clock", "isolated synthetic availability delay"))
        result = sample(cfg)
        self.assertEqual((result.truth_epoch_ns,result.acquisition_ns,result.publication_ns), (400_000_000,400_000_000,600_000_000))
        args = dict(maximum_age_ns=300_000_000, expected_fingerprint=cfg.fingerprint())
        self.assertFalse(result.usable_at(500_000_000, **args).valid)
        self.assertTrue(result.usable_at(600_000_000, **args).valid)
        self.assertFalse(result.usable_at(800_000_000, **args).valid)
        self.assertFalse(result.usable_at(600_000_000, maximum_age_ns=300_000_000, expected_fingerprint="wrong").valid)
        mismatch = TAMModel(TAMConfig()).acquire([1e-5,0,0], truth_epoch_ns=0, acquisition_ns=1, context=quiet(1), field_source="test")
        self.assertFalse(mismatch.valid)
        self.assertIsNotNone(mismatch.measurement_S_T)

    def test_only_ideal_supported_live_and_configuration_is_not_flight(self):
        self.assertFalse(TAMConfig().runtime_usable_for_flight)
        TAMConfig().validate_live()
        for name in ("TEST_BIAS_ONLY","TEST_SCALE_ONLY","TEST_NOISE_ONLY"):
            with self.assertRaises(ValueError):
                profile_config(name).validate_live()
        with self.assertRaises(ValueError):
            replace(profile_config("TEST_BIAS_ONLY"), profile="IDEAL_REGRESSION").validate_live()


if __name__ == "__main__":
    unittest.main()
