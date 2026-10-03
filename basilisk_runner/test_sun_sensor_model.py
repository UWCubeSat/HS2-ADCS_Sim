"""8C-1 independent synthetic geometry/calibration/validity tests, not flight tests."""
from dataclasses import replace
import math
import unittest

import numpy as np

from sun_sensor_model import (CSSChannel, IDENTITY, RESPONSE_UNITS, SunAvailability,
                              SunConfig, SunModel, fixture, profile_config)


def acquire(cfg=None, direction=(.2, -.3, .7), c_bn=IDENTITY, tick=0, **kwargs):
    model = SunModel(cfg or profile_config("TEST_ARRAY_GEOMETRY"))
    options = dict(truth_epoch_ns=tick, acquisition_ns=tick,
                   availability=SunAvailability(tick, True, "ASSUMED TEST-ONLY visible; no eclipse model"),
                   source="synthetic unit-test Sun, 2026-10-02")
    options.update(kwargs)
    return model.acquire(direction, c_bn, **options)


def changed(cfg, **parameters):
    return replace(cfg, **{name: replace(getattr(cfg, name), value=value) for name, value in parameters.items()})


def single(normal=(0., 0., 1.), **parameters):
    channel = CSSChannel("synthetic", normal_S=fixture(normal, "1", "S_i", "unit-test axis"))
    return SunConfig(profile="TEST-ONLY single channel", mode="CSS_ARRAY", channels=(changed(channel, **parameters),))


def angle(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return float(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b)))


class SunModelTests(unittest.TestCase):
    def test_direct_regression_retains_legacy_magnitude_and_separate_truth(self):
        reference = np.array([.3, .8, -.5])
        c = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        result = acquire(SunConfig(), reference, c)
        np.testing.assert_array_equal(result.reconstruction.measurement_B, c @ reference)
        np.testing.assert_allclose(result.truth.direction_B, (c @ reference)/np.linalg.norm(reference), atol=2e-16)
        self.assertNotAlmostEqual(np.linalg.norm(result.reconstruction.measurement_B), 1.)
        self.assertEqual(result.channels, ())
        self.assertEqual(result.reconstruction.method, "development_direct_vector")

    def test_inertial_to_body_direction_not_transpose(self):
        c = ((0., 1., 0.), (-1., 0., 0.), (0., 0., 1.))
        result = acquire(SunConfig(), (1., 0., 0.), c)
        np.testing.assert_array_equal(result.truth.direction_B, (0., -1., 0.))
        self.assertFalse(np.array_equal(result.truth.direction_B, np.array(c).T @ [1., 0., 0.]))

    def test_sensor_mount_transpose_maps_normal_to_body(self):
        cfg = single((1., 0., 0.))
        c = ((0., 1., 0.), (-1., 0., 0.), (0., 0., 1.))
        cfg = replace(cfg, channels=(changed(cfg.channels[0], mounting_C_SB=c),))
        result = acquire(cfg, (0., 1., 0.))
        np.testing.assert_array_equal(result.channels[0].normal_B, (0., 1., 0.))
        self.assertEqual(result.channels[0].response, 1.)

    def test_on_normal_sixty_perpendicular_and_behind(self):
        cases = [((0., 0., 1.), 1., True), ((math.sqrt(3)/2, 0., .5), .5, True),
                 ((1., 0., 0.), 0., False), ((0., 0., -1.), 0., False)]
        for sun, expected, valid in cases:
            with self.subTest(sun=sun):
                channel = acquire(single(), sun).channels[0]
                self.assertAlmostEqual(channel.response, expected, delta=3e-16)
                self.assertEqual(channel.valid, valid)

    def test_fov_inclusive_edge_and_outside(self):
        cfg = single(half_fov_rad=math.pi/3)
        for a, eligible in ((math.pi/3, True), (math.pi/3-1e-6, True), (math.pi/3+1e-6, False)):
            channel = acquire(cfg, (math.sin(a), 0., math.cos(a))).channels[0]
            self.assertEqual(channel.in_fov, eligible)
            self.assertEqual(channel.valid, eligible)
            self.assertAlmostEqual(channel.response, math.cos(a) if eligible else 0., delta=3e-16)

    def test_synthetic_array_positive_negative_axes_and_arbitrary_directions(self):
        errors = []
        for sun in [*(sign*axis for sign in (-1, 1) for axis in np.eye(3)), (.2, -.3, .7), (.4, .8, -.1)]:
            sample = acquire(direction=sun)
            self.assertTrue(sample.reconstruction.valid, sample.reconstruction.reasons)
            self.assertGreaterEqual(len(sample.reconstruction.used_channels), 3)
            errors.append(angle(sample.reconstruction.direction_B, sun))
        self.assertLess(max(errors), 1e-14)

    def test_arbitrary_rotated_array_and_nonidentity_attitude(self):
        # Independent Rodrigues active rotation, not Basilisk or MEKF helpers.
        axis = np.array([1., 2., 3.])/math.sqrt(14)
        skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
        r = np.eye(3)+math.sin(.47)*skew+(1-math.cos(.47))*(skew @ skew)
        cfg = profile_config("TEST_ARRAY_GEOMETRY")
        cfg = replace(cfg, channels=tuple(changed(c, mounting_C_SB=tuple(map(tuple, r.T))) for c in cfg.channels))
        ref = np.array([.2, -.3, .7])
        sample = acquire(cfg, ref, r)
        self.assertTrue(sample.reconstruction.valid)
        self.assertLess(angle(sample.reconstruction.direction_B, r @ ref), 1e-14)
        for measurement, config in zip(sample.channels, cfg.channels):
            np.testing.assert_allclose(measurement.normal_B, r @ np.array(config.normal_S.value), atol=2e-16)

    def test_valid_channels_do_not_imply_valid_reconstruction(self):
        result = acquire(single(), (0., 0., 1.))
        self.assertTrue(result.channels[0].valid)
        self.assertFalse(result.reconstruction.valid)
        self.assertIn("insufficient_independent_geometry", result.reconstruction.reasons)
        self.assertIsNone(result.reconstruction.direction_B)

    def test_no_illuminated_and_cube_edge_rank_loss(self):
        result = acquire(single(), (0., 0., -1.))
        self.assertIn("no_illuminated_channels", result.reconstruction.reasons)
        # Only two positive independent rows; zero-incidence rows are not used.
        edge = acquire(direction=(1., 1., 0.))
        self.assertIn("insufficient_independent_geometry", edge.reconstruction.reasons)

    def test_poor_conditioning_is_separate_from_rank(self):
        eps = 1e-8
        normals = [np.array([0., 0., 1.]), np.array([eps, 0., 1.]), np.array([0., eps, 1.])]
        channels = tuple(CSSChannel(str(i), normal_S=fixture(tuple(n/np.linalg.norm(n)), "1", "S_i", "poor geometry test"))
                         for i, n in enumerate(normals))
        cfg = changed(SunConfig(profile="TEST-ONLY poor geometry", mode="CSS_ARRAY", channels=channels), minimum_singular_ratio=1e-6)
        result = acquire(cfg, (0., 0., 1.))
        self.assertEqual(result.reconstruction.rank, 3)
        self.assertIn("ill_conditioned_geometry", result.reconstruction.reasons)

    def test_reconstruction_is_measurement_only_and_scale_invariant(self):
        cfg = profile_config("TEST_ARRAY_GEOMETRY")
        result = acquire(cfg)
        model = SunModel(cfg)
        for gain in (.1, 2., 10.):
            channels = tuple(replace(c, response=c.response*gain) for c in result.channels)
            reconstructed = model.reconstruct(channels, 0)
            self.assertTrue(reconstructed.valid)
            self.assertLess(angle(reconstructed.direction_B, result.reconstruction.direction_B), 1e-14)
        channels = tuple(replace(c, response=0.) for c in result.channels)
        self.assertIn("degenerate_reconstruction_norm", model.reconstruct(channels, 0).reasons)

    def test_differential_gain_changes_direction_and_calibration_recovers_it(self):
        base = profile_config("TEST_ARRAY_GEOMETRY")
        nominal = acquire(base)
        selected = next(i for i, c in enumerate(nominal.channels) if c.valid)
        channels = list(base.channels)
        channels[selected] = changed(channels[selected], gain=1.2, offset=.03)
        perturbed = acquire(replace(base, channels=tuple(channels)))
        self.assertTrue(perturbed.reconstruction.valid)
        self.assertGreater(angle(nominal.reconstruction.direction_B, perturbed.reconstruction.direction_B), 1e-3)
        channels[selected] = changed(channels[selected], calibration_gain=1.2, calibration_offset=.03)
        calibrated = acquire(replace(base, channels=tuple(channels)))
        self.assertLess(angle(nominal.reconstruction.direction_B, calibrated.reconstruction.direction_B), 1e-14)

    def test_eclipse_and_occlusion_are_distinct_and_never_valid_zero(self):
        cfg = profile_config("TEST_ARRAY_GEOMETRY")
        for mode in (SunConfig(), cfg):
            sample = acquire(mode, availability=SunAvailability(0, False, "TEST-ONLY eclipse provider", reason="eclipse"))
            self.assertIsNotNone(sample.truth)
            self.assertFalse(sample.reconstruction.valid)
            self.assertIsNone(sample.reconstruction.measurement_B)
            self.assertIn("sun_unavailable", sample.reconstruction.reasons)
        masks = (True,)*len(cfg.channels)
        occluded = acquire(cfg, availability=SunAvailability(0, True, "TEST-ONLY mask", masks))
        self.assertTrue(all("occluded" in c.reasons for c in occluded.channels))
        self.assertFalse(occluded.reconstruction.valid)
        self.assertNotIn("sun_unavailable", occluded.reconstruction.reasons)

    def test_clipping_preserves_reading_but_excludes_channel(self):
        sample = acquire(single(maximum_response=.25), (0., 0., 1.))
        channel = sample.channels[0]
        self.assertEqual(channel.response, .25)
        self.assertTrue(channel.saturated)
        self.assertFalse(channel.valid)
        self.assertIsNone(sample.reconstruction.measurement_B)

    def test_noise_seed_reproducible_and_no_default_rng(self):
        base = profile_config("TEST_ARRAY_GEOMETRY")
        self.assertIsNone(SunModel(base).rng)
        self.assertIsNone(SunModel(SunConfig()).rng)
        channels = tuple(changed(c, noise_sigma=1e-4) for c in base.channels)
        with self.assertRaises(ValueError):
            replace(base, channels=channels)
        cfg = replace(changed(base, seed=8301), channels=channels)
        self.assertEqual(acquire(cfg), acquire(cfg))
        self.assertTrue(any(c.noise != 0 for c in acquire(cfg).channels))

    def test_acquisition_publication_reconstruction_processing_separate(self):
        cfg = changed(SunConfig(), latency_ns=200)
        sample = acquire(cfg, tick=100)
        self.assertEqual((sample.truth_epoch_ns, sample.acquisition_ns, sample.publication_ns, sample.reconstruction_ns), (100,100,300,300))
        self.assertEqual(sample.usable_at(200, maximum_age_ns=500, expected_fingerprint=cfg.fingerprint()), (False, ("not_yet_published",)))
        self.assertEqual(sample.usable_at(300, maximum_age_ns=500, expected_fingerprint=cfg.fingerprint()), (True, ()))
        self.assertFalse(sample.usable_at(700, maximum_age_ns=500, expected_fingerprint=cfg.fingerprint())[0])
        self.assertFalse(sample.usable_at(300, maximum_age_ns=500, expected_fingerprint="wrong")[0])

    def test_stale_truth_and_availability_metadata_reject(self):
        self.assertIn("truth_acquisition_epoch_mismatch", acquire(SunConfig(), truth_epoch_ns=1).reconstruction.reasons)
        for availability in (SunAvailability(1, True, "test"), SunAvailability(0, True, ""), SunAvailability(0, True, "test", (True,))):
            sample = acquire(SunConfig(), availability=availability)
            self.assertIn("invalid_availability_metadata", sample.reconstruction.reasons)
            self.assertIsNone(sample.reconstruction.measurement_B)

    def test_nonfinite_truth_bad_rotation_and_nonfinite_channels(self):
        for direction in ((0.,0.,0.), (float("nan"),0.,1.)):
            self.assertFalse(acquire(direction=direction).reconstruction.valid)
        self.assertFalse(acquire(c_bn=-np.eye(3)).reconstruction.valid)
        cfg = profile_config("TEST_ARRAY_GEOMETRY")
        channels = list(acquire(cfg).channels)
        i = next(i for i, c in enumerate(channels) if c.valid)
        channels[i] = replace(channels[i], response=float("nan"))
        self.assertIn("nonfinite_channel_value", SunModel(cfg).reconstruct(tuple(channels), 0).reasons)

    def test_channel_metadata_identity_and_epochs_checked(self):
        cfg = profile_config("TEST_ARRAY_GEOMETRY")
        original = acquire(cfg).channels
        for kwargs in (dict(identifier="wrong"), dict(acquisition_ns=1), dict(publication_ns=1),
                       dict(configuration_fingerprint="wrong"), dict(units="V"), dict(normal_B=(0.,0.,1.))):
            channels = (replace(original[0], **kwargs), *original[1:])
            self.assertIn("invalid_channel_metadata", SunModel(cfg).reconstruct(channels, 0).reasons)

    def test_config_provenance_fingerprints_and_live_allowlist(self):
        direct, array = SunConfig(), profile_config("TEST_ARRAY_GEOMETRY")
        self.assertFalse(array.runtime_usable_for_flight)
        self.assertNotEqual(array.fingerprint(), direct.fingerprint())
        direct.validate_live()
        for cfg in (array, replace(array, profile="IDEAL_REGRESSION"), changed(direct, latency_ns=1)):
            with self.assertRaisesRegex(ValueError, "live Sun permits only"):
                cfg.validate_live()
        for fn in (lambda: replace(direct, runtime_usable_for_flight=True),
                   lambda: single((0.,0.,2.)), lambda: single(gain="nan"),
                   lambda: single(gain=True), lambda: single(half_fov_rad=math.pi),
                   lambda: replace(array, channels=(array.channels[0], array.channels[0]))):
            with self.assertRaises(ValueError):
                fn()

    def test_duplicate_acquisition_rejected(self):
        model = SunModel(SunConfig())
        kwargs = dict(truth_epoch_ns=0, acquisition_ns=0, availability=SunAvailability(0, True, "test"), source="test")
        model.acquire([1.,0.,0.], IDENTITY, **kwargs)
        with self.assertRaisesRegex(ValueError, "must advance"):
            model.acquire([1.,0.,0.], IDENTITY, **kwargs)


if __name__ == "__main__":
    unittest.main()
