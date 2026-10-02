"""8A-2C focused live composition tests; prior isolated suites are not rerun."""
import unittest

import numpy as np

from validate_gyro_bias_noise_response import INJECTED_BIAS, composition, validate


class GyroBiasNoiseResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.frame, cls.repeated = validate()

    def require_checks(self, *names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.report["checks"][name])

    def test_exact_combined_profile_and_measurement_composition(self):
        self.require_checks("only_authorized_combined_TEST_ONLY_profile", "current_truth_matches_ideal_reference",
            "exact_bias_plus_noise_composition_S_and_B", "same_independent_seeded_draws_as_noise_only",
            "deterministic_bias_once_relative_to_same_noise_case", "acquisition_publication_processing_epochs_preserved")

    def test_exact_live_repeatability(self):
        self.require_checks("repeat_all_input_samples_intervals_events_exact", "repeat_estimator_states_covariance_counts_exact",
            "repeat_innovations_witnesses_and_status_exact", "repeat_native_navigation_telemetry_exact")

    def test_bias_sign_finite_response_and_update_continuity(self):
        self.require_checks("finite_valid_response", "final_bias_and_matched_noise_difference_have_injected_sign",
            "bias_held_in_propagation_and_explained_by_vector_updates", "posterior_bias_subtracted_once")

    def test_independent_combined_endpoint_interpolation(self):
        self.require_checks("independent_endpoint_interpolated_short_interval")

    def test_vector_innovations_and_corrections(self):
        self.require_checks("finite_consistent_vector_innovation_direction", "every_actual_vector_update_reduces_measured_residual",
            "both_vector_histories_expose_combined_errors", "same_vector_events_replays_and_no_rejections")

    def test_covariance_numerical_health(self):
        self.require_checks("covariance_finite_symmetric_positive_definite", "no_covariance_growth_beyond_zero_Q_kinematic_bound")

    def test_shadow_authority_and_production_preservation(self):
        self.require_checks("host_spacecraft_actuators_match_all_references_and_repeat", "SimpleNav_control_without_development_command_owner",
            "combined_modeled_gyro_closed_loop_rejected", "shared_runtime_helpers_and_Q_R_P0_unchanged")
        self.assertTrue(self.report["passed"], self.report["checks"])

    def test_composition_gate_rejects_missing_or_doubled_effects(self):
        for effect in ("missing_bias", "double_bias", "double_noise"):
            with self.subTest(effect=effect):
                changed = self.frame.copy(deep=True)
                batch = changed.attrs["shadow_input_trace"][5]
                delta = -INJECTED_BIAS if effect == "missing_bias" else INJECTED_BIAS if effect == "double_bias" else np.array(batch.gyro_sample.noise_S_rad_s)
                batch.gyro_point_B += delta
                result = composition(changed, self.report["cases"])
                self.assertFalse(result["checks"]["exact_bias_plus_noise_composition_S_and_B"])


if __name__ == "__main__":
    unittest.main()
