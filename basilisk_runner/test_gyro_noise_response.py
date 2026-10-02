"""8A-2B focused live noise delivery/response tests; no statistical campaign."""
from copy import deepcopy
from dataclasses import replace
import unittest

from validate_gyro_noise_response import repeatability, sequence_evidence, validate


class GyroNoiseResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.ideal, cls.noisy, cls.repeated, cls.noise_case, cls.repeat_case = validate()

    def require_checks(self, *names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.report["checks"][name])

    def test_exact_noise_only_sequence_and_epochs(self):
        self.require_checks("existing_noise_only_configuration", "ideal_measurements_are_truth",
            "exact_seeded_draws_without_extra_consumption", "noise_applied_exactly_once_in_S_and_B",
            "measured_minus_ideal_matches_expected_to_roundoff", "all_original_sample_epochs_preserved")

    def test_live_deterministic_repeatability(self):
        self.require_checks("repeat_all_input_samples_intervals_events_exact", "repeat_estimator_states_covariance_counts_exact",
            "repeat_innovations_witnesses_and_status_exact", "repeat_native_navigation_telemetry_exact")

    def test_finite_response_and_independent_propagation(self):
        self.require_checks("finite_initialized_response", "posterior_bias_subtracted_once",
            "bias_changes_only_on_vector_processing", "random_samples_not_assigned_as_fixed_bias",
            "independent_short_interval_direction_and_magnitude")

    def test_vector_innovations_and_corrections(self):
        self.require_checks("finite_geometrically_consistent_innovations", "actual_updates_reduce_measured_vector_residuals",
            "both_vector_residual_histories_perturbed", "same_vector_events_replays_and_no_rejections")

    def test_covariance_math_health(self):
        self.require_checks("covariance_finite_symmetric_positive_definite", "no_covariance_growth_beyond_zero_Q_kinematic_bound")

    def test_ideal_preservation_and_no_actuator_authority(self):
        self.require_checks("host_spacecraft_commands_torques_byte_identical", "SimpleNav_control_without_development_command_owner",
            "modeled_gyro_closed_loop_rejected", "ideal_reference_preserved", "shared_runtime_and_Q_R_P0_unchanged")
        self.assertTrue(self.report["passed"], self.report["checks"])

    def test_delivery_check_rejects_double_noise(self):
        changed = self.noisy.copy(deep=True)
        batch = changed.attrs["shadow_input_trace"][5]
        batch.gyro_point_B += batch.gyro_sample.noise_S_rad_s
        self.assertFalse(sequence_evidence(self.ideal, changed)["checks"]["noise_applied_exactly_once_in_S_and_B"])

    def test_sequence_check_rejects_shifted_rng_draw(self):
        changed = self.noisy.copy(deep=True)
        batches = changed.attrs["shadow_input_trace"]
        batches[0].gyro_sample = replace(batches[0].gyro_sample, noise_S_rad_s=batches[1].gyro_sample.noise_S_rad_s)
        self.assertFalse(sequence_evidence(self.ideal, changed)["checks"]["exact_seeded_draws_without_extra_consumption"])

    def test_repeat_check_rejects_changed_covariance(self):
        changed = deepcopy(self.repeat_case)
        changed["history"][5]["covariance"][0][0] += .001
        checks = repeatability(self.noisy, self.repeated, self.noise_case, changed)
        self.assertFalse(checks["repeat_innovations_witnesses_and_status_exact"])


if __name__ == "__main__":
    unittest.main()
