"""8A-2A one live deterministic perturbation; no repeated sensor/core tests."""
import unittest

from validate_gyro_bias_response import assess, validate


class GyroBiasResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.ideal, cls.biased, cls.iw, cls.bw = validate()

    def require_checks(self, *names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.report["evidence"]["checks"][name])

    def test_live_bias_frame_and_single_addition_subtraction(self):
        self.require_checks("bias_applied_once_at_all_live_samples", "original_acquisition_publication_epochs",
                            "posterior_bias_subtracted_once_in_native_NavAtt")

    def test_bias_state_and_independent_early_error_direction(self):
        self.require_checks("finite_valid_bias_estimator", "bias_state_moves_toward_injected_sign",
                            "first_and_subsequent_bias_estimates_have_consistent_sign",
                            "first_unupdated_interval_matches_independent_bias_tendency")

    def test_vector_innovations_and_actual_corrections(self):
        self.require_checks("innovation_sign_norm_and_direction", "every_update_reduces_its_measured_vector_residual",
                            "both_vector_types_expose_bias_error", "same_vector_events_updates_replay_and_no_rejections")

    def test_covariance_math_health(self):
        self.require_checks("covariance_numerically_healthy", "no_covariance_growth_beyond_zero_Q_kinematic_bound")

    def test_ideal_preservation_and_no_actuator_authority(self):
        self.require_checks("shadow_host_actuation_and_state_byte_identical", "ideal_reference_preserved",
                            "modeled_gyro_closed_loop_guard_effective", "shared_runtime_and_Q_R_P0_unchanged",
                            "only_specified_bias_no_noise_or_mount_change")
        self.assertTrue(self.report["passed"], self.report["evidence"]["checks"])

    def test_validator_rejects_corrupted_truth_to_measurement_contract(self):
        changed = self.biased.copy(deep=True)
        changed.loc[5, "omega_B_x_rad_s"] += .003
        result = assess(self.ideal, changed, self.iw, self.bw)
        self.assertFalse(result["checks"]["bias_applied_once_at_all_live_samples"])

    def test_validator_rejects_wrong_correction_direction_evidence(self):
        from copy import deepcopy
        changed = deepcopy(self.bw)
        for event in changed.events:
            if event["initialized_before"] and event["result"] == "updated":
                event["before"]["q_BN"], event["after"]["q_BN"] = event["after"]["q_BN"], event["before"]["q_BN"]
                break
        result = assess(self.ideal, self.biased, self.iw, changed)
        self.assertFalse(result["checks"]["every_update_reduces_its_measured_vector_residual"])


if __name__ == "__main__":
    unittest.main()
