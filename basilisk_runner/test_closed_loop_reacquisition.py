"""7G-2B one live lifecycle; existing gate/estimator unit policies are reused."""
import unittest

from validate_closed_loop_reacquisition import assess, validate


class ClosedLoopReacquisitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.frame, cls.fixture = validate()
        cls.evidence = cls.report["evidence"]

    def require_checks(self, *names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.evidence["checks"][name])

    def test_fault_reset_and_reacquiring_all_keep_native_zero(self):
        self.require_checks("real_pre_fault_actuation", "fault_removes_actual_command",
            "reset_changes_generation_and_keeps_zero", "reacquiring_is_not_valid_absolute_navigation",
            "zero_persists_until_fresh_authority")
        self.assertEqual(self.evidence["zero_publications_before_resume"], 78)
        self.assertEqual(self.evidence["reacquiring_boundaries"], 64)

    def test_reacquisition_alone_does_not_authorize_even_new_same_epoch_command(self):
        self.require_checks("exact_reacquisition_and_generation", "reacquisition_alone_not_authority")

    def test_unchanged_pre_reset_command_replay_rejected_by_provenance(self):
        self.require_checks("old_envelope_rejected_by_provenance_before_deadline_check")
        self.assertEqual(self.evidence["old_generation"], (0, 1, 0))
        decision = self.evidence["replay_decision"]
        self.assertTrue(decision["current_health"]["healthy"])
        self.assertTrue(decision["mathematical_command_valid"])
        self.assertFalse(decision["command_usable"])

    def test_fresh_chain_new_generation_reaches_native_actuator(self):
        self.require_checks("fresh_snapshot_exact_epoch_and_generation", "resumed_command_is_only_new_generation",
                            "native_input_equals_gate_owner_and_message", "fresh_inputs_match_independent_native_records")
        self.assertEqual(self.evidence["new_generation"], (1, 2, 1))

    def test_resumed_native_torque_and_continuous_plant(self):
        self.require_checks("native_torque_off_then_resumes_next_interval", "native_torque_matches_independent_physics",
                            "finite_continuous_plant_through_reset_and_resume", "energy_work_balance")

    def test_no_fallback_cycle_compatibility_and_preservation(self):
        self.require_checks("complete_live_evidence", "one_fault_reset_and_replay_only", "single_owner_and_fresh_publications",
                            "no_automatic_fallback", "quiet_zero_and_TAM_not_corrupted", "unchanged_stage_order")
        self.assertTrue(self.report["production_preservation"]["passed"])
        self.assertTrue(self.report["passed"], self.evidence["checks"])

    def test_validator_rejects_early_command_after_reacquisition(self):
        changed = self.frame.copy(deep=True)
        changed.loc[76, "mcmd_x_Am2"] = .1
        result = assess(changed, self.fixture)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["zero_persists_until_fresh_authority"])
        self.assertFalse(result["checks"]["native_input_equals_gate_owner_and_message"])

    def test_validator_rejects_false_resumed_native_torque(self):
        changed = self.frame.copy(deep=True)
        changed.loc[87, "native_mtbNetTorque_B_x_Nm"] += 1e-6
        result = assess(changed, self.fixture)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["native_torque_matches_independent_physics"])


if __name__ == "__main__":
    unittest.main()
