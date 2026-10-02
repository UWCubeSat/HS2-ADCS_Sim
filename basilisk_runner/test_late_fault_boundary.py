"""7G-2C focused decision/publication boundary checks; no fault campaign."""
import unittest

from validate_late_fault_boundary import assess, validate


class LateFaultBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.frame, cls.fixture = validate()
        cls.evidence = cls.report["evidence"]

    def require_checks(self, *names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.evidence["checks"][name])

    def test_fault_is_after_real_approval_before_owner(self):
        self.require_checks("actual_Basilisk_order", "gate_then_fault_then_owner_at_same_epoch",
            "real_current_health_approved_before_fault", "supported_fault_preserves_original_quality_epoch")

    def test_first_native_command_after_fault_is_earlier_approved_envelope(self):
        self.require_checks("already_approved_envelope_survives_exactly_one_publication",
            "native_input_independently_matches_owner_and_CSV")
        self.assertEqual(self.evidence["surviving_publications_after_fault"], 1)

    def test_first_zero_boundary_and_no_further_stale_command(self):
        self.require_checks("first_eligible_boundary_and_all_later_commands_zero", "no_approval_after_fault")
        self.assertEqual(self.evidence["epochs_ns"]["fault_to_zero_command"], 100_000_000)
        self.assertEqual(self.evidence["subsequent_zero_publications"], 22)

    def test_persistent_source_fault_and_no_fallback(self):
        self.require_checks("persistent_exposed_source_fault", "one_owner_no_automatic_fallback",
            "quiet_and_later_bursts_remain_zero")

    def test_last_torque_interval_and_spacecraft_continuity(self):
        self.require_checks("held_inputs_and_epochs_prove_following_interval",
            "one_surviving_torque_interval_then_exact_zero", "native_torque_matches_independent_physics",
            "finite_continuous_plant", "energy_work_accounts_for_last_approved_interval")

    def test_production_isolation_and_complete_evidence(self):
        self.require_checks("complete_evidence")
        self.assertTrue(self.report["production_preservation"]["passed"])
        self.assertTrue(self.report["passed"], self.evidence["checks"])

    def test_validator_rejects_additional_retained_command(self):
        changed = self.frame.copy(deep=True)
        changed.loc[9, "mcmd_x_Am2"] = .1
        evidence = assess(changed, self.fixture)
        self.assertFalse(evidence["passed"])
        self.assertFalse(evidence["checks"]["first_eligible_boundary_and_all_later_commands_zero"])
        self.assertFalse(evidence["checks"]["native_input_independently_matches_owner_and_CSV"])

    def test_validator_rejects_unaccounted_native_torque(self):
        changed = self.frame.copy(deep=True)
        changed.loc[10, "native_mtbNetTorque_B_x_Nm"] = 1e-6
        evidence = assess(changed, self.fixture)
        self.assertFalse(evidence["passed"])
        self.assertFalse(evidence["checks"]["native_torque_matches_independent_physics"])
        self.assertFalse(evidence["checks"]["one_surviving_torque_interval_then_exact_zero"])

    def test_validator_rejects_reversed_fault_publication_order(self):
        # Mutate only the witness; retain the physical run and original fixture.
        from copy import deepcopy
        changed = deepcopy(self.fixture)
        changed.after_fault["sequence"] = changed.publications[8]["sequence"]+1
        evidence = assess(self.frame, changed)
        self.assertFalse(evidence["passed"])
        self.assertFalse(evidence["checks"]["gate_then_fault_then_owner_at_same_epoch"])


if __name__ == "__main__":
    unittest.main()
