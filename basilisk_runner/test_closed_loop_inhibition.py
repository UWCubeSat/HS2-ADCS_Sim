"""7G-2A focused actuator-boundary integration, not new 7F fault policy tests."""
import unittest

from attitude_mekf_adapter import ShadowOptions
from disconnected_command_observer import ObserverOptions
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_closed_loop_inhibition import assess, validate


class ClosedLoopInhibitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.frame, cls.nominal, cls.events = validate()
        cls.evidence = cls.report["evidence"]

    def require_checks(self, *names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.evidence["checks"][name])

    def test_real_nonzero_command_applied_before_fault(self):
        self.require_checks("pre_fault_real_nonzero_actuation", "identical_nominal_configuration_and_pre_fault_state")
        self.assertAlmostEqual(self.evidence["pre_fault_actuation_s"], .2)
        before = self.evidence["pre_fault"]["owner"]
        self.assertEqual(before["sample_epoch_ns"], 400_000_000)
        self.assertEqual(before["computation_epoch_ns"], 500_000_000)

    def test_live_fault_and_first_eligible_native_zero(self):
        self.require_checks("live_fault_seen_at_first_boundary", "one_existing_fault_no_reset",
                            "existing_numeric_command_revoked_not_erased")
        epochs = self.evidence["epochs"]
        for key in ("fault_event_ns", "first_health_detection_ns", "first_zero_publication_ns",
                    "first_zero_native_input_ns", "first_zero_interval_start_ns"):
            self.assertEqual(epochs[key], 800_000_000)
        self.assertEqual(epochs["first_zero_native_torque_record_ns"], 900_000_000)

    def test_persistent_zero_no_old_command_retention(self):
        self.require_checks("owner_and_native_zero_on_every_faulted_boundary", "persistent_fault_all_bursts_inhibited",
            "fresh_zero_publication_and_single_owner", "independent_message_record_equals_native_readback_and_gate")
        self.assertEqual(self.evidence["faulted_publications"], 23)
        self.assertEqual(self.evidence["faulted_actuation_boundaries"], 10)

    def test_zero_torque_and_continuous_rigid_body_response(self):
        self.require_checks("held_input_confirms_next_interval", "native_torque_zero_for_all_inhibited_intervals",
            "finite_truth", "continuous_independent_rigid_body_response", "independent_native_torque",
            "energy_work_balance", "continues_rotating_without_new_magnetic_torque")

    def test_cycle_acquisition_and_quiet_bookkeeping_survive(self):
        self.require_checks("quiet_sample_and_faulted_actuation_zero", "valid_TAM_during_persistent_fault",
            "quiet_history_from_actual_zero_command", "no_false_coherent_snapshot_after_failed_capture",
            "unchanged_live_stage_order")

    def test_no_fallback_and_default_preservation(self):
        self.require_checks("no_fallback_while_simplenav_still_available")
        self.assertTrue(self.report["production_preservation"]["passed"])
        self.assertTrue(self.report["passed"], self.evidence["checks"])

    def test_fault_fixture_is_explicit_python_only_mekf_opt_in(self):
        with self.assertRaisesRegex(ValueError, "MEKF_DEVELOPMENT ownership"):
            run(mekf_fault_test=ObserverOptions(lambda *args: None))
        options = dict(stop_time_s=3., write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), shadow=ShadowOptions(ideal_sun=True), control_source="MEKF_DEVELOPMENT")
        for bad in (ObserverOptions(), object()):
            with self.assertRaisesRegex(ValueError, "test-only fault callback"):
                run(**options, mekf_fault_test=bad)

    def test_validator_rejects_nonzero_native_torque_while_inhibited(self):
        changed = self.frame.copy(deep=True)
        changed.loc[9, "native_mtbNetTorque_B_x_Nm"] = 1e-6
        result = assess(changed, self.nominal, self.events)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["native_torque_zero_for_all_inhibited_intervals"])
        self.assertFalse(result["checks"]["independent_native_torque"])

    def test_validator_rejects_retained_nonzero_command_record(self):
        changed = self.frame.copy(deep=True)
        changed.loc[8, "mcmd_x_Am2"] = -.2
        result = assess(changed, self.nominal, self.events)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["owner_and_native_zero_on_every_faulted_boundary"])
        self.assertFalse(result["checks"]["independent_message_record_equals_native_readback_and_gate"])


if __name__ == "__main__":
    unittest.main()
