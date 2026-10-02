"""8B-1 ideal live value/validity equivalence and no actuator authority."""
import unittest

from validate_tam_sensor_model import compare, validate


class TAMShadowIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report,cls.native,cls.modeled = validate()

    def require(self,*names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.report["checks"][name])

    def test_live_measurements_and_estimator_equivalence(self):
        self.require("live_ideal_equivalence","representative_WMM_history_equivalence")
        self.assertEqual(self.report["native_WMM_history"]["samples"],31)
        self.assertEqual(len(self.report["model_samples"]),3)

    def test_epochs_transport_and_configuration_provenance(self):
        self.require("all_acquisition_field_state_publication_epochs_equal",
            "delayed_processing_keeps_acquisition_epoch","ideal_model_no_rng_and_matching_provenance")

    def test_rejected_metadata_does_not_destroy_finite_field(self):
        self.require("rejected_sequence_equivalence","ineligible_finite_measurement_preserved","shadow_fault_does_not_change_actuation")

    def test_committed_default_and_authority_guards(self):
        self.require("disabled_host_matches_HEAD_adapter","disabled_estimator_inputs_states_exact","no_development_owner",
            "perturbed_TAM_live_selection_rejected","all_TAM_closed_loop_selection_rejected")
        self.assertTrue(self.report["passed"])

    def test_validator_rejects_wrong_units_in_delivered_vector(self):
        changed = self.modeled.copy(deep=True)
        event = next(e for b in changed.attrs["shadow_input_trace"] for e in b.deliveries if e.sensor == "magnetic")
        from dataclasses import replace
        for batch in changed.attrs["shadow_input_trace"]:
            batch.deliveries = [replace(e,measured=[float(x)*1e6 for x in e.measured]) if e.event_id == event.event_id else e
                for e in batch.deliveries]
        self.assertFalse(compare(self.native,changed)["checks"]["magnetic_vectors_equal_to_transform_roundoff"])


if __name__ == "__main__":
    unittest.main()
