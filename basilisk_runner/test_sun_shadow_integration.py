"""8C-1 actual live equivalence, fail-sensitive comparison and authority guards."""
from copy import deepcopy
from dataclasses import replace
import unittest

from validate_sun_sensor_model import compare, validate


class SunShadowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.baseline, cls.modeled = validate()

    def test_live_and_disabled_equivalence(self):
        for key in ("live_ideal_equivalence", "disabled_HEAD_equivalence", "disabled_sources_and_metadata_unchanged",
                    "cold_start_acquisition_unchanged", "production_preserved"):
            self.assertTrue(self.report["checks"][key], key)
        self.assertEqual(self.report["nominal"]["max_vector_difference"], 0.)
        self.assertTrue(all(value == 0. for value in self.report["nominal"]["estimator_max_abs_differences"].values()))

    def test_timing_invalid_sequence_and_source_identity(self):
        for key in ("epochs_and_provenance_preserved", "delayed_processing_not_reacquisition",
                    "source_identifies_direct_mode", "rejected_event_equivalence", "rejected_delivery_keeps_finite_sample"):
            self.assertTrue(self.report["checks"][key], key)

    def test_control_guards_and_all_validator_checks(self):
        self.assertTrue(self.report["passed"], self.report["checks"])

    def test_comparison_detects_changed_direction(self):
        changed = deepcopy(self.modeled)
        for batch in changed.attrs["shadow_input_trace"]:
            batch.deliveries = [replace(e, measured=[1., 0., 0.]) if e.sensor == "sun" else e for e in batch.deliveries]
        self.assertFalse(compare(self.baseline, changed)["passed"])

    def test_comparison_does_not_hide_unknown_provenance(self):
        changed = deepcopy(self.modeled)
        for batch in changed.attrs["shadow_input_trace"]:
            batch.deliveries = [replace(e, source="wrong provenance") if e.sensor == "sun" else e for e in batch.deliveries]
        self.assertFalse(compare(self.baseline, changed)["passed"])

    def test_comparison_detects_changed_epoch_and_missing_event(self):
        for remove in (False, True):
            changed = deepcopy(self.modeled)
            for batch in changed.attrs["shadow_input_trace"]:
                batch.deliveries = ([e for e in batch.deliveries if e.sensor != "sun"] if remove else
                    [replace(e, epoch_ns=e.epoch_ns+1) if e.sensor == "sun" else e for e in batch.deliveries])
            self.assertFalse(compare(self.baseline, changed)["passed"])


if __name__ == "__main__":
    unittest.main()
