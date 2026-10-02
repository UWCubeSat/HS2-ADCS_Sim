"""8B-2B deterministic acquisition predicate/boundaries only; no state response."""
from dataclasses import replace
import unittest

import numpy as np

from validate_acquisition_boundary import actual,fixture,independent,validate


class AcquisitionBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report=validate()

    def require(self,*names):
        for name in names:
            with self.subTest(check=name): self.assertTrue(self.report["checks"][name])

    def test_independent_predicate_matches_actual_adapter(self):
        self.require("independent_predicate_matches_all_actual_acquisitions","zero_bias_accepts")
        self.assertTrue(self.report["passed"])

    def test_original_rejection_reconstruction_in_T(self):
        self.require("original_bias_rejection_reconstructed")
        case=next(r for r in self.report["rows"] if r["label"]=="original_8B2A")
        np.testing.assert_array_equal(case["bias_S_T"],[1e-6,-2e-6,3e-6])
        self.assertAlmostEqual(case["independent"]["metric"],0.08692867113186659,places=14)
        self.assertTrue(case["sensor_valid"])
        self.assertFalse(case["actual"]["initialized"])

    def test_basis_is_right_handed_and_physically_aligned(self):
        self.require("geometric_basis_orthonormal_right_handed")
        g=self.report["geometry"]
        self.assertGreater(np.dot(g["in_plane_B"],g["Sun_unit_B"]),0)
        self.assertLess(abs(np.dot(g["cross_plane_B"],g["Sun_unit_B"])),1e-14)

    def test_parallel_magnitude_behavior_and_exceptions(self):
        self.require("parallel_magnitude_only_cases_accept","parallel_zero_and_polarity_reversal_are_separate_rejections")

    def test_both_signed_perpendicular_sensitivities(self):
        self.require("perpendicular_signed_cases_show_both_decisions")
        boundaries=self.report["boundaries"]
        for sign in ("positive","negative"):
            self.assertLess(boundaries["in_plane_"+sign]["smallest_tested_rejected"]["bias_magnitude_T"],
                boundaries["cross_plane_"+sign]["largest_tested_accepted"]["bias_magnitude_T"])

    def test_actual_brackets_straddle_fixed_tolerance(self):
        self.require("all_boundaries_bracket_unchanged_tolerance")
        for b in self.report["boundaries"].values():
            self.assertEqual(b["largest_tested_accepted"]["actual"]["decision"],"ACCEPT")
            self.assertEqual(b["smallest_tested_rejected"]["actual"]["decision"],"REJECT")
            self.assertGreater(b["width_T"],0)

    def test_scaled_original_direction_boundary_is_below_one(self):
        b=self.report["boundaries"]["original_direction"]
        self.assertGreater(b["scale_interval"][0],0)
        self.assertLess(b["scale_interval"][1],1)

    def test_endpoint_repeatability_and_numerical_resolution(self):
        self.require("all_endpoint_decisions_repeat_across_order_and_idle_history",
            "binary64_predicate_error_within_declared_arithmetic_band","endpoint_measurement_and_independent_calculation_repeat_exactly")

    def test_cosine_metric_and_B_angle_are_distinct(self):
        self.require("raw_metric_and_vector_angle_match_basis_geometry")
        a=self.report["boundaries"]["in_plane_positive"]["largest_tested_accepted"]
        b=self.report["boundaries"]["cross_plane_positive"]["largest_tested_accepted"]
        self.assertGreater(b["magnetic_direction_change_rad"],1000*a["magnetic_direction_change_rad"])

    def test_sensor_values_validity_and_runtime_preserved(self):
        self.require("all_finite_values_and_quiet_sensor_validity_preserved","runtime_models_gate_Q_R_P0_unchanged")

    def test_consistent_collinear_pair_hits_separate_geometry_gate(self):
        # Pure algebraic edge case, outside the fixed-WMM sweep: keep the Sun
        # input unchanged; make both magnetic vectors parallel to that Sun.
        f=fixture(); sun=next(e for e in f["pair"].deliveries if e.sensor=="sun")
        pair=replace(f["pair"],deliveries=[replace(e,measured=sun.measured,reference_n=sun.reference_n,frame="B",c_sb=None)
            if e.sensor=="magnetic" else e for e in f["pair"].deliveries])
        prediction=independent(sun.measured,sun.measured,sun.reference_n,sun.reference_n,
            f["policy"]["acquisition_pair_tolerance"]["value"],f["policy"]["minimum_acquisition_sine"]["value"])
        self.assertLess(prediction["metric"],1e-60)  # 70-digit normalization roundoff, not a gate adjustment.
        self.assertEqual(prediction["reason"],"acquisition_geometry")
        self.assertIn("acquisition_geometry",actual(f,pair)["rejected"])


if __name__=="__main__": unittest.main()
