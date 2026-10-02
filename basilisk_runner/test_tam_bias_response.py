"""8B-2A delivered bias, acquisition boundary and actuator exclusion witnesses."""
from dataclasses import replace
from copy import deepcopy
import unittest

from attitude_mekf_adapter import ShadowOptions
from gyro_sensor_model import profile_config as gyro_profile
from tam_sensor_model import profile_config
from validate_tam_bias_response import ENABLE_NS, composition, response, validate


class TAMBiasResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report,cls.ideal,cls.biased,cls.iw,cls.bw,cls.post,cls.pw=validate()

    def require(self,*names):
        for name in names:
            with self.subTest(check=name):
                self.assertTrue(self.report["checks"][name])

    def test_actual_deliveries_apply_exact_bias_in_T_S_and_B_once(self):
        self.require("unchanged_TEST_ONLY_bias_profile","exact_once_bias_in_delivered_sensor_and_body_vectors",
            "unchanged_event_frames_epochs_and_input_validity","field_acquisition_publication_and_provenance",
            "gyro_and_Sun_paths_exact")

    def test_cold_start_cannot_be_misreported_as_healthy_response(self):
        self.require("cold_start_rejection_explained_by_independent_pair_geometry")
        self.assertEqual(self.report["engineering_gate"],"PASS")
        self.assertTrue(self.report["passed"])
        self.assertFalse(self.report["cold_start_bias_acquisition_supported"])
        self.assertTrue(all(self.report["checks"].values()))
        self.assertFalse(self.report["final_status"]["bias"]["initialized"])
        self.assertFalse(any(e["result"] == "updated" for e in self.bw.events))
        self.assertIsNone(self.report["summaries"]["bias"]["final_attitude_error_rad"])
        self.assertIsNone(self.report["summaries"]["bias"]["covariance_min_eigenvalue"])

    def test_post_acquisition_boundary_and_exact_composition(self):
        self.require("post_acquisition_composition_hard_gate","normal_ideal_acquisition_before_explicit_bias_boundary",
            "initial_ideal_then_only_two_subsequent_biased_acquisitions","all_estimator_states_exact_before_first_biased_delivery")
        self.assertEqual(self.report["transition"]["biased_processing_epochs_ns"],[1_600_000_000,2_600_000_000])

    def test_actual_magnetic_corrections_and_independent_geometry(self):
        self.require("actual_magnetic_and_Sun_innovations_match_vector_geometry","actual_updates_reduce_their_measured_vector_residual",
            "known_bias_innovation_sign_and_first_correction")

    def test_ideal_Sun_interaction(self):
        self.require("ideal_Sun_observes_and_corrects_magnetic_disagreement","initialized_response_valid_without_rejected_events")

    def test_covariance_and_coupled_bias_state_health(self):
        self.require("covariance_finite_symmetric_positive_definite","covariance_within_unchanged_zero_Q_bound",
            "estimated_gyro_bias_changes_only_in_observed_vector_updates")

    def test_reversed_correction_witness_cannot_pass(self):
        changed=deepcopy(self.pw)
        e=next(e for e in changed.events if e["result"] == "updated" and e["sensor"] == "magnetic")
        e["before"]["q_BN"],e["after"]["q_BN"]=e["after"]["q_BN"],e["before"]["q_BN"]
        result=response(self.post,changed,self.ideal,self.report["estimator_policy"])
        self.assertFalse(result["checks"]["actual_updates_reduce_their_measured_vector_residual"])
        self.assertFalse(result["checks"]["known_bias_innovation_sign_and_first_correction"])

    def test_negative_covariance_witness_cannot_pass(self):
        changed=deepcopy(self.pw)
        e=next(e for e in changed.events if e["result"] == "updated")
        e["after"]["P"][0,0]=-1.
        result=response(self.post,changed,self.ideal,self.report["estimator_policy"])
        self.assertFalse(result["checks"]["covariance_finite_symmetric_positive_definite"])

    def test_invalid_values_preserved_separately_from_validity(self):
        self.require("rejected_finite_biased_vector_preserved")

    def test_native_control_and_committed_ideal_preserved(self):
        self.require("ideal_reference_matches_committed_8B1","native_control_spacecraft_and_actuators_byte_identical",
            "no_development_command_owner","biased_TAM_actuator_selection_rejected",
            "physical_models_controller_cycle_and_Q_R_P0_unchanged")

    def test_only_explicit_unchanged_bias_fixture_can_opt_in(self):
        cfg=profile_config("TEST_BIAS_ONLY")
        ShadowOptions(ideal_sun=True,tam_model=cfg,tam_bias_test_only=True).validate(100_000_000)
        invalid=[ShadowOptions(ideal_sun=True,tam_model=cfg),
            ShadowOptions(ideal_sun=True,tam_bias_test_only=True),
            ShadowOptions(ideal_sun=True,tam_model=replace(cfg,profile="IDEAL_REGRESSION"),tam_bias_test_only=True),
            ShadowOptions(ideal_sun=True,tam_model=cfg,tam_bias_test_only=True,gyro_model=gyro_profile("TEST_BIAS_ONLY")),
            ShadowOptions(ideal_sun=True,tam_model=cfg,tam_bias_test_only=True,initial_q_BN=(1.,0.,0.,0.)),
            ShadowOptions(ideal_sun=False,tam_model=cfg,tam_bias_test_only=True),
            ShadowOptions(ideal_sun=True,tam_model=cfg,tam_bias_enable_ns=ENABLE_NS),
            ShadowOptions(ideal_sun=True,tam_model=cfg,tam_bias_test_only=True,tam_bias_enable_ns=ENABLE_NS+1)]
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ValueError):
                options.validate(100_000_000)

    def test_noise_and_scale_are_not_authorized_by_bias_opt_in(self):
        for profile in ("TEST_NOISE_ONLY","TEST_SCALE_ONLY"):
            with self.assertRaises(ValueError):
                ShadowOptions(ideal_sun=True,tam_model=profile_config(profile),tam_bias_test_only=True).validate(100_000_000)

    def test_composition_gate_detects_wrong_units_and_sign(self):
        for factor in (1e6,-1.):
            changed=self.biased.copy(deep=True)
            batch=next(b for b in changed.attrs["shadow_input_trace"] if b.deliveries)
            batch.deliveries=[replace(e,measured=[factor*x for x in e.measured]) if e.sensor == "magnetic" else e
                for e in batch.deliveries]
            self.assertFalse(composition(self.ideal,changed)["passed"])
        changed=self.post.copy(deep=True)
        for batch in changed.attrs["shadow_input_trace"]:
            batch.deliveries=[replace(e,measured=[v+d for v,d in zip(e.measured,(1e-6,-2e-6,3e-6))])
                if e.sensor == "magnetic" and e.epoch_ns == ENABLE_NS else e for e in batch.deliveries]
        self.assertFalse(composition(self.ideal,changed,ENABLE_NS)["passed"])


if __name__ == "__main__":
    unittest.main()
