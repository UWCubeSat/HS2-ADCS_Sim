"""7G-1 nominal live ownership/plant integration; no new fault scenarios."""
import contextlib
import io
import unittest

import numpy as np

from attitude_mekf_adapter import ShadowOptions
from disconnected_command_observer import ObserverOptions
from magnetic_control_cycle import diagnostic_cycle_config
from scenario_huskysat2_detumble import run
from validate_mekf_closed_loop import integration, physics, validate, vector


class NominalClosedLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.a, cls.b = validate()

    def test_explicit_bounded_nominal_selection_only(self):
        options = dict(stop_time_s=6., write_outputs=False, make_plots=False,
            cycle=diagnostic_cycle_config(), shadow=ShadowOptions(ideal_sun=True), control_source="MEKF_DEVELOPMENT")
        for change in ({"stop_time_s": None}, {"stop_time_s": 11.}, {"cycle": None}, {"actuator": "direct"},
                       {"shadow": None}, {"shadow": ShadowOptions()}, {"disconnected_commands": ObserverOptions()},
                       {"control_source": "unrecognized"}):
            with self.subTest(change=change), self.assertRaises(ValueError), contextlib.redirect_stdout(io.StringIO()):
                run(**(options | change))

    def test_exact_owner_gated_native_readback_and_live_provenance(self):
        chain = self.report["integration"]
        self.assertTrue(chain["passed"], chain["checks"])
        self.assertEqual(chain["component_replay"]["exact_comparisons"], 103)

    def test_first_command_following_interval_and_startup(self):
        chain = self.report["integration"]
        first = chain["first_command"]
        self.assertEqual(chain["first_valid_mekf_ns"], 400_000_000)
        self.assertEqual(first["sample_epoch_ns"], 400_000_000)
        self.assertEqual(first["computation_epoch_ns"], 500_000_000)
        self.assertEqual(first["publication_epoch_ns"], 600_000_000)
        self.assertAlmostEqual(self.report["B_mekf"]["first_native_torque_s"], .7)
        self.assertTrue(self.report["comparison"]["pre_first_application_identical"])

    def test_zero_every_prohibited_interval_and_boundary_torque(self):
        m = vector(self.b, "mcmd", "Am2")
        native = vector(self.b, "native_mtbNetTorque_B", "Nm")
        ticks = self.b.time_ns.to_numpy(dtype=np.int64)
        self.assertTrue(np.all(m[ticks % 1_000_000_000 < 600_000_000] == 0))
        # End-of-burst COIL_OFF output still records the preceding interval;
        # the next interval is zero. This is not a quiet-phase violation.
        self.assertTrue(np.any(native[10]))
        self.assertFalse(np.any(native[11]))
        self.assertEqual(self.report["B_mekf"]["zero_command_intervals"], 36)

    def test_independent_native_torque_rate_and_energy(self):
        for name in ("A_simple_nav", "B_mekf"):
            self.assertTrue(self.report[name]["passed"], self.report[name]["checks"])

    def test_validator_rejects_changed_independent_native_evidence(self):
        changed = self.b.copy(deep=True)
        changed.loc[7, "native_mtbNetTorque_B_x_Nm"] += 1e-6
        result = physics(changed)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["native_final_stage_torque"])

    def test_validator_rejects_changed_gate_to_native_evidence(self):
        changed = self.b.copy(deep=True)
        changed.loc[6, "mcmd_x_Am2"] = 0.
        self.assertFalse(integration(changed)["checks"]["native_input_equals_gated_output"])

    def test_validator_uses_independent_mekf_sample_message(self):
        changed = self.b.copy(deep=True)
        changed.loc[4, "mekf_nav_omega_B_x_rad_s"] += 1e-4
        self.assertFalse(physics(changed)["existing_checks"]["cycle_sample_state_and_field"]["passed"])

    def test_controlled_ab_and_committed_default_preservation(self):
        self.assertTrue(self.report["passed"], self.report["preservation"])
        self.assertTrue(self.report["preservation"]["passed"])
        for name in ("A_simple_nav", "B_mekf"):
            self.assertEqual(self.report[name]["completed_cycles"], 6)
            self.assertAlmostEqual(self.report[name]["active_time_s"], 2.4)
        self.assertGreater(self.report["comparison"]["max_rate_vector_difference_rad_s"], 0.)


if __name__ == "__main__":
    unittest.main()
