"""7F-2C focused live integration tests; no duplicated controller/estimator math."""
import ast
from dataclasses import replace
import inspect
import unittest

import disconnected_command_observer as observer
from command_health_gate import CommandHealthGate
from magnetic_control_cycle import diagnostic_cycle_config
from validate_disconnected_command_chain import validate


class LiveCommandChainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = validate()

    def test_nominal_live_schedule_and_exact_sample_provenance(self):
        case = self.report["cases"]["nominal"]
        self.assertTrue(case["passed"], case["checks"])

    def test_live_invalid_views_rejected(self):
        for name in ("previous_nav", "stale_wrong_cycle_tam", "invalid_tam", "late_quality"):
            with self.subTest(name=name):
                self.assertTrue(self.report["cases"][name]["passed"], self.report["cases"][name]["checks"])

    def test_fault_after_compute_and_fault_during_burst_inhibit(self):
        for name in ("fault_before_application", "mid_burst_fault"):
            self.assertTrue(self.report["cases"][name]["passed"], self.report["cases"][name]["checks"])

    def test_live_reset_reacquisition_requires_fresh_generation(self):
        for name in ("fault_reset_reacquire", "reset_after_compute"):
            self.assertTrue(self.report["cases"][name]["passed"], self.report["cases"][name]["checks"])

    def test_live_component_equivalence_exact_for_all_cases(self):
        for name, case in self.report["cases"].items():
            with self.subTest(name=name):
                self.assertTrue(case["component_equivalence"]["passed"])
                self.assertGreater(case["component_equivalence"]["exact_record_comparisons"]["application_decisions"], 0)

    def test_production_preservation_disabled_and_enabled(self):
        self.assertTrue(self.report["preservation"]["passed"], self.report["preservation"])
        self.assertTrue(all(case["checks"]["production_bytes_exact"] for case in self.report["cases"].values()))

    def test_structural_actuator_isolation(self):
        tree = ast.parse(inspect.getsource(observer))
        attrs = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertTrue({"write", "addDynamicEffector", "MTBCmdMsg", "CmdTorqueBodyMsg", "mtbCmdOutMsg",
                         "cmdTorqueOutMsg", "torqueExternalPntB_B"}.isdisjoint(attrs))
        self.assertTrue(self.report["preservation"]["actuator_and_sensor_wiring_AST_identical"])
        self.assertTrue(all(not row["actuator_authority"] for case in self.report["cases"].values() for row in case["telemetry"]))

    def test_application_extension_rejects_wrong_cycle_without_relabeled_epochs(self):
        # One focused boundary contract check supplements the live cases. It does
        # not retest math/fault policies already verified in 7F-1/2A/2B.
        from test_command_health_gate import calculation, current, ready_gate, MS
        snapshot, command = calculation()
        gate = ready_gate()
        self.assertTrue(gate.evaluate(snapshot, command, *current(500), 500*MS).command_usable)
        cycle = diagnostic_cycle_config()
        self.assertTrue(gate.evaluate(snapshot, command, *current(600), 600*MS, application_cycle=cycle).command_usable)
        bad = replace(cycle, sample_to_compute=replace(cycle.sample_to_compute, value=0.2))
        self.assertEqual(gate.evaluate(snapshot, command, *current(600), 600*MS,
                                      application_cycle=bad).inhibition_reason, "application_cycle_mismatch")
        self.assertEqual(gate.evaluate(snapshot, command, *current(1000), 1000*MS,
                                      application_cycle=cycle).inhibition_reason, "outside_application_window")


if __name__ == "__main__":
    unittest.main()
