"""Phase 5 physical-profile and sensitivity-report regressions; no flight claims."""
import contextlib
from dataclasses import replace
from fractions import Fraction
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from hs2_sim_config import (DEFAULT_CONFIG, HS2SimConfig, get_regression_baseline_config,
                           get_hs2_candidate_config, get_profile_config, physical_profile_name)
import scenario_huskysat2_detumble as scenario
from compare_physical_profiles import (inertia_sanity, load_run, compare_runs,
                                      threshold_observation, attitude_separation_deg, main as compare_main)


class PhysicalProfileTests(unittest.TestCase):
    def test_baseline_identity_and_fingerprint_are_unchanged(self):
        baseline = get_regression_baseline_config()
        self.assertIs(baseline, DEFAULT_CONFIG)
        self.assertEqual(physical_profile_name(baseline), "regression_baseline")
        # Frozen hash from the committed Phase 4/static-cleanup snapshot, covering
        # every original value AND provenance field, not a self-comparison.
        self.assertEqual(baseline.fingerprint(), "99de374af2f512e27d34e40fdc3327356662ef48804f7ba7945b7a703a68ad8f")
        self.assertEqual(len(baseline.to_dict()), 8)
        self.assertEqual(baseline.spacecraft.mass.value, 2.6)
        self.assertEqual(baseline.spacecraft.dimensions.value, (0.1, 0.1, 0.2))

    def test_candidate_values_and_unresolved_provenance(self):
        sc = get_hs2_candidate_config().spacecraft
        self.assertEqual(sc.mass.value, 3.72911)
        self.assertEqual(sc.mass.status, "ASSUMED")
        self.assertIn("E43", sc.mass.source)
        self.assertIn("TBD", sc.mass.notes)
        self.assertEqual(sc.dimensions.value, (0.100, 0.100, 0.3405))
        self.assertEqual(sc.dimensions.status, "TBC")
        self.assertEqual(sc.dimensions.frame, "B xyz")
        for conflict in ("340 mm", "338.6 mm", "physical HS-2 axis mapping"):
            self.assertIn(conflict, sc.dimensions.notes)
        self.assertEqual(sc.com.value, (0, 0, 0))
        self.assertEqual(sc.com.status, "ASSUMED")
        self.assertIn("not a measured", sc.com.notes)
        self.assertEqual(sc.inertia.status, "ASSUMED")
        self.assertIn("Geometry-only", sc.inertia.notes)

    def test_candidate_inertia_independent_integral_and_rational_anchors(self):
        candidate = get_hs2_candidate_config()
        result = inertia_sanity(candidate)
        self.assertTrue(result["passed"], result)
        # Exact-rational evaluation of kg*(m^2)/12, separately calculated from
        # decimal mass/envelope values. The production profile uses float arithmetic.
        expected = [float(Fraction(187858018271, 4800000000000)),
                    float(Fraction(187858018271, 4800000000000)), float(Fraction(372911, 60000000))]
        np.testing.assert_allclose(candidate.spacecraft.inertia.value, np.diag(expected), atol=1e-17, rtol=0)
        self.assertLess(result["max_volume_integral_error_kg_m2"], 5e-17)
        self.assertEqual(candidate.spacecraft.inertia.units, "kg m^2")
        self.assertFalse(np.array_equal(candidate.spacecraft.inertia.value, DEFAULT_CONFIG.spacecraft.inertia.value))

    def test_candidate_changes_only_physical_section_and_never_defaults(self):
        candidate = get_hs2_candidate_config()
        self.assertEqual(replace(candidate, spacecraft=DEFAULT_CONFIG.spacecraft), DEFAULT_CONFIG)
        self.assertNotEqual(candidate.fingerprint(), DEFAULT_CONFIG.fingerprint())
        self.assertEqual(get_regression_baseline_config().spacecraft.mass.value, 2.6)
        self.assertEqual(physical_profile_name(HS2SimConfig.from_dict(candidate.to_dict())), "hs2_candidate")
        self.assertEqual(physical_profile_name(candidate.with_run_options(0.2)), "hs2_candidate")
        custom = replace(candidate, spacecraft=replace(candidate.spacecraft, mass=replace(candidate.spacecraft.mass, value=3.8)))
        self.assertEqual(physical_profile_name(custom), "custom")

    def test_explicit_factory_and_cli_selection(self):
        for name in ("regression_baseline", "hs2_candidate"):
            self.assertEqual(physical_profile_name(get_profile_config(name)), name)
            with patch.object(scenario, "run") as run:
                scenario.main(["--profile", name, "--no-plots"])
                self.assertEqual(run.call_args.kwargs["config"], get_profile_config(name))
        with patch.object(scenario, "run") as run:
            scenario.main(["--no-plots"])
            self.assertIs(run.call_args.kwargs["config"], DEFAULT_CONFIG)
        with self.assertRaises(ValueError):
            get_profile_config("typo")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            scenario.main(["--profile", "hs2_candidate", "--config", "unused.json"])

    def test_saved_profiles_validation_and_manifest_tampering(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(scenario, "OUT_DATA", Path(directory)), contextlib.redirect_stdout(io.StringIO()):
            root = Path(directory)
            for config in (DEFAULT_CONFIG, get_hs2_candidate_config()):
                df = scenario.run(2.0, make_plots=False, config=config)
                self.assertEqual(df.attrs["physical_profile"], physical_profile_name(config))
            baseline = root / "detumble_output.csv"
            candidate = root / "detumble_output_hs2_candidate.csv"
            _, loaded, manifest = load_run(candidate)
            self.assertEqual(loaded, get_hs2_candidate_config().with_run_options(2.0))
            self.assertEqual(manifest["configuration"]["spacecraft"]["mass"]["value"], 3.72911)
            self.assertEqual(manifest["configuration"]["spacecraft"]["dimensions"]["status"], "TBC")
            self.assertTrue(compare_runs(baseline, candidate)["passed"])
            report = root / "comparison.json"
            args = ["--baseline", str(baseline), "--candidate", str(candidate), "--report", str(report)]
            self.assertEqual(compare_main(args), 0)
            manifest["physical_profile"] = "regression_baseline"
            candidate.with_name(candidate.stem + "_run.json").write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "mismatch"):
                load_run(candidate)
            self.assertEqual(compare_main(args), 1)
            self.assertFalse(json.loads(report.read_text())["passed"])

    def test_sensitivity_comparison_rejects_nonphysical_changes(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(scenario, "OUT_DATA", Path(directory)), contextlib.redirect_stdout(io.StringIO()):
            root = Path(directory)
            scenario.run(0.2, make_plots=False)
            candidate = get_hs2_candidate_config()
            candidate = replace(candidate, initial=replace(candidate.initial,
                body_rate=replace(candidate.initial.body_rate, value=(0.1, 0.1, 0.1))))
            scenario.run(0.2, make_plots=False, config=candidate)
            with self.assertRaisesRegex(ValueError, "only in spacecraft"):
                compare_runs(root / "detumble_output.csv", root / "detumble_output_hs2_candidate.csv")

    def test_sampled_crossing_does_not_treat_transient_as_sustained(self):
        times = np.array([0, 1, 2, 3, 4], dtype=float)
        result = threshold_observation(times, np.array([2, 0.4, 0.8, 0.3, 0.2]), 0.5)
        self.assertEqual(result["first_below_s"], 1)
        self.assertEqual(result["below_through_end_from_s"], 3)
        self.assertEqual(result["remaining_observed_window_s"], 1)
        self.assertIsNone(threshold_observation(times, np.ones(5), 0.5)["first_below_s"])

    def test_attitude_difference_respects_shadow_sets_and_known_rotation(self):
        sigma = np.array([[0.2, -0.1, 0.3]])
        shadow = -sigma / np.sum(sigma * sigma)
        np.testing.assert_allclose(attitude_separation_deg(sigma, shadow), [0], atol=1e-13, rtol=0)
        np.testing.assert_allclose(attitude_separation_deg(np.zeros((1, 3)), np.array([[0, 0, np.tan(np.pi / 8)]])),
                                   [90], atol=1e-13, rtol=0)


if __name__ == "__main__":
    unittest.main()
