"""Phase 4 input/provenance regressions; synthetic overrides never change defaults."""
import contextlib
from dataclasses import FrozenInstanceError, asdict, fields, replace
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from hs2_sim_config import DEFAULT_CONFIG, HS2SimConfig, Parameter, STATUSES
from basilisk_adcs_adapter import ADCSConfig
import scenario_huskysat2_detumble as scenario
import compare_reference_vs_basilisk as comparison


def changed(config, section, name, value):
    part = getattr(config, section)
    parameter = replace(getattr(part, name), value=value, source="Synthetic configuration regression",
                        revision="Phase 4 test", notes="Test input, not an HS-2 engineering selection.")
    return replace(config, **{section: replace(part, **{name: parameter})})


class HS2ConfigurationTests(unittest.TestCase):
    def test_default_preserves_exact_phase3_inputs(self):
        c = DEFAULT_CONFIG
        c.validate()
        self.assertEqual(c.spacecraft.mass.value, 2.6)
        self.assertEqual(c.spacecraft.dimensions.value, (0.1, 0.1, 0.2))
        # Independent frozen Phase 3 expressions; exact equality, not allclose.
        ixy = (2.6/12.0) * (0.1**2 + 0.2**2)
        iz = (2.6/12.0) * (0.1**2 + 0.1**2)
        self.assertEqual(c.spacecraft.inertia.value, ((ixy, 0, 0), (0, ixy, 0), (0, 0, iz)))
        self.assertEqual(c.spacecraft.com.value, (0, 0, 0))
        self.assertEqual((c.orbit.altitude.value, c.orbit.inclination.value, c.orbit.eccentricity.value), (600e3, 56.0, 0))
        self.assertEqual(c.environment.mu_earth.value, 3.986004418e14)
        self.assertEqual(c.environment.orbit_reference_radius.value, 6.371e6)
        self.assertEqual(c.environment.wmm_reference_radius.value, 6371.2e3)
        self.assertEqual(c.environment.epoch_utc.value, "2026-01-01T00:00:00+00:00")
        self.assertEqual(c.environment.epoch_fractional_year.value, 2026.0)
        self.assertEqual(c.environment.magnetic_model.value, "WMM2025")
        self.assertEqual(c.environment.coefficient_filename.value, "WMM2025.COF")
        self.assertEqual(c.environment.earth_orientation_model.value, "IAU_EARTH_pck00011_low_order")
        self.assertEqual(c.environment.gravity_model.value, "central_earth")
        self.assertEqual(c.environment.disturbances.value, "none")
        for name in ("dynamics_step", "environment_step", "sensor_step", "control_step"):
            self.assertEqual(getattr(c.timing, name).value, 0.1)
        self.assertEqual(c.timing.record_step.value, 1.0)
        self.assertEqual(c.timing.duration_orbits.value, 1.0)
        self.assertIsNone(c.timing.duration_override.value)
        self.assertEqual(c.duration_s, 2*math.pi*math.sqrt((6.371e6+600e3)**3/3.986004418e14))
        r, v = c.initial_orbit_state()
        speed = math.sqrt(3.986004418e14/6971000.0)
        self.assertEqual(r, [6971000.0, 0, 0])
        self.assertEqual(v, [0, speed*math.cos(math.radians(56)), speed*math.sin(math.radians(56))])
        self.assertEqual(c.initial.body_rate.value, (0.8, -0.2, 0.3))
        self.assertEqual(c.initial.sigma_BN.value, (0, 0, 0))
        self.assertEqual(c.magnetorquers.count.value, 3)
        np.testing.assert_array_equal(c.magnetorquers.axes_B.value, np.eye(3))
        self.assertEqual(c.magnetorquers.rod_voltage_limits.value, (5, 5))
        self.assertEqual(c.magnetorquers.rod_dipole_gains.value, (2.3, 2.3))
        self.assertEqual(c.magnetorquers.aircoil_power_limit.value, 1.75)
        self.assertEqual(c.magnetorquers.implementation.value, "native")
        self.assertEqual(asdict(ADCSConfig.from_sim_config(c)), {
            "dipoleCommandGain": 67200.0,
            "mtqDipoleGain_Am2_A": (2.3, 2.3, 0.85/math.sqrt(1.75/4.4)),
            "mtqResistance_Ohm": (51.0, 51.0, 4.4),
            "mtqCurrentLimit_A": (5.0/51.0, 5.0/51.0, math.sqrt(1.75/4.4)),
            "mtqDipoleLimit_Am2": (0.2, 0.2, 0.85),
            "minMagField_T": 1e-12, "use_cpp_core_if_available": True,
        })
        self.assertEqual(ADCSConfig(), ADCSConfig.from_sim_config(c))
        np.testing.assert_array_equal(c.sensors.magnetometer_dcm_SB.value, np.eye(3))
        self.assertEqual(c.sensors.magnetometer_scale.value, 1)
        self.assertEqual(c.sensors.magnetometer_bias.value, (0, 0, 0))
        self.assertEqual(c.sensors.magnetometer_noise_std.value, (0, 0, 0))
        self.assertEqual(c.sensors.magnetometer_min_output.value, -1e200)
        self.assertEqual(c.sensors.magnetometer_max_output.value, 1e200)
        self.assertEqual(c.sensors.navigation_model.value, "SimpleNav_truth")
        np.testing.assert_array_equal(c.sensors.navigation_noise_matrix.value, np.zeros((18, 18)))
        np.testing.assert_array_equal(c.sensors.navigation_walk_bounds.value, np.zeros(18))

    def test_immutable_and_json_round_trip(self):
        with self.assertRaises(FrozenInstanceError):
            DEFAULT_CONFIG.spacecraft.mass.value = 3
        with self.assertRaises(TypeError):
            DEFAULT_CONFIG.magnetorquers.axes_B.value[0][0] = 2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"config.json"
            path.write_text(json.dumps(DEFAULT_CONFIG.to_dict()), encoding="utf-8")
            loaded = HS2SimConfig.load(path)
            self.assertEqual(loaded, DEFAULT_CONFIG)
            self.assertEqual(loaded.fingerprint(), DEFAULT_CONFIG.fingerprint())
        bad = DEFAULT_CONFIG.to_dict()
        del bad["spacecraft"]["mass"]["source"]
        with self.assertRaises(ValueError):
            HS2SimConfig.from_dict(bad)

    def test_status_and_provenance_are_mandatory(self):
        for section in fields(DEFAULT_CONFIG):
            for item in fields(getattr(DEFAULT_CONFIG, section.name)):
                p = getattr(getattr(DEFAULT_CONFIG, section.name), item.name)
                self.assertIsInstance(p, Parameter)
                self.assertIn(p.status, STATUSES)
                self.assertTrue(all((p.units, p.source, p.revision, p.frame)))
        for section, names in (("spacecraft", ("mass", "dimensions", "inertia", "com")),
                               ("magnetorquers", ("dipole_limits", "axes_B", "resistance")),
                               ("orbit", ("altitude", "inclination", "eccentricity"))):
            for name in names:
                self.assertEqual(getattr(getattr(DEFAULT_CONFIG, section), name).status, "ASSUMED")
        parameter = DEFAULT_CONFIG.spacecraft.mass
        for kwargs in ({"status": "VERIFIED"}, {"source": ""}, {"revision": ""}, {"units": ""},
                       {"frame": ""}, {"notes": []}, {"status": ["ASSUMED"]}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                replace(parameter, **kwargs)
        for status in STATUSES:
            self.assertEqual(replace(parameter, status=status).status, status)
        for kwargs in ({"mass": 2.6}, {"mass": replace(parameter, units="g")}):
            with self.assertRaises(ValueError):
                replace(DEFAULT_CONFIG, spacecraft=replace(DEFAULT_CONFIG.spacecraft, **kwargs))

    def test_nonfinite_numbers_are_rejected_at_any_depth(self):
        for value in (float("nan"), float("inf"), -float("inf"), ((0.0, float("nan")),)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replace(DEFAULT_CONFIG.spacecraft.mass, value=value)

    def test_mass_dimensions_and_inertia_validation(self):
        for value in (0, -1):
            with self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "spacecraft", "mass", value)
        for value in ((0.1, 0, 0.2), (0.1, 0.2)):
            with self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "spacecraft", "dimensions", value)
        for tensor in (((1, 1, 0), (0, 1, 0), (0, 0, 1)),
                       ((1, 0, 0), (0, 0, 0), (0, 0, 1)),
                       ((1, 2, 0), (2, 1, 0), (0, 0, 1)), ((1, 0), (0, 1))):
            with self.subTest(tensor=tensor), self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "spacecraft", "inertia", tensor)
        c = changed(DEFAULT_CONFIG, "spacecraft", "inertia", ((0.02, 0.002, 0), (0.002, 0.025, 0), (0, 0, 0.03)))
        c.validate()

    def test_actuator_axis_and_count_validation(self):
        for matrix in (((1, 0), (0, 1), (0, 0)), ((0, 0, 0), (0, 1, 0), (0, 0, 1)),
                       ((2, 0, 0), (0, 1, 0), (0, 0, 1))):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "magnetorquers", "axes_B", matrix)
        for count in (0, 2, 4, True):
            with self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "magnetorquers", "count", count)
        with self.assertRaisesRegex(ValueError, "allocation"):
            changed(DEFAULT_CONFIG, "magnetorquers", "axes_B", ((0, 1, 0), (1, 0, 0), (0, 0, 1)))

    def test_dipole_electrical_validation_and_derived_limits(self):
        for limits in ((0, 0.2, 0.85), (-0.2, 0.2, 0.85), (0.2, 0.2)):
            with self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "magnetorquers", "dipole_limits", limits)
        for name, value in (("resistance", (0, 51, 4.4)), ("aircoil_power_limit", -1),
                            ("rod_voltage_limits", (0, 5)), ("rod_dipole_gains", (0, 2.3))):
            with self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, "magnetorquers", name, value)
        c = changed(DEFAULT_CONFIG, "magnetorquers", "rod_voltage_limits", (4, 3))
        payload = ADCSConfig.from_sim_config(c)
        self.assertEqual(payload.mtqCurrentLimit_A[:2], (4/51, 3/51))

    def test_timing_and_duration_validation(self):
        for name in ("dynamics_step", "environment_step", "sensor_step", "control_step", "record_step"):
            for value in (0, -0.1, 1e-10):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    changed(DEFAULT_CONFIG, "timing", name, value)
        with self.assertRaisesRegex(ValueError, "equal"):
            changed(DEFAULT_CONFIG, "timing", "sensor_step", 0.2)
        with self.assertRaisesRegex(ValueError, "multiple"):
            changed(DEFAULT_CONFIG, "timing", "record_step", 0.15)
        for value in (0, -1, 86400):
            with self.assertRaises(ValueError):
                DEFAULT_CONFIG.with_run_options(duration_s=value)

    def test_orbit_and_model_domain_validation(self):
        for section, name, value in (("orbit", "altitude", -1), ("orbit", "inclination", 181),
                                     ("orbit", "eccentricity", -0.1), ("orbit", "eccentricity", 1),
                                     ("orbit", "eccentricity", 0.1), ("environment", "mu_earth", 0),
                                     ("environment", "epoch_utc", "2027-01-01T00:00:00+00:00"),
                                     ("environment", "earth_orientation_model", "SPICE")):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                changed(DEFAULT_CONFIG, section, name, value)

    def test_scenario_consumes_supplied_configuration(self):
        c = DEFAULT_CONFIG
        for section, name, value in (("spacecraft", "mass", 3.1),
                                     ("spacecraft", "inertia", ((0.02, 0.002, 0), (0.002, 0.025, 0), (0, 0, 0.03))),
                                     ("orbit", "altitude", 650e3), ("orbit", "inclination", 45.0),
                                     ("initial", "body_rate", (0.05, 0.01, 0.02)),
                                     ("initial", "sigma_BN", (0.1, -0.05, 0.02)),
                                     ("controller", "dipole_command_gain", 33600.0),
                                     ("magnetorquers", "dipole_limits", (0.15, 0.16, 0.7))):
            c = changed(c, section, name, value)
        timing = c.timing
        for name in ("dynamics_step", "environment_step", "sensor_step", "control_step", "record_step"):
            timing = replace(timing, **{name: replace(getattr(timing, name), value=0.2)})
        c = replace(c, timing=timing).with_run_options(duration_s=0.4)
        sc = scenario.configure_spacecraft(c)
        self.assertEqual(sc.hub.mHub, 3.1)
        np.testing.assert_array_equal(sc.hub.IHubPntBc_B, c.spacecraft.inertia.value)
        adapter = ADCSConfig.from_sim_config(c)
        self.assertEqual(adapter.dipoleCommandGain, 33600)
        mtb_config = scenario.configure_mtb_config_message(adapter, c)
        mtb_message = mtb_config.read()  # SWIG payload view requires its message owner to remain alive.
        self.assertEqual(mtb_message.numMTB, 3)
        np.testing.assert_array_equal(mtb_message.maxMtbDipoles[:3], (0.15, 0.16, 0.7))
        # Poison compatibility aliases. A live run must still consume c.
        with patch.object(scenario, "MASS_KG", -123), patch.object(scenario, "CONTROL_DT_S", 999), \
                patch.object(scenario, "INITIAL_RATES", [999]*3), contextlib.redirect_stdout(io.StringIO()):
            df = scenario.run(write_outputs=False, make_plots=False, config=c)
        np.testing.assert_array_equal(df.time_ns, [0, 200_000_000, 400_000_000])
        np.testing.assert_array_equal(df.loc[0, [f"omega_B_{a}_rad_s" for a in "xyz"]], c.initial.body_rate.value)
        np.testing.assert_array_equal(df.loc[0, [f"sigma_BN_{i}" for i in (1, 2, 3)]], c.initial.sigma_BN.value)
        np.testing.assert_array_equal(df.loc[0, [f"r_N_{a}_m" for a in "xyz"]], c.initial_orbit_state()[0])
        np.testing.assert_array_equal(df.loc[0, [f"v_N_{a}_m_s" for a in "xyz"]], c.initial_orbit_state()[1])
        self.assertTrue((df.simulation_config_sha256 == c.fingerprint()).all())
        self.assertEqual(HS2SimConfig.from_dict(df.attrs["simulation_config"]), c)
        metrics = comparison.telemetry_metrics(df)
        self.assertTrue(metrics["telemetry_timing_valid"])
        self.assertTrue(metrics["native_configuration_unchanged"])
        self.assertLess(metrics["rigid_body_step_max_rate_error_rad_s"], 1e-14)
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            comparison.telemetry_metrics(df, DEFAULT_CONFIG)


if __name__ == "__main__":
    unittest.main()
