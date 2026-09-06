"""Focused Phase 5B protections: isolation, physical response, work and sign."""
import unittest
import numpy as np

from hs2_sim_config import DEFAULT_CONFIG, get_hs2_candidate_config
from basilisk_adcs_adapter import ADCSConfig, controller_step
from analyze_physical_profile_dynamics import configurations, isolation_report, bench, rk_work, intervals


class PhysicalDynamicsTests(unittest.TestCase):
    def test_inertia_factorial_isolation_and_unchanged_defaults(self):
        self.assertTrue(isolation_report()["passed"])
        cases = configurations()
        a, d = np.diag(DEFAULT_CONFIG.spacecraft.inertia.value), np.diag(get_hs2_candidate_config().spacecraft.inertia.value)
        np.testing.assert_array_equal(np.diag(cases["B"].spacecraft.inertia.value), [d[0], d[1], a[2]])
        np.testing.assert_array_equal(np.diag(cases["C"].spacecraft.inertia.value), [a[0], a[1], d[2]])
        np.testing.assert_array_equal(cases["D"].spacecraft.inertia.value, get_hs2_candidate_config().spacecraft.inertia.value)
        np.testing.assert_array_equal(cases["E"].spacecraft.inertia.value, DEFAULT_CONFIG.spacecraft.inertia.value)
        f, g = np.diag(cases["F"].spacecraft.inertia.value), np.diag(cases["G"].spacecraft.inertia.value)
        self.assertAlmostEqual(f[2]/f[0], a[2]/a[0])
        self.assertAlmostEqual(g[2]/g[0], d[2]/d[0])
        self.assertEqual(DEFAULT_CONFIG.fingerprint(), "99de374af2f512e27d34e40fdc3327356662ef48804f7ba7945b7a703a68ad8f")

    def test_native_response_identifies_each_candidate_principal_moment(self):
        config = get_hs2_candidate_config()
        for j in range(3):
            m, b = np.eye(3)[(j+1)%3]*0.1, np.eye(3)[(j+2)%3]*4e-5
            output = bench(config, [0., 0., 0.], b, dt=0.001, duration=0.001, fixed_dipole=m, euler=True)
            expected_tau = np.eye(3)[j]*4e-6
            np.testing.assert_allclose(output["torque"][-1], expected_tau, atol=1e-20, rtol=0)
            np.testing.assert_allclose(output["omega"][-1], 0.001*expected_tau/np.diag(config.spacecraft.inertia.value), atol=1e-18, rtol=0)

    def test_work_quadrature_against_independent_native_propagation(self):
        config = get_hs2_candidate_config()
        m, b = [0.1, -0.05, 0.2], [1e-5, -2e-5, 3e-5]
        result = bench(config, [0.8, -0.2, 0.3], b, duration=2., fixed_dipole=m)
        n = len(result["omega"])-1
        work, tau, _, _, _ = rk_work(result["sigma"][:-1], result["omega"][:-1],
            np.tile(m, (n, 1)), np.tile(b, (n, 1)), np.diff(result["time_s"]), config.spacecraft.inertia.value)
        np.testing.assert_allclose(tau, result["torque"][1:], atol=1e-18, rtol=0)
        energy = 0.5*np.sum(result["omega"]**2*np.diag(config.spacecraft.inertia.value), axis=1)
        self.assertLess(abs((energy[-1]-energy[0])-work.sum()), 1e-11)
        self.assertGreater(abs((energy[-1]-energy[0])+work.sum()), 1e-7)  # Reversing work sign must fail.

    def test_component_clipping_cannot_add_energy_at_same_acquisition_state(self):
        cfg = ADCSConfig(use_cpp_core_if_available=False)
        for w in ([0.01, 0.02, -0.03], [0.8, -0.2, 0.3], [-2., 3., 4.]):
            field = np.array([1e-5, -2e-5, 4e-5])
            omega = np.asarray(w)
            result = controller_step(0., omega, field, cfg)
            applied = np.asarray(result["commanded_magnetic_dipole_B_Am2"])
            requested = cfg.dipoleCommandGain*np.cross(omega, field)
            parallel = np.dot(omega, field)*field/np.dot(field, field)
            ideal_tau = -cfg.dipoleCommandGain*np.dot(field, field)*(omega-parallel)
            np.testing.assert_allclose(np.cross(requested, field), ideal_tau, atol=1e-18, rtol=0)
            power = float(np.dot(np.cross(applied, field), omega))
            self.assertAlmostEqual(power, -float(applied@requested)/cfg.dipoleCommandGain, places=18)
            self.assertLessEqual(power, 1e-18)
            self.assertGreaterEqual(power, float(ideal_tau@omega)-1e-18)

    def test_saturation_intervals_are_following_hold_durations(self):
        blocks = intervals(np.array([True, True, False, True]), np.array([0., 0.1, 0.2, 0.3, 0.4]))
        self.assertEqual(len(blocks), 2)
        self.assertAlmostEqual(sum(item["duration_s"] for item in blocks), 0.3)
        self.assertEqual(blocks[0]["start_s"], 0.)
        self.assertEqual(blocks[0]["end_s"], 0.2)
        self.assertEqual(intervals(np.zeros(4, dtype=bool), np.arange(5)), [])


if __name__ == "__main__":
    unittest.main()
