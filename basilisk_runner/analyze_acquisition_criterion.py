"""Phase 8B-2C design checks, 2026-10-02; no runtime imports or file writes.

ACQUISITION CRITERION DESIGN BASIS / NO THRESHOLD CHANGE /
NO FLIGHT CRITERION SELECTED. Geometry is the rounded, committed 8B-2B
0.4 s example in docs/ATTITUDE_ESTIMATOR_ARCHITECTURE.md at cfcf724.
All perturbations, covariance factors and numerical check tolerances below
are ASSUMED / TEST-ONLY algebra fixtures, not installed sensor performance.
Angles are radians; unit vectors and cosine residuals are dimensionless.
See docs/ACQUISITION_CRITERION_DESIGN.md for derivation and evidence limits.
"""
from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

Vector = NDArray[np.float64]
COSINE = -0.495114043767
FIELD_UT = 22.7306052815
THETA = math.acos(COSINE)
SINE = math.sqrt(1 - COSINE**2)
# Rotated analysis coordinates, not spacecraft mounting or reference settings.
M = np.array([1., 0., 0.])
S = np.array([COSINE, SINE, 0.])
NORMAL = np.array([0., 0., 1.])
TM = np.array([0., 1., 0.])
TS = np.array([SINE, -COSINE, 0.])


def unit(v: Vector) -> Vector:
    length = float(np.linalg.norm(v))
    if not math.isfinite(length) or length == 0:
        raise ValueError("finite nonzero vector required")
    return v / length


def rotate(v: Vector, axis: Vector, angle: float) -> Vector:
    """Active Rodrigues rotation: dv = angle * (unit(axis) cross v)."""
    n = unit(axis)
    return (v * math.cos(angle) + np.cross(n, v) * math.sin(angle)
            + n * float(n @ v) * (1 - math.cos(angle)))


def residual(m: Vector, s: Vector) -> float:
    return float(unit(m) @ unit(s)) - COSINE


def information(m: Vector, s: Vector, wm: float, ws: float) -> Vector:
    """Local direction information; illustrative positive weights, not MEKF R."""
    m, s = unit(m), unit(s)
    return wm * (np.eye(3) - np.outer(m, m)) + ws * (np.eye(3) - np.outer(s, s))


def examples() -> dict[str, float]:
    a = 1e-4  # TEST-ONLY deterministic angle, rad; not a standard deviation.
    return {
        "TAM_toward_Sun": abs(residual(rotate(M, NORMAL, a), S)),
        "Sun_toward_TAM": abs(residual(M, rotate(S, NORMAL, -a))),
        "both_toward_each_other": abs(residual(rotate(M, NORMAL, a), rotate(S, NORMAL, -a))),
        "common_rotation": abs(residual(rotate(M, NORMAL, a), rotate(S, NORMAL, a))),
        "TAM_out_of_plane": abs(residual(M * math.cos(a) + NORMAL * math.sin(a), S)),
        "both_out_of_plane_same_side": abs(residual(M * math.cos(a) + NORMAL * math.sin(a),
                                                    S * math.cos(a) + NORMAL * math.sin(a))),
    }


class DesignChecks(unittest.TestCase):
    """Independent finite rotations/differences and limiting geometries."""

    def test_committed_geometry(self) -> None:
        self.assertAlmostEqual(THETA, 2.08876241514, delta=1e-11)
        self.assertAlmostEqual(float(np.linalg.norm(np.cross(M, S))), SINE, delta=3e-16)

    def test_all_measured_and_reference_rotation_derivatives(self) -> None:
        # Different coordinate orientation for the reference pair exercises signs.
        refs = [rotate(v, np.array([1., -2., 3.]), .7) for v in (M, S)]
        vectors = [M, S, *refs]
        c_b, c_n = np.cross(M, S), np.cross(*refs)
        expected = np.concatenate((c_b, -c_b, -c_n, c_n))
        h = 1e-5  # central-difference step, rad; analysis precision only.
        for i in range(12):
            signed = []
            for sign in (-1, 1):
                changed = list(vectors)
                changed[i // 3] = rotate(changed[i // 3], np.eye(3)[i % 3], sign * h)
                signed.append(float(changed[0] @ changed[1] - changed[2] @ changed[3]))
            self.assertAlmostEqual((signed[1] - signed[0]) / (2 * h), expected[i], delta=5e-11)

    def test_both_in_plane_errors_add_with_toward_convention(self) -> None:
        for a, b in ((1e-4, 2e-4), (-2e-4, 1e-4), (1e-4, -1e-4)):
            exact = residual(rotate(M, NORMAL, a), rotate(S, NORMAL, -b))
            self.assertAlmostEqual(exact, math.cos(THETA - a - b) - COSINE, delta=4e-16)
            self.assertLessEqual(abs(exact - SINE * (a + b)), (a + b)**2 / 2 + 4e-16)

    def test_cross_plane_second_order(self) -> None:
        a, b = 1e-4, -2e-4
        exact = residual(M * math.cos(a) + NORMAL * math.sin(a),
                         S * math.cos(b) + NORMAL * math.sin(b))
        self.assertAlmostEqual(exact, COSINE * (math.cos(a) * math.cos(b) - 1)
                               + math.sin(a) * math.sin(b), delta=4e-16)
        self.assertAlmostEqual(exact, a * b - COSINE * (a*a + b*b) / 2, delta=1e-15)

    def test_general_tangent_expansion_has_cubic_remainder(self) -> None:
        u, v = .7 * TM + .3 * NORMAL, -.2 * TS + .6 * NORMAL
        errors = []
        for h in (1e-3, 5e-4):
            second = h * float(S @ u + M @ v) + h*h * float(u @ v - COSINE * (u @ u + v @ v) / 2)
            errors.append(abs(residual(M + h*u, S + h*v) - second))
        self.assertGreater(errors[0], 1e-11)
        self.assertLess(errors[1] / errors[0], .14)

    def test_common_rotation_is_exactly_invisible(self) -> None:
        for axis in np.eye(3):
            for a in (.2, -.7, 1.1):
                self.assertAlmostEqual(residual(rotate(M, axis, a), rotate(S, axis, a)), 0., delta=4e-16)

    def test_magnitude_normalization_and_invalid_inputs(self) -> None:
        for scale in (.1, 1., 10.):
            self.assertAlmostEqual(residual(scale * M, S), 0., delta=3e-16)
        self.assertGreater(abs(residual(-M, S)), .9)
        for bad in (np.zeros(3), np.array([float("nan"), 0., 0.])):
            with self.assertRaises(ValueError):
                unit(bad)

    def test_joint_covariance_with_correlations(self) -> None:
        # Unitless synthetic factor; h supplies a small angular scale (rad).
        # Symmetric finite rotations independently recover J P J^T without
        # random draws or assuming independent TAM/Sun components.
        factor = np.eye(6) + .2 * np.tril(np.ones((6, 6)), -1)
        cross = np.cross(M, S)
        jacobian = np.concatenate((cross, -cross))
        expected = float(jacobian @ factor @ factor.T @ jacobian)
        h = 1e-5
        finite_variance = 0.
        for column in factor.T:
            dm, ds = column[:3], column[3:]
            def perturbed(sign: int) -> float:
                mm = rotate(M, dm, sign * h * float(np.linalg.norm(dm))) if np.any(dm) else M
                ss = rotate(S, ds, sign * h * float(np.linalg.norm(ds))) if np.any(ds) else S
                return residual(mm, ss)
            derivative = (perturbed(1) - perturbed(-1)) / (2 * h)
            finite_variance += derivative**2
        self.assertAlmostEqual(finite_variance, expected, delta=1e-9)
        common = np.vstack((np.eye(3), np.eye(3)))
        np.testing.assert_allclose(jacobian @ common, 0., atol=1e-15, rtol=0)

    def test_information_eigenvalues_and_collinear_rank_loss(self) -> None:
        # Weights are arbitrary TEST-ONLY inverse-angular-variance units.
        for angle in (0., 1e-4, .3, math.pi/2, math.pi-1e-4, math.pi):
            s = np.array([math.cos(angle), math.sin(angle), 0.])
            for wm, ws in ((1., 1.), (2., 3.)):
                total = wm + ws
                separation = math.sqrt((wm - ws)**2 + 4*wm*ws*math.cos(angle)**2)
                expected = sorted(((total-separation)/2, (total+separation)/2, total))
                np.testing.assert_allclose(np.linalg.eigvalsh(information(M, s, wm, ws)),
                                           expected, atol=2e-15, rtol=1e-7)
        for sign in (-1, 1):
            self.assertEqual(np.linalg.matrix_rank(information(M, sign*M, 1., 1.)), 2)

    def test_cosine_sensitivity_disappears_at_collinearity(self) -> None:
        a = 1e-4
        for c in (-1., 1.):
            measured = float(rotate(M, NORMAL, a) @ (c*M))
            self.assertAlmostEqual(abs(measured - c), a*a/2, delta=1e-16)
        self.assertAlmostEqual(abs(float(rotate(M, NORMAL, a) @ TM)), a, delta=2e-13)

    def test_existing_policy_remains_test_only(self) -> None:
        path = Path(__file__).resolve().parent / "config/attitude_mekf_test_only.json"
        policy = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(policy["acquisition_pair_tolerance"]["value"], 1e-8)
        self.assertEqual(policy["minimum_acquisition_sine"]["value"], 1e-6)
        self.assertEqual(policy["status"], "ASSUMED")
        self.assertFalse(policy["flight_runtime_usable"])


def main() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(DesignChecks)
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(suite)
    print(json.dumps({
        "scope": "DESIGN BASIS ONLY / NO THRESHOLD CHANGE / NO FLIGHT CRITERION SELECTED",
        "source": "committed Phase 8B-2B geometry at cfcf724; rounded inputs",
        "revision": "2026-10-02", "passed": result.wasSuccessful(),
        "tests": result.testsRun, "theta_rad": THETA, "cosine": COSINE, "sine": SINE,
        "field_microtesla": FIELD_UT, "illustrative_angle_rad_TEST_ONLY": 1e-4,
        "absolute_cosine_residuals": examples(),
        "equal_weight_information_eigenvalues_per_unit_weight": np.linalg.eigvalsh(information(M, S, 1., 1.)).tolist(),
    }, indent=2, allow_nan=False))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
