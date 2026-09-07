"""Phase 7B mathematical checks, not a runtime estimator or flight tuning.

All rotations, step sizes, SPD matrices and tolerances below are synthetic
algebra fixtures. They are not HS-2 parameters, Q/R, gates or accuracy claims.
No scenario is imported and no simulation output is created.
"""

import unittest

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def unit(v: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(v))
    if not np.isfinite(length) or length == 0.0:
        raise ValueError("A finite nonzero vector is required")
    return v / length


def product(q: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Scalar-first Hamilton product; passive DCM composition reverses order."""
    return np.r_[q[0] * p[0] - q[1:] @ p[1:],
                 q[0] * p[1:] + p[0] * q[1:] + np.cross(q[1:], p[1:])]


def inverse(q: np.ndarray) -> np.ndarray:
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def exp_q(a: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(a))
    return np.r_[np.cos(theta / 2.0), 0.5 * np.sinc(theta / (2.0 * np.pi)) * a]


def log_q(q: np.ndarray) -> np.ndarray:
    q = unit(q)
    if q[0] < 0.0:
        q = -q
    s = float(np.linalg.norm(q[1:]))
    return 2.0 * q[1:] if s == 0.0 else 2.0 * np.arctan2(s, q[0]) * q[1:] / s


def passive_rodrigues(a: np.ndarray) -> np.ndarray:
    """Independent matrix exponential of -[a]x, without quaternion conversion."""
    theta = float(np.linalg.norm(a))
    if theta == 0.0:
        return np.eye(3)
    k = skew(a / theta)
    return np.eye(3) - np.sin(theta) * k + (1.0 - np.cos(theta)) * (k @ k)


def tangent(u: np.ndarray) -> np.ndarray:
    seed = np.eye(3)[int(np.argmin(np.abs(u)))]
    first = unit(np.cross(u, seed))
    return np.column_stack((first, np.cross(u, first)))


def triad(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    e1 = unit(first)
    e2 = unit(np.cross(first, second))
    return np.column_stack((e1, e2, np.cross(e1, e2)))


def right_jacobian(a: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(a))
    k = skew(a)
    if theta == 0.0:
        return np.eye(3)
    return (np.eye(3) - (1.0 - np.cos(theta)) / theta**2 * k
            + (theta - np.sin(theta)) / theta**3 * (k @ k))


class TestAttitudeEstimatorArchitecture(unittest.TestCase):
    def test_literal_rotation_and_wrong_transpose(self):
        a = np.array([0.0, 0.0, np.pi / 2.0])
        q = exp_q(a)
        c = np.asarray(rbk.EP2C(q), dtype=float)
        expected = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(c, expected, atol=4e-16)
        np.testing.assert_allclose(c @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0], atol=4e-16)
        self.assertGreater(float(np.linalg.norm(c - c.T)), 2.0)

    def test_composition_and_body_rate_propagation(self):
        a, b = np.array([0.2, -0.3, 0.4]), np.array([-0.4, 0.1, 0.2])
        q, p = exp_q(a), exp_q(b)
        np.testing.assert_allclose(rbk.EP2C(product(q, p)), passive_rodrigues(b) @ passive_rodrigues(a), atol=8e-16)
        self.assertGreater(float(np.linalg.norm(product(q, p) - product(p, q))), 0.05)
        omega = np.array([0.12, 0.21, -0.08])
        qdot = 0.5 * product(q, np.r_[0.0, omega])
        np.testing.assert_allclose(qdot, 0.5 * np.asarray(rbk.BmatEP(q)) @ omega, atol=1e-16)
        h = 1e-5
        c_dot = (np.asarray(rbk.EP2C(product(q, exp_q(h * omega))))
                 - np.asarray(rbk.EP2C(product(q, exp_q(-h * omega))))) / (2.0 * h)
        np.testing.assert_allclose(c_dot, -skew(omega) @ passive_rodrigues(a), atol=2e-11)

    def test_quaternion_mrp_shadow_and_normalization(self):
        for angle in (0.0, np.pi - 1e-9, np.pi, np.pi + 1e-9, 4.1):
            a = unit(np.array([2.0, -1.0, 3.0])) * angle
            q = exp_q(a)
            c = passive_rodrigues(a)
            for signed_q in (q, -q, unit(1.0002 * q)):
                sigma = np.asarray(rbk.EP2MRP(signed_q), dtype=float)
                np.testing.assert_allclose(rbk.MRP2C(sigma), c, atol=2e-15)
                if float(sigma @ sigma) > 0.0:
                    np.testing.assert_allclose(rbk.MRP2C(-sigma / (sigma @ sigma)), c, atol=2e-15)
            np.testing.assert_allclose(c @ c.T, np.eye(3), atol=2e-15)
            self.assertAlmostEqual(float(np.linalg.det(c)), 1.0, places=14)
        with self.assertRaises(ValueError):
            unit(np.zeros(4))

    def test_sensor_earth_and_body_transform_chain(self):
        # Nonidentity mounts and Earth rotation expose hidden transpose errors.
        c_pn = passive_rodrigues(np.array([0.0, 0.0, 0.8]))
        c_bn = passive_rodrigues(np.array([0.3, -0.2, 0.5]))
        c_sb = passive_rodrigues(np.array([-0.4, 0.2, 0.1]))
        field_p = np.array([1.0, 2.0, -3.0])
        field_n = c_pn.T @ field_p
        measured_s = c_sb @ c_bn @ field_n
        np.testing.assert_allclose(c_sb.T @ measured_s, c_bn @ field_n, atol=2e-15)
        for reference_n in (unit(field_n), unit(np.array([-2.0, 3.0, 1.0]))):
            self.assertGreater(float(np.linalg.norm(c_bn.T @ reference_n - c_bn @ reference_n)), 0.1)
        wrong = c_pn @ field_p
        self.assertAlmostEqual(float(np.linalg.norm(wrong)), float(np.linalg.norm(field_n)))
        self.assertGreater(float(np.linalg.norm(wrong - field_n)), 1.0)

    def test_vector_measurement_jacobian_sign(self):
        q = exp_q(np.array([0.3, 0.2, -0.4]))
        u_n = unit(np.array([1.0, -2.0, 4.0]))
        predicted = np.asarray(rbk.EP2C(q)) @ u_n
        eps = 1e-6
        numerical = np.column_stack([
            (np.asarray(rbk.EP2C(product(q, exp_q(eps * axis)))) @ u_n
             - np.asarray(rbk.EP2C(product(q, exp_q(-eps * axis)))) @ u_n) / (2.0 * eps)
            for axis in np.eye(3)
        ])
        np.testing.assert_allclose(numerical, skew(predicted), atol=2e-10)
        self.assertGreater(float(np.linalg.norm(numerical + skew(predicted))), 2.0)

    def test_normalized_covariance_and_tangent_projection(self):
        v = np.array([2.0, -3.0, 4.0])
        u = unit(v)
        j = (np.eye(3) - np.outer(u, u)) / np.linalg.norm(v)
        eps = 1e-5
        numerical = np.column_stack([(unit(v + eps * axis) - unit(v - eps * axis)) / (2 * eps) for axis in np.eye(3)])
        np.testing.assert_allclose(numerical, j, atol=1e-11)
        e = tangent(u)
        r3 = j @ np.diag([1.0, 2.0, 3.0]) @ j.T
        self.assertEqual(int(np.linalg.matrix_rank(r3)), 2)
        self.assertGreater(float(np.linalg.eigvalsh(e.T @ r3 @ e).min()), 0.0)
        np.testing.assert_allclose(e.T @ u, np.zeros(2), atol=1e-16)

    def test_error_dynamics_attitude_and_bias_signs(self):
        q = exp_q(np.array([0.2, -0.4, 0.1]))
        omega = np.array([0.4, -0.2, 0.3])
        eps, h = 1e-5, 1e-4
        columns = []
        for axis in np.eye(6):
            errors = []
            for sign in (-1.0, 1.0):
                perturb = sign * eps * axis
                true_q = product(q, exp_q(perturb[:3]))
                # z-bhat=omega; true rate differs by negative residual bias.
                true_next = product(true_q, exp_q(h * (omega - perturb[3:])))
                nominal_next = product(q, exp_q(h * omega))
                errors.append(log_q(product(inverse(nominal_next), true_next)))
            columns.append((errors[1] - errors[0]) / (2.0 * eps))
        phi_top = np.column_stack(columns)
        numerical_f = (phi_top - np.c_[np.eye(3), np.zeros((3, 3))]) / h
        np.testing.assert_allclose(numerical_f, np.c_[-skew(omega), -np.eye(3)], atol=2.1e-5)

    def test_triad_acquisition_and_collinear_rejection(self):
        c = passive_rodrigues(np.array([1.0, -0.6, 0.2]))
        n1, n2 = unit(np.array([1.0, 2.0, 3.0])), unit(np.array([-2.0, 3.0, 1.0]))
        recovered = triad(c @ n1, c @ n2) @ triad(n1, n2).T
        np.testing.assert_allclose(recovered, c, atol=7e-16)
        with self.assertRaises(ValueError):
            triad(np.array([1.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]))

    def test_delayed_vector_transport_retains_original_reference_epoch(self):
        c0 = passive_rodrigues(np.array([0.2, 0.1, -0.3]))
        relative = passive_rodrigues(np.array([-0.3, 0.4, 0.2]))
        c1 = relative @ c0
        u0, u1 = unit(np.array([1.0, 2.0, 3.0])), unit(np.array([2.0, 1.0, 3.0]))
        old_measured = c0 @ u0
        transported = relative @ old_measured
        np.testing.assert_allclose(transported, c1 @ u0, atol=3e-16)
        self.assertGreater(float(np.linalg.norm(transported - c1 @ u1)), 0.2)
        self.assertGreater(float(np.linalg.norm(old_measured - c1 @ u0)), 0.1)
        self.assertAlmostEqual(float(np.linalg.norm(old_measured)), float(np.linalg.norm(c1 @ u0)))

    def test_single_vector_and_temporal_bias_observability(self):
        def h_for(u):
            return np.c_[tangent(u).T @ skew(u), np.zeros((2, 3))]

        h1 = h_for(np.array([0.0, 0.0, 1.0]))
        h2 = h_for(np.array([1.0, 0.0, 0.0]))
        phi = np.block([[np.eye(3), -np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        self.assertEqual(int(np.linalg.matrix_rank(h1)), 2)
        self.assertEqual(int(np.linalg.matrix_rank(np.vstack((h1, h1 @ phi)))), 4)
        both = np.vstack((h1, h2))
        self.assertEqual(int(np.linalg.matrix_rank(both)), 3)
        self.assertEqual(int(np.linalg.matrix_rank(np.vstack((both, both @ phi)))), 6)

    def test_zero_rate_discrete_noise_and_psd(self):
        h = 0.3
        sg, sb = np.diag([1.0, 2.0, 3.0]), np.diag([0.3, 0.2, 0.1])
        closed = np.block([[h * sg + h**3 * sb / 3.0, -h**2 * sb / 2.0],
                           [-h**2 * sb / 2.0, h * sb]])
        # Independent positive-weight Gauss quadrature integrates this polynomial exactly.
        nodes, weights = np.polynomial.legendre.leggauss(3)
        integral = np.zeros((6, 6))
        qc = np.block([[sg, np.zeros((3, 3))], [np.zeros((3, 3)), sb]])
        g = np.diag([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0])
        for node, weight in zip(nodes, weights):
            remaining = h * (1.0 - node) / 2.0
            phi = np.block([[np.eye(3), -remaining * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
            integral += h * weight / 2.0 * phi @ g @ qc @ g.T @ phi.T
        np.testing.assert_allclose(integral, closed, atol=3e-16)
        self.assertGreater(float(np.linalg.eigvalsh(closed).min()), 0.0)
        self.assertLess(float(closed[0, 3]), 0.0)

    def test_covariance_reset_jacobian(self):
        a = np.array([0.3, -0.2, 0.4])
        eps = 1e-6
        numerical = np.column_stack([
            (log_q(product(inverse(exp_q(a)), exp_q(a + eps * axis)))
             - log_q(product(inverse(exp_q(a)), exp_q(a - eps * axis)))) / (2.0 * eps)
            for axis in np.eye(3)
        ])
        np.testing.assert_allclose(numerical, right_jacobian(a), atol=1e-10)
        self.assertGreater(float(np.linalg.norm(numerical - right_jacobian(a).T)), 0.5)

    def test_joseph_and_reset_preserve_covariance(self):
        a = np.arange(36, dtype=float).reshape(6, 6) / 36.0
        p = a @ a.T + np.eye(6)
        u = unit(np.array([1.0, 2.0, 3.0]))
        h = np.c_[tangent(u).T @ skew(u), np.zeros((2, 3))]
        r = np.diag([0.7, 0.9])
        s = h @ p @ h.T + r
        k = np.linalg.solve(s, h @ p).T
        residual_map = np.eye(6) - k @ h
        joseph = residual_map @ p @ residual_map.T + k @ r @ k.T
        np.testing.assert_allclose(joseph, p - k @ s @ k.T, atol=2e-15)
        gamma = np.block([[right_jacobian(np.array([0.3, -0.1, 0.2])), np.zeros((3, 3))],
                          [np.zeros((3, 3)), np.eye(3)]])
        reset = gamma @ joseph @ gamma.T
        for candidate in (joseph, reset):
            np.testing.assert_allclose(candidate, candidate.T, atol=1e-15)
            self.assertGreater(float(np.linalg.eigvalsh(candidate).min()), 0.0)
        self.assertGreater(float(np.linalg.norm(reset[:3, 3:] - joseph[:3, 3:])), 0.01)

    def test_antipodal_vector_is_not_a_zero_error_update(self):
        u = np.array([0.0, 0.0, 1.0])
        measured = -u
        np.testing.assert_array_equal(tangent(u).T @ (measured - u), np.zeros(2))
        angle = np.arctan2(np.linalg.norm(np.cross(measured, u)), measured @ u)
        self.assertAlmostEqual(float(angle), np.pi)


if __name__ == "__main__":
    unittest.main()
