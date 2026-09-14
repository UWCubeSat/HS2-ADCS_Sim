"""Phase 7C opt-in mathematical prototype; no flight noise/accuracy model.

Source/convention: docs/ATTITUDE_ESTIMATOR_ARCHITECTURE.md, Phase 7B, 2026-09-07.
N is inertial; B is the mathematical body frame. C_BN maps N components to B.
Scalar-first Hamilton q satisfies C(q*p)=C(p)C(q). Right multiplicative error:
q_true=qhat*Exp(delta_theta_B), delta_b=b_true-bhat (rad/s, fixed B axes).
The module owns no plant, controller, sensor simulation or flight configuration.
"""
from __future__ import annotations

from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass
from typing import Literal

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


def epoch(value: int) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError("epoch must be a nonnegative integer nanosecond count")
    return int(value)


def vector(value: ArrayLike, size: int = 3) -> Array:
    result = np.array(value, dtype=float, copy=True)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError("finite vector with declared dimension required")
    return result


def unit(value: ArrayLike) -> Array:
    result = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(result))
    if not np.isfinite(norm) or norm == 0:
        raise ValueError("finite nonzero vector required")
    return result / norm


def skew(v: ArrayLike) -> Array:
    x, y, z = vector(v)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def multiply(q: Array, p: Array) -> Array:
    return np.r_[q[0]*p[0] - q[1:] @ p[1:],
                 q[0]*p[1:] + p[0]*q[1:] + np.cross(q[1:], p[1:])]


def rotation_quaternion(a: ArrayLike) -> Array:
    a = vector(a)
    angle = float(np.linalg.norm(a))
    return np.r_[np.cos(angle/2), 0.5*np.sinc(angle/(2*np.pi))*a]


def dcm(q: ArrayLike) -> Array:
    q = unit(vector(q, 4))
    return (q[0]**2-q[1:] @ q[1:])*np.eye(3) + 2*np.outer(q[1:], q[1:]) - 2*q[0]*skew(q[1:])


def quaternion_from_dcm(c: Array) -> Array:
    # Largest-component eigenvector of Davenport's exact rotation matrix.
    # Passive C requires C23-C32, not the active-rotation sign.
    z = np.array([c[1, 2]-c[2, 1], c[2, 0]-c[0, 2], c[0, 1]-c[1, 0]])
    k = np.block([[np.array([[np.trace(c)]]), z[None, :]],
                  [z[:, None], c+c.T-np.trace(c)*np.eye(3)]])
    q = np.linalg.eigh(k)[1][:, -1]
    return q if q[0] >= 0 else -q


def right_jacobian(a: ArrayLike) -> Array:
    a = vector(a)
    theta = float(np.linalg.norm(a))
    k = skew(a)
    if theta < 1e-4:  # Numerical series switch, not an operational gate.
        aa = 0.5-theta**2/24+theta**4/720
        bb = 1/6-theta**2/120+theta**4/5040
    else:
        aa = (1-np.cos(theta))/theta**2
        bb = (theta-np.sin(theta))/theta**3
    return np.eye(3)-aa*k+bb*(k @ k)


def transition(omega: Array, seconds: float) -> Array:
    """Exact constant-rate F exponential, including attitude/bias coupling."""
    a = omega*seconds
    return np.block([[dcm(rotation_quaternion(a)), -seconds*right_jacobian(a)],
                     [np.zeros((3, 3)), np.eye(3)]])


def covariance(value: ArrayLike, size: int, *, positive: bool = False) -> Array:
    p = np.array(value, dtype=float, copy=True)
    if p.shape != (size, size) or not np.isfinite(p).all():
        raise ValueError("finite covariance with declared dimension required")
    tolerance = 128*np.finfo(float).eps*max(1., float(np.linalg.norm(p)))
    if np.linalg.norm(p-p.T) > tolerance:
        raise ValueError("asymmetric covariance")
    p = (p+p.T)/2
    minimum = float(np.linalg.eigvalsh(p).min())
    if minimum < -tolerance or (positive and minimum <= 0):
        raise ValueError("covariance must be PSD (innovation weights strictly positive)")
    return p  # Never clip eigenvalues or add unexplained jitter.


def triad(body1: ArrayLike, body2: ArrayLike, reference1: ArrayLike,
          reference2: ArrayLike, minimum_sine: float) -> tuple[Array, float]:
    """Return q_BN and conditioning indicator; caller supplies TEST-ONLY gate."""
    if not 0 < minimum_sine < 1:
        raise ValueError("explicit collinearity gate required")

    def basis(first, second):
        first, second = unit(vector(first)), unit(vector(second))
        cross = np.cross(first, second)
        sine = float(np.linalg.norm(cross))
        if sine <= minimum_sine:
            raise ValueError("collinear/ill-conditioned acquisition vectors")
        normal = cross/sine
        return np.column_stack((first, normal, np.cross(first, normal))), sine

    body, sb = basis(body1, body2)
    reference, sn = basis(reference1, reference2)
    return quaternion_from_dcm(body @ reference.T), min(sb, sn)


@dataclass(frozen=True)
class NumericalPolicy:
    """Explicit algorithm-test weights/policies, never sensor specifications."""
    p0: Array
    qc: Array
    vector_weight: float
    maximum_residual_rad: float
    minimum_acquisition_sine: float
    history_ns: int
    max_events: int
    label: str
    acquisition_pair_tolerance: float

    def __post_init__(self):
        covariance(self.p0, 6, positive=True)
        covariance(self.qc, 6)
        if (not np.isfinite(self.vector_weight) or self.vector_weight <= 0
                or not 0 < self.maximum_residual_rad < np.pi/2
                or not 0 < self.minimum_acquisition_sine < 1
                or not 0 < self.acquisition_pair_tolerance < 1
                or self.history_ns <= 0 or self.max_events < 4
                or self.label != "TEST-ONLY; NOT FLIGHT TUNING"):
            raise ValueError("explicit valid TEST-ONLY numerical policy required")


@dataclass(frozen=True)
class VectorSample:
    event_id: str
    sensor: Literal["magnetic", "sun"]
    epoch_ns: int
    reference_epoch_ns: int
    measured: ArrayLike
    reference_n: ArrayLike
    source: str
    valid: bool = True
    invalid_reason: str = "sensor_invalid"
    frame: Literal["B", "S"] = "B"
    c_sb: ArrayLike | None = None

    def body_and_reference(self) -> tuple[Array, Array]:
        epoch(self.epoch_ns)
        epoch(self.reference_epoch_ns)
        if self.sensor not in ("magnetic", "sun") or not self.event_id or not self.source:
            raise ValueError("sensor/source/event identity required")
        if self.reference_epoch_ns != self.epoch_ns:
            raise ValueError("reference_epoch_mismatch")
        body = vector(self.measured)
        if self.frame == "S":
            c = np.asarray(self.c_sb, dtype=float)
            if (c.shape != (3, 3) or not np.isfinite(c).all()
                    or not np.allclose(c @ c.T, np.eye(3), atol=1e-12, rtol=0)
                    or not np.isclose(np.linalg.det(c), 1., atol=1e-12, rtol=0)):
                raise ValueError("proper supplied C_SB required for sensor-frame input")
            body = c.T @ body
        elif self.frame != "B" or self.c_sb is not None:
            raise ValueError("unambiguous body/sensor frame required")
        return unit(body), unit(vector(self.reference_n))


@dataclass(frozen=True)
class LossEvent:
    event_id: str
    epoch_ns: int
    source: str = "explicit TEST-ONLY lost-attitude command"


class MEKF:
    """Local filter kernel. All epochs are physical monotonic integer ns."""
    def __init__(self, policy: NumericalPolicy, epoch_ns: int = 0,
                 prior_q: ArrayLike | None = None, prior_bias: ArrayLike = (0., 0., 0.)):
        self.policy = deepcopy(policy)
        self.epoch_ns = epoch(epoch_ns)
        self.q = unit(vector(prior_q, 4)) if prior_q is not None else np.array([1., 0., 0., 0.])
        self.initialized = prior_q is not None
        self.bias = vector(prior_bias)
        self.initial_bias = self.bias.copy()
        self.p = covariance(policy.p0, 6, positive=True)
        self.last_rate: Array | None = None
        self.rate_support_ns: tuple[int, int] | None = None
        self.pending: dict[str, VectorSample] = {}
        self.updates: Counter = Counter()
        self.rejections: Counter = Counter()
        self.acquisitions = 0
        self.instantaneous_attitude_rank = 0
        self.last_update: dict | None = None

    def propagate(self, end_ns: int, gyro: ArrayLike):
        end_ns = epoch(end_ns)
        if end_ns <= self.epoch_ns:
            raise ValueError("gyro interval must move forward with explicit coverage")
        h = (end_ns-self.epoch_ns)*1e-9
        omega = vector(gyro)-self.bias
        phi = transition(omega, h)
        qd = np.zeros((6, 6))
        if np.any(self.policy.qc):
            # Positive-weight quadrature preserves PSD. Constant-rate interval
            # approximation; convergence against refined quadrature is tested.
            nodes, weights = np.polynomial.legendre.leggauss(8)
            g = np.diag([-1., -1., -1., 1., 1., 1.])
            spectral = g @ self.policy.qc @ g.T
            for node, weight in zip(nodes, weights):
                sub = transition(omega, h*(1-node)/2)
                qd += h*weight/2 * (sub @ spectral @ sub.T)
        self.p = covariance(phi @ self.p @ phi.T+qd, 6)
        self.q = unit(multiply(self.q, rotation_quaternion(omega*h)))
        self.last_rate = omega
        self.rate_support_ns = (self.epoch_ns, end_ns)
        self.epoch_ns = end_ns
        self.instantaneous_attitude_rank = 0
        self.pending.clear()  # Acquisition requires a genuine common-epoch pair.

    def observe(self, sample: VectorSample) -> str:
        if sample.epoch_ns != self.epoch_ns:
            raise ValueError("measurement_state_epoch_mismatch")
        if not sample.valid:
            self.rejections[sample.invalid_reason] += 1
            return sample.invalid_reason
        try:
            body, reference = sample.body_and_reference()
        except ValueError as error:
            self.rejections[str(error)] += 1
            return str(error)
        if not self.initialized:
            self.pending[sample.sensor] = deepcopy(sample)
            if len(self.pending) < 2:
                return "awaiting_noncollinear_pair"
            mag, sun = self.pending["magnetic"], self.pending["sun"]
            mb, mn = mag.body_and_reference()
            sb, sn = sun.body_and_reference()
            # Rotation preserves the angle between vectors. This TEST-ONLY
            # ideal-data consistency check is separate from collinearity.
            if abs(float(mb @ sb-mn @ sn)) > self.policy.acquisition_pair_tolerance:
                self.rejections["inconsistent_acquisition_pair"] += 1
                return "inconsistent_acquisition_pair"
            try:
                self.q, _ = triad(mb, sb, mn, sn, self.policy.minimum_acquisition_sine)
            except ValueError:
                self.rejections["acquisition_geometry"] += 1
                return "acquisition_geometry"
            self.bias = self.initial_bias.copy()  # Declared reset policy, not truth bias.
            self.p = covariance(self.policy.p0, 6)
            self.initialized = True
            self.acquisitions += 1
            self.updates.update(("magnetic", "sun"))
            self.instantaneous_attitude_rank = 3
            self.last_rate = None
            self.rate_support_ns = None
            self.last_update = {"epoch_ns": self.epoch_ns, "kind": "TRIAD", "events": [mag.event_id, sun.event_id],
                                "sources": [mag.source, sun.source]}
            self.pending.clear()
            return "acquired"
        predicted = dcm(self.q) @ reference
        angle = float(np.arctan2(np.linalg.norm(np.cross(body, predicted)), body @ predicted))
        if angle > self.policy.maximum_residual_rad:
            self.rejections["outside_local_gate"] += 1
            return "outside_local_gate"  # Includes antipodal zero-tangent trap.
        seed = np.eye(3)[int(np.argmin(np.abs(predicted)))]
        e1 = unit(np.cross(predicted, seed))
        e = np.column_stack((e1, np.cross(predicted, e1)))
        residual = e.T @ (body-predicted)
        h = np.c_[e.T @ skew(predicted), np.zeros((2, 3))]  # Positive sign.
        r = self.policy.vector_weight*np.eye(2)  # TEST-ONLY tangent algebra weight.
        innovation = covariance(h @ self.p @ h.T+r, 2, positive=True)
        gain = np.linalg.solve(innovation, h @ self.p).T
        correction = gain @ residual
        residual_map = np.eye(6)-gain @ h
        pe = residual_map @ self.p @ residual_map.T + gain @ r @ gain.T
        gamma = np.block([[right_jacobian(correction[:3]), np.zeros((3, 3))],
                          [np.zeros((3, 3)), np.eye(3)]])
        self.p = covariance(gamma @ pe @ gamma.T, 6)
        self.q = unit(multiply(self.q, rotation_quaternion(correction[:3])))
        self.bias += correction[3:]
        # The previous interval's rate must not be relabeled as a current rate
        # after a bias correction. A subsequent covered interval supplies it.
        self.last_rate = None
        self.rate_support_ns = None
        self.updates[sample.sensor] += 1
        self.instantaneous_attitude_rank = 2
        self.last_update = {"epoch_ns": self.epoch_ns, "kind": sample.sensor,
                            "events": [sample.event_id], "sources": [sample.source],
                            "residual_angle_rad": angle, "tangent_innovation": residual.tolist()}
        return "updated"

    def output(self) -> dict:
        q = self.q if self.q[0] >= 0 else -self.q
        return {"q_BN": self.q.copy(), "sigma_BN": q[1:]/(1+q[0]),
                "bias_B_rad_s": self.bias.copy(), "P": self.p.copy(), "epoch_ns": self.epoch_ns,
                "initialized": self.initialized, "instantaneous_attitude_rank": self.instantaneous_attitude_rank,
                "omega_B_rad_s": None if self.last_rate is None else self.last_rate.copy(),
                "rate_support_ns": self.rate_support_ns, "last_update": deepcopy(self.last_update),
                "covariance_status": "TEST-ONLY; NOT CALIBRATED CONFIDENCE"}


@dataclass(frozen=True)
class GyroInterval:
    start_ns: int
    end_ns: int
    rate_B_rad_s: Array


class ReplayEstimator:
    """Bounded fixed-lag event replay, with canonical same-epoch loss/mag/Sun order.

    Gyro intervals arrive contiguously in forward order, providing piecewise
    constant rate support. Vector events can arrive late within retained history.
    Replays include ALL admitted vector events, reevaluating local gates. Counts
    reflect the final event history, not how often an event was replayed.
    """
    def __init__(self, initial: MEKF):
        self.core = deepcopy(initial)
        self.base = deepcopy(initial)
        self.sealed_base = False
        self.gyros: list[GyroInterval] = []
        self.events: list[VectorSample | LossEvent] = []
        self.checkpoints: dict[int, MEKF] = {}
        self.receipts = deque(maxlen=initial.policy.max_events)
        self.ingress_rejections: Counter = Counter()
        self.replay_count = 0
        self.revision = 0
        self.dispositions: dict[str, str] = {}

    def advance(self, start_ns: int, end_ns: int, rate_B_rad_s: ArrayLike):
        start_ns, end_ns = epoch(start_ns), epoch(end_ns)
        if start_ns != self.core.epoch_ns:
            raise ValueError("missing/overlapping gyro coverage")
        gyro = vector(rate_B_rad_s)
        self.core.propagate(end_ns, gyro)
        self.gyros.append(GyroInterval(start_ns, end_ns, gyro))
        self.checkpoints[end_ns] = deepcopy(self.core)
        self._prune()

    def _prune(self):
        cutoff = self.core.epoch_ns-self.core.policy.history_ns
        while self.gyros and (self.gyros[0].end_ns <= cutoff
                             or len(self.gyros)+len(self.events) > self.core.policy.max_events):
            first = self.gyros.pop(0)
            self.base = self.checkpoints.pop(first.end_ns)
            self.sealed_base = True
            self.events = [event for event in self.events if event.epoch_ns > first.end_ns]
            retained_ids = {event.event_id for event in self.events}
            self.dispositions = {key: value for key, value in self.dispositions.items() if key in retained_ids}

    @staticmethod
    def _key(event: VectorSample | LossEvent):
        order = 0 if isinstance(event, LossEvent) else 1 if event.sensor == "magnetic" else 2
        return event.epoch_ns, order, event.event_id

    def submit(self, event: VectorSample | LossEvent, receipt_ns: int) -> str:
        epoch(event.epoch_ns)
        receipt_ns = epoch(receipt_ns)
        reason = None
        if receipt_ns < event.epoch_ns or receipt_ns < self.core.epoch_ns:
            reason = "receipt_before_acquisition_or_output"
        elif event.epoch_ns > self.core.epoch_ns:
            reason = "future_or_missing_gyro_coverage"
        elif event.epoch_ns < self.base.epoch_ns or (self.sealed_base and event.epoch_ns == self.base.epoch_ns):
            reason = "outside_history"
        elif any(old.event_id == event.event_id for old in self.events):
            reason = "duplicate_event"
        elif len(self.events)+len(self.gyros) >= self.core.policy.max_events:
            reason = "history_capacity"
        elif isinstance(event, VectorSample):
            if not event.valid:
                reason = event.invalid_reason
            else:
                try:
                    event.body_and_reference()
                except ValueError as error:
                    reason = str(error)
        if reason:
            self.ingress_rejections[reason] += 1
            self.receipts.append({"id": event.event_id, "receipt_ns": receipt_ns, "epoch_ns": event.epoch_ns, "result": reason})
            return reason
        delayed = event.epoch_ns < self.core.epoch_ns
        self.events.append(deepcopy(event))
        self.events.sort(key=self._key)
        results = self._rebuild(event.epoch_ns)
        revisions = {key: {"previous": old, "current": results[key]} for key, old in self.dispositions.items()
                     if key in results and old != results[key]}
        self.dispositions = results
        self.revision += 1
        self.replay_count += int(delayed)
        result = results[event.event_id]
        self.receipts.append({"id": event.event_id, "receipt_ns": receipt_ns, "epoch_ns": event.epoch_ns,
                              "result": result, "revision": self.revision, "replayed": delayed,
                              "revised_dispositions": revisions})
        self._prune()
        return result

    def _rebuild(self, changed_epoch_ns: int) -> dict[str, str]:
        preceding = [tick for tick in self.checkpoints if tick < changed_epoch_ns]
        state = deepcopy(self.checkpoints[max(preceding)] if preceding else self.base)
        restored_epoch = state.epoch_ns
        # A checkpoint includes all events at its epoch. The original unsealed
        # base is the sole exception: it precedes initial-epoch measurements.
        initial_base = not preceding and not self.sealed_base
        pending = iter(event for event in self.events if event.epoch_ns > restored_epoch
                       or (initial_base and event.epoch_ns == restored_epoch))
        event = next(pending, None)
        results = {key: value for key, value in self.dispositions.items()
                   if any(item.event_id == key and (item.epoch_ns < restored_epoch or
                          (not initial_base and item.epoch_ns == restored_epoch)) for item in self.events)}
        self.checkpoints = {tick: checkpoint for tick, checkpoint in self.checkpoints.items() if tick <= restored_epoch}

        def apply(item):
            if isinstance(item, LossEvent):
                state.initialized = False
                state.pending.clear()
                state.last_rate = None
                state.rate_support_ns = None
                state.instantaneous_attitude_rank = 0
                return "declared_lost"
            return state.observe(item)

        while event is not None and event.epoch_ns == state.epoch_ns:
            results[event.event_id] = apply(event)
            event = next(pending, None)
        for gyro in self.gyros:
            if gyro.end_ns <= restored_epoch:
                continue
            while event is not None and event.epoch_ns <= gyro.end_ns:
                if event.epoch_ns > state.epoch_ns:
                    state.propagate(event.epoch_ns, gyro.rate_B_rad_s)
                results[event.event_id] = apply(event)
                event = next(pending, None)
            if state.epoch_ns < gyro.end_ns:
                state.propagate(gyro.end_ns, gyro.rate_B_rad_s)
            self.checkpoints[gyro.end_ns] = deepcopy(state)
        if event is not None:
            raise ValueError("event lacks gyro coverage")
        self.core = state
        return results
