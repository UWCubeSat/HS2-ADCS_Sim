"""Phase 8A-1 parametric DEVELOPMENT gyro, 2026-10-01; NOT flight performance.

omega_S = C_SB omega_B; y_S = (diag(scale) + cross_axis) omega_S + bias_S + n_S;
clip in S if enabled; y_B = C_SB.T y_S. C_SB is a proper B-to-S rotation.
The MEKF alone subtracts its estimated B-frame bias; this model never does.
n_S is independent Gaussian noise per acquired sample with explicit rad/s sigma,
NOT a conversion of vendor noise density. Interpolating samples later correlates
subinterval errors; it does not create independent higher-rate observations.

Samples are point acquisitions on offset + k*period, integer simulation ns.
Publication availability is acquisition + latency. The caller owns delivery;
no queue, aperture/filter, quantization, temperature or bias drift is inferred.
Delayed/asynchronous gyro ingestion is NOT added to the existing MEKF adapter.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
import hashlib
import json

import numpy as np
from numpy.typing import ArrayLike

from hs2_sim_config import Parameter

SCOPE = "GYRO SENSOR MODEL FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL / NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"
IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
ZERO_MATRIX = ((0., 0., 0.),)*3
PROFILES = ("IDEAL_REGRESSION", "TEST_BIAS_ONLY", "TEST_SCALE_ONLY", "TEST_NOISE_ONLY", "TEST_DELAYED_SAMPLE")


def fixture(value, units, frame, treatment):
    return Parameter(value, units, "ASSUMED", "Phase 8A-1 software fixture; gyro_sensor_model.py",
                     "2026-10-01", frame, "TEST-ONLY: "+treatment)


def epoch(value):
    if type(value) is not int or value < 0:
        raise ValueError("gyro epochs/periods require nonnegative integer simulation ns")
    return value


def numeric(value, shape):
    a = np.asarray(value)
    if a.shape != shape or a.dtype.kind not in "fiu" or not np.isfinite(a).all():
        raise ValueError(f"finite numeric gyro value of shape {shape} required")
    return a.astype(float)


@dataclass(frozen=True)
class GyroConfig:
    profile: str = "IDEAL_REGRESSION"
    runtime_usable_for_flight: bool = False
    mounting_C_SB: Parameter = fixture(IDENTITY, "1", "B to S", "proper mounting rotation; installed transform TBD")
    bias_S: Parameter = fixture((0., 0., 0.), "rad/s", "S", "constant additive bias, not in-run stability")
    scale: Parameter = fixture((1., 1., 1.), "1", "S", "per-axis gain; scale error is gain minus one")
    cross_axis: Parameter = fixture(ZERO_MATRIX, "1", "S", "off-diagonal additive gains; diagonal must be zero")
    noise_sigma_S: Parameter = fixture((0., 0., 0.), "rad/s", "S", "independent discrete-sample Gaussian standard deviations")
    sample_period_ns: Parameter = fixture(100_000_000, "ns", "simulation clock", "existing 0.1 s development bridge cadence; not flight ODR")
    sample_offset_ns: Parameter = fixture(0, "ns", "simulation clock", "first acquisition; no sample before this epoch")
    latency_ns: Parameter = fixture(0, "ns", "simulation clock", "fixed acquisition-to-publication availability delay; no hardware claim")
    range_S: Parameter = fixture(None, "rad/s", "S", "None disables clipping; enabled positive per-axis half-ranges are test-only")
    seed: Parameter = fixture(None, "1", "software RNG", "PCG64 seed required only for nonzero noise; zero noise consumes no RNG draws")

    def __post_init__(self):
        if not isinstance(self.profile, str) or not self.profile.strip() or self.runtime_usable_for_flight is not False:
            raise ValueError("named development-only gyro configuration required")
        expected_units = dict(mounting_C_SB="1", bias_S="rad/s", scale="1", cross_axis="1",
            noise_sigma_S="rad/s", sample_period_ns="ns", sample_offset_ns="ns", latency_ns="ns", range_S="rad/s", seed="1")
        expected_frames = dict(mounting_C_SB="B to S", bias_S="S", scale="S", cross_axis="S",
            noise_sigma_S="S", sample_period_ns="simulation clock", sample_offset_ns="simulation clock",
            latency_ns="simulation clock", range_S="S", seed="software RNG")
        for name, units in expected_units.items():
            p = getattr(self, name)
            if not isinstance(p, Parameter) or p.units != units or p.frame != expected_frames[name] or not p.notes.strip():
                raise ValueError(f"gyro {name} requires provenance/treatment, units {units}, frame {expected_frames[name]}")
        c = numeric(self.mounting_C_SB.value, (3, 3))
        if not np.allclose(c @ c.T, np.eye(3), rtol=0, atol=1e-12) or abs(np.linalg.det(c)-1.) > 1e-12:
            raise ValueError("C_SB must be a right-handed orthonormal B-to-S DCM")
        numeric(self.bias_S.value, (3,))
        if np.any(numeric(self.scale.value, (3,)) <= 0):
            raise ValueError("positive scale gains required; polarity belongs in C_SB")
        if np.any(np.diag(numeric(self.cross_axis.value, (3, 3)))):
            raise ValueError("cross-axis diagonal must be zero; use scale for diagonal gains")
        sigma = numeric(self.noise_sigma_S.value, (3,))
        if np.any(sigma < 0):
            raise ValueError("nonnegative discrete noise sigma required")
        if epoch(self.sample_period_ns.value) == 0:
            raise ValueError("positive sample period required")
        epoch(self.sample_offset_ns.value)
        epoch(self.latency_ns.value)
        if self.range_S.value is not None and np.any(numeric(self.range_S.value, (3,)) <= 0):
            raise ValueError("positive gyro range half-widths required")
        if self.seed.value is not None:
            epoch(self.seed.value)
        if np.any(sigma) and self.seed.value is None:
            raise ValueError("nonzero noise requires an explicit reproducible seed")

    def to_dict(self):
        result = {"profile": self.profile, "runtime_usable_for_flight": False, "scope": SCOPE}
        for f in fields(self):
            p = getattr(self, f.name)
            if isinstance(p, Parameter):
                result[f.name] = asdict(p) | {"treatment": p.notes}
        return result

    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True, allow_nan=False).encode()).hexdigest()

    def validate_live(self, step_ns: int):
        if self.sample_period_ns.value != step_ns or self.sample_offset_ns.value != 0 or self.latency_ns.value != 0:
            raise ValueError("live gyro requires existing task cadence, zero offset/latency; delayed gyro is isolated-only")


def profile_config(name="IDEAL_REGRESSION"):
    config = GyroConfig(profile=name)
    if name == "IDEAL_REGRESSION":
        return config
    if name == "TEST_BIAS_ONLY":
        return replace(config, bias_S=fixture((.003, -.002, .001), "rad/s", "S",
            "synthetic vector reused from config/attitude_mekf_test_only.json, Phase 7C 2026-09-07; not installed bias"))
    if name == "TEST_SCALE_ONLY":
        return replace(config, scale=fixture((1.01, .98, 1.03), "1", "S", "synthetic +1%, -2%, +3% gain errors"))
    if name == "TEST_NOISE_ONLY":
        return replace(config, noise_sigma_S=fixture((.001, .002, .003), "rad/s", "S",
            "synthetic per-sample sigma; no density/bandwidth conversion"),
            seed=fixture(8101, "1", "software RNG", "repeatable PCG64 software fixture"))
    if name == "TEST_DELAYED_SAMPLE":
        return replace(config, latency_ns=fixture(200_000_000, "ns", "simulation clock",
            "synthetic 0.2 s publication delay, isolated model only; live adapter does not accept delayed gyro"))
    raise ValueError(f"unknown gyro profile {name!r}")


@dataclass(frozen=True)
class GyroSample:
    acquisition_ns: int
    publication_ns: int  # availability, not proof of caller delivery
    measurement_S_rad_s: tuple[float, ...] | None
    measurement_B_rad_s: tuple[float, ...] | None
    noise_S_rad_s: tuple[float, ...] | None
    saturated_axes: tuple[bool, ...]
    valid: bool
    reason: str
    configuration_fingerprint: str

    def age_ns(self, processing_ns: int):
        if epoch(processing_ns) < self.publication_ns:
            raise ValueError("measurement is not available before publication")
        return processing_ns-self.acquisition_ns


class GyroModel:
    def __init__(self, config: GyroConfig):
        if not isinstance(config, GyroConfig):
            raise ValueError("GyroModel requires explicit GyroConfig")
        self.config = config
        self.c = numeric(config.mounting_C_SB.value, (3, 3))
        self.gain = np.diag(numeric(config.scale.value, (3,))) + numeric(config.cross_axis.value, (3, 3))
        self.bias, self.sigma = numeric(config.bias_S.value, (3,)), numeric(config.noise_sigma_S.value, (3,))
        self.limit = None if config.range_S.value is None else numeric(config.range_S.value, (3,))
        self.rng = np.random.Generator(np.random.PCG64(config.seed.value)) if np.any(self.sigma) else None
        self.identity = (np.array_equal(self.c, np.eye(3)) and np.array_equal(self.gain, np.eye(3))
                         and not np.any(self.bias) and self.rng is None and self.limit is None)
        self.last_call_ns = -1
        self.fingerprint = config.fingerprint()

    def acquire(self, true_rate_B_rad_s: ArrayLike, acquisition_ns: int) -> GyroSample | None:
        tick = epoch(acquisition_ns)
        if tick <= self.last_call_ns:
            raise ValueError("gyro observation calls must advance; duplicate/out-of-order sample rejected")
        self.last_call_ns = tick
        offset, period = self.config.sample_offset_ns.value, self.config.sample_period_ns.value
        if tick < offset or (tick-offset) % period:
            return None  # No acquisition and no RNG draw off the configured grid.
        publication = tick+self.config.latency_ns.value
        try:
            truth = numeric(true_rate_B_rad_s, (3,))
        except ValueError:
            return GyroSample(tick, publication, None, None, None, (False,)*3, False,
                              "invalid_true_rate", self.fingerprint)
        noise = np.zeros(3) if self.rng is None else self.rng.normal(size=3)*self.sigma
        if self.identity:
            # Preserve all existing floating-point endpoint values, including
            # signed zero. The bridge's existing interpolation is untouched.
            measured_s, measured_b = truth.copy(), truth.copy()
            saturated = np.zeros(3, dtype=bool)
        else:
            measured_s = self.gain @ (self.c @ truth) + self.bias + noise
            saturated = np.zeros(3, dtype=bool) if self.limit is None else np.abs(measured_s) > self.limit
            if self.limit is not None:
                measured_s = np.clip(measured_s, -self.limit, self.limit)
            measured_b = self.c.T @ measured_s
        finite = bool(np.isfinite(measured_s).all() and np.isfinite(measured_b).all())
        valid = finite and not bool(np.any(saturated))
        reason = "" if valid else "gyro_saturated" if finite else "nonfinite_gyro_measurement"
        return GyroSample(tick, publication, tuple(measured_s) if finite else None,
            tuple(measured_b) if finite else None, tuple(noise), tuple(bool(x) for x in saturated),
            valid, reason, self.fingerprint)
