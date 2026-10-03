"""8C-1 CSS / SUN-VECTOR MEASUREMENT FRAMEWORK, 2026-10-02.

PARAMETRIC DEVELOPMENT MODEL / NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.
Direction points from spacecraft toward Sun. C_BN maps N to B; C_SB maps B
to each channel's S. Generic cosine response units are dimensionless normalized
response, NOT volts or a TensorCSS transfer function. Truth, channel readings,
and inferred direction are distinct records. No Basilisk/actuator dependency.

IDEAL_REGRESSION preserves the old unnormalized dimensionless vector sent to
MEKF (which normalizes it), and is explicitly NOT a physical array. The array
uses calibrated positive-front/FOV-eligible rows in an unconstrained linear
least-squares fit, then normalizes. Eligibility uses truth incidence in this
development model; a flight channel-selection algorithm is not established.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np
from numpy.typing import ArrayLike

from hs2_sim_config import Parameter

SCOPE = "CSS / SUN-VECTOR MEASUREMENT FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL / NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"
IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
PROFILES = ("IDEAL_REGRESSION", "TEST_ARRAY_GEOMETRY")
RESPONSE_UNITS = "dimensionless normalized response"
GEOMETRY_EPS = 16 * np.finfo(float).eps  # Dot/FOV arithmetic only, not a shadow or signal threshold.


def fixture(value, units, frame, treatment):
    return Parameter(value, units, "ASSUMED", "Phase 8C-1 software fixture; sun_sensor_model.py",
                     "2026-10-02", frame, "TEST-ONLY: " + treatment)


def epoch(value):
    if type(value) is not int or value < 0:
        raise ValueError("Sun epochs require nonnegative integer simulation ns")
    return value


def numeric(value: ArrayLike, shape):
    array = np.asarray(value)
    if array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
        raise ValueError(f"finite numeric Sun value of shape {shape} required")
    return array.astype(float)


def rotation(value):
    c = numeric(value, (3, 3))
    if not np.allclose(c @ c.T, np.eye(3), atol=1e-12, rtol=0) or abs(np.linalg.det(c)-1) > 1e-12:
        raise ValueError("proper rotation required")
    return c


def unit(value):
    v = numeric(value, (3,))
    norm = float(np.linalg.norm(v))
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("nonzero finite Sun direction required")
    return v / norm


def parameter(p, units, frame):
    if not isinstance(p, Parameter) or p.units != units or p.frame != frame or not p.notes.strip():
        raise ValueError(f"Sun parameter needs provenance/treatment, units {units}, frame {frame}")
    return p.value


@dataclass(frozen=True)
class CSSChannel:
    identifier: str
    normal_S: Parameter = fixture((0., 0., 1.), "1", "S_i", "unit normal; installed channel axes TBD")
    mounting_C_SB: Parameter = fixture(IDENTITY, "1", "B to S_i", "proper channel mounting; installed transform TBD")
    gain: Parameter = fixture(1., "1", "channel", "forward response gain")
    offset: Parameter = fixture(0., RESPONSE_UNITS, "channel", "forward additive response offset")
    calibration_gain: Parameter = fixture(1., "1", "channel", "gain known to reconstruction, independently configurable")
    calibration_offset: Parameter = fixture(0., RESPONSE_UNITS, "channel", "offset known to reconstruction")
    half_fov_rad: Parameter = fixture(math.pi/2, "rad", "S_i", "generic front hemisphere; NOT TensorCSS FOV")
    noise_sigma: Parameter = fixture(0., RESPONSE_UNITS, "channel", "discrete independent per-acquisition noise; NOT vendor noise")
    maximum_response: Parameter = fixture(None, RESPONSE_UNITS, "channel", "None disables upper clipping; no installed voltage range")

    def __post_init__(self):
        if not isinstance(self.identifier, str) or not self.identifier.strip():
            raise ValueError("channel identity required")
        n = numeric(parameter(self.normal_S, "1", "S_i"), (3,))
        if abs(np.linalg.norm(n)-1) > 1e-12:
            raise ValueError("channel normal must be unit length")
        rotation(parameter(self.mounting_C_SB, "1", "B to S_i"))
        for name in ("gain", "calibration_gain"):
            if float(numeric(parameter(getattr(self, name), "1", "channel"), ())) <= 0:
                raise ValueError("positive response/calibration gain required")
        for name in ("offset", "calibration_offset", "noise_sigma"):
            value = float(numeric(parameter(getattr(self, name), RESPONSE_UNITS, "channel"), ()))
            if name == "noise_sigma" and value < 0:
                raise ValueError("nonnegative noise sigma required")
        if not 0 < float(numeric(parameter(self.half_fov_rad, "rad", "S_i"), ())) <= math.pi/2:
            raise ValueError("generic cosine channel FOV must be in (0,pi/2]")
        limit = parameter(self.maximum_response, RESPONSE_UNITS, "channel")
        if limit is not None and float(numeric(limit, ())) <= 0:
            raise ValueError("positive clipping limit required")

    def normal_B(self):
        return rotation(self.mounting_C_SB.value).T @ numeric(self.normal_S.value, (3,))


@dataclass(frozen=True)
class SunConfig:
    profile: str = "IDEAL_REGRESSION"
    mode: str = "DIRECT_VECTOR"
    channels: tuple[CSSChannel, ...] = ()
    runtime_usable_for_flight: bool = False
    latency_ns: Parameter = fixture(0, "ns", "simulation clock", "acquisition-to-channel publication; reconstruction at publication")
    minimum_singular_ratio: Parameter = fixture(1e-10, "1", "reconstruction", "numerical conditioning floor only; no flight quality criterion")
    seed: Parameter = fixture(None, "1", "software RNG", "PCG64 seed; zero noise consumes no draws")

    def __post_init__(self):
        if not isinstance(self.profile, str) or not self.profile.strip() or self.runtime_usable_for_flight is not False:
            raise ValueError("named non-flight Sun profile required")
        if self.mode not in ("DIRECT_VECTOR", "CSS_ARRAY") or not isinstance(self.channels, tuple):
            raise ValueError("explicit Sun mode and immutable channel tuple required")
        if any(not isinstance(c, CSSChannel) for c in self.channels):
            raise ValueError("CSSChannel configuration required")
        if (self.mode == "DIRECT_VECTOR" and self.channels) or (self.mode == "CSS_ARRAY" and not self.channels):
            raise ValueError("direct mode has no array; CSS mode requires explicit channels")
        if len({c.identifier for c in self.channels}) != len(self.channels):
            raise ValueError("unique channel identities required")
        epoch(parameter(self.latency_ns, "ns", "simulation clock"))
        if not 0 < float(numeric(parameter(self.minimum_singular_ratio, "1", "reconstruction"), ())) < 1:
            raise ValueError("explicit numerical singular-ratio floor required")
        seed = parameter(self.seed, "1", "software RNG")
        if seed is not None:
            epoch(seed)
        if any(c.noise_sigma.value for c in self.channels) and seed is None:
            raise ValueError("nonzero channel noise requires explicit seed")

    def to_dict(self):
        return dict(scope=SCOPE, status="ASSUMED", revision="2026-10-02",
                    source="Phase 8C-1 software fixture; sun_sensor_model.py", **asdict(self))

    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True, allow_nan=False).encode()).hexdigest()

    def validate_live(self):
        if self != SunConfig():
            raise ValueError("live Sun permits only unchanged IDEAL_REGRESSION; array/error/latency configurations are isolated-only")


def profile_config(name="IDEAL_REGRESSION"):
    if name == "IDEAL_REGRESSION":
        return SunConfig()
    if name == "TEST_ARRAY_GEOMETRY":
        # Eight cube-corner normals: synthetic algebra fixture, NOT an HS-2 count.
        normals = [(x/math.sqrt(3), y/math.sqrt(3), z/math.sqrt(3))
                   for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
        channels = tuple(CSSChannel(f"synthetic-{i}", normal_S=fixture(n, "1", "S_i",
                         "synthetic cube-corner direction; NOT installed geometry")) for i, n in enumerate(normals))
        return SunConfig(profile=name, mode="CSS_ARRAY", channels=channels)
    raise ValueError(f"unknown Sun profile {name!r}")


@dataclass(frozen=True)
class SunAvailability:
    epoch_ns: int
    available: bool
    source: str  # Explicit eclipse/visibility provider, not an inferred eclipse model.
    channel_occluded: tuple[bool, ...] = ()  # Empty: explicitly no per-channel mask supplied.
    reason: str = ""  # E.g. TEST-ONLY eclipse input; never silently inferred from darkness.


@dataclass(frozen=True)
class SunTruth:
    epoch_ns: int
    reference_N: tuple[float, ...]
    direction_N: tuple[float, ...]
    direction_B: tuple[float, ...]
    source: str


@dataclass(frozen=True)
class ChannelMeasurement:
    identifier: str
    normal_B: tuple[float, ...]
    incidence: float
    in_fov: bool
    illuminated: bool
    response: float | None
    noise: float | None
    saturated: bool
    valid: bool
    reasons: tuple[str, ...]
    acquisition_ns: int
    publication_ns: int
    configuration_fingerprint: str
    units: str = RESPONSE_UNITS


@dataclass(frozen=True)
class SunReconstruction:
    direction_B: tuple[float, ...] | None
    measurement_B: tuple[float, ...] | None  # Legacy direct mode retains raw dimensionless magnitude.
    valid: bool
    reasons: tuple[str, ...]
    used_channels: tuple[str, ...]
    rank: int | None
    condition_number: float | None
    method: str


@dataclass(frozen=True)
class SunSample:
    truth: SunTruth | None
    channels: tuple[ChannelMeasurement, ...]
    reconstruction: SunReconstruction
    truth_epoch_ns: int
    acquisition_ns: int
    publication_ns: int
    reconstruction_ns: int
    availability: SunAvailability
    configuration_fingerprint: str

    def usable_at(self, processing_ns, *, maximum_age_ns, expected_fingerprint):
        now, limit = epoch(processing_ns), epoch(maximum_age_ns)
        reasons = list(self.reconstruction.reasons)
        if now < self.publication_ns or now < self.reconstruction_ns:
            reasons.append("not_yet_published")
        if now < self.acquisition_ns or now-self.acquisition_ns > limit:
            reasons.append("invalid_sample_age")
        if self.configuration_fingerprint != expected_fingerprint:
            reasons.append("configuration_fingerprint_mismatch")
        return self.reconstruction.valid and not reasons, tuple(reasons)


def rejected(reasons, method, used=(), rank=None, condition=None):
    return SunReconstruction(None, None, False, tuple(reasons), tuple(used), rank, condition, method)


class SunModel:
    def __init__(self, config: SunConfig):
        if not isinstance(config, SunConfig):
            raise ValueError("explicit SunConfig required")
        self.config = config
        self.fingerprint = config.fingerprint()
        self.rng = (np.random.Generator(np.random.PCG64(config.seed.value))
                    if any(c.noise_sigma.value for c in config.channels) else None)
        self.last_acquisition_ns = -1

    def reconstruct(self, channels: tuple[ChannelMeasurement, ...], acquisition_ns: int) -> SunReconstruction:
        """Measurement-only least squares. No Sun truth or attitude input here.

        Known offsets/gains are removed; rows are channel normals in B.
        Require >=3 independent valid rows, singular ratio above configured
        numerical floor and finite nonzero solution. Invalid channels are excluded;
        malformed/nonfinite purportedly valid channels reject the reconstruction.
        This positive-incidence linear inversion is not a general CSS flight solver.
        """
        tick = epoch(acquisition_ns)
        method = "calibrated_linear_least_squares"
        if len(channels) != len(self.config.channels):
            return rejected(("invalid_channel_metadata",), method)
        normals, values, used = [], [], []
        for measurement, config in zip(channels, self.config.channels):
            if (measurement.identifier != config.identifier or measurement.acquisition_ns != tick
                    or type(measurement.acquisition_ns) is not int or type(measurement.publication_ns) is not int
                    or any(type(flag) is not bool for flag in (measurement.valid, measurement.in_fov, measurement.illuminated, measurement.saturated))
                    or measurement.publication_ns != tick+self.config.latency_ns.value
                    or measurement.configuration_fingerprint != self.fingerprint
                    or measurement.units != RESPONSE_UNITS
                    or not np.array_equal(measurement.normal_B, config.normal_B())):
                return rejected(("invalid_channel_metadata",), method)
            if not measurement.valid:
                continue
            if measurement.reasons or not measurement.in_fov or not measurement.illuminated or measurement.saturated:
                return rejected(("invalid_channel_metadata",), method)
            if measurement.response is None or not math.isfinite(measurement.response):
                return rejected(("nonfinite_channel_value",), method)
            value = (measurement.response-config.calibration_offset.value)/config.calibration_gain.value
            if not math.isfinite(value):
                return rejected(("nonfinite_calibrated_value",), method)
            normals.append(config.normal_B())
            values.append(value)
            used.append(config.identifier)
        if not used:
            reason = "no_illuminated_channels" if not any(c.illuminated for c in channels) else "no_usable_channels"
            return rejected((reason,), method)
        a = np.array(normals)
        solution, _, rank, singular = np.linalg.lstsq(a, np.array(values), rcond=None)
        if rank < 3:
            return rejected(("insufficient_independent_geometry",), method, used, int(rank))
        condition = float(singular[0]/singular[-1])
        if singular[-1]/singular[0] <= self.config.minimum_singular_ratio.value:
            return rejected(("ill_conditioned_geometry",), method, used, int(rank), condition)
        try:
            direction = tuple(unit(solution))
        except ValueError:
            return rejected(("degenerate_reconstruction_norm",), method, used, int(rank), condition)
        return SunReconstruction(direction, direction, True, (), tuple(used), int(rank), condition, method)

    def acquire(self, reference_N: ArrayLike, c_bn: ArrayLike, *, truth_epoch_ns: int,
                acquisition_ns: int, availability: SunAvailability, source: str) -> SunSample:
        tick, truth_tick = epoch(acquisition_ns), epoch(truth_epoch_ns)
        if tick <= self.last_acquisition_ns:
            raise ValueError("Sun acquisitions must advance; no duplicate RNG consumption")
        self.last_acquisition_ns = tick
        publication = tick+self.config.latency_ns.value
        reasons = []
        if truth_tick != tick:
            reasons.append("truth_acquisition_epoch_mismatch")
        if not isinstance(source, str) or not source.strip():
            reasons.append("missing_truth_provenance")
        if (not isinstance(availability, SunAvailability) or type(availability.epoch_ns) is not int
                or availability.epoch_ns != tick or type(availability.available) is not bool
                or not isinstance(availability.source, str) or not availability.source.strip()
                or not isinstance(availability.channel_occluded, tuple)
                or (availability.channel_occluded and len(availability.channel_occluded) != len(self.config.channels))
                or any(type(x) is not bool for x in availability.channel_occluded)):
            reasons.append("invalid_availability_metadata")
        elif not availability.available:
            reasons.append("sun_unavailable")
        truth, raw_b = None, None
        try:
            ref = numeric(reference_N, (3,))
            raw_b = rotation(c_bn) @ ref
            truth = SunTruth(truth_tick, tuple(ref), tuple(unit(ref)), tuple(unit(raw_b)), source)
        except (ValueError, TypeError):
            reasons.append("invalid_sun_truth_or_frame")
        channels = []
        if self.config.mode == "DIRECT_VECTOR":
            if reasons:
                reconstruction = rejected(reasons, "development_direct_vector")
            else:
                assert truth is not None and raw_b is not None
                reconstruction = SunReconstruction(truth.direction_B, tuple(raw_b), True, (), (), None, None, "development_direct_vector")
        else:
            if truth is not None:
                for i, config in enumerate(self.config.channels):
                    n = config.normal_B()
                    q = float(np.clip(n @ np.array(truth.direction_B), -1., 1.))
                    lit = bool(q > 0)
                    fov = bool(q + GEOMETRY_EPS >= math.cos(config.half_fov_rad.value))
                    channel_reasons = list(reasons)
                    if not lit:
                        channel_reasons.append("not_illuminated")
                    if not fov:
                        channel_reasons.append("outside_fov")
                    if not reasons and availability.channel_occluded and availability.channel_occluded[i]:
                        channel_reasons.append("occluded")
                    response = noise = None
                    saturated = False
                    if not reasons:
                        # Unilluminated/FOV-excluded/occluded channels report dark
                        # offset/noise, but are never used as illuminated rows.
                        signal = max(0., q) if not channel_reasons else 0.
                        noise = float(self.rng.standard_normal()*config.noise_sigma.value) if self.rng is not None else 0.
                        response = config.gain.value*signal + config.offset.value + noise
                        if not math.isfinite(response):
                            response = None
                            channel_reasons.append("nonfinite_channel_value")
                        elif config.maximum_response.value is not None and response > config.maximum_response.value:
                            response = float(config.maximum_response.value)
                            saturated = True
                            channel_reasons.append("saturated")
                    channels.append(ChannelMeasurement(config.identifier, tuple(n), q, fov, lit, response, noise,
                        saturated, not channel_reasons, tuple(channel_reasons), tick, publication, self.fingerprint))
            reconstruction = rejected(reasons, "calibrated_linear_least_squares") if reasons else self.reconstruct(tuple(channels), tick)
        return SunSample(truth, tuple(channels), reconstruction, truth_tick, tick, publication,
                         publication, availability, self.fingerprint)
