"""Phase 8B-1 TAM MEASUREMENT / VALIDITY MODEL FRAMEWORK, 2026-10-02.

PARAMETRIC DEVELOPMENT MODEL / NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED.
C_SB maps B to S. Canonical field unit is Tesla:
y_S = clip((diag(scale)+cross_axis) C_SB B_local_B + bias_S + noise_S).
Reconstructed y_B = C_SB.T y_S; it is a measurement, never labeled truth.
B_local_B is the insertion point for future environment + spacecraft + coil
fields. No contamination amplitude, decay, aperture or filter is inferred here.
Value production and eligibility are separate. TEST_ONLY_SCHEDULER quiet evidence
is deliberately not a claim of measured physical settling or current decay.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
import hashlib
import json

import numpy as np
from numpy.typing import ArrayLike

from hs2_sim_config import Parameter

SCOPE = "TAM MEASUREMENT / VALIDITY MODEL FRAMEWORK / PARAMETRIC DEVELOPMENT MODEL / NOT INSTALLED HS-2 PERFORMANCE / NOT FLIGHT VALIDATED"
IDENTITY = ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
ZERO_MATRIX = ((0.,0.,0.),)*3
PROFILES = ("IDEAL_REGRESSION", "TEST_BIAS_ONLY", "TEST_SCALE_ONLY", "TEST_NOISE_ONLY")


def fixture(value, units, frame, treatment):
    return Parameter(value, units, "ASSUMED", "Phase 8B-1 software fixture; tam_sensor_model.py",
        "2026-10-02", frame, "TEST-ONLY: "+treatment)


def epoch(value):
    if type(value) is not int or value < 0:
        raise ValueError("TAM timestamps require nonnegative integer simulation ns")
    return value


def numeric(value, shape):
    a = np.asarray(value)
    if a.shape != shape or a.dtype.kind not in "fiu" or not np.isfinite(a).all():
        raise ValueError(f"finite numeric TAM value of shape {shape} required")
    return a.astype(float)


def microtesla_to_tesla(value: ArrayLike):
    return numeric(value, (3,))*1e-6


def tesla_to_microtesla(value: ArrayLike):
    return numeric(value, (3,))*1e6


@dataclass(frozen=True)
class TAMConfig:
    profile: str = "IDEAL_REGRESSION"
    runtime_usable_for_flight: bool = False
    mounting_C_SB: Parameter = fixture(IDENTITY, "1", "B to S", "proper mounting rotation; installed transform TBD")
    bias_S: Parameter = fixture((0.,0.,0.), "T", "S", "additive offset; not measured hard-iron or remanence")
    scale: Parameter = fixture((1.,1.,1.), "1", "S", "gain; error is gain minus one")
    cross_axis: Parameter = fixture(ZERO_MATRIX, "1", "S", "off-diagonal error; diagonal is owned by scale")
    noise_sigma_S: Parameter = fixture((0.,0.,0.), "T", "S", "discrete per-acquisition sigma; no vendor density conversion")
    range_S: Parameter = fixture(None, "T", "S", "None disables clipping; no installed range selected")
    latency_ns: Parameter = fixture(0, "ns", "simulation clock", "acquisition-to-publication availability; not transport/processing delay")
    seed: Parameter = fixture(None, "1", "software RNG", "PCG64 seed; zero noise consumes no draws")

    def __post_init__(self):
        if not isinstance(self.profile, str) or not self.profile.strip() or self.runtime_usable_for_flight is not False:
            raise ValueError("named TEST-ONLY TAM configuration required")
        contract = dict(mounting_C_SB=("1","B to S"), bias_S=("T","S"), scale=("1","S"),
            cross_axis=("1","S"), noise_sigma_S=("T","S"), range_S=("T","S"),
            latency_ns=("ns","simulation clock"), seed=("1","software RNG"))
        for name,(units,frame) in contract.items():
            p = getattr(self, name)
            if not isinstance(p, Parameter) or p.units != units or p.frame != frame or not p.notes.strip():
                raise ValueError(f"TAM {name} requires provenance/treatment, units {units}, frame {frame}")
        c = numeric(self.mounting_C_SB.value, (3,3))
        if not np.allclose(c @ c.T, np.eye(3), rtol=0, atol=1e-12) or abs(np.linalg.det(c)-1.) > 1e-12:
            raise ValueError("C_SB must be a proper B-to-S rotation")
        numeric(self.bias_S.value, (3,))
        if np.any(numeric(self.scale.value, (3,)) <= 0):
            raise ValueError("positive TAM scale required; polarity belongs in mounting")
        if np.any(np.diag(numeric(self.cross_axis.value, (3,3)))):
            raise ValueError("cross-axis diagonal must be zero")
        sigma = numeric(self.noise_sigma_S.value, (3,))
        if np.any(sigma < 0):
            raise ValueError("nonnegative TAM sigma required")
        if self.range_S.value is not None and np.any(numeric(self.range_S.value, (3,)) <= 0):
            raise ValueError("positive TAM half-range required")
        epoch(self.latency_ns.value)
        if self.seed.value is not None:
            epoch(self.seed.value)
        if np.any(sigma) and self.seed.value is None:
            raise ValueError("TAM noise requires explicit seed")

    def to_dict(self):
        return dict(profile=self.profile, runtime_usable_for_flight=False, scope=SCOPE,
            **{f.name:asdict(getattr(self, f.name)) for f in fields(self) if isinstance(getattr(self, f.name), Parameter)})

    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True, allow_nan=False).encode()).hexdigest()

    def validate_live(self):
        # Only ideal shadow integration is authorized in 8B-1. Isolated profiles
        # cannot enter the live bridge merely by renaming a perturbed config.
        if self != TAMConfig():
            raise ValueError("live TAM permits only unchanged IDEAL_REGRESSION; perturbations/delay are isolated-only")


def profile_config(name="IDEAL_REGRESSION"):
    cfg = TAMConfig(profile=name)
    if name == "IDEAL_REGRESSION":
        return cfg
    if name == "TEST_BIAS_ONLY":
        return replace(cfg, bias_S=fixture((1e-6,-2e-6,3e-6), "T", "S", "synthetic unit-test offset, not HS-2 bias"))
    if name == "TEST_SCALE_ONLY":
        return replace(cfg, scale=fixture((1.01,.98,1.03), "1", "S", "synthetic gain errors"))
    if name == "TEST_NOISE_ONLY":
        return replace(cfg, noise_sigma_S=fixture((1e-7,2e-7,3e-7), "T", "S", "synthetic discrete sigma, not vendor density"),
            seed=fixture(8201, "1", "software RNG", "reproducible isolated software fixture"))
    raise ValueError(f"unknown TAM profile {name!r}")


@dataclass(frozen=True)
class CleanlinessContext:
    """Explicit acquisition evidence; command axes differ from effective B axes.

    A is per actuator axis. None current means unmeasured, never measured zero.
    Only ASSUMED scheduler eligibility is implemented, not hardware acceptance.
    """
    epoch_ns: int
    phase: str
    cycle_index: int
    commanded_axis_dipoles_Am2: tuple[float, ...]
    effective_dipole_B_Am2: tuple[float, ...]
    quiet_elapsed_ns: int | None
    required_quiet_ns: int | None
    acquisition_eligible: bool
    evidence_scope: str
    source: str
    cycle_fingerprint: str
    coil_current_A: tuple[float, ...] | None = None


@dataclass(frozen=True)
class TAMValidity:
    valid: bool
    reasons: tuple[str, ...]


def cleanliness(context: CleanlinessContext | None, acquisition_ns: int) -> list[str]:
    reasons = []
    if context is None:
        return ["missing_cleanliness_context"]
    try:
        if epoch(context.epoch_ns) != acquisition_ns:
            reasons.append("stale_cleanliness_context")
        epoch(context.cycle_index)
        if context.phase != "SAMPLE" or context.acquisition_eligible is not True:
            reasons.append("cycle_acquisition_ineligible")
        if context.evidence_scope != "TEST_ONLY_SCHEDULER" or not context.source.strip() or not context.cycle_fingerprint.strip():
            reasons.append("quiet_evidence_unestablished")
        if context.quiet_elapsed_ns is None or context.required_quiet_ns is None:
            reasons.append("missing_quiet_evidence")
        elif epoch(context.quiet_elapsed_ns) < epoch(context.required_quiet_ns):
            reasons.append("insufficient_quiet_history")
        if np.any(numeric(context.commanded_axis_dipoles_Am2, (3,))) or np.any(numeric(context.effective_dipole_B_Am2, (3,))):
            reasons.append("nonzero_command_or_effective_dipole")
        if context.coil_current_A is not None and np.any(numeric(context.coil_current_A, (3,))):
            reasons.append("coil_current_nonzero")
    except (ValueError, TypeError, AttributeError):
        reasons.append("invalid_cleanliness_metadata")
    return reasons


@dataclass(frozen=True)
class TAMSample:
    truth_epoch_ns: int
    acquisition_ns: int
    publication_ns: int
    measurement_S_T: tuple[float, ...] | None
    reconstructed_B_T: tuple[float, ...] | None
    noise_S_T: tuple[float, ...] | None
    saturated_axes: tuple[bool, ...]
    valid: bool  # Acquisition eligibility; future publication is not availability yet.
    rejection_reasons: tuple[str, ...]
    configuration_fingerprint: str
    field_source: str
    cleanliness_context: CleanlinessContext | None
    output_frame: str = "S; reconstructed_B_T separately in B"
    units: str = "T"

    def usable_at(self, processing_ns: int, *, maximum_age_ns: int, expected_fingerprint: str) -> TAMValidity:
        now, limit = epoch(processing_ns), epoch(maximum_age_ns)
        reasons = list(self.rejection_reasons)
        if now < self.publication_ns:
            reasons.append("not_yet_published")
        if now < self.acquisition_ns or now-self.acquisition_ns > limit:
            reasons.append("invalid_sample_age")
        if self.configuration_fingerprint != expected_fingerprint:
            reasons.append("configuration_fingerprint_mismatch")
        return TAMValidity(self.valid and not reasons, tuple(reasons))


class TAMModel:
    def __init__(self, config: TAMConfig):
        if not isinstance(config, TAMConfig):
            raise ValueError("explicit TAMConfig required")
        self.config = config
        self.c = numeric(config.mounting_C_SB.value, (3,3))
        self.gain = np.diag(numeric(config.scale.value, (3,)))+numeric(config.cross_axis.value, (3,3))
        self.bias, self.sigma = numeric(config.bias_S.value, (3,)), numeric(config.noise_sigma_S.value, (3,))
        self.limit = None if config.range_S.value is None else numeric(config.range_S.value, (3,))
        self.rng = np.random.Generator(np.random.PCG64(config.seed.value)) if np.any(self.sigma) else None
        self.last_acquisition_ns = -1
        self.identity = np.array_equal(self.c, np.eye(3)) and np.array_equal(self.gain, np.eye(3)) and not np.any(self.bias) and self.rng is None and self.limit is None

    def acquire(self, local_field_B_T: ArrayLike, *, truth_epoch_ns: int, acquisition_ns: int,
                context: CleanlinessContext | None, field_source: str) -> TAMSample:
        tick, truth_tick = epoch(acquisition_ns), epoch(truth_epoch_ns)
        if tick <= self.last_acquisition_ns:
            raise ValueError("TAM acquisitions must advance; no duplicate RNG consumption")
        self.last_acquisition_ns = tick
        reasons = cleanliness(context, tick)
        if truth_tick != tick:
            reasons.append("truth_acquisition_epoch_mismatch")
        if not isinstance(field_source, str) or not field_source.strip():
            reasons.append("missing_field_provenance")
        measured_s = measured_b = noise = None
        saturated = np.zeros(3, dtype=bool)
        try:
            truth = numeric(local_field_B_T, (3,))
            noise = np.zeros(3) if self.rng is None else self.rng.standard_normal(3)*self.sigma
            with np.errstate(over="ignore", invalid="ignore"):
                raw = truth.copy() if self.identity else self.gain @ (self.c @ truth)+self.bias+noise
                if not np.isfinite(raw).all():
                    reasons.append("nonfinite_measurement")
                else:
                    saturated = np.zeros(3, dtype=bool) if self.limit is None else np.abs(raw) > self.limit
                    measured_s = raw if self.limit is None else np.clip(raw, -self.limit, self.limit)
                    measured_b = measured_s.copy() if self.identity else self.c.T @ measured_s
                    if not np.isfinite(measured_b).all():
                        measured_b = None
                        reasons.append("nonfinite_reconstruction")
                    if np.any(saturated):
                        reasons.append("saturated")
        except (ValueError, TypeError):
            reasons.append("invalid_true_field")
        return TAMSample(truth_tick, tick, tick+self.config.latency_ns.value,
            None if measured_s is None else tuple(measured_s), None if measured_b is None else tuple(measured_b),
            None if noise is None else tuple(noise), tuple(bool(x) for x in saturated), not reasons, tuple(reasons),
            self.config.fingerprint(), field_source, context)
