"""Phase 7F-1 CONTROL SNAPSHOT / SCHEDULING DEVELOPMENT CONTRACT, 2026-09-30.

NO CONTROLLER AUTHORITY / NO ACTUATOR AUTHORITY / NOT FLIGHT VALIDATED.
Source: Phase 6A acquisition/compute schedule and Phase 7E navigation contract.
This unscheduled, opt-in data layer reads supplied evidence only. It does not
call a controller, publish messages, access truth, or retrieve estimator history.
All epochs are integer simulation ns; vectors are SI in declared N/B/S frames.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from attitude_navigation_consumer import Snapshot, Source, State, assess


def _epoch(value) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError("nonnegative integer ns required")
    return int(value)


def _vector(value) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError("finite three-vector required")
    return float(array[0]), float(array[1]), float(array[2])


@dataclass(frozen=True)
class CycleWindow:
    """Exact selected-cycle relationship, not a generic age tolerance.

    Supply the unchanged MagneticCycleConfig's derived integer ns and provenance.
    No default timing is selected by this module. Construction errors reject
    an invalid schedule before any input snapshot is considered.
    """
    period_ns: int
    sample_offset_ns: int
    compute_offset_ns: int
    evaluation_ns: int
    provenance: str

    def __post_init__(self):
        for value in (self.period_ns, self.sample_offset_ns, self.compute_offset_ns, self.evaluation_ns):
            _epoch(value)
        if not (0 <= self.sample_offset_ns < self.compute_offset_ns < self.period_ns):
            raise ValueError("ordered sample/compute offsets within positive period required")
        if (self.evaluation_ns % self.period_ns != self.compute_offset_ns
                or not isinstance(self.provenance, str) or not self.provenance):
            raise ValueError("evaluation must be the selected cycle's compute event with provenance")

    @property
    def sample_ns(self) -> int:
        return self.evaluation_ns-self.compute_offset_ns+self.sample_offset_ns


@dataclass(frozen=True)
class MagneticSample:
    """Stored acquisition evidence, never current truth B or a refreshed WMM.

    tam_S_T is the actual TAM payload. C_SB maps B to S, explicitly supplied by
    the existing sensor configuration. valid/quiet_valid are producer evidence,
    not a new coil model. The original acquisition event/phase are retained when
    delivered later. Optional reference_N_T is acquisition-epoch WMM provenance;
    the existing rate-based control law itself needs no inertial reference.
    """
    tam_S_T: tuple[float, float, float]
    dcm_SB: tuple[tuple[float, float, float], ...]
    acquisition_ns: int
    publication_ns: int
    valid: bool
    quiet_valid: bool
    acquisition_phase: str
    acquisition_event: bool
    provenance: str
    reference_N_T: tuple[float, float, float] | None = None
    reference_epoch_ns: int | None = None

    def __post_init__(self):
        # Own even caller-provided lists/arrays; no writable nested storage.
        object.__setattr__(self, "tam_S_T", tuple(self.tam_S_T))
        object.__setattr__(self, "dcm_SB", tuple(tuple(row) for row in self.dcm_SB))
        if self.reference_N_T is not None:
            object.__setattr__(self, "reference_N_T", tuple(self.reference_N_T))


@dataclass(frozen=True)
class ControlSnapshot:
    """Capture-time coherence evidence, NOT continuing command authorization.

    A future consumer must also check current fault/revocation state at actual
    evaluation. Freezing coherent data does not promise future estimator health.
    """
    navigation_source: str
    sigma_BN: tuple[float, float, float]
    omega_BN_B_rad_s: tuple[float, float, float]
    navigation_state_ns: int
    navigation_publication_ns: int
    navigation_quality_ns: int
    navigation_valid: bool
    initialized: bool
    estimator_state: str
    fault: str
    consumer_state: str
    tam_S_T: tuple[float, float, float]
    tam_B_T: tuple[float, float, float]
    dcm_SB: tuple[tuple[float, float, float], ...]
    tam_acquisition_ns: int
    tam_publication_ns: int
    tam_valid: bool
    quiet_valid: bool
    reference_N_T: tuple[float, float, float] | None
    reference_epoch_ns: int | None
    capture_ns: int
    window: CycleWindow
    navigation_provenance: str
    magnetic_provenance: str
    coherence_status: str = "COHERENT_AT_CAPTURE"
    rejection_reason: str = ""

    def evaluation_rejection(self, actual_epoch_ns: int) -> str:
        """Only checks scheduled use, not latest health or command authorization."""
        try:
            return "" if _epoch(actual_epoch_ns) == self.window.evaluation_ns else "wrong_evaluation_epoch"
        except ValueError:
            return "invalid_evaluation_epoch"


@dataclass(frozen=True)
class SnapshotResult:
    snapshot: ControlSnapshot | None
    rejection_reason: str = ""

    @property
    def accepted(self) -> bool:
        return self.snapshot is not None


def capture_control_snapshot(nav: Snapshot, navigation_decision: Mapping,
                             magnetic: MagneticSample, window: CycleWindow,
                             capture_ns: int, *, navigation_provenance: str) -> SnapshotResult:
    """Compose Phase 7E's accepted decision with exact cycle/sample checks.

    s=window.sample_ns, e=window.evaluation_ns, c=capture_ns:
      nav.state = TAM.acquisition = optional reference.epoch = s
      s <= nav.publication <= quality.publication <= c <= e
      s <= TAM.publication <= c
    Later publication may describe s, but changing a header cannot relabel a
    current state as historical. The caller must provide the real state/timeTag.
    """
    try:
        c, s, e = _epoch(capture_ns), window.sample_ns, window.evaluation_ns
        if not s <= c <= e:
            return SnapshotResult(None, "capture_outside_selected_cycle_window")
        if not isinstance(nav.source, Source) or nav.source not in (Source.MEKF, Source.SIMPLE_NAV):
            return SnapshotResult(None, "unsupported_navigation_source")
        if not isinstance(navigation_provenance, str) or not navigation_provenance:
            return SnapshotResult(None, "missing_navigation_provenance")
        # Recheck native payload/quality against the selected epoch. This is a
        # derived exact-cycle allowance (c-s), never an invented flight age limit.
        state, reason, _ = assess(nav, c, c-s, s)
        if state != State.VALID:
            return SnapshotResult(None, "navigation:"+reason)
        d = navigation_decision
        if (d.get("accepted") is not True or d.get("consumer_state") != "VALID"
                or d.get("fault_latched") is not False):
            return SnapshotResult(None, "navigation_consumer_not_accepting")
        if (d.get("selected_source") != nav.source.value or d.get("consumer_epoch_ns") != c
                or d.get("state_epoch_ns") != s or d.get("publication_epoch_ns") != nav.header_ns
                or d.get("quality_epoch_ns") != nav.quality_header_ns
                or d.get("estimator_valid") is not True or d.get("initialized") is not True
                or _vector(d.get("received_sigma_BN")) != _vector(nav.sigma)
                or _vector(d.get("received_omega_BN_B_rad_s")) != _vector(nav.omega)):
            return SnapshotResult(None, "navigation_decision_does_not_describe_this_input")
        acquisition, publication = _epoch(magnetic.acquisition_ns), _epoch(magnetic.publication_ns)
        if magnetic.valid is not True or magnetic.quiet_valid is not True:
            return SnapshotResult(None, "invalid_magnetic_acquisition")
        if magnetic.acquisition_phase != "SAMPLE" or magnetic.acquisition_event is not True:
            return SnapshotResult(None, "not_a_sample_acquisition")
        if acquisition > c or publication > c or publication < acquisition:
            return SnapshotResult(None, "impossible_magnetic_epoch")
        if acquisition != s:
            return SnapshotResult(None, "tam_does_not_match_selected_sample_epoch")
        if not isinstance(magnetic.provenance, str) or not magnetic.provenance:
            return SnapshotResult(None, "missing_magnetic_provenance")
        measured = _vector(magnetic.tam_S_T)
        dcm = np.asarray(magnetic.dcm_SB, dtype=float)
        # TEST-ONLY 1e-12 numerical rotation check, not mounting accuracy. Epochs
        # use exact integer equality and have no numerical time tolerance.
        if (dcm.shape != (3, 3) or not np.isfinite(dcm).all()
                or not np.allclose(dcm @ dcm.T, np.eye(3), atol=1e-12, rtol=0)
                or not np.isclose(np.linalg.det(dcm), 1., atol=1e-12, rtol=0)):
            return SnapshotResult(None, "invalid_C_SB")
        reference = None
        if magnetic.reference_N_T is not None:
            reference = _vector(magnetic.reference_N_T)
            if _epoch(magnetic.reference_epoch_ns) != s:
                return SnapshotResult(None, "reference_epoch_mismatch")
        elif magnetic.reference_epoch_ns is not None:
            return SnapshotResult(None, "reference_epoch_without_vector")
        q = nav.quality
        if not isinstance(q.get("fault"), str):
            return SnapshotResult(None, "invalid_fault_metadata")
        return SnapshotResult(ControlSnapshot(nav.source.value, _vector(nav.sigma), _vector(nav.omega),
            s, _epoch(nav.header_ns), _epoch(nav.quality_header_ns), True, True, q["state"], q["fault"],
            "VALID", measured, _vector(dcm.T @ measured), tuple(tuple(float(v) for v in row) for row in dcm),
            acquisition, publication, True, True, reference, magnetic.reference_epoch_ns,
            c, window, navigation_provenance, magnetic.provenance))
    except (ValueError, TypeError, KeyError, OverflowError) as error:
        return SnapshotResult(None, "invalid_snapshot_input:"+str(error))
