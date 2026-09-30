"""Phase 7F-2B EVALUATION-TIME COMMAND HEALTH GATE, 2026-09-30.

NO ACTUATOR AUTHORITY / NOT CLOSED LOOP / NOT FLIGHT VALIDATED.
Development contract from Phase 7D/E quality and Phase 7F-1/2A frozen records.
Observe every source-status event; forward explicit reset_acquisition requests.
No controller call, command retention, fallback, native message or scheduling.
All epochs are integer simulation ns. Decisions describe one evaluation only.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from typing import Mapping

from attitude_navigation_consumer import Snapshot, Source, State, assess
from control_input_snapshot import ControlSnapshot, _epoch
from disconnected_detumble_math import CommandMathematics


@dataclass(frozen=True)
class HealthObservation:
    epoch_ns: int
    quality_epoch_ns: int | None
    source_state: str
    healthy: bool
    reason: str
    lifecycle_state: str
    acquisition_epoch_ns: int | None
    acquisition_count: int
    reset_epoch_ns: int | None


@dataclass(frozen=True)
class CommandUseDecision:
    """Historical result, never a reusable authorization or held command.

    Numeric command/provenance is immutable and may remain recorded when unusable.
    Consumers must evaluate again at the intended use event, not reuse this bit.
    """
    command: CommandMathematics | None
    command_usable: bool
    inhibition_reason: str
    evaluation_epoch_ns: int
    sample_epoch_ns: int | None
    snapshot_capture_epoch_ns: int | None
    command_computation_epoch_ns: int | None
    capture_time_valid: bool
    mathematical_command_valid: bool
    current_health: HealthObservation
    actuator_authority: bool = False
    scope: str = "EVALUATION-TIME COMMAND HEALTH GATE / NOT CLOSED LOOP / NOT FLIGHT VALIDATED"


def _current_health(nav: Snapshot, decision: Mapping, tick: int) -> tuple[str, int]:
    """Reuse Phase 7E validation with EXACT current-point quality/nav epochs.

    Stored command inputs remain at s. This separate live health input is at e.
    No flight age threshold or resampling is introduced.
    """
    q = nav.quality
    if not isinstance(q, dict) or not isinstance(decision, Mapping):
        return "invalid_or_nonfinite_health_metadata", 0
    try:
        # Published status/consumer records are JSON-compatible. Reject NaN/Inf
        # even in companion diagnostic fields rather than accepting partial data.
        json.dumps(q, allow_nan=False)
        json.dumps(dict(decision), allow_nan=False)
        if not isinstance(nav.source, Source) or nav.source != Source.MEKF:
            return "wrong_source_no_fallback", 0
        if not isinstance(q.get("fault"), str):
            return "invalid_fault_metadata", 0
        count = _epoch(q["acquisitions"])
        if (type(q.get("acquisition_event")) is not bool
                or type(q.get("reacquisition_event")) is not bool
                or q["reacquisition_event"] != (q["acquisition_event"] and count > 1)):
            return "inconsistent_acquisition_metadata", count
        state, reason, _ = assess(nav, tick, 0, tick)
        if state != State.VALID:
            return reason, count
        if decision.get("fault_latched") is True:
            return "navigation_fault_latched", count
        if (decision.get("fault_latched") is not False
                or decision.get("accepted") is not True
                or decision.get("consumer_state") != State.VALID.value
                or decision.get("requested_source") != Source.MEKF.value
                or decision.get("selected_source") != Source.MEKF.value
                or decision.get("estimator_valid") is not True
                or decision.get("initialized") is not True
                or decision.get("fault") != ""
                or decision.get("rejection_reason") != ""):
            return "current_consumer_not_accepting", count
        for key in ("consumer_epoch_ns", "state_epoch_ns", "publication_epoch_ns", "quality_epoch_ns"):
            if _epoch(decision[key]) != tick:
                return "current_consumer_epoch_mismatch", count
        return "", count
    except (KeyError, TypeError, ValueError, OverflowError):
        return "invalid_or_nonfinite_health_metadata", 0


def _capture_valid(snapshot: ControlSnapshot | None) -> bool:
    # Accepted Phase 7F-1 input is a precondition; do not repeat frame/TAM math.
    return (isinstance(snapshot, ControlSnapshot)
            and snapshot.coherence_status == "COHERENT_AT_CAPTURE"
            and snapshot.rejection_reason == ""
            and snapshot.navigation_valid is True and snapshot.initialized is True
            and snapshot.estimator_state == "INITIALIZED" and snapshot.fault == ""
            and snapshot.consumer_state == "VALID"
            and snapshot.tam_valid is True and snapshot.quiet_valid is True)


def _record_reason(snapshot: ControlSnapshot | None, command: CommandMathematics | None) -> str:
    """Check binding/provenance, never recompute command equations."""
    if snapshot is None or command is None:
        return "fresh_snapshot_and_command_required"
    if not _capture_valid(snapshot):
        return "invalid_capture_provenance"
    if command.snapshot != snapshot:
        return "command_snapshot_mismatch"
    try:
        if (snapshot.navigation_source != Source.MEKF.value
                or not all(isinstance(value, str) and value.strip() for value in
                           (snapshot.navigation_provenance, snapshot.magnetic_provenance, snapshot.window.provenance))):
            return "invalid_snapshot_provenance"
        json.dumps(asdict(command), allow_nan=False)
        config_json = json.dumps(json.loads(command.configuration_json), sort_keys=True, allow_nan=False)
        if (hashlib.sha256(config_json.encode()).hexdigest() != command.configuration_fingerprint
                or command.controller_entry_point != "basilisk_adcs_adapter.controller_step"
                or command.actuator_authority is not False):
            return "invalid_command_provenance"
        if command.command_valid is not True:
            return "mathematical_command_invalid"
        if (_epoch(command.evaluation_epoch_ns) != snapshot.window.evaluation_ns
                or not snapshot.navigation_state_ns == snapshot.tam_acquisition_ns == snapshot.window.sample_ns
                or not snapshot.window.sample_ns <= snapshot.capture_ns <= command.evaluation_epoch_ns):
            return "inconsistent_command_epochs"
    except (ValueError, TypeError, KeyError, OverflowError):
        return "invalid_command_provenance"
    return ""


class CommandHealthGate:
    """MEKF-only, fail-closed development lifecycle; stores NO command.

    Startup requires an observed fresh acquisition event. Any subsequent health
    loss latches inhibition until an explicit reset and increased acquisition
    count. Forward the existing InputBatch.reset_acquisition event via notify_reset;
    merely clearing a fault/valid bit is insufficient. Full process/adapter Reset
    (which clears counters) needs a new integration contract, not silent reuse.

    After revocation/reset, sample must be STRICTLY after reacquisition. Equal-ns
    events have no ordering token in ControlSnapshot; reject ambiguity instead
    of inventing order. No duration/flight age threshold is selected.
    """
    def __init__(self):
        self._last_epoch_ns = -1
        self._state = "AWAITING_ACQUISITION"
        self._count = 0
        self._acquired_ns: int | None = None
        self._reset_ns: int | None = None
        self._revoked_ns: int | None = None

    def _revoke(self, tick: int):
        self._revoked_ns = max(tick, self._revoked_ns if self._revoked_ns is not None else tick)
        self._acquired_ns = None

    def notify_reset(self, epoch_ns: int):
        """Explicit reset notification, not an estimator reset implementation."""
        tick = _epoch(epoch_ns)
        if tick < self._last_epoch_ns:
            raise ValueError("reset must not precede observed source history")
        self._last_epoch_ns = tick
        self._reset_ns = tick
        self._revoke(tick)
        self._state = "REACQUIRING"

    def observe(self, nav: Snapshot, decision: Mapping, epoch_ns: int) -> HealthObservation:
        """Consume current source evidence even when there is no command to use."""
        tick = _epoch(epoch_ns)
        reason, count = _current_health(nav, decision, tick)
        if tick < self._last_epoch_ns:
            reason = "nonmonotonic_health_epoch"
        self._last_epoch_ns = max(self._last_epoch_ns, tick)
        if reason:
            waiting = self._state in ("AWAITING_ACQUISITION", "REACQUIRING") and reason == "not_initialized"
            if not waiting:
                self._revoke(self._last_epoch_ns)
                self._state = "FAULTED"
        elif self._state == "FAULTED":
            reason = "explicit_reset_required"
        elif self._state in ("AWAITING_ACQUISITION", "REACQUIRING"):
            if not nav.quality["acquisition_event"] or count <= self._count:
                reason = "fresh_acquisition_event_required"
            else:
                self._count, self._acquired_ns, self._state = count, tick, "HEALTHY"
        elif (count != self._count
              or (nav.quality["acquisition_event"] and tick != self._acquired_ns)):
            reason = "acquisition_sequence_changed_without_reset"
            self._revoke(tick)
            self._state = "FAULTED"
        try:
            quality_epoch = _epoch(nav.quality_header_ns)
        except (TypeError, ValueError):
            quality_epoch = None
        source_state = str(nav.quality.get("state", "UNKNOWN")) if isinstance(nav.quality, dict) else "UNKNOWN"
        return HealthObservation(tick, quality_epoch, source_state,
                                 not reason, reason, self._state, self._acquired_ns, self._count, self._reset_ns)

    def evaluate(self, snapshot: ControlSnapshot | None, command: CommandMathematics | None,
                 nav: Snapshot, decision: Mapping, epoch_ns: int) -> CommandUseDecision:
        """Reassess NOW. An earlier usable result grants no later use permission.

        Phase 7F-2A evaluation_epoch_ns is the command COMPUTATION epoch. This
        gate's epoch is USE evaluation: ordered after computation at the same
        existing compute tick. Later ticks reject the old command; no hold added.
        """
        health = self.observe(nav, decision, epoch_ns)
        reason = health.reason or _record_reason(snapshot, command)
        if not reason and snapshot is not None and command is not None:
            sample = snapshot.navigation_state_ns
            if self._acquired_ns is None or sample < self._acquired_ns:
                reason = "snapshot_predates_current_acquisition"
            elif self._revoked_ns is not None and (sample <= self._revoked_ns or sample <= self._acquired_ns):
                reason = "fresh_post_reacquisition_snapshot_required"
            else:
                reason = snapshot.evaluation_rejection(epoch_ns)
        return CommandUseDecision(command, not reason, reason, epoch_ns,
            snapshot.navigation_state_ns if snapshot else None, snapshot.capture_ns if snapshot else None,
            command.evaluation_epoch_ns if command else None, _capture_valid(snapshot),
            command.command_valid is True if command else False, health)
