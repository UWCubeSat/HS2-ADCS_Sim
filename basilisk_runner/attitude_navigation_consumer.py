"""Phase 7E DUMMY NAVIGATION CONSUMER / NO CONTROL AUTHORITY / NOT FLIGHT VALIDATED.

Source: Phase 7D NavAtt/status contract and Phase 7E interface tests, 2026-09-14.
All policies here are ASSUMED / TEST-ONLY, not a flight failover or stale limit.
Inputs are published NavAtt and quality only; no truth/sensor/actuator access.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf import epoch, vector
from attitude_mekf_adapter import DevelopmentChannel


class Source(str, Enum):
    SIMPLE_NAV = "SIMPLE_NAV"
    MEKF = "MEKF"
    NONE = "NONE"


class State(str, Enum):
    UNINITIALIZED = "UNINITIALIZED"
    VALID = "VALID"
    STALE = "STALE"
    DEGRADED = "DEGRADED"
    FAULTED = "FAULTED"
    REACQUIRING = "REACQUIRING"


@dataclass(frozen=True)
class ConsumerOptions:
    """Explicit selection schedule, ns; no automatic substitution/fallback.

    Empty schedule selects SIMPLE_NAV. These are diagnostic commands only.
    """
    selections: tuple[tuple[int, Source], ...] = ()

    def requested(self, tick: int) -> Source:
        previous = -1
        selected = Source.SIMPLE_NAV
        for stamp, source in self.selections:
            epoch(stamp)
            if stamp <= previous:
                raise ValueError("selection commands must be strictly time ordered")
            previous = stamp
            source = Source(source)
            if stamp <= tick:
                selected = source
        return selected


@dataclass
class Snapshot:
    source: Source
    header_ns: int | None
    time_tag_s: float | None
    sigma: object
    omega: object
    quality_header_ns: int | None
    quality: dict


class NavPort:
    """Source identity is a fixed subscription binding, not inferred from values."""
    def __init__(self, source: Source, nav_message, quality: DevelopmentChannel[dict]):
        self.source, self.quality = source, quality
        self.reader = messaging.NavAttMsgReader()
        self.reader.subscribeTo(nav_message)

    def snapshot(self) -> Snapshot:
        status = self.quality.read() if self.quality.time_written_ns is not None else {}
        if not self.reader.isWritten():
            return Snapshot(self.source, None, None, None, None, self.quality.time_written_ns, status)
        payload = self.reader()
        return Snapshot(self.source, int(self.reader.timeWritten()), float(payload.timeTag),
                        list(payload.sigma_BN), list(payload.omega_BN_B), self.quality.time_written_ns, status)


class SimpleNavQuality(sysModel.SysModel):
    """ASSUMED ideal SimpleNav producer contract, NOT flight validity evidence.

    Native NavAtt has no quality bit. This explicit development facade declares
    initialized availability only for a current written header/timeTag. The
    consumer independently checks fields, epochs and finiteness.
    """
    def __init__(self, message):
        super().__init__()
        self.reader = messaging.NavAttMsgReader()
        self.reader.subscribeTo(message)
        self.out = DevelopmentChannel[dict]()

    def UpdateState(self, current_time):
        tick = epoch(int(current_time))
        stamp = int(self.reader.timeWritten()) if self.reader.isWritten() else None
        available = stamp == tick and self.reader().timeTag == tick*1e-9
        self.out.write({"source": Source.SIMPLE_NAV.value, "valid": available,
            "initialized": available, "state": "INITIALIZED" if available else "UNINITIALIZED",
            "state_epoch_ns": stamp, "nav_published_epoch_ns": stamp,
            "status_epoch_ns": tick, "sim_epoch_ns": tick, "fault": "",
            "acquisitions": 0, "acquisition_event": False, "reacquisition_event": False,
            "scope": "ASSUMED / TEST-ONLY ideal SimpleNav availability"}, tick)


def representation(sigma, omega):
    """Canonical MRPs and scalar-first quaternion for C_BN: N components -> B.

    Body rate remains omega_BN_B [rad/s]; no reference subtraction or controller.
    """
    sigma, omega = vector(sigma), vector(omega)
    norm2 = float(sigma @ sigma)
    if not np.isfinite(norm2):
        raise ValueError("MRP norm not numerically representable")
    if norm2 > 1:
        sigma = -sigma/norm2
    q = np.asarray(rbk.MRP2EP(sigma), dtype=float)
    return {"sigma_BN": sigma.tolist(), "q_BN": q.tolist(),
            "C_BN": np.asarray(rbk.MRP2C(sigma)).tolist(), "omega_BN_B_rad_s": omega.tolist()}


def attitude_distance(q1, q2) -> float:
    a, b = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    if a @ b < 0:
        b = -b
    return float(4*np.arctan2(np.linalg.norm(a-b), np.linalg.norm(a+b)))


def assess(sample: Snapshot, tick: int, max_age_ns: int, required_epoch_ns: int | None):
    """Pure input checks. Integer state age, not refreshed publication, gates use."""
    q = sample.quality
    if not q or sample.quality_header_ns is None:
        return State.DEGRADED, "missing_quality", None
    try:
        quality_epoch = epoch(q["status_epoch_ns"])
        if epoch(sample.quality_header_ns) != quality_epoch or epoch(q["sim_epoch_ns"]) != quality_epoch:
            return State.FAULTED, "quality_epoch_mismatch", None
        if quality_epoch > tick:
            return State.FAULTED, "future_quality", None
        if tick-quality_epoch > max_age_ns:
            return State.STALE, "stale_quality", None
        if q.get("source", sample.source.value) != sample.source.value:
            return State.FAULTED, "source_identity_mismatch", None
        epoch(q.get("acquisitions", 0))
        if q.get("fault") or q.get("state") == "FAULT":
            return State.FAULTED, "estimator_fault:"+str(q.get("fault", "FAULT")), None
        if q.get("initialized") is not True:
            return State.UNINITIALIZED, "not_initialized", None
        if q.get("valid") is not True:
            return State.DEGRADED, "quality_not_valid", None
        if q.get("state") != "INITIALIZED":
            return State.FAULTED, "inconsistent_estimator_state", None
        stamp, publication = epoch(q["state_epoch_ns"]), epoch(sample.header_ns)
        if epoch(q["nav_published_epoch_ns"]) != publication:
            return State.FAULTED, "publication_epoch_mismatch", None
        if stamp > tick or publication > tick:
            return State.FAULTED, "future_navigation", None
        if stamp > publication or publication > quality_epoch:
            return State.FAULTED, "state_publication_order", None
        if sample.time_tag_s != stamp*1e-9:
            return State.FAULTED, "payload_state_epoch_mismatch", None
        if sample.source == Source.MEKF and epoch(q["gyro_epoch_ns"]) != stamp:
            return State.FAULTED, "gyro_state_epoch_mismatch", None
        if tick-stamp > max_age_ns:
            return State.STALE, "stale_state", None
        if required_epoch_ns is not None and stamp != required_epoch_ns:
            return State.STALE, "wrong_required_sample_epoch", None
        return State.VALID, "", representation(sample.sigma, sample.omega)
    except (ValueError, KeyError, TypeError, OverflowError) as error:
        return State.FAULTED, "invalid_navigation_contract:"+str(error), None


class DummyNavigationConsumer(sysModel.SysModel):
    """Diagnostic-only source gate; owns no output NavAtt or actuator messages.

    Faults latch per source. Recovery requires fresh post-fault explicit
    acquisition evidence with an increased acquisition count. STALE may recover
    with fresh data; invalid quality never authorizes use or automatic failover.
    """
    def __init__(self, ports: dict[Source, NavPort], options: ConsumerOptions = ConsumerOptions(),
                 max_age_ns: int = 0):
        super().__init__()
        self.ports, self.options = dict(ports), options
        self.max_age_ns = epoch(max_age_ns)  # TEST-ONLY; 0 = current-point contract.
        options.requested(0)
        self.latches: dict[Source, tuple[int, int]] = {}
        self.selected = Source.NONE
        self.previous: dict | None = None
        self.history: list[dict] = []

    def UpdateState(self, current_time):
        self.consume(epoch(int(current_time)))

    def consume(self, tick: int, snapshots: dict[Source, Snapshot] | None = None,
                required_epoch_ns: int | None = None):
        tick = epoch(tick)
        snapshots = snapshots if snapshots is not None else {key: port.snapshot() for key, port in self.ports.items()}
        requested = self.options.requested(tick)
        sample = snapshots.get(requested)
        status, reason, values = State.DEGRADED, "explicit_none" if requested == Source.NONE else "source_unavailable", None
        quality = sample.quality if sample else {}
        if sample is not None and requested != Source.NONE:
            if sample.source != requested:
                status, reason = State.FAULTED, "subscription_source_mismatch"
            else:
                status, reason, values = assess(sample, tick, self.max_age_ns, required_epoch_ns)
            if status == State.FAULTED:
                count = quality.get("acquisitions", 0)
                self.latches.setdefault(requested, (tick, count if type(count) is int and count >= 0 else 0))
            if requested in self.latches and status != State.FAULTED:
                fault_tick, acquisitions = self.latches[requested]
                recovered = (status == State.VALID and quality.get("acquisition_event") is True
                             and quality.get("acquisitions", 0) > acquisitions
                             and quality["state_epoch_ns"] > fault_tick)
                if recovered:
                    del self.latches[requested]
                else:
                    status = State.REACQUIRING if quality.get("initialized") is False else State.FAULTED
                    reason, values = "explicit_reacquisition_required", None
        accepted = status == State.VALID
        selected = requested if accepted else Source.NONE
        raw_stamp = quality.get("state_epoch_ns")
        try:
            stamp = epoch(raw_stamp)
        except ValueError:
            stamp = None
        # Preserve raw invalid data as null plus explicit rejection, not zero nav.
        def safe_vector(value):
            try:
                return vector(value).tolist()
            except (ValueError, TypeError):
                return None
        row = {"consumer_epoch_ns": tick, "requested_source": requested.value,
               "selected_source": selected.value, "consumer_state": status.value,
               "accepted": accepted, "estimator_valid": quality.get("valid", False),
               "initialized": quality.get("initialized", False), "state_epoch_ns": stamp,
               "publication_epoch_ns": sample.header_ns if sample else None,
               "quality_epoch_ns": sample.quality_header_ns if sample else None,
               "navigation_age_ns": tick-stamp if isinstance(stamp, int) else None,
               "required_epoch_ns": required_epoch_ns, "max_age_ns_test_only": self.max_age_ns,
               "rejection_reason": reason, "fault": quality.get("fault", ""),
               "fault_latched": requested in self.latches,
               "invalid_state_epoch_repr": repr(raw_stamp) if raw_stamp is not None and stamp is None else None,
               "fault_event": status == State.FAULTED and (self.previous is None or self.previous["consumer_state"] != State.FAULTED.value),
               "acquisition_event": quality.get("acquisition_event", False),
               "reacquisition_event": quality.get("reacquisition_event", False),
               "received_sigma_BN": safe_vector(sample.sigma) if sample else None,
               "received_omega_BN_B_rad_s": safe_vector(sample.omega) if sample else None,
               "navigation": values if accepted else None,
               "handover_event": selected != self.selected, "previous_source": self.selected.value,
               "handover_attitude_jump_rad": None, "handover_rate_jump_rad_s": None,
               "handover_state_epoch_jump_ns": None, "handover_publication_epoch_jump_ns": None,
               "handover_comparison": "not_available",
               "state_advanced": self.previous is None or stamp != self.previous["state_epoch_ns"],
               "publication_advanced": self.previous is None or (sample.header_ns if sample else None) != self.previous["publication_epoch_ns"],
               "scope": "DUMMY NAVIGATION CONSUMER / NO CONTROL AUTHORITY / NOT FLIGHT VALIDATED"}
        # Compare both published sources at THIS epoch, not successive moving
        # spacecraft states. This diagnostic has no role in selection/gating.
        if accepted and selected != self.selected and self.selected in snapshots:
            old = snapshots[self.selected]
            old_state, _, old_values = assess(old, tick, self.max_age_ns, required_epoch_ns)
            if old_state == State.VALID and self.selected not in self.latches:
                row["handover_state_epoch_jump_ns"] = stamp-old.quality["state_epoch_ns"]
                row["handover_publication_epoch_jump_ns"] = sample.header_ns-old.header_ns
                if row["handover_state_epoch_jump_ns"] == 0:
                    row["handover_attitude_jump_rad"] = attitude_distance(old_values["q_BN"], values["q_BN"])
                    row["handover_rate_jump_rad_s"] = float(np.linalg.norm(
                        np.array(old_values["omega_BN_B_rad_s"])-values["omega_BN_B_rad_s"]))
                    row["handover_comparison"] = "same_state_epoch"
                else:
                    row["handover_comparison"] = "different_state_epochs"
        self.selected, self.previous = selected, deepcopy(row)
        self.history.append(row)
        return row


class FrozenSampleProbe(sysModel.SysModel):
    """Mirror the EXISTING cycle acquisition/compute epochs without controlling.

    Capture after MEKF publication; preserve both original NavAtt/status headers.
    Compute requires that exact captured epoch, not a nearest-time or age-only join.
    """
    def __init__(self, ports, options, cycle):
        super().__init__()
        self.cycle = cycle
        self.consumer = DummyNavigationConsumer(ports, options, cycle.compute_offset_ns-cycle.sample_offset_ns)
        self.snapshots: dict[Source, Snapshot] = {}

    def UpdateState(self, current_time):
        tick = epoch(int(current_time))
        offset = tick % self.cycle.period_ns
        if offset == self.cycle.sample_offset_ns:
            self.snapshots = {key: port.snapshot() for key, port in self.consumer.ports.items()}
        if offset == self.cycle.compute_offset_ns:
            acquisition = tick-offset+self.cycle.sample_offset_ns
            self.consumer.consume(tick, self.snapshots, acquisition)


def attach_consumers(sim, simple_message, mekf_message, mekf_quality, cycle, options: ConsumerOptions):
    """Explicit scheduling opt-in; only published data, no estimator/plant handles."""
    facade = SimpleNavQuality(simple_message)
    ports = {Source.SIMPLE_NAV: NavPort(Source.SIMPLE_NAV, simple_message, facade.out),
             Source.MEKF: NavPort(Source.MEKF, mekf_message, mekf_quality)}
    point = DummyNavigationConsumer(ports, options)
    frozen = FrozenSampleProbe(ports, options, cycle) if cycle is not None else None
    for model, tag, priority in ((facade, "DummySimpleNavQuality", 565),
                                 (point, "DummyNavConsumer", 560), (frozen, "DummyFrozenNav", 555)):
        if model is not None:
            model.ModelTag = tag
            sim.AddModelToTask("DynamicsTask", model, ModelPriority=priority)
    return point, frozen
