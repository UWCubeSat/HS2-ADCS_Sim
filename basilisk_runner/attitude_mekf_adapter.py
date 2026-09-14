"""Phase 7D SHADOW DEVELOPMENT INTEGRATION / NOT FLIGHT VALIDATED.

Sources: Phase 7B/7C architecture and existing Phase 6A cycle, reviewed 2026-09-14.
N/B/P conventions and the MEKF equations are unchanged. This module translates
live messages, carries validity/epochs, and logs; it never commands an actuator.
All ideal bridges, cadences/delays and conditioning weights are TEST-ONLY.
"""
from __future__ import annotations

from collections import Counter, deque
from copy import deepcopy
from dataclasses import asdict, dataclass, field, replace
from typing import Generic, TypeVar

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk

from attitude_mekf import MEKF, ReplayEstimator, VectorSample, epoch, vector
from attitude_mekf_prototype import independent_error, load_test_policy, magnetic_cycle_sample

T = TypeVar("T")


class DevelopmentChannel(Generic[T]):
    """Python-only typed companion channel; NOT a SWIG/flight message ABI.

    Snapshot ownership and publication time are explicit. Native navigation
    values travel on NavAttMsg; this channel carries mandatory validity metadata.
    """
    def __init__(self):
        self._payload: T | None = None
        self.time_written_ns: int | None = None

    def write(self, payload: T, tick: int):
        self._payload = deepcopy(payload)
        self.time_written_ns = epoch(tick)

    def read(self) -> T:
        if self._payload is None:
            raise ValueError("development channel unwritten")
        return deepcopy(self._payload)


@dataclass(frozen=True)
class ShadowOptions:
    ideal_sun: bool = False
    magnetic_enabled: bool = True
    magnetic_delay_ns: int = 0
    sun_delay_ns: int = 0
    delay_after_ns: int = 1_000_000_000
    sun_period_ns: int = 700_000_000
    sun_offset_ns: int = 400_000_000
    drop_sun_after_ns: int | None = None
    initial_q_BN: tuple[float, float, float, float] | None = None
    label: str = "ASSUMED / TEST-ONLY; SHADOW DEVELOPMENT INTEGRATION"

    def validate(self, step_ns: int):
        for value in (self.magnetic_delay_ns, self.sun_delay_ns, self.delay_after_ns,
                      self.sun_period_ns, self.sun_offset_ns):
            epoch(value)
            if value % step_ns:
                raise ValueError("development event times must lie on the existing task grid")
        if self.sun_period_ns == 0:
            raise ValueError("positive TEST-ONLY Sun cadence required")
        if self.drop_sun_after_ns is not None:
            epoch(self.drop_sun_after_ns)


@dataclass
class InputBatch:
    sim_epoch_ns: int
    gyro_epoch_ns: int | None
    gyro_point_B: np.ndarray
    interval_start_ns: int | None
    intervals: list[tuple[int, int, np.ndarray]] = field(default_factory=list)
    deliveries: list[VectorSample] = field(default_factory=list)
    gyro_valid: bool = True
    gyro_reason: str = ""
    source_status: dict[str, str] = field(default_factory=dict)
    reset_acquisition: bool = False


def causal_gyro_intervals(start: int, end: int, first, last, parts: int):
    """TEST-ONLY linear endpoint-rate reconstruction, received at end.

    Unlike the offline Phase 7C cubic synthesizer, no future truth sample is used.
    Rates at subinterval midpoints approximate the covered angular-rate history;
    this is not an installed IMU aperture/filter model or a plant timing change.
    """
    start, end = epoch(start), epoch(end)
    first, last = vector(first), vector(last)
    if end <= start or parts <= 0 or (end-start) % parts:
        raise ValueError("invalid ideal gyro interval partition")
    step = (end-start)//parts
    return [(start+j*step, start+(j+1)*step,
             first+(last-first)*(j+0.5)/parts) for j in range(parts)]


class IdealLiveBridge(sysModel.SysModel):
    """Read actual SCStates/TAM/WMM messages AFTER current cycle acquisition."""
    def __init__(self, state_msg, tam_msg, field_msg, cycle_driver, c_sb,
                 options: ShadowOptions, step_ns: int):
        super().__init__()
        options.validate(step_ns)
        self.options, self.driver, self.step_ns = options, cycle_driver, step_ns
        self.state = messaging.SCStatesMsgReader()
        self.tam = messaging.TAMSensorMsgReader()
        self.magnetic = messaging.MagneticFieldMsgReader()
        for reader, source in ((self.state, state_msg), (self.tam, tam_msg), (self.magnetic, field_msg)):
            reader.subscribeTo(source)
        self.c_sb = np.array(c_sb, dtype=float)
        policy, fixture = load_test_policy()
        self.sun_n = vector(fixture["sun_reference_n"]["value"])
        self.parts = fixture["gyro_substeps"]["value"]
        self.delivery_capacity = policy.max_events
        self.out = DevelopmentChannel[InputBatch]()
        self.Reset(0)

    def Reset(self, tick):
        self.previous: tuple[int, np.ndarray] | None = None
        self.pending: list[tuple[int, VectorSample]] = []

    def magnetic_acquisition(self, tick: int) -> VectorSample:
        """Invalid/missing native sensor data remain explicit rejected inputs."""
        try:
            if not self.tam.isWritten() or not self.magnetic.isWritten():
                raise ValueError("missing_magnetic_message")
            row = dict(self.driver.history[-1])
            if round(row["time_s"]*1e9) != tick:
                raise ValueError("stale_cycle_metadata")
            row.update(time_ns=tick, tam_message_time_ns=self.tam.timeWritten(),
                       field_evaluation_time_ns=self.magnetic.timeWritten())
            raw_tam = vector(self.tam().tam_S)
            torque = vector(np.asarray(self.driver.effector.torqueExternalPntB_B).reshape(3))
            for j, axis in enumerate("xyz"):
                row[f"tam_sample_B_B_{axis}_T"] = float(raw_tam[j])
                row[f"applied_torque_B_{axis}_Nm"] = float(torque[j])
            sample = magnetic_cycle_sample(row, self.driver.cycle)
            return replace(sample, frame="S", c_sb=self.c_sb,
                source="live TAMSensorMsg.tam_S + acquisition WMM B_N; explicit C_SB")
        except (ValueError, IndexError) as error:
            return VectorSample(f"invalid-live-mag-{tick}", "magnetic", tick, tick,
                np.zeros(3), np.zeros(3), "invalid live magnetic input; no truth substitution",
                valid=False, invalid_reason=str(error))

    def UpdateState(self, current_time):
        tick = epoch(int(current_time))
        actual_gyro_epoch = int(self.state.timeWritten()) if self.state.isWritten() else None
        batch = InputBatch(tick, actual_gyro_epoch, np.zeros(3), None)
        try:
            if not self.state.isWritten() or self.state.timeWritten() != tick:
                raise ValueError("stale_or_missing_SCStates")
            state = self.state()
            gyro = vector(state.omega_BN_B)
            sigma = vector(state.sigma_BN)
            batch.gyro_point_B = gyro
            if self.previous is not None:
                start, previous = self.previous
                if tick-start != self.step_ns:
                    raise ValueError("missing_gyro_coverage")
                batch.interval_start_ns = start
                batch.intervals = causal_gyro_intervals(start, tick, previous, gyro, self.parts)
            self.previous = (tick, gyro.copy())
        except ValueError as error:
            batch.gyro_valid, batch.gyro_reason = False, str(error)
            self.previous = None
            self.out.write(batch, tick)
            return
        batch.source_status["magnetic"] = "no_quiet_contract" if self.driver is None else "no_new_acquisition"
        if self.options.magnetic_enabled and self.driver is not None:
            if tick % self.driver.cycle.period_ns == self.driver.cycle.sample_offset_ns:
                sample = self.magnetic_acquisition(tick)
                # Driver's historical *_B_B labels currently hold raw tam_S.
                # The new bridge explicitly carries C_SB instead of relabeling S.
                delay = self.options.magnetic_delay_ns if tick >= self.options.delay_after_ns else 0
                self.pending.append((tick+delay, sample))
                batch.source_status["magnetic"] = "acquired" if sample.valid else "invalid_acquisition"
        if not self.options.magnetic_enabled:
            batch.source_status["magnetic"] = "disabled_test_only"
        batch.source_status["sun"] = "absent"
        if self.options.ideal_sun:
            batch.source_status["sun"] = "no_new_acquisition"
            if tick >= self.options.sun_offset_ns and (tick-self.options.sun_offset_ns) % self.options.sun_period_ns == 0:
                valid = self.options.drop_sun_after_ns is None or tick < self.options.drop_sun_after_ns
                sample = VectorSample(f"live-sun-{tick}", "sun", tick, tick,
                    np.asarray(rbk.MRP2C(sigma)) @ self.sun_n, self.sun_n,
                    "TEST-ONLY ideal Sun direction from current truth; no CSS model",
                    valid=valid, invalid_reason="missing_sun_test_only")
                delay = self.options.sun_delay_ns if tick >= self.options.delay_after_ns else 0
                self.pending.append((tick+delay, sample))
                batch.source_status["sun"] = "acquired" if valid else "missing"
        remaining = []
        for delivery_tick, sample in self.pending:
            if delivery_tick <= tick:
                batch.deliveries.append(sample)
            else:
                remaining.append((delivery_tick, sample))
        self.pending = remaining
        # Bound delayed delivery storage independently of the core replay window.
        while len(self.pending) > self.delivery_capacity:
            _, overflow = self.pending.pop(0)
            batch.deliveries.append(replace(overflow, valid=False, invalid_reason="bridge_delivery_capacity"))
        self.out.write(batch, tick)


class MEKFNavigationAdapter(sysModel.SysModel):
    """Interface-only consumer; the Phase 7C core owns every estimator update.

    NavAtt fields have current-point semantics only when status.valid is true.
    Before acquisition/on gyro faults no new NavAtt is written. Readers must
    require the companion status AND current NavAtt header/timeTag. This is not
    an unconditional replacement for SimpleNav.
    """
    def __init__(self, inputs: DevelopmentChannel[InputBatch], options: ShadowOptions):
        super().__init__()
        self.inputs, self.options = inputs, options
        self.policy, _ = load_test_policy()
        self.navOutMsg = messaging.NavAttMsg()
        self.statusOut = DevelopmentChannel[dict]()
        self.Reset(0)

    def Reset(self, tick):
        self.engine: ReplayEstimator | None = None
        self.fault = ""
        self.rejected: Counter = Counter()
        self.sensor_epochs: dict[str, int] = {}
        self.accepted_epochs: dict[str, int] = {}
        self.total_acquisitions = 0
        self.history: list[dict] = []
        self.input_trace: list[InputBatch] = []
        self.trace: list[dict] = []
        self.prior_used = False

    def UpdateState(self, current_time):
        tick = epoch(int(current_time))
        results: list[dict] = []
        acquired = False
        try:
            if self.inputs.time_written_ns != tick:
                raise ValueError("stale_or_missing_input_batch")
            batch = self.inputs.read()
            self.input_trace.append(deepcopy(batch))
            if epoch(batch.sim_epoch_ns) != tick or epoch(batch.gyro_epoch_ns) != tick:
                raise ValueError("gyro_or_batch_epoch_mismatch")
            gyro = vector(batch.gyro_point_B)
            if not batch.gyro_valid:
                raise ValueError(batch.gyro_reason or "invalid_gyro")
            if batch.reset_acquisition:
                self.engine, self.fault = None, ""
            if self.fault:
                raise ValueError(self.fault)
            if self.engine is None and self.options.initial_q_BN is not None and not self.prior_used:
                self.engine = ReplayEstimator(MEKF(self.policy, epoch_ns=tick, prior_q=self.options.initial_q_BN))
                self.prior_used = True
            if self.engine is not None:
                start = self.engine.core.epoch_ns
                if start < tick:
                    if batch.interval_start_ns != start or not batch.intervals:
                        raise ValueError("missing_gyro_coverage")
                    if batch.intervals[-1][1] != tick:
                        raise ValueError("gyro_interval_end_mismatch")
                    # Validate the whole batch before mutating the estimator.
                    for begin, end, rate in batch.intervals:
                        if epoch(begin) != start or epoch(end) <= begin:
                            raise ValueError("gyro_interval_order")
                        vector(rate)
                        start = end
                    for begin, end, rate in batch.intervals:
                        self.engine.advance(begin, end, rate)
            else:
                # No arbitrary-attitude gyro propagation while uninitialized.
                # Initial acquisition is deliberately restricted to a fresh,
                # common-epoch pair; delayed local updates work after acquisition.
                candidate = ReplayEstimator(MEKF(self.policy, epoch_ns=tick))
                for sample in batch.deliveries:
                    if sample.epoch_ns != tick:
                        self.rejected["initial_pair_not_current"] += 1
                        results.append({"id": sample.event_id, "result": "initial_pair_not_current"})
                        continue
                    result = candidate.submit(sample, tick)
                    results.append({"id": sample.event_id, "sensor": sample.sensor,
                                    "epoch_ns": sample.epoch_ns, "result": result,
                                    "input_valid": sample.valid, "source": sample.source})
                    self.sensor_epochs[sample.sensor] = sample.epoch_ns
                if candidate.core.initialized:
                    self.engine = candidate
                    acquired = True
                    self.total_acquisitions += 1
                else:
                    self.rejected.update(candidate.ingress_rejections+candidate.core.rejections)
                batch = replace(batch, deliveries=[])  # Acquisition events already consumed.
            if self.engine is not None:
                for sample in batch.deliveries:
                    result = self.engine.submit(sample, tick)
                    results.append({"id": sample.event_id, "sensor": sample.sensor,
                                    "epoch_ns": sample.epoch_ns, "result": result,
                                    "input_valid": sample.valid, "source": sample.source})
                    self.sensor_epochs[sample.sensor] = sample.epoch_ns
                payload = messaging.NavAttMsgPayload()
                payload.sigma_BN = self.engine.core.output()["sigma_BN"].tolist()
                payload.omega_BN_B = self.engine.core.point_rate(gyro, tick).tolist()
                payload.timeTag = tick*1e-9
                # vehSunPntB is not supplied/validated; shadow consumers use only
                # sigma_BN, omega_BN_B and timeTag under the companion contract.
                self.navOutMsg.write(payload, tick, self.moduleID)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.fault = str(error)
            self.rejected[self.fault] += 1
        core = self.engine.core if self.engine else None
        valid = core is not None and core.initialized and not self.fault and core.epoch_ns == tick
        rejected = self.rejected.copy()
        if self.engine:
            rejected.update(self.engine.ingress_rejections+core.rejections)
            # Recompute retained accepted epochs after replay; receipt of a bad
            # or late-rejected sample must never make accepted-data age fresher.
            self.accepted_epochs = {sensor: stamp for sensor, stamp in self.accepted_epochs.items()
                                    if stamp <= self.engine.base.epoch_ns}
            for event in self.engine.events:
                if self.engine.dispositions.get(event.event_id) in ("updated", "acquired", "awaiting_noncollinear_pair"):
                    self.accepted_epochs[event.sensor] = max(event.epoch_ns, self.accepted_epochs.get(event.sensor, -1))
        status = {"sim_epoch_ns": tick, "status_epoch_ns": tick, "gyro_epoch_ns": None,
                  "state_epoch_ns": core.epoch_ns if core else None, "nav_published_epoch_ns": tick if valid else None,
                  "valid": bool(valid), "initialized": bool(core and core.initialized),
                  "state": "FAULT" if self.fault else "INITIALIZED" if valid else "UNINITIALIZED",
                  "fault": self.fault, "results": results, "rejected": dict(rejected),
                  "sensor_epochs": dict(self.sensor_epochs),
                  "sample_ages_ns": {key: tick-value for key, value in self.sensor_epochs.items()},
                  "accepted_sensor_epochs": dict(self.accepted_epochs),
                  "accepted_sample_ages_ns": {key: tick-value for key, value in self.accepted_epochs.items()},
                  "updates": dict(core.updates) if core else {}, "acquisitions": self.total_acquisitions,
                  "acquisition_event": acquired, "reacquisition_event": acquired and self.total_acquisitions > 1,
                  "replays": self.engine.replay_count if self.engine else 0,
                  "last_update": deepcopy(core.last_update) if core else None,
                  "instantaneous_attitude_rank": core.instantaneous_attitude_rank if core else 0,
                  "rate_semantics": "current point gyro minus posterior bias; rad/s in B",
                  "scope": "SHADOW DEVELOPMENT INTEGRATION / NOT FLIGHT VALIDATED"}
        if self.inputs.time_written_ns == tick:
            latest = self.inputs.read()
            try:
                status["gyro_epoch_ns"] = epoch(latest.gyro_epoch_ns)
            except ValueError:
                status["invalid_gyro_epoch_repr"] = repr(latest.gyro_epoch_ns)
            status["source_status"] = latest.source_status
        self.statusOut.write(status, tick)
        self.history.append(deepcopy(status))
        self.trace.append({"epoch_ns": tick, "core": deepcopy(core.output()) if core else None,
                           "updates": dict(core.updates) if core else {}, "results": deepcopy(results)})


class ShadowNavigationObserver(sysModel.SysModel):
    """Only consumer of shadow NavAtt; validates actual header/payload timing."""
    def __init__(self, adapter, state_msg, simple_nav_msg):
        super().__init__()
        self.adapter = adapter
        self.state = messaging.SCStatesMsgReader()
        self.simple = messaging.NavAttMsgReader()
        self.estimated = messaging.NavAttMsgReader()
        self.state.subscribeTo(state_msg)
        self.simple.subscribeTo(simple_nav_msg)
        self.estimated.subscribeTo(adapter.navOutMsg)
        self.history: list[dict] = []

    def UpdateState(self, current_time):
        tick = int(current_time)
        status = self.adapter.statusOut.read()
        if self.adapter.statusOut.time_written_ns != tick:
            raise ValueError("observer scheduled before estimator publication")
        if self.state.timeWritten() != tick or self.simple.timeWritten() != tick:
            raise ValueError("truth/SimpleNav epoch mismatch in shadow observer")
        truth, simple = self.state(), self.simple()
        row = {"time_ns": tick, "truth_epoch_ns": self.state.timeWritten(),
               "simple_nav_epoch_ns": self.simple.timeWritten(), "simple_nav_time_tag_s": simple.timeTag,
               "gyro_epoch_ns": status["gyro_epoch_ns"], "estimator_state_epoch_ns": status["state_epoch_ns"],
               "status_epoch_ns": status["status_epoch_ns"], "valid": status["valid"], "state": status["state"],
               "initialized": status["initialized"], "fault": status["fault"],
               "acquisitions": status["acquisitions"], "replays": status["replays"],
               "acquisition_event": status["acquisition_event"], "reacquisition_event": status["reacquisition_event"],
               "magnetic_epoch_ns": status["sensor_epochs"].get("magnetic"),
               "sun_epoch_ns": status["sensor_epochs"].get("sun"),
               "magnetic_age_ns": status["sample_ages_ns"].get("magnetic"),
               "sun_age_ns": status["sample_ages_ns"].get("sun"),
               "magnetic_accepted_epoch_ns": status["accepted_sensor_epochs"].get("magnetic"),
               "sun_accepted_epoch_ns": status["accepted_sensor_epochs"].get("sun"),
               "magnetic_accepted_age_ns": status["accepted_sample_ages_ns"].get("magnetic"),
               "sun_accepted_age_ns": status["accepted_sample_ages_ns"].get("sun"),
               "last_update_epoch_ns": status["last_update"]["epoch_ns"] if status["last_update"] else None,
               "last_update_kind": status["last_update"]["kind"] if status["last_update"] else "none",
               "rejected_count": sum(status["rejected"].values()), "rejected_reasons": str(status["rejected"]),
               "nav_written_ns": self.estimated.timeWritten() if self.estimated.isWritten() else None}
        for j in range(3):
            row[f"truth_sigma_{j+1}"] = truth.sigma_BN[j]
            row[f"simple_sigma_{j+1}"] = simple.sigma_BN[j]
            row[f"truth_omega_{j}_rad_s"] = truth.omega_BN_B[j]
        if status["valid"]:
            estimated = self.estimated()
            if self.estimated.timeWritten() != tick or estimated.timeTag != tick*1e-9:
                raise ValueError("shadow NavAtt header/state/timeTag mismatch")
            core = self.adapter.engine.core
            error, _ = independent_error(core.q, truth.sigma_BN)
            eig = np.linalg.eigvalsh(core.p)
            row.update(nav_time_tag_s=estimated.timeTag, attitude_error_rad=error,
                       covariance_eigenvalue_min=float(eig.min()), covariance_eigenvalue_max=float(eig.max()))
            for j in range(3):
                row[f"mekf_sigma_{j+1}"] = estimated.sigma_BN[j]
                row[f"mekf_omega_{j}_rad_s"] = estimated.omega_BN_B[j]
                row[f"bias_{j}_rad_s"] = core.bias[j]
            for j in range(4):
                row[f"mekf_q_{j}"] = core.q[j]
            for j in range(6):
                row[f"P_diagonal_{j}"] = core.p[j, j]
        self.history.append(row)
