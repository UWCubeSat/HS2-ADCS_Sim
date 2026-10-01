"""Phase 7F-2C LIVE DISCONNECTED COMMAND-CHAIN OBSERVER, 2026-09-30.

NO ACTUATOR AUTHORITY / NOT CLOSED LOOP / NOT FLIGHT VALIDATED.
Orchestration of unchanged 7F-1/2A algorithms and the explicit 7F-2B application
boundary interface. Reads native TAM/NavAtt and development quality/input copies.
Only Python diagnostic records leave this module; no native command endpoint.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
from typing import Callable

from Basilisk.architecture import messaging, sysModel

from command_health_gate import CommandHealthGate
from control_input_snapshot import ControlSnapshot, CycleWindow, MagneticSample, capture_control_snapshot
from disconnected_detumble_math import CommandMathematics, evaluate_snapshot


SCOPE = "LIVE DISCONNECTED COMMAND-CHAIN OBSERVER / NO ACTUATOR AUTHORITY / NOT CLOSED LOOP / NOT FLIGHT VALIDATED"


@dataclass(frozen=True)
class ObserverOptions:
    # Explicit validation-only fixture attachment. Never enabled by the CLI.
    test_setup: Callable | None = None
    label: str = "DEVELOPMENT CANDIDATE; 2026-09-30; no actuator authority"


@dataclass(frozen=True)
class PendingCalculation:
    snapshot: ControlSnapshot
    command: CommandMathematics
    generation: tuple[int, int, int]  # reset serial, acquisition count, revocation serial
    cycle_index: int
    command_id: int
    application_start_ns: int
    application_end_ns: int


class Stage(sysModel.SysModel):
    def __init__(self, callback):
        super().__init__()
        self.callback = callback

    def UpdateState(self, tick):
        self.callback(int(tick))


class DisconnectedCommandObserver:
    """No plant, effector, production controller or command-message handle.

    Pending math is diagnostic data only, discarded on a new cycle/failed compute.
    A reset does not erase evidence, but invalidates its generation immediately.
    No previous usability decision is carried forward as authority.
    """
    def __init__(self, cycle, config, tam_message, input_channel, port, consumer, options):
        self.cycle, self.config, self.port, self.consumer, self.options = cycle, config, port, consumer, options
        self.acquisition_inputs = self.health_inputs = input_channel
        self.tam = messaging.TAMSensorMsgReader()
        self.tam.subscribeTo(tam_message)
        self.gate = CommandHealthGate()
        self.sample: MagneticSample | None = None
        self.snapshot: ControlSnapshot | None = None
        self.pending: PendingCalculation | None = None
        self.reset_generation = self.revocation_generation = self.command_id = 0
        self.capture_generation = (0, 0, 0)
        self.was_healthy = False
        self.last_reset_ns = None
        self.early_nav = None
        self.capture_nav_override = None  # owned diagnostic copy, validation hook only
        self.history, self.trace, self.order = [], [], []
        self.models = []  # Keep scheduled Python callbacks alive.

    def generation(self, health):
        return self.reset_generation, health.acquisition_count, self.revocation_generation

    def early(self, tick):
        self.early_nav = self.port.snapshot()
        self.order.append((tick, 595, "pre_MEKF", self.early_nav.header_ns,
                           int(self.tam.timeWritten()) if self.tam.isWritten() else None))

    def acquire(self, tick):
        if tick % self.cycle.period_ns == 0:
            self.sample = self.snapshot = self.pending = None
        self.order.append((tick, 587, "input_tap", self.acquisition_inputs.time_written_ns))
        if tick % self.cycle.period_ns != self.cycle.sample_offset_ns:
            return
        batch = self.acquisition_inputs.read()
        selected = [s for s in batch.deliveries if s.sensor == "magnetic" and s.epoch_ns == tick]
        self.sample = None
        if len(selected) == 1:
            field = selected[0]
            # Existing bridge already validates actual native dipole/torque/quiet
            # history via magnetic_cycle_sample. Do not substitute truth field.
            self.sample = MagneticSample(tuple(self.tam().tam_S),
                self.config.sensors.magnetometer_dcm_SB.value, field.epoch_ns,
                int(self.tam.timeWritten()), field.valid, field.valid, "SAMPLE", True,
                field.source + "; stored by 7F-2C input tap, 2026-09-30",
                tuple(field.reference_n), field.reference_epoch_ns)

    def capture(self, tick):
        nav, decision = self.port.snapshot(), deepcopy(self.consumer.history[-1])
        reset = (self.health_inputs.time_written_ns == tick and self.health_inputs.read().reset_acquisition)
        if reset and self.last_reset_ns != tick:
            self.gate.notify_reset(tick)
            self.last_reset_ns = tick
            self.reset_generation += 1
        health = self.gate.observe(nav, decision, tick)
        if self.was_healthy and not health.healthy:
            self.revocation_generation += 1
        self.was_healthy = health.healthy
        row = {"tick": tick, "nav": nav, "decision": decision, "reset": reset,
               "health": health, "sample": self.sample, "capture_result": None,
               "capture_nav": None, "compute": None, "application": None}
        if tick % self.cycle.period_ns == self.cycle.sample_offset_ns:
            self.snapshot = None
            start = tick // self.cycle.period_ns * self.cycle.period_ns
            plan = CycleWindow(self.cycle.period_ns, self.cycle.sample_offset_ns, self.cycle.compute_offset_ns,
                start+self.cycle.compute_offset_ns, "Existing magnetic cycle: "+self.cycle.fingerprint())
            capture_nav = self.capture_nav_override or nav
            self.capture_nav_override = None
            row["capture_nav"], row["plan"] = capture_nav, plan
            if self.sample is not None:
                result = capture_control_snapshot(capture_nav, decision, self.sample, plan, tick,
                    navigation_provenance="Live MEKF NavAtt + Phase 7E quality/consumer; 7F-2C 2026-09-30")
                self.snapshot, row["capture_result"] = result.snapshot, result
            self.capture_generation = self.generation(health)
        self.trace.append(row)
        self.order.append((tick, 554, "capture_health", nav.header_ns, nav.quality_header_ns))

    def compute(self, tick):
        if tick % self.cycle.period_ns != self.cycle.compute_offset_ns:
            return
        self.pending = None
        if self.snapshot is not None:
            command = evaluate_snapshot(self.snapshot, tick, self.config)
            self.command_id += 1
            start = tick // self.cycle.period_ns * self.cycle.period_ns
            self.pending = PendingCalculation(self.snapshot, command, self.capture_generation,
                tick//self.cycle.period_ns, self.command_id, start+self.cycle.actuation_offset_ns,
                start+self.cycle.period_ns)
        nav, decision = self.port.snapshot(), deepcopy(self.consumer.history[-1])
        result = self.gate.evaluate(self.snapshot, self.pending.command if self.pending else None, nav, decision, tick)
        self.trace[-1]["compute"] = (self.pending, nav, decision, result)
        self.order.append((tick, 552, "compute", result.command_computation_epoch_ns))

    def evaluate(self, tick):
        application = tick % self.cycle.period_ns >= self.cycle.actuation_offset_ns
        nav, decision = self.port.snapshot(), deepcopy(self.consumer.history[-1])
        pending = self.pending
        result = None
        reason = "not_application_boundary"
        generation_match = cycle_match = False
        if application:
            result = self.gate.evaluate(pending.snapshot if pending else None, pending.command if pending else None,
                                       nav, decision, tick, application_cycle=self.cycle)
            generation_match = pending is not None and pending.generation == self.generation(result.current_health)
            cycle_match = (pending is not None and pending.cycle_index == tick//self.cycle.period_ns
                           and pending.application_start_ns <= tick < pending.application_end_ns)
            reason = (result.inhibition_reason or ("source_generation_mismatch" if not generation_match else
                      "cycle_generation_mismatch" if not cycle_match else ""))
            self.trace[-1]["application"] = (pending, nav, decision, result)
        health = result.current_health if result else self.trace[-1]["health"]
        sample = pending.snapshot if pending else self.snapshot
        command = pending.command if pending else None
        capture_result = self.trace[-1]["capture_result"]
        self.history.append({"sim_epoch_ns": tick, "cycle_index": tick//self.cycle.period_ns,
            "selected_sample_epoch_ns": tick//self.cycle.period_ns*self.cycle.period_ns+self.cycle.sample_offset_ns,
            "stored_tam_acquisition_ns": self.sample.acquisition_ns if self.sample else None,
            "stored_tam_publication_ns": self.sample.publication_ns if self.sample else None,
            "stored_tam_valid": self.sample.valid if self.sample else False,
            "stored_tam_quiet_valid": self.sample.quiet_valid if self.sample else False,
            "application_boundary": application, "intended_application_start_ns": pending.application_start_ns if pending else None,
            "intended_application_end_ns": pending.application_end_ns if pending else None,
            "sample_epoch_ns": sample.navigation_state_ns if sample else None,
            "tam_acquisition_epoch_ns": sample.tam_acquisition_ns if sample else None,
            "mekf_sample_publication_ns": sample.navigation_publication_ns if sample else None,
            "snapshot_capture_ns": sample.capture_ns if sample else None,
            "capture_rejection": capture_result.rejection_reason if capture_result else
                ("missing_selected_TAM" if tick % self.cycle.period_ns == self.cycle.sample_offset_ns and self.sample is None else ""),
            "command_computation_ns": command.evaluation_epoch_ns if command else None,
            "command_id": pending.command_id if pending else None, "command_generation": pending.generation if pending else None,
            "source_generation": self.generation(health), "generation_match": generation_match, "cycle_match": cycle_match,
            "reset_generation": self.reset_generation, "acquisition_generation": health.acquisition_count,
            "revocation_generation": self.revocation_generation,
            "current_mekf_state_ns": nav.quality.get("state_epoch_ns"), "current_mekf_publication_ns": nav.header_ns,
            "current_quality_epoch_ns": health.quality_epoch_ns, "estimator_state": health.source_state,
            "estimator_valid": nav.quality.get("valid", False), "source_healthy": health.healthy,
            "reset_epoch_ns": health.reset_epoch_ns, "lifecycle_state": health.lifecycle_state,
            "mathematical_command_valid": command.command_valid if command else False,
            "command_usable": application and not reason, "inhibition_reason": reason,
            "requested_dipole_B_Am2": command.requested_dipole_B_Am2 if command else None,
            "clipped_dipole_B_Am2": command.clipped_dipole_B_Am2 if command else None,
            "predicted_torque_B_Nm": command.predicted_torque_B_Nm if command else None,
            "configuration_fingerprint": command.configuration_fingerprint if command else None,
            "actuator_authority": False, "scope": SCOPE})
        self.order.append((tick, 550, "application" if application else "diagnostic", nav.header_ns))

    def records(self):
        # Command records are logged once, never substituted for applied telemetry.
        commands = [asdict(row["compute"][0]) for row in self.trace if row["compute"] and row["compute"][0]]
        return {"scope": SCOPE, "label": self.options.label, "cycle": self.cycle.to_dict(),
                "telemetry": self.history, "commands": commands, "execution_order": self.order}


def attach_observer(sim, cycle, config, tam_message, bridge, adapter, consumer, options):
    from attitude_navigation_consumer import Source
    observer = DisconnectedCommandObserver(cycle, config, tam_message, bridge.out,
                                           consumer.ports[Source.MEKF], consumer, options)
    for priority, name, callback in ((595, "ChainEarlyRead", observer.early), (587, "ChainInputTap", observer.acquire),
                                    (554, "ChainCapture", observer.capture), (552, "ChainCompute", observer.compute),
                                    (550, "ChainApplication", observer.evaluate)):
        model = Stage(callback)
        model.ModelTag = name
        observer.models.append(model)
        sim.AddModelToTask("DynamicsTask", model, ModelPriority=priority)
    if options.test_setup is not None:
        options.test_setup(sim, bridge.out, adapter, observer)
    return observer
