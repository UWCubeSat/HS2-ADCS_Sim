"""Opt-in Phase 6A cycle; no released HS-2 hardware timing is specified here.

Source: docs/PHYSICAL_PARAMETERS.md, I1 sections 4.1.3/4.2.4. Coil-off sampling
intent is documented; burst, settling, sample phase and carrier remain TBD.
Diagnostic durations are ASSUMED / TEST-ONLY, revision 2026-09-06.

N/B and native RK4 contracts are unchanged. Only TAM acquisition and controller
execution are gated. A snapshot of BOTH nav and TAM at the sample epoch drives
the unchanged controller at the later computation event. Native MtbEffector
receives a body-axis dipole command every plant tick, held over the next step.
Current WMM B_N continues driving native dynamics independently of sample age.
No carrier, inductance, residual dipole, current decay or interference is modeled.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import hashlib
import json
from pathlib import Path
from typing import cast

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import MtbEffector
from Basilisk.utilities import RigidBodyKinematics

from hs2_sim_config import Parameter
from basilisk_adcs_adapter import MAX_EFF_CNT, PythonBdotMTQController


def test_duration(value: float, description: str) -> Parameter:
    return Parameter(value, "s", "ASSUMED", "Phase 6A architecture test; not HS-2 hardware timing",
                     "2026-09-06", "simulation clock", description + "; TEST-ONLY")


@dataclass(frozen=True)
class MagneticCycleConfig:
    quiet: Parameter
    settling: Parameter
    sample_to_compute: Parameter
    compute_to_actuate: Parameter
    actuation: Parameter

    def durations_ns(self) -> dict[str, int]:
        result = {}
        for f in fields(self):
            p = getattr(self, f.name)
            if not isinstance(p, Parameter) or p.units != "s":
                raise ValueError("Cycle durations require provenance-bearing seconds")
            p.__post_init__()
            if isinstance(p.value, bool) or not isinstance(p.value, (float, int)) or p.value < 0:
                raise ValueError("Cycle durations must be finite nonnegative seconds")
            ticks = round(p.value * 1e9)
            if abs(p.value * 1e9 - ticks) > 1e-5:
                raise ValueError("Cycle durations must resolve to integer nanoseconds")
            result[f.name] = ticks
        return result

    def validate(self, step_ns: int):
        d = self.durations_ns()
        if step_ns <= 0 or any(v % step_ns for v in d.values()):
            raise ValueError("Every cycle duration must be on the unchanged plant task clock")
        # Distinct sample/compute/apply ticks make the first architecture explicit.
        # This is a supported software granularity, not a hardware latency claim.
        if any(d[n] < step_ns for n in ("quiet", "sample_to_compute", "compute_to_actuate", "actuation")):
            raise ValueError("Quiet, compute delay, application delay and burst need at least one task step")

    @property
    def period_ns(self) -> int:
        return sum(self.durations_ns().values())

    @property
    def sample_offset_ns(self) -> int:
        d = self.durations_ns()
        return d["quiet"] + d["settling"]

    @property
    def compute_offset_ns(self) -> int:
        return self.sample_offset_ns + self.durations_ns()["sample_to_compute"]

    @property
    def actuation_offset_ns(self) -> int:
        return self.compute_offset_ns + self.durations_ns()["compute_to_actuate"]

    def phase(self, tick: int, step_ns: int) -> tuple[str, int, int]:
        start = tick // self.period_ns * self.period_ns
        offset = tick - start
        q = self.durations_ns()["quiet"]
        boundaries = (("COIL_OFF", 0, step_ns), ("QUIET", step_ns, q),
                      ("SETTLING", q, self.sample_offset_ns),
                      ("SAMPLE", self.sample_offset_ns, self.compute_offset_ns),
                      ("COMPUTE", self.compute_offset_ns, self.actuation_offset_ns),
                      ("ACTUATE", self.actuation_offset_ns, self.period_ns))
        for name, lo, hi in boundaries:
            if lo <= offset < hi:
                return name, start + lo, start + hi
        raise ValueError("Tick outside the configured cycle")

    def to_dict(self):
        return {"durations": asdict(self), "derived": {
            "period_s": self.period_ns * 1e-9, "sample_integration_duration_s": 0.0,
            "sample_offset_s": self.sample_offset_ns * 1e-9,
            "compute_offset_s": self.compute_offset_ns * 1e-9,
            "actuation_offset_s": self.actuation_offset_ns * 1e-9,
            "coil_off_duration_s": self.actuation_offset_ns * 1e-9,
            "actuation_duty_fraction": self.durations_ns()["actuation"] / self.period_ns},
            "sample_model": "ASSUMED ideal instantaneous TAM acquisition, not a physical integration aperture"}

    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()

    @classmethod
    def from_dict(cls, value):
        config = cls(**{name: Parameter(**p) for name, p in value["durations"].items()})
        if config.to_dict() != value:
            raise ValueError("Cycle derived values or metadata disagree with its durations")
        return config

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def diagnostic_cycle_config() -> MagneticCycleConfig:
    return MagneticCycleConfig(test_duration(0.2, "Initial quiet portion"),
        test_duration(0.2, "Additional settling portion; no electrical decay model"),
        test_duration(0.1, "Sample-to-compute event delay"),
        test_duration(0.1, "Compute-to-application event delay"),
        test_duration(0.4, "Held dipole burst"))


class MagneticCycleDriver(sysModel.SysModel):
    """Own the actual gated TAM/controller calls and native command messages.

    Scheduled after current nav/WMM and native post-plant torque publication.
    The wrapped TAM/controller are NOT independently scheduled. Commands are
    published each task tick for the following interval; computation epochs are
    separate and are not relabeled when a held command is republished.
    """
    def __init__(self, cycle, step_ns, tam, nav_message, state_message, field_message,
                 controller_config, strict=True):
        super().__init__()
        cycle.validate(step_ns)
        self.cycle, self.step_ns, self.tam, self.strict = cycle, step_ns, tam, strict
        self.nav_message, self.state_message, self.field_message = nav_message, state_message, field_message
        self.current_readers = (messaging.NavAttMsgReader(), messaging.SCStatesMsgReader(), messaging.MagneticFieldMsgReader())
        for reader, message in zip(self.current_readers, (nav_message, state_message, field_message)):
            reader.subscribeTo(message)
        self.sample_nav = messaging.NavAttMsg()
        self.controller = PythonBdotMTQController(controller_config)
        self.controller.navAttInMsg.subscribeTo(self.sample_nav)
        self.controller.tamSensorInMsg.subscribeTo(tam.tamDataOutMsg)
        self.config = controller_config
        self.mtbCmdOutMsg = messaging.MTBCmdMsg()
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        self.effector: MtbEffector.MtbEffector | None = None
        self.history = []

    def Reset(self, tick):
        self.tam.Reset(tick)
        self.tam.tamDataOutMsg.write(messaging.TAMSensorMsgPayload(), tick)
        self.sample_nav.write(messaging.NavAttMsgPayload(), tick)
        self.controller.Reset(tick)
        self.mtbCmdOutMsg.write(messaging.MTBCmdMsgPayload(), tick)
        self.cmdTorqueOutMsg.write(messaging.CmdTorqueBodyMsgPayload(), tick)
        self.sample_epoch = self.compute_epoch = -1
        self.valid = False
        self.disabled_since = int(tick)  # Known zero startup, no invented pre-run settling credit.
        self.valid_samples = self.rejected_samples = 0
        self.requested = np.zeros(3)
        self.clipped = np.zeros(3)
        self.sample_b = self.sample_bn = self.sample_w = self.sample_sigma = np.zeros(3)
        self.sample_controller_torque = np.zeros(3)
        self.flags = [False] * 3
        self.core_used = False
        self.history.clear()

    def native_dipoles(self):
        if self.effector is None:
            raise ValueError("Cycle driver requires its attached native effector")
        command = np.asarray(self.effector.mtbCmdInMsg().mtbDipoleCmds, dtype=float)[:3]
        p = self.effector.mtbParamsInMsg()
        limits = np.asarray(p.maxMtbDipoles, dtype=float)[:3]
        axes = np.asarray(p.GtMatrix_B, dtype=float)[:9].reshape(3, 3)
        return command, axes @ np.clip(command, -limits, limits)

    def acquire(self, tick):
        command, applied = self.native_dipoles()
        # native_dipoles() has checked attachment; its guard is not inferred here.
        effector = cast(MtbEffector.MtbEffector, self.effector)
        torque = np.asarray(effector.torqueExternalPntB_B, dtype=float).reshape(3)
        start = tick // self.cycle.period_ns * self.cycle.period_ns
        quiet_age = tick - self.disabled_since if self.disabled_since >= 0 else -1
        valid = (tick == start + self.cycle.sample_offset_ns
                 and not np.any(command) and not np.any(applied) and not np.any(torque)
                 and quiet_age >= self.cycle.sample_offset_ns)
        if not valid:
            self.rejected_samples += 1
            self.valid = False
            if self.strict:
                raise ValueError("INVALID magnetic acquisition: phase, native dipole/torque or quiet age")
            return
        if any(not msg.isWritten() or msg.timeWritten() != tick for msg in
               (*self.current_readers, self.tam.stateInMsg, self.tam.magInMsg)):
            raise ValueError("Magnetic acquisition requires current field, state and nav epochs")
        self.tam.UpdateState(tick)  # Actual sensor execution occurs ONLY here.
        self.sample_nav.write(self.nav_message.read(), tick, self.moduleID)
        self.sample_epoch = tick
        self.sample_b = np.asarray(self.tam.tamDataOutMsg.read().tam_S, dtype=float)
        self.sample_bn = np.asarray(self.field_message.read().magField_N, dtype=float)
        nav = self.sample_nav.read()
        self.sample_w = np.asarray(nav.omega_BN_B, dtype=float)
        self.sample_sigma = np.asarray(nav.sigma_BN, dtype=float)
        self.valid = True
        self.valid_samples += 1

    def compute(self, tick):
        start = tick // self.cycle.period_ns * self.cycle.period_ns
        if (not self.valid or tick != start + self.cycle.compute_offset_ns
                or self.sample_epoch != start + self.cycle.sample_offset_ns
                or self.controller.navAttInMsg.timeWritten() != self.sample_epoch
                or self.controller.tamSensorInMsg.timeWritten() != self.sample_epoch
                or self.compute_epoch == tick):
            raise ValueError("Controller refused INVALID/stale sample or inconsistent computation epoch")
        self.controller.UpdateState(tick)
        last = self.controller.history[-1]
        if not last["controller_valid"]:
            raise ValueError("Cycle controller returned invalid command")
        self.compute_epoch = tick
        self.requested = self.config.dipoleCommandGain * np.cross(self.sample_w, self.sample_b)
        self.clipped = np.asarray(self.controller.mtbCmdOutMsg.read().mtbDipoleCmds, dtype=float)[:3]
        self.sample_controller_torque = np.asarray(self.controller.cmdTorqueOutMsg.read().torqueRequestBody, dtype=float)
        self.flags = [bool(last[f"saturation_{a}"]) for a in "xyz"]
        self.core_used = bool(last["using_cpp_core"])
        # Confirm the unchanged controller actually consumed the frozen snapshot.
        # These keys are written as floats in the controller's heterogeneous history.
        if (not np.array_equal([cast(float, last[f"B_B_{a}_T"]) for a in "xyz"], self.sample_b)
                or not np.array_equal([cast(float, last[f"omega_B_{a}_rad_s"]) for a in "xyz"], self.sample_w)):
            raise ValueError("Controller consumed a different field/state snapshot")

    def UpdateState(self, tick):
        tick = int(tick)
        start = tick // self.cycle.period_ns * self.cycle.period_ns
        phase, phase_start, phase_end = self.cycle.phase(tick, self.step_ns)
        pre_command, pre_applied = self.native_dipoles()
        # native_dipoles() has checked attachment before any effector readback.
        effector = cast(MtbEffector.MtbEffector, self.effector)
        # Any independently observed energization interrupts the quiet history,
        # including a command injected outside this driver during a fault test.
        if (np.any(pre_command) or np.any(pre_applied)
                or np.any(np.asarray(effector.torqueExternalPntB_B, dtype=float))):
            self.disabled_since = -1
        if tick == start:
            self.valid = False  # Previous cycle samples never authorize a new burst.
            self.requested = np.zeros(3)
            self.clipped = np.zeros(3)
            self.flags = [False]*3
        sample_event = tick == start + self.cycle.sample_offset_ns
        compute_event = tick == start + self.cycle.compute_offset_ns
        if sample_event:
            self.acquire(tick)
        consumed = False
        if compute_event and (self.valid or self.strict):
            self.compute(tick)
            consumed = True
        command = np.zeros(3)
        if phase == "ACTUATE" and self.valid:
            if self.compute_epoch != start + self.cycle.compute_offset_ns or tick < self.compute_epoch:
                raise ValueError("Actuation before this cycle's computed command")
            command = self.clipped.copy()
        if np.any(command):
            self.disabled_since = -1
        elif self.disabled_since < 0:
            self.disabled_since = tick
        payload = messaging.MTBCmdMsgPayload()
        payload.mtbDipoleCmds = command.tolist() + [0.]*(MAX_EFF_CNT-3)
        self.mtbCmdOutMsg.write(payload, tick, self.moduleID)
        native_command, effective = self.native_dipoles()
        if not np.array_equal(command, native_command):
            raise ValueError("Native subscriber does not receive the gated electrical command")
        # Diagnostic expectation at CURRENT truth epoch, never used by controller
        # or native dynamics. The controller's sample-epoch torque is separate.
        state = self.state_message.read()
        field_b = np.asarray(RigidBodyKinematics.MRP2C(state.sigma_BN)) @ np.asarray(self.field_message.read().magField_N)
        expected = np.cross(effective, field_b)
        tp = messaging.CmdTorqueBodyMsgPayload()
        tp.torqueRequestBody = expected.tolist()
        self.cmdTorqueOutMsg.write(tp, tick, self.moduleID)
        currents = command / np.asarray(self.config.mtqDipoleGain_Am2_A)
        powers = currents**2 * np.asarray(self.config.mtqResistance_Ohm)
        limits = np.minimum(np.asarray(self.config.mtqDipoleLimit_Am2),
                           np.asarray(self.config.mtqCurrentLimit_A)*np.asarray(self.config.mtqDipoleGain_Am2_A))
        row = {"time_s": tick*1e-9, "cycle_phase": phase, "cycle_index": tick//self.cycle.period_ns,
            "cycle_start_ns": start, "cycle_phase_start_ns": phase_start, "cycle_phase_end_ns": phase_end,
            "cycle_sample_event": sample_event, "cycle_sample_valid": self.valid,
            "cycle_controller_consumed_sample": consumed, "cycle_sample_epoch_ns": self.sample_epoch,
            "cycle_compute_epoch_ns": self.compute_epoch, "cycle_actuation_start_ns": start+self.cycle.actuation_offset_ns,
            "cycle_actuation_end_ns": start+self.cycle.period_ns,
            "cycle_disabled_since_ns": self.disabled_since,
            "cycle_time_since_disabled_ns": tick-self.disabled_since if self.disabled_since >= 0 else -1,
            "cycle_valid_sample_count": self.valid_samples, "cycle_rejected_sample_count": self.rejected_samples,
            "pcoil_total_W": float(powers.sum()), "controller_valid": consumed, "using_cpp_core": self.core_used}
        for j, a in enumerate("xyz"):
            row[f"i{a}_A"] = float(currents[j])
            row[f"pcoil_{a}_W"] = float(powers[j])
            row[f"saturation_{a}"] = bool(self.flags[j])
            for prefix, value in (("cycle_requested_dipole", self.requested), ("cycle_clipped_dipole", self.clipped),
                    ("cycle_electrical_normalized", command/limits), ("cycle_native_input_effective_dipole", effective),
                    ("cycle_pre_native_command", pre_command), ("cycle_pre_native_effective_dipole", pre_applied),
                    ("cycle_sample_B_B", self.sample_b), ("cycle_sample_B_N", self.sample_bn),
                    ("cycle_sample_omega_B", self.sample_w), ("cycle_sample_sigma_BN", self.sample_sigma),
                    ("cycle_controller_sample_torque", self.sample_controller_torque)):
                row[f"{prefix}_{a}"] = float(value[j])
        self.history.append(row)
