"""Phase 7G-1 FIRST MEKF CLOSED-LOOP DEVELOPMENT INTEGRATION, 2026-09-30.

NOMINAL SHORT-RUN ONLY / NOT FLIGHT VALIDATED. This is the sole opt-in native
MTBCmd publisher. The 7F observer and its snapshot/math/health algorithms remain
unchanged and own no command endpoint. Truth is used only by the cycle driver's
independent expected-torque diagnostic, never as a replacement control input.
"""
from __future__ import annotations

import numpy as np
from Basilisk.architecture import messaging, sysModel

from basilisk_adcs_adapter import MAX_EFF_CNT
from disconnected_command_observer import DisconnectedCommandObserver
from magnetic_control_cycle import MagneticCycleDriver

SIMPLE_NAV_REFERENCE = "SIMPLE_NAV_REFERENCE"
MEKF_DEVELOPMENT = "MEKF_DEVELOPMENT"
SCOPE = "FIRST MEKF CLOSED-LOOP DEVELOPMENT INTEGRATION / NOMINAL SHORT-RUN ONLY / NOT FLIGHT VALIDATED"


class MEKFCommandOwner(sysModel.SysModel):
    """Publish after the current application decision, for [t, t + plant step).

    A fresh zero payload is the default on EVERY invocation. No last usable
    command is retained. Generation/health/cycle policy belongs to the existing
    7F chain; this boundary checks its delivery epoch and publishes its result.
    """
    def __init__(self, driver: MagneticCycleDriver):
        super().__init__()
        if not driver.external_command_owner:
            raise ValueError("MEKF ownership requires the cycle driver's publisher to be disabled")
        self.driver = driver
        self.observer: DisconnectedCommandObserver | None = None
        self.mtbCmdOutMsg = messaging.MTBCmdMsg()
        self.unused_cycle_input = messaging.MTBCmdMsgReader()
        self.unused_cycle_input.subscribeTo(driver.mtbCmdOutMsg)
        # Diagnostic message only; the driver computes its current-truth m x B.
        self.cmdTorqueOutMsg = driver.cmdTorqueOutMsg
        self.history: list[dict] = []

    def Reset(self, tick):
        self.mtbCmdOutMsg.write(messaging.MTBCmdMsgPayload(), tick, self.moduleID)
        self.history.clear()

    def UpdateState(self, tick):
        tick = int(tick)
        observer = self.observer
        row = observer.history[-1] if observer is not None and observer.history else {}
        current = row.get("sim_epoch_ns") == tick
        pending = observer.pending if observer is not None and current else None
        usable = bool(current and row.get("command_usable") and pending is not None)
        command = np.zeros(3)
        if usable and pending is not None:
            command = np.asarray(pending.command.clipped_dipole_B_Am2, dtype=float)
        payload = messaging.MTBCmdMsgPayload()
        payload.mtbDipoleCmds = command.tolist() + [0.]*(MAX_EFF_CNT-3)
        self.mtbCmdOutMsg.write(payload, tick, self.moduleID)

        # Populate controller diagnostics from the actual immutable MEKF
        # calculation. Do not run or report the unused SimpleNav calculation.
        driver = self.driver
        snapshot = observer.snapshot if observer is not None and current else None
        if snapshot is not None:
            # Capture is already complete at this point, including on SAMPLE
            # ticks before a calculation exists. Keep these fields frozen from
            # that acquisition onward, not switched from SimpleNav at compute.
            driver.sample_w = np.array(snapshot.omega_BN_B_rad_s)
            driver.sample_sigma = np.array(snapshot.sigma_BN)
        if pending is not None:
            math = pending.command
            driver.compute_epoch = math.evaluation_epoch_ns
            driver.requested = np.array(math.requested_dipole_B_Am2)
            driver.clipped = np.array(math.clipped_dipole_B_Am2)
            driver.sample_controller_torque = np.array(math.predicted_torque_B_Nm)
            driver.flags = list(math.saturation_flags)
            driver.core_used = math.optional_core_available and math.optional_core_enabled
        # Updates quiet history using the command ACTUALLY published now, with
        # native readback equality asserted; finishes the provisional 600 row.
        driver.complete_external_tick(tick, command)
        native, effective = driver.native_dipoles()
        effector = driver.effector
        if effector is None:  # Attachment also guarded by native_dipoles().
            raise ValueError("Missing native actuator")
        self.history.append({"publication_epoch_ns": tick, "priority": 548,
            "owner": MEKF_DEVELOPMENT, "publisher_module_id": self.moduleID,
            "subscriber_module_id": int(effector.mtbCmdInMsg.moduleID()),
            "subscriber_epoch_ns": int(effector.mtbCmdInMsg.timeWritten()),
            "gate_epoch_ns": row.get("sim_epoch_ns"), "command_usable": usable,
            "inhibition_reason": row.get("inhibition_reason", "missing_current_chain_decision"),
            "command_id": pending.command_id if pending else None,
            "generation": pending.generation if pending else None,
            "cycle_index": row.get("cycle_index"), "sample_epoch_ns": row.get("sample_epoch_ns"),
            "computation_epoch_ns": row.get("command_computation_ns"),
            "published_dipole_Am2": command.tolist(), "native_input_dipole_Am2": native.tolist(),
            "effective_dipole_B_Am2": effective.tolist(),
            "unused_cycle_message_written": bool(self.unused_cycle_input.isWritten()),
            "application_start_ns": tick, "application_end_ns": tick+driver.step_ns})
