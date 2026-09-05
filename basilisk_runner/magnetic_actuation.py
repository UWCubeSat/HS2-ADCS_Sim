"""Actuation timing evidence and an explicit command-replay test source.

CONFIRMED software contract: Basilisk v2.10.2 MtbEffector.cpp/.h, inspected
2026-09-05, https://github.com/AVSLab/basilisk/tree/v2.10.2/src/simulation/dynamics/MtbEffector
and exercised by test_native_magnetic_actuation.py against the installed wheel.

mtbCmdInMsg: MTBCmdMsg.mtbDipoleCmds, signed scalar dipoles [A m^2] along
columns of MTBArrayConfigMsg.GtMatrix_B (row-major 3 x numMTB). Each scalar
is clipped to +/-maxMtbDipoles BEFORE mapping into B. magInMsg consumes
MagneticFieldMsg.magField_N [T], NOT body/sensor components. During every
computeForceTorque call, tau_B = (G clip(mu)) x (C_BN(hubSigma) B_N).
addDynamicEffector links the hub attitude and causes dynamics calls. Scheduled
UpdateState only publishes the last computed torque in mtbOutMsg.mtbNetTorque_B.
It does not latch commands or calculate a fresh torque. With default RK4 the
post-plant readback is the FINAL RK STAGE torque, not a held or averaged torque
and not exactly the torque evaluated at the accepted post-step attitude.

ASSUMED hardware configuration remains ADCSConfig's legacy three ideal body
axes and +/-[0.2, 0.2, 0.85] A m^2 limits. Native clipping is instantaneous;
current/PWM, electrical dynamics, remanence, alignment errors, interference,
thermal/duty-cycle limits and released HS-2 hardware remain outside this model.
No reference object in this file applies torque to spacecraft dynamics.
"""

import numpy as np
from Basilisk.architecture import messaging, sysModel


class MagneticInputGuard(sysModel.SysModel):
    """Observe the ACTUAL native subscribers immediately before propagation.

    At plant tick t>0, both input messages must have been acquired/published at
    t-dt and remain unchanged throughout that integration call. At t=0 there
    is no preceding interval; the zero initialization is explicitly excluded.
    Earth/WMM/sensors still acquire the new state at t AFTER propagation.
    """

    def __init__(self, effector, step_ns):
        super().__init__()
        self.effector = effector
        self.step_ns = step_ns
        self.history = []

    def UpdateState(self, current_time_ns):
        command = self.effector.mtbCmdInMsg
        field = self.effector.magInMsg
        if current_time_ns > 0:
            expected = current_time_ns - self.step_ns
            if any(not msg.isLinked() or not msg.isWritten() or msg.timeWritten() != expected
                   for msg in (command, field)):
                raise ValueError("Native magnetic inputs must both belong to the preceding command epoch")
        self.history.append((current_time_ns, command.timeWritten(), field.timeWritten()))


class ReplayDipoles(sysModel.SysModel):
    """Validation only: replay exact native command ticks, compute direct m x B.

    The controller may still run for diagnostics, but these separate messages
    drive the reference actuator. Neither native torque nor plant truth is
    copied into this source. B_B is acquired on the reference spacecraft.
    """

    def __init__(self, commands, tam_message):
        super().__init__()
        self.commands = {int(row[0]): np.asarray(row[1:], dtype=float) for row in commands}
        if len(self.commands) != len(commands) or any(len(m) != 3 or not np.isfinite(m).all()
                                                       for m in self.commands.values()):
            raise ValueError("Replay requires unique integer ticks and three finite dipoles")
        if any(float(row[0]) != int(row[0]) for row in commands):
            raise ValueError("Replay epochs must be integer nanoseconds")
        self.tam_input = messaging.TAMSensorMsgReader()
        self.tam_input.subscribeTo(tam_message)
        self.mtbCmdOutMsg = messaging.MTBCmdMsg()
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        self.mtbCmdOutMsg.write(messaging.MTBCmdMsgPayload(), 0)
        self.cmdTorqueOutMsg.write(messaging.CmdTorqueBodyMsgPayload(), 0)

    def UpdateState(self, current_time_ns):
        if current_time_ns not in self.commands or self.tam_input.timeWritten() != current_time_ns:
            raise ValueError("Replay command or reference field is missing at the exact task tick")
        dipole = self.commands[current_time_ns]
        command = messaging.MTBCmdMsgPayload()
        command.mtbDipoleCmds = dipole.tolist() + [0.0] * (len(command.mtbDipoleCmds) - 3)
        expected = messaging.CmdTorqueBodyMsgPayload()
        expected.torqueRequestBody = np.cross(dipole, self.tam_input().tam_S).tolist()
        self.mtbCmdOutMsg.write(command, current_time_ns, self.moduleID)
        self.cmdTorqueOutMsg.write(expected, current_time_ns, self.moduleID)
