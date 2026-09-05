"""Basilisk-to-ADCS adapter layer.

This file intentionally does not import Basilisk.ExternalModules. It provides a
Python Basilisk SysModel for fast iteration and optionally calls a separately
built pybind module named ``adcs_core`` if it is importable.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, List, Sequence
import math
import numpy as np

try:
    import adcs_core as _adcs_core  # optional pybind module from cpp_adcs_core
except Exception:  # pragma: no cover - local optional dependency
    _adcs_core = None

from Basilisk.architecture import sysModel, messaging, bskLogging


MAX_EFF_CNT = 36


@dataclass
class ADCSConfig:
    dipoleCommandGain: float = 67200.0
    mtqDipoleGain_Am2_A: Sequence[float] = (2.3, 2.3, 0.85 / math.sqrt(1.75 / 4.4))
    mtqResistance_Ohm: Sequence[float] = (51.0, 51.0, 4.4)
    mtqCurrentLimit_A: Sequence[float] = (5.0 / 51.0, 5.0 / 51.0, math.sqrt(1.75 / 4.4))
    mtqDipoleLimit_Am2: Sequence[float] = (0.2, 0.2, 0.85)
    minMagField_T: float = 1.0e-12
    use_cpp_core_if_available: bool = True


def _cross(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    return np.cross(np.asarray(a, dtype=float), np.asarray(b, dtype=float))


def _controller_step_python(time_s: float, omega_B_rad_s: Sequence[float], mag_B_T: Sequence[float], cfg: ADCSConfig) -> Dict[str, object]:
    omega = np.asarray(omega_B_rad_s, dtype=float)
    mag = np.asarray(mag_B_T, dtype=float)

    result = {
        "commanded_magnetic_dipole_B_Am2": [0.0, 0.0, 0.0],
        "commanded_coil_current_A": [0.0, 0.0, 0.0],
        "commanded_control_torque_B_Nm": [0.0, 0.0, 0.0],
        "coil_power_W": [0.0, 0.0, 0.0],
        "coil_power_total_W": 0.0,
        "saturation_flags": [False, False, False],
        "valid": True,
    }

    if not (np.isfinite(time_s) and np.all(np.isfinite(omega)) and np.all(np.isfinite(mag))):
        result["valid"] = False
        return result

    if float(np.linalg.norm(mag)) < cfg.minMagField_T:
        result["valid"] = False
        return result

    desired_dipole = cfg.dipoleCommandGain * _cross(omega, mag)
    gains = np.asarray(cfg.mtqDipoleGain_Am2_A, dtype=float)
    resist = np.asarray(cfg.mtqResistance_Ohm, dtype=float)
    current_limits = np.asarray(cfg.mtqCurrentLimit_A, dtype=float)
    dipole_limits = np.asarray(cfg.mtqDipoleLimit_Am2, dtype=float)

    currents = np.zeros(3)
    dipoles = np.zeros(3)
    powers = np.zeros(3)
    flags = [False, False, False]

    for i in range(3):
        if gains[i] <= 0.0 or current_limits[i] <= 0.0 or dipole_limits[i] <= 0.0 or resist[i] < 0.0:
            result["valid"] = False
            flags[i] = True
            continue
        allowed_current = min(current_limits[i], dipole_limits[i] / gains[i])
        raw_current = desired_dipole[i] / gains[i]
        cmd_current = float(np.clip(raw_current, -allowed_current, allowed_current))
        currents[i] = cmd_current
        dipoles[i] = cmd_current * gains[i]
        powers[i] = resist[i] * cmd_current * cmd_current
        flags[i] = abs(raw_current - cmd_current) > 1.0e-15

    torque = _cross(dipoles, mag)
    result.update({
        "commanded_magnetic_dipole_B_Am2": dipoles.tolist(),
        "commanded_coil_current_A": currents.tolist(),
        "commanded_control_torque_B_Nm": torque.tolist(),
        "coil_power_W": powers.tolist(),
        "coil_power_total_W": float(np.sum(powers)),
        "saturation_flags": flags,
    })
    return result


def controller_step(time_s: float, omega_B_rad_s: Sequence[float], mag_B_T: Sequence[float], cfg: ADCSConfig | None = None) -> Dict[str, object]:
    cfg = cfg or ADCSConfig()
    if cfg.use_cpp_core_if_available and _adcs_core is not None:
        try:
            return _adcs_core.step(float(time_s), list(omega_B_rad_s), list(mag_B_T), asdict(cfg))
        except Exception:
            # Fall back to Python so scenario bring-up is not blocked by pybind packaging.
            pass
    return _controller_step_python(time_s, omega_B_rad_s, mag_B_T, cfg)


class PythonBdotMTQController(sysModel.SysModel):
    """Fast-prototyping Basilisk Python module for magnetorquer detumble.

    Inputs:
        navAttInMsg: NavAttMsgPayload, usually from simpleNav
        tamSensorInMsg: TAMSensorMsgPayload, magnetometer output; assumes S == B

    Output:
        mtbCmdOutMsg: MTBCmdMsgPayload, first 3 dipoles populated
    """

    def __init__(self, config: ADCSConfig | None = None, *args):
        super().__init__(*args)
        self.config = config or ADCSConfig()
        self.navAttInMsg = messaging.NavAttMsgReader()
        self.tamSensorInMsg = messaging.TAMSensorMsgReader()
        self.mtbCmdOutMsg = messaging.MTBCmdMsg()
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        self.history: List[Dict[str, object]] = []

    def Reset(self, CurrentSimNanos):
        if not self.navAttInMsg.isLinked():
            self.bskLogger.bskLog(bskLogging.BSK_ERROR, "PythonBdotMTQController.navAttInMsg is not linked.")
        if not self.tamSensorInMsg.isLinked():
            self.bskLogger.bskLog(bskLogging.BSK_ERROR, "PythonBdotMTQController.tamSensorInMsg is not linked.")
        payload = self.mtbCmdOutMsg.zeroMsgPayload
        payload.mtbDipoleCmds = [0.0] * MAX_EFF_CNT
        self.mtbCmdOutMsg.write(payload, CurrentSimNanos, self.moduleID)
        torque_payload = self.cmdTorqueOutMsg.zeroMsgPayload
        torque_payload.torqueRequestBody = [0.0, 0.0, 0.0]
        self.cmdTorqueOutMsg.write(torque_payload, CurrentSimNanos, self.moduleID)
        self.history.clear()

    def UpdateState(self, CurrentSimNanos):
        nav = self.navAttInMsg()
        tam = self.tamSensorInMsg()

        time_s = CurrentSimNanos * 1.0e-9
        omega = np.asarray(nav.omega_BN_B, dtype=float)[:3]
        mag_B = np.asarray(tam.tam_S, dtype=float)[:3]

        cmd = controller_step(time_s, omega, mag_B, self.config)

        payload = self.mtbCmdOutMsg.zeroMsgPayload
        dipoles3 = list(cmd["commanded_magnetic_dipole_B_Am2"])
        # SWIG's fixed-array getter returns a copy. Assign the whole array so
        # the published command contains the computed dipoles, not zeros.
        payload.mtbDipoleCmds = dipoles3 + [0.0] * (MAX_EFF_CNT - 3)
        self.mtbCmdOutMsg.write(payload, CurrentSimNanos, self.moduleID)

        torque3 = list(cmd["commanded_control_torque_B_Nm"])
        torque_payload = self.cmdTorqueOutMsg.zeroMsgPayload
        torque_payload.torqueRequestBody = torque3
        self.cmdTorqueOutMsg.write(torque_payload, CurrentSimNanos, self.moduleID)

        self.history.append({
            "time_s": time_s,
            "omega_B_x_rad_s": float(omega[0]),
            "omega_B_y_rad_s": float(omega[1]),
            "omega_B_z_rad_s": float(omega[2]),
            "B_B_x_T": float(mag_B[0]),
            "B_B_y_T": float(mag_B[1]),
            "B_B_z_T": float(mag_B[2]),
            "mcmd_x_Am2": float(dipoles3[0]),
            "mcmd_y_Am2": float(dipoles3[1]),
            "mcmd_z_Am2": float(dipoles3[2]),
            "ix_A": float(cmd["commanded_coil_current_A"][0]),
            "iy_A": float(cmd["commanded_coil_current_A"][1]),
            "iz_A": float(cmd["commanded_coil_current_A"][2]),
            "pcoil_x_W": float(cmd["coil_power_W"][0]),
            "pcoil_y_W": float(cmd["coil_power_W"][1]),
            "pcoil_z_W": float(cmd["coil_power_W"][2]),
            "pcoil_total_W": float(cmd["coil_power_total_W"]),
            "control_torque_B_x_Nm": float(torque3[0]),
            "control_torque_B_y_Nm": float(torque3[1]),
            "control_torque_B_z_Nm": float(torque3[2]),
            "control_torque_B_mag_Nm": float(np.linalg.norm(torque3)),
            "saturation_x": bool(cmd["saturation_flags"][0]),
            "saturation_y": bool(cmd["saturation_flags"][1]),
            "saturation_z": bool(cmd["saturation_flags"][2]),
            "controller_valid": bool(cmd["valid"]),
            "using_cpp_core": bool(_adcs_core is not None and self.config.use_cpp_core_if_available),
        })
