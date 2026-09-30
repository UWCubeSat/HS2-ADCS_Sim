"""Phase 7F-2A DISCONNECTED CONTROLLER COMMAND MATHEMATICS, 2026-09-30.

NO ACTUATOR AUTHORITY / NOT CLOSED LOOP / NOT FLIGHT VALIDATED.
Source: unchanged basilisk_adcs_adapter.controller_step and hs2_sim_config.
Inputs must be accepted Phase 7F-1 snapshots. This stateless calculation does
not assess current health, implement inhibition/retention, or authorize commands.
It returns immutable Python data, never Basilisk command messages.
"""
from __future__ import annotations

from dataclasses import dataclass
import json

import basilisk_adcs_adapter as production
from control_input_snapshot import ControlSnapshot
from hs2_sim_config import DEFAULT_CONFIG, HS2SimConfig


Vector = tuple[float, float, float]


@dataclass(frozen=True)
class CommandMathematics:
    """command_valid is the existing computation's flag, NOT permission to use.

    Dipoles are mathematical body-axis commands [A m^2]; current is [A]. Torque
    is predicted m_B cross sampled B_B [N m], not applied/effector telemetry.
    The snapshot retains input vectors, attitude, source, sample/publication/capture
    epochs and provenance. No native message ABI or subscriber exists here.
    """
    snapshot: ControlSnapshot
    evaluation_epoch_ns: int
    requested_dipole_B_Am2: Vector
    clipped_dipole_B_Am2: Vector
    coil_current_A: Vector
    predicted_torque_B_Nm: Vector
    saturation_flags: tuple[bool, bool, bool]
    command_valid: bool
    configuration_fingerprint: str
    configuration_json: str
    optional_core_available: bool
    optional_core_enabled: bool
    controller_entry_point: str = "basilisk_adcs_adapter.controller_step"
    actuator_authority: bool = False
    scope: str = "DISCONNECTED CONTROLLER COMMAND MATHEMATICS / NOT CLOSED LOOP / NOT FLIGHT VALIDATED"


def evaluate_snapshot(snapshot: ControlSnapshot, evaluation_epoch_ns: int,
                      config: HS2SimConfig = DEFAULT_CONFIG) -> CommandMathematics:
    """Call production math once, using the frozen B-frame rate and TAM.

    Preconditions are interface assertions, not a fault-management policy. Phase
    7F-2B must separately establish evaluation-time health/inhibition before any
    usable command path exists. No SimpleNav substitution or retained command.
    """
    if not isinstance(snapshot, ControlSnapshot) or snapshot.coherence_status != "COHERENT_AT_CAPTURE":
        raise ValueError("accepted Phase 7F-1 ControlSnapshot required")
    error = snapshot.evaluation_rejection(evaluation_epoch_ns)
    if error:
        raise ValueError(error)
    cfg = production.ADCSConfig.from_sim_config(config)
    result = production.controller_step(evaluation_epoch_ns*1e-9,
        snapshot.omega_BN_B_rad_s, snapshot.tam_B_T, cfg)
    # The production result omits the unclipped request. This sole diagnostic
    # expression mirrors MagneticCycleDriver's request telemetry and reuses its
    # controller cross-product helper. Clipping/current/torque are NOT duplicated.
    requested = cfg.dipoleCommandGain * production._cross(snapshot.omega_BN_B_rad_s, snapshot.tam_B_T)

    def vector(values: production.VectorInput) -> Vector:
        return float(values[0]), float(values[1]), float(values[2])

    flags = result["saturation_flags"]
    return CommandMathematics(snapshot, int(evaluation_epoch_ns), vector(requested),
        vector(result["commanded_magnetic_dipole_B_Am2"]), vector(result["commanded_coil_current_A"]),
        vector(result["commanded_control_torque_B_Nm"]),
        (bool(flags[0]), bool(flags[1]), bool(flags[2])), bool(result["valid"]),
        config.fingerprint(), json.dumps(config.to_dict(), sort_keys=True, allow_nan=False),
        production._adcs_core is not None, cfg.use_cpp_core_if_available)
