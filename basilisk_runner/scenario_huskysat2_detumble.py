"""HuskySat-style Basilisk WMM + magnetorquer detumble scenario.

This is the fast workflow path:
- pip-installed Basilisk
- no Basilisk source rebuild
- no Basilisk.ExternalModules import
- Python Basilisk controller module with optional separate pybind core

Telemetry contract (schema 4; seconds/nanoseconds from 2026-01-01 00:00 UTC):
N is Earth-centered ICRF/J2000; P is the low-order IAU Earth-fixed frame
defined in magnetic_environment.py; B is the unchanged simulated body frame
(physical HS-2 axis mapping remains unresolved). sigma_BN represents B relative
to N: C_BN maps inertial components into body components, v_B = C_BN v_N.
WMM consumes r_BN_N [m], forms r_P = C_PN r_N, evaluates the field in P, and
returns B_N = C_PN.T B_P [T]. TAM uses B_S = C_SB C_BN B_N; C_SB = identity
in this ideal baseline, hence saved tam_S is B_B. No silent frame transpose.

* time_s/time_ns is the recorder task tick, after spacecraft propagation.
  r_N, v_N, sigma_BN, Euler angles and omega_B describe state_time_ns.
* sensor_state_* is a snapshot AFTER propagation and BEFORE environment/nav/TAM.
  Its epoch, the WMM input position epoch, Earth orientation epoch, WMM output
  epoch and sensor/nav/controller input epoch ALL equal the current tick.
  Earth absolute time is also saved as seconds past J2000 TDB. WMM internally
  rounds its secular-variation time to a whole second (wmm_coefficient_time_ns);
  geometry and message times retain the full task-clock precision.
* mcmd is MTBCmdMsg.mtbDipoleCmds[0:3]; control_torque is the recorded
  CmdTorqueBodyMsg.torqueRequestBody, expected m x B at the CURRENT epoch.
  Dipole and inertial field are held over the following interval [t, t+0.1 s].
* applied_torque is the selected effector's torqueExternalPntB_B sampled AFTER
  propagation and BEFORE environment/commands update. Native MtbEffector
  transforms the held B_N using hub attitude at EVERY RK4 stage. Its readback
  and native_mtbNetTorque_B message contain the FINAL RK4 STAGE torque, not an
  interval average or the exact accepted end-state torque. Their timestamps
  denote publication/sampling at t; the field/dipole inputs belong to t-0.1 s.
  In explicit direct-reference mode only, body torque is constant over the step.
  held_* are actual message/state recordings taken before propagation, including
  the PRIOR dipole, field, expected torque and pre-integration state. Native
  validation reconstructs the RK4 stages from these independent inputs; it
  never equates final-stage torque to the earlier controller torque.
  At t=0 the plant is initialized; there is no completed application interval
  or previous field/command. Application checks explicitly exclude that row.
* Currents, coil powers, saturation/validity and core-availability flags remain
  controller diagnostics at diagnostic_time_ns; they are not hardware readings.
  command_mode=replay uses separate validation messages; those diagnostics then
  describe the unused controller command, as controller_diagnostics_applied states.

All sources are matched on exact integer task ticks, never nearest-time joined.
Use sensor_state_sigma_BN with B_N/B_B for frame checks, and held/post omega for
the applied-torque dynamics check. No interpolation or time relabeling is used.
These are CONFIRMED software scheduling/source semantics for this development
configuration, not confirmed HS-2 hardware or flight timing requirements.
"""

from __future__ import annotations

from pathlib import Path
import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    import Basilisk
    from Basilisk.utilities import SimulationBaseClass, macros
    from Basilisk.simulation import spacecraft, gravityEffector, magneticFieldWMM, simpleNav, magnetometer, MtbEffector, extForceTorque
    from Basilisk.architecture import messaging
except ImportError as exc:
    raise SystemExit(
        "Basilisk is not installed in this Python environment. Run: python -m pip install -r requirements.txt"
    ) from exc

from basilisk_adcs_adapter import ADCSConfig, PythonBdotMTQController, MAX_EFF_CNT
from magnetic_environment import EarthOrientation, WMMInputGuard, MODEL_NAME
from magnetic_actuation import MagneticInputGuard, ReplayDipoles

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT_DATA = HERE / "output_data"
OUT_PLOTS = HERE / "output_plots"
OUT_DATA.mkdir(parents=True, exist_ok=True)
OUT_PLOTS.mkdir(parents=True, exist_ok=True)

MASS_KG = 2.6
LX_M, LY_M, LZ_M = 0.10, 0.10, 0.20
IXX = (MASS_KG / 12.0) * (LY_M**2 + LZ_M**2)
IYY = (MASS_KG / 12.0) * (LX_M**2 + LZ_M**2)
IZZ = (MASS_KG / 12.0) * (LX_M**2 + LY_M**2)
MU_EARTH = 3.986004418e14
EARTH_RADIUS_M = 6.371e6
ALTITUDE_M = 600e3
INCLINATION_DEG = 56.0
INITIAL_RATES = [0.8, -0.2, 0.3]
CONTROL_DT_S = 0.1
RECORD_DT_S = 1.0
MAG_NOISE_STD_T = 0.0
EPOCH_FRACTIONAL_YEAR = 2026.0
# Verified by native torque/state tests, matched-evaluation direct trajectories,
# and direct-hold step refinement. The legacy RK4 body-hold trajectory differs.
DEFAULT_ACTUATOR = "native"


def find_wmm2025_path() -> str:
    """Find WMM2025.COF without hard-coding a user-specific Basilisk build path."""
    candidates = []
    for base in [Path(Basilisk.__path__[0]), ROOT / "reference_standalone" / "original_project"]:
        if base.exists():
            candidates.extend(base.glob("**/WMM2025.COF"))
    if candidates:
        return str(candidates[0])
    raise FileNotFoundError(
        "Could not find WMM2025.COF. Install Basilisk support data or keep the standalone reference file at "
        "reference_standalone/original_project/reference/WMM2025.COF."
    )


def mrp_to_dcm(sigma):
    """Convert an MRP attitude vector to the associated direction cosine matrix.

    This is used only for diagnostic Euler-angle output.  Basilisk internally
    propagates MRPs, which remain the preferred attitude representation for
    simulation and control.
    """
    s = np.asarray(sigma, dtype=float)
    s2 = float(np.dot(s, s))
    sx = np.array([
        [0.0, -s[2], s[1]],
        [s[2], 0.0, -s[0]],
        [-s[1], s[0], 0.0],
    ])
    return np.eye(3) + (8.0 * sx @ sx - 4.0 * (1.0 - s2) * sx) / (1.0 + s2) ** 2


def dcm_to_euler321(C):
    """Return 3-2-1 Euler angles [phi, theta, psi] from a DCM."""
    C = np.asarray(C, dtype=float)
    phi = math.atan2(C[1, 2], C[2, 2])
    theta = -math.asin(float(np.clip(C[0, 2], -1.0, 1.0)))
    psi = math.atan2(C[0, 1], C[0, 0])
    return np.array([phi, theta, psi])


def configure_spacecraft():
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "HuskySat2_detumble"
    sc.hub.mHub = MASS_KG
    sc.hub.r_BcB_B = [0.0, 0.0, 0.0]
    sc.hub.IHubPntBc_B = [[IXX, 0.0, 0.0], [0.0, IYY, 0.0], [0.0, 0.0, IZZ]]

    r0 = EARTH_RADIUS_M + ALTITUDE_M
    v0 = math.sqrt(MU_EARTH / r0)
    inc = math.radians(INCLINATION_DEG)
    sc.hub.r_CN_NInit = [r0, 0.0, 0.0]
    sc.hub.v_CN_NInit = [0.0, v0 * math.cos(inc), v0 * math.sin(inc)]
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [INITIAL_RATES[0], INITIAL_RATES[1], INITIAL_RATES[2]]

    earth = gravityEffector.GravBodyData()
    earth.planetName = "earth_planet_data"
    earth.mu = MU_EARTH
    earth.isCentralBody = True
    sc.gravField.gravBodies = spacecraft.GravBodyVector([earth])
    return sc


def configure_mtb_config_message(config: ADCSConfig):
    """Create MTB layout message.

    Basilisk's MtbEffector reads GtMatrix_B as a row-major 3 x numMTB
    matrix.  For three body-aligned torque bars, the active portion must be

        [1, 0, 0,
         0, 1, 0,
         0, 0, 1]

    Do not stride by MAX_EFF_CNT here.  MtbEffector only reads the first
    3*numMTB entries of this array.
    """
    gt = [0.0] * (3 * MAX_EFF_CNT)
    gt[0] = 1.0
    gt[4] = 1.0
    gt[8] = 1.0

    max_dipoles = [0.0] * MAX_EFF_CNT
    max_dipoles[0:3] = list(config.mtqDipoleLimit_Am2)

    payload = messaging.MTBArrayConfigMsgPayload()
    payload.numMTB = 3
    payload.GtMatrix_B = gt
    payload.maxMtbDipoles = max_dipoles
    return messaging.MTBArrayConfigMsg().write(payload)


def sample_at_ticks(ticks, source_ticks, values, source_name):
    """Select exact samples; missing/duplicate/out-of-order evidence is an error."""
    ticks = np.asarray(ticks, dtype=np.int64)
    source_ticks = np.asarray(source_ticks, dtype=np.int64)
    values = np.asarray(values)
    if (len(source_ticks) == 0 or len(source_ticks) != len(values)
            or np.any(np.diff(source_ticks) <= 0)):
        raise ValueError(f"Invalid timestamps/length for {source_name}")
    indices = np.searchsorted(source_ticks, ticks)
    if np.any(indices >= len(source_ticks)) or not np.array_equal(source_ticks[indices], ticks):
        raise ValueError(f"Missing exact task-tick sample for {source_name}")
    return values[indices]


def run(stop_time_s=None, write_outputs=True, actuator=DEFAULT_ACTUATOR,
        replay_commands=None, capture_commands=False, make_plots=True):
    if actuator not in ("native", "direct"):
        raise ValueError("actuator must be native or direct")
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("DynamicsProcess")
    proc.addTask(sim.CreateNewTask("DynamicsTask", macros.sec2nano(CONTROL_DT_S)))

    sc = configure_spacecraft()

    mag = magneticFieldWMM.MagneticFieldWMM()
    mag.ModelTag = "WMM2025"
    mag.wmmDataFullPath = find_wmm2025_path()
    mag.epochDateFractionalYear = EPOCH_FRACTIONAL_YEAR
    mag.planetRadius = 6371.2e3
    mag.addSpacecraftToModel(sc.scStateOutMsg)
    # Initialize only the pre-run field sentinel. The t=0 row has no preceding
    # application interval; actual current-epoch WMM output is computed at t=0.
    mag.envOutMsgs[0].write(messaging.MagneticFieldMsgPayload(), 0)
    earth_orientation = EarthOrientation(EPOCH_FRACTIONAL_YEAR)
    earth_orientation.ModelTag = "EarthOrientation"
    mag.planetPosInMsg.subscribeTo(earth_orientation.planetOutMsg)
    wmm_guard = WMMInputGuard(mag)
    wmm_guard.ModelTag = "WMMInputGuard"

    nav = simpleNav.SimpleNav()
    nav.ModelTag = "SimpleNav"
    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)

    tam = magnetometer.Magnetometer()
    tam.ModelTag = "Magnetometer"
    tam.dcm_SB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    tam.scaleFactor = 1.0
    tam.senBias = [0.0, 0.0, 0.0]
    tam.senNoiseStd = [MAG_NOISE_STD_T, MAG_NOISE_STD_T, MAG_NOISE_STD_T]
    tam.stateInMsg.subscribeTo(sc.scStateOutMsg)
    tam.magInMsg.subscribeTo(mag.envOutMsgs[0])

    adcs_cfg = ADCSConfig()
    ctrl = PythonBdotMTQController(adcs_cfg)
    ctrl.ModelTag = "PythonBdotMTQController"
    ctrl.navAttInMsg.subscribeTo(nav.attOutMsg)
    ctrl.tamSensorInMsg.subscribeTo(tam.tamDataOutMsg)

    command_source = ctrl
    if replay_commands is not None:
        command_source = ReplayDipoles(replay_commands, tam.tamDataOutMsg)
        command_source.ModelTag = "ValidationCommandReplay"

    mtb_cfg_msg = configure_mtb_config_message(adcs_cfg)

    # Exactly one magnetic dynamic effector is attached. Native 2.10.2 reads
    # dipole/config/B_N messages and hub attitude on EVERY dynamics evaluation.
    # ExtForceTorque is retained only as the legacy constant-body-torque reference.
    if actuator == "native":
        effector = MtbEffector.MtbEffector()
        effector.mtbCmdInMsg.subscribeTo(command_source.mtbCmdOutMsg)
        effector.magInMsg.subscribeTo(mag.envOutMsgs[0])
        effector.mtbParamsInMsg.subscribeTo(mtb_cfg_msg)
    else:
        effector = extForceTorque.ExtForceTorque()
        effector.cmdTorqueInMsg.subscribeTo(command_source.cmdTorqueOutMsg)
    effector.ModelTag = "NativeMTB" if actuator == "native" else "DirectMagneticTorqueReference"
    sc.addDynamicEffector(effector)

    # Capture the previous command/field/state before propagation. These are
    # evidence for the magnetic inputs held during the interval ending now.
    rec_dt = macros.sec2nano(RECORD_DT_S)
    held_state_log = sc.scStateOutMsg.recorder(rec_dt)
    held_mag_log = mag.envOutMsgs[0].recorder(rec_dt)
    held_cmd_log = command_source.mtbCmdOutMsg.recorder(rec_dt)
    held_torque_log = command_source.cmdTorqueOutMsg.recorder(rec_dt)
    for recorder in [held_state_log, held_mag_log, held_cmd_log, held_torque_log]:
        sim.AddModelToTask("DynamicsTask", recorder, ModelPriority=1100)
    if actuator == "native":
        actuator_guard = MagneticInputGuard(effector, macros.sec2nano(CONTROL_DT_S))
        actuator_guard.ModelTag = "NativeInputEpochGuard"
        sim.AddModelToTask("DynamicsTask", actuator_guard, ModelPriority=1050)

    # Plant first: all command/field messages still belong to the preceding tick.
    sim.AddModelToTask("DynamicsTask", sc, ModelPriority=1000)
    if actuator == "native":
        # UpdateState publishes the most recent dynamics torque; it does not
        # calculate torque or latch inputs. Publish before environment/commands change.
        sim.AddModelToTask("DynamicsTask", effector, ModelPriority=980)
        native_output_log = effector.mtbOutMsg.recorder(rec_dt)
        sim.AddModelToTask("DynamicsTask", native_output_log, ModelPriority=975)
    effector_log = effector.logger(["torqueExternalPntB_B"])
    sim.AddModelToTask("DynamicsTask", effector_log, ModelPriority=975)
    sensor_state_log = sc.scStateOutMsg.recorder(rec_dt)
    sim.AddModelToTask("DynamicsTask", sensor_state_log, ModelPriority=950)

    # All environment and sensor inputs now represent this same propagated epoch.
    sim.AddModelToTask("DynamicsTask", earth_orientation, ModelPriority=925)
    sim.AddModelToTask("DynamicsTask", wmm_guard, ModelPriority=910)
    sim.AddModelToTask("DynamicsTask", mag, ModelPriority=900)
    sim.AddModelToTask("DynamicsTask", nav, ModelPriority=800)
    sim.AddModelToTask("DynamicsTask", tam, ModelPriority=700)
    sim.AddModelToTask("DynamicsTask", ctrl, ModelPriority=600)
    if replay_commands is not None:
        sim.AddModelToTask("DynamicsTask", command_source, ModelPriority=550)
    if actuator == "direct":
        sim.AddModelToTask("DynamicsTask", effector, ModelPriority=500)
    # Only ExtForceTorque must latch the NEW torque for the NEXT step here.

    sc_log = sc.scStateOutMsg.recorder(rec_dt)
    mag_log = mag.envOutMsgs[0].recorder(rec_dt)
    tam_log = tam.tamDataOutMsg.recorder(rec_dt)
    cmd_log = command_source.mtbCmdOutMsg.recorder(rec_dt)
    expected_torque_log = command_source.cmdTorqueOutMsg.recorder(rec_dt)
    nav_log = nav.attOutMsg.recorder(rec_dt)
    earth_log = earth_orientation.planetOutMsg.recorder(rec_dt)
    for recorder in [sc_log, mag_log, tam_log, cmd_log, expected_torque_log, nav_log, earth_log]:
        sim.AddModelToTask("DynamicsTask", recorder)
    if capture_commands:
        full_cmd_log = command_source.mtbCmdOutMsg.recorder()
        sim.AddModelToTask("DynamicsTask", full_cmd_log)

    r0 = EARTH_RADIUS_M + ALTITUDE_M
    orbit_period_s = 2.0 * math.pi * math.sqrt(r0**3 / MU_EARTH)
    duration_s = orbit_period_s if stop_time_s is None else float(stop_time_s)
    print(f"Running detumble scenario ({actuator}): {duration_s:.2f} s")

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration_s))
    sim.ExecuteSimulation()

    ticks = np.asarray(sc_log.times(), dtype=np.int64)
    t = ticks * macros.NANO2SEC

    def sampled(log, field):
        return sample_at_ticks(ticks, log.times(), getattr(log, field), field)

    def written(log):
        return sample_at_ticks(ticks, log.times(), log.timesWritten(), "message write time")

    r = np.asarray(sc_log.r_BN_N, dtype=float)
    v = np.asarray(sc_log.v_BN_N, dtype=float)
    sigma = np.asarray(sc_log.sigma_BN, dtype=float)
    omega = np.asarray(sc_log.omega_BN_B, dtype=float)
    omega_mag = np.linalg.norm(omega, axis=1)
    euler = np.array([dcm_to_euler321(mrp_to_dcm(s)) for s in sigma])

    B_N = sampled(mag_log, "magField_N")
    B_B = sampled(tam_log, "tam_S")
    dipole = sampled(cmd_log, "mtbDipoleCmds")[:, :3]
    expected_torque_B = sampled(expected_torque_log, "torqueRequestBody")
    applied_torque_B = sampled(effector_log, "torqueExternalPntB_B")

    diag = pd.DataFrame(ctrl.history)
    # History time_s is produced from the integer controller task tick. Recover
    # that tick to avoid floating-point equality joins; no sample shifting.
    diag_ticks = np.rint(diag["time_s"].to_numpy() * 1e9).astype(np.int64)
    diag_sampled = pd.DataFrame(sample_at_ticks(ticks, diag_ticks, diag.to_numpy(), "controller history"),
                                columns=diag.columns)
    guard_data = pd.DataFrame(wmm_guard.history)
    guard_ticks = guard_data["time_ns"].to_numpy(dtype=np.int64)

    df = pd.DataFrame({
        "telemetry_schema_version": 4,
        "actuator_mode": actuator,
        "command_mode": "replay" if replay_commands is not None else "controller",
        "controller_diagnostics_applied": replay_commands is None,
        "expected_torque_source": ("ReplayDipoles" if replay_commands is not None else "PythonBdotMTQController") + ".CmdTorqueBodyMsg",
        "applied_torque_evaluation": "final_RK4_stage" if actuator == "native" else "constant_body_hold",
        "time_s": t,
        "time_ns": ticks,
        "control_step_ns": macros.sec2nano(CONTROL_DT_S),
        "application_interval_valid": ticks > 0,
        "state_time_ns": written(sc_log),
        "sensor_state_time_ns": written(sensor_state_log),
        "wmm_state_time_ns": sample_at_ticks(ticks, guard_ticks, guard_data["wmm_state_time_ns"], "WMM input epoch"),
        "earth_orientation_time_ns": sample_at_ticks(ticks, guard_ticks, guard_data["earth_orientation_time_ns"], "Earth input epoch"),
        "earth_orientation_tdb_s": sampled(earth_log, "J2000Current"),
        "earth_orientation_enabled": sampled(earth_log, "computeOrient"),
        "earth_orientation_model": MODEL_NAME,
        "field_evaluation_time_ns": written(mag_log),
        "wmm_coefficient_time_ns": ((ticks + 500_000_000) // 1_000_000_000) * 1_000_000_000,
        "held_state_time_ns": written(held_state_log),
        "held_field_time_ns": written(held_mag_log),
        "held_dipole_time_ns": written(held_cmd_log),
        "held_expected_torque_time_ns": written(held_torque_log),
        "tam_message_time_ns": written(tam_log),
        "nav_message_time_ns": written(nav_log),
        "nav_time_tag_s": sampled(nav_log, "timeTag"),
        "dipole_command_time_ns": written(cmd_log),
        "expected_torque_time_ns": written(expected_torque_log),
        "applied_torque_time_ns": sample_at_ticks(ticks, effector_log.times(),
                                                  effector_log.times(), "effector sample time"),
        "diagnostic_time_ns": sample_at_ticks(ticks, diag_ticks, diag_ticks, "diagnostic time"),
        "applied_torque_source": ("MtbEffector" if actuator == "native" else "ExtForceTorque") + ".torqueExternalPntB_B",
        "r_N_x_m": r[:, 0], "r_N_y_m": r[:, 1], "r_N_z_m": r[:, 2],
        "v_N_x_m_s": v[:, 0], "v_N_y_m_s": v[:, 1], "v_N_z_m_s": v[:, 2],
        "sigma_BN_1": sigma[:, 0], "sigma_BN_2": sigma[:, 1], "sigma_BN_3": sigma[:, 2],
        "euler321_phi_rad": euler[:, 0],
        "euler321_theta_rad": euler[:, 1],
        "euler321_psi_rad": euler[:, 2],
        "euler321_phi_deg": np.degrees(euler[:, 0]),
        "euler321_theta_deg": np.degrees(euler[:, 1]),
        "euler321_psi_deg": np.degrees(euler[:, 2]),
        "omega_B_x_rad_s": omega[:, 0], "omega_B_y_rad_s": omega[:, 1], "omega_B_z_rad_s": omega[:, 2],
        "omega_mag_rad_s": omega_mag,
        "B_N_x_T": B_N[:, 0], "B_N_y_T": B_N[:, 1], "B_N_z_T": B_N[:, 2],
        "B_N_mag_T": np.linalg.norm(B_N, axis=1),
        "B_B_x_T": B_B[:, 0], "B_B_y_T": B_B[:, 1], "B_B_z_T": B_B[:, 2],
        "B_B_mag_T": np.linalg.norm(B_B, axis=1),
        "mcmd_x_Am2": dipole[:, 0], "mcmd_y_Am2": dipole[:, 1], "mcmd_z_Am2": dipole[:, 2],
        "applied_torque_B_x_Nm": applied_torque_B[:, 0],
        "applied_torque_B_y_Nm": applied_torque_B[:, 1],
        "applied_torque_B_z_Nm": applied_torque_B[:, 2],
        "applied_torque_B_mag_Nm": np.linalg.norm(applied_torque_B, axis=1),
        "control_torque_B_x_Nm": expected_torque_B[:, 0],
        "control_torque_B_y_Nm": expected_torque_B[:, 1],
        "control_torque_B_z_Nm": expected_torque_B[:, 2],
        "control_torque_B_mag_Nm": np.linalg.norm(expected_torque_B, axis=1),
    })

    for prefix, log, field, suffixes in [
        ("held_omega_B", held_state_log, "omega_BN_B", ["x_rad_s", "y_rad_s", "z_rad_s"]),
        ("held_sigma_BN", held_state_log, "sigma_BN", ["1", "2", "3"]),
        ("held_B_N", held_mag_log, "magField_N", ["x_T", "y_T", "z_T"]),
        ("held_mcmd", held_cmd_log, "mtbDipoleCmds", ["x_Am2", "y_Am2", "z_Am2"]),
        ("held_control_torque_B", held_torque_log, "torqueRequestBody", ["x_Nm", "y_Nm", "z_Nm"]),
        ("sensor_state_r_N", sensor_state_log, "r_BN_N", ["x_m", "y_m", "z_m"]),
        ("sensor_state_v_N", sensor_state_log, "v_BN_N", ["x_m_s", "y_m_s", "z_m_s"]),
        ("sensor_state_sigma_BN", sensor_state_log, "sigma_BN", ["1", "2", "3"]),
        ("sensor_state_omega_B", sensor_state_log, "omega_BN_B", ["x_rad_s", "y_rad_s", "z_rad_s"]),
        ("nav_sigma_BN", nav_log, "sigma_BN", ["1", "2", "3"]),
        ("nav_omega_B", nav_log, "omega_BN_B", ["x_rad_s", "y_rad_s", "z_rad_s"]),
    ]:
        values = sampled(log, field)
        for i, suffix in enumerate(suffixes):
            df[f"{prefix}_{suffix}"] = values[:, i]
    earth_matrix = sampled(earth_log, "J20002Pfix").reshape(-1, 3, 3)
    for i in range(3):
        for j in range(3):
            df[f"earth_C_PN_{i+1}{j+1}"] = earth_matrix[:, i, j]

    diagnostic_cols = [
        "ix_A", "iy_A", "iz_A",
        "pcoil_x_W", "pcoil_y_W", "pcoil_z_W", "pcoil_total_W",
        "saturation_x", "saturation_y", "saturation_z",
        "controller_valid", "using_cpp_core",
    ]
    for col in diagnostic_cols:
        df[col] = diag_sampled[col].to_numpy(dtype=float)

    if actuator == "native":
        configuration = mtb_cfg_msg.read()
        for i, axis in enumerate("xyz"):
            df[f"mtb_max_dipole_{axis}_Am2"] = configuration.maxMtbDipoles[i]
        for i in range(3):
            for j in range(3):
                df[f"mtb_Gt_B_{i+1}{j+1}"] = configuration.GtMatrix_B[3*i+j]
        df["native_output_time_ns"] = written(native_output_log)
        inputs = np.asarray(actuator_guard.history, dtype=np.int64)
        df["native_input_dipole_time_ns"] = sample_at_ticks(ticks, inputs[:, 0], inputs[:, 1], "native dipole epoch")
        df["native_input_field_time_ns"] = sample_at_ticks(ticks, inputs[:, 0], inputs[:, 2], "native field epoch")
        for i, axis in enumerate("xyz"):
            df[f"native_mtbNetTorque_B_{axis}_Nm"] = sampled(native_output_log, "mtbNetTorque_B")[:, i]
    if capture_commands:
        df.attrs["command_history"] = [(int(tick), *row[:3]) for tick, row in
                                       zip(full_cmd_log.timesWritten(), full_cmd_log.mtbDipoleCmds)]
    out_csv = OUT_DATA / ("detumble_output.csv" if actuator == "native" else "detumble_direct_output.csv")
    if replay_commands is not None:
        out_csv = OUT_DATA / f"detumble_{actuator}_replay.csv"
    if write_outputs:
        df.to_csv(out_csv, index=False)
        print(f"Wrote {out_csv}")
    print(f"Initial |omega| [rad/s]: {omega_mag[0]:.12g}")
    print(f"Final   |omega| [rad/s]: {omega_mag[-1]:.12g}")
    print(f"Mean |applied magnetic torque| [N m]: {np.linalg.norm(applied_torque_B, axis=1).mean():.12g}")
    print(f"Peak |applied magnetic torque| [N m]: {np.linalg.norm(applied_torque_B, axis=1).max():.12g}")

    if write_outputs and make_plots:
        from plot_results import plot_all
        plot_all(df, OUT_PLOTS if actuator == "native" else OUT_PLOTS / "direct_reference")
    return df


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--actuator", choices=("native", "direct"), default=DEFAULT_ACTUATOR)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    run(stop_time_s=args.duration, actuator=args.actuator, make_plots=not args.no_plots)
