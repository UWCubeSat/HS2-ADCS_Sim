"""HuskySat-style Basilisk WMM + magnetorquer detumble scenario.

This is the fast workflow path:
- pip-installed Basilisk
- no Basilisk source rebuild
- no Basilisk.ExternalModules import
- Python Basilisk controller module with optional separate pybind core

Telemetry contract (schema 2; seconds/nanoseconds from simulation start):
* time_s/time_ns is the recorder task tick, after spacecraft propagation.
  r_N, v_N, sigma_BN, Euler angles and omega_B describe state_time_ns.
* sensor_state_* is an additional, pre-environment snapshot of scStateOutMsg.
  sensor_state_time_ns is its actual message write time, normally one 0.1 s
  integration step earlier (both states have epoch zero on the initial row).
  WMM, SimpleNav and TAM all consume this state; their task order is unchanged.
* B_N is WMM's field evaluated at field_evaluation_time_ns using the POSITION
  in sensor_state_*. B_B is the recorded TAM tam_S output (S=B only because
  dcm_SB is identity here), using the ATTITUDE in that same sensor state.
  The WMM evaluation epoch and input position epoch are deliberately separate.
* nav_* comes from SimpleNav; nav_time_tag_s is its output/update tag, NOT
  the consumed spacecraft state's epoch. *_message_time_ns are message headers.
* mcmd is MTBArrayCmdMsg.mtbDipoleCmds[0:3]. control_torque is the independent
  recording of CmdTorqueBodyMsg.torqueRequestBody, the controller's expected
  m x B torque. Both commands are published at their respective *_time_ns.
* applied_torque is ExtForceTorque.torqueExternalPntB_B sampled AFTER the plant
  update, at applied_torque_time_ns. It is the combined dynamic-effector torque,
  not a command alias and not native MtbEffector output. In this constant-body-
  torque bridge it is held over [sensor_state_time_ns, state_time_ns]; the zero
  row is initialization, with no elapsed integration interval.
* Currents, coil powers, saturation/validity and core-availability flags remain
  controller diagnostics at diagnostic_time_ns; they are not hardware readings.

All sources are matched on exact integer task ticks, never nearest-time joined.
Use sensor_state_sigma_BN with B_N/B_B for frame checks, and pre/post omega for
the applied-torque dynamics check. Do not pair B_B with post-step sigma_BN.
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
USE_DIRECT_TORQUE_FALLBACK = True  # Directly applies m x B through Basilisk extForceTorque while MtbEffector wiring is investigated.


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


def run():
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

    mtb_cfg_msg = configure_mtb_config_message(adcs_cfg)

    # Primary fast-development actuator path for now:
    # The controller computes the magnetic torque tau_B = m_B x B_B and sends it to
    # Basilisk's native ExtForceTorque dynamic effector. This proves the Basilisk
    # spacecraft dynamics/control loop without requiring a custom Basilisk C++ module.
    # MtbEffector remains the target native MTB actuator, but is not used here because
    # its output was zero in the first local test.
    direct_torque = extForceTorque.ExtForceTorque()
    direct_torque.ModelTag = "DirectMagneticTorqueFallback"
    direct_torque.cmdTorqueInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)
    sc.addDynamicEffector(direct_torque)

    # Keep a native MtbEffector object available for later debugging, but do not attach
    # it while the direct torque bridge is active; otherwise torque could be double-counted
    # once MtbEffector wiring is fixed.
    mtb = MtbEffector.MtbEffector()
    mtb.ModelTag = "MtbEffector_debug_not_attached"
    mtb.mtbCmdInMsg.subscribeTo(ctrl.mtbCmdOutMsg)
    mtb.magInMsg.subscribeTo(mag.envOutMsgs[0])
    mtb.mtbParamsInMsg.subscribeTo(mtb_cfg_msg)

    # Observe the exact state consumed by WMM/nav/TAM, before any model updates.
    # This recorder is passive: preserve all original physics/control priorities.
    rec_dt = macros.sec2nano(RECORD_DT_S)
    sensor_state_log = sc.scStateOutMsg.recorder(rec_dt)
    sim.AddModelToTask("DynamicsTask", sensor_state_log, ModelPriority=1000)

    # Higher priority runs earlier. Controller writes command before torque/dynamics updates.
    sim.AddModelToTask("DynamicsTask", mag, ModelPriority=900)
    sim.AddModelToTask("DynamicsTask", nav, ModelPriority=800)
    sim.AddModelToTask("DynamicsTask", tam, ModelPriority=700)
    sim.AddModelToTask("DynamicsTask", ctrl, ModelPriority=600)
    sim.AddModelToTask("DynamicsTask", direct_torque, ModelPriority=500)
    sim.AddModelToTask("DynamicsTask", sc, ModelPriority=100)

    sc_log = sc.scStateOutMsg.recorder(rec_dt)
    mag_log = mag.envOutMsgs[0].recorder(rec_dt)
    tam_log = tam.tamDataOutMsg.recorder(rec_dt)
    cmd_log = ctrl.mtbCmdOutMsg.recorder(rec_dt)
    expected_torque_log = ctrl.cmdTorqueOutMsg.recorder(rec_dt)
    direct_torque_log = direct_torque.logger(["torqueExternalPntB_B"])
    nav_log = nav.attOutMsg.recorder(rec_dt)
    for recorder in [sc_log, mag_log, tam_log, cmd_log, expected_torque_log, direct_torque_log, nav_log]:
        sim.AddModelToTask("DynamicsTask", recorder)

    r0 = EARTH_RADIUS_M + ALTITUDE_M
    orbit_period_s = 2.0 * math.pi * math.sqrt(r0**3 / MU_EARTH)
    print(f"Running detumble scenario for one orbit: {orbit_period_s:.2f} s")

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(orbit_period_s))
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
    applied_torque_B = sampled(direct_torque_log, "torqueExternalPntB_B")

    diag = pd.DataFrame(ctrl.history)
    # History time_s is produced from the integer controller task tick. Recover
    # that tick to avoid floating-point equality joins; no sample shifting.
    diag_ticks = np.rint(diag["time_s"].to_numpy() * 1e9).astype(np.int64)
    diag_sampled = pd.DataFrame(sample_at_ticks(ticks, diag_ticks, diag.to_numpy(), "controller history"),
                                columns=diag.columns)

    df = pd.DataFrame({
        "telemetry_schema_version": 2,
        "time_s": t,
        "time_ns": ticks,
        "control_step_ns": macros.sec2nano(CONTROL_DT_S),
        "state_time_ns": written(sc_log),
        "sensor_state_time_ns": written(sensor_state_log),
        "field_evaluation_time_ns": written(mag_log),
        "tam_message_time_ns": written(tam_log),
        "nav_message_time_ns": written(nav_log),
        "nav_time_tag_s": sampled(nav_log, "timeTag"),
        "dipole_command_time_ns": written(cmd_log),
        "expected_torque_time_ns": written(expected_torque_log),
        "applied_torque_time_ns": sample_at_ticks(ticks, direct_torque_log.times(),
                                                  direct_torque_log.times(), "effector sample time"),
        "diagnostic_time_ns": sample_at_ticks(ticks, diag_ticks, diag_ticks, "diagnostic time"),
        "applied_torque_source": "ExtForceTorque.torqueExternalPntB_B",
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

    diagnostic_cols = [
        "ix_A", "iy_A", "iz_A",
        "pcoil_x_W", "pcoil_y_W", "pcoil_z_W", "pcoil_total_W",
        "saturation_x", "saturation_y", "saturation_z",
        "controller_valid", "using_cpp_core",
    ]
    for col in diagnostic_cols:
        df[col] = diag_sampled[col].to_numpy(dtype=float)

    out_csv = OUT_DATA / "detumble_output.csv"
    df.to_csv(out_csv, index=False)
    print(f"Wrote {out_csv}")
    print(f"Initial |omega| [rad/s]: {omega_mag[0]:.12g}")
    print(f"Final   |omega| [rad/s]: {omega_mag[-1]:.12g}")
    print(f"Mean |applied magnetic torque| [N m]: {np.linalg.norm(applied_torque_B, axis=1).mean():.12g}")
    print(f"Peak |applied magnetic torque| [N m]: {np.linalg.norm(applied_torque_B, axis=1).max():.12g}")

    from plot_results import plot_all
    plot_all(df, OUT_PLOTS)
    return df


if __name__ == "__main__":
    run()
