"""HuskySat-style Basilisk WMM + magnetorquer detumble scenario.

This is the fast workflow path:
- pip-installed Basilisk
- no Basilisk source rebuild
- no Basilisk.ExternalModules import
- Python Basilisk controller module with optional separate pybind core
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

    # Higher priority runs earlier. Controller writes command before torque/dynamics updates.
    sim.AddModelToTask("DynamicsTask", mag, ModelPriority=900)
    sim.AddModelToTask("DynamicsTask", nav, ModelPriority=800)
    sim.AddModelToTask("DynamicsTask", tam, ModelPriority=700)
    sim.AddModelToTask("DynamicsTask", ctrl, ModelPriority=600)
    sim.AddModelToTask("DynamicsTask", direct_torque, ModelPriority=500)
    sim.AddModelToTask("DynamicsTask", sc, ModelPriority=100)

    rec_dt = macros.sec2nano(RECORD_DT_S)
    sc_log = sc.scStateOutMsg.recorder(rec_dt)
    mag_log = mag.envOutMsgs[0].recorder(rec_dt)
    tam_log = tam.tamDataOutMsg.recorder(rec_dt)
    cmd_log = ctrl.mtbCmdOutMsg.recorder(rec_dt)
    direct_torque_log = direct_torque.logger(["torqueExternalPntB_B"])
    nav_log = nav.attOutMsg.recorder(rec_dt)
    for recorder in [sc_log, mag_log, tam_log, cmd_log, direct_torque_log, nav_log]:
        sim.AddModelToTask("DynamicsTask", recorder)

    r0 = EARTH_RADIUS_M + ALTITUDE_M
    orbit_period_s = 2.0 * math.pi * math.sqrt(r0**3 / MU_EARTH)
    print(f"Running detumble scenario for one orbit: {orbit_period_s:.2f} s")

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(orbit_period_s))
    sim.ExecuteSimulation()

    t = sc_log.times() * macros.NANO2SEC
    r = np.asarray(sc_log.r_BN_N, dtype=float)
    v = np.asarray(sc_log.v_BN_N, dtype=float)
    sigma = np.asarray(sc_log.sigma_BN, dtype=float)
    omega = np.asarray(sc_log.omega_BN_B, dtype=float)
    omega_mag = np.linalg.norm(omega, axis=1)
    euler = np.array([dcm_to_euler321(mrp_to_dcm(s)) for s in sigma])

    B_N = np.asarray(mag_log.magField_N, dtype=float)
    B_B = np.asarray(tam_log.tam_S, dtype=float)
    dipole_all = np.asarray(cmd_log.mtbDipoleCmds, dtype=float)
    dipole = dipole_all[:, :3]
    # The direct_torque logger samples at the task rate, while the spacecraft
    # recorders sample at RECORD_DT_S.  Align the direct-torque log onto the
    # canonical spacecraft time vector before building the output DataFrame.
    applied_torque_raw = np.asarray(direct_torque_log.torqueExternalPntB_B, dtype=float)
    applied_torque_times = np.asarray(direct_torque_log.times(), dtype=float) * macros.NANO2SEC

    if applied_torque_raw.ndim == 1:
        applied_torque_raw = applied_torque_raw.reshape((-1, 3))

    if len(applied_torque_raw) == 0:
        applied_torque_B = np.zeros((len(t), 3))
    else:
        n_torque = min(len(applied_torque_raw), len(applied_torque_times))
        torque_df = pd.DataFrame({
            "time_s": applied_torque_times[:n_torque],
            "applied_torque_B_x_Nm": applied_torque_raw[:n_torque, 0],
            "applied_torque_B_y_Nm": applied_torque_raw[:n_torque, 1],
            "applied_torque_B_z_Nm": applied_torque_raw[:n_torque, 2],
        }).sort_values("time_s")

        torque_sampled = pd.merge_asof(
            pd.DataFrame({"time_s": t}),
            torque_df,
            on="time_s",
            direction="nearest",
            tolerance=max(CONTROL_DT_S, RECORD_DT_S),
        )
        applied_torque_B = torque_sampled[[
            "applied_torque_B_x_Nm",
            "applied_torque_B_y_Nm",
            "applied_torque_B_z_Nm",
        ]].fillna(0.0).to_numpy(dtype=float)

    diag = pd.DataFrame(ctrl.history)
    diag_sampled = pd.merge_asof(
        pd.DataFrame({"time_s": t}),
        diag.sort_values("time_s") if not diag.empty else pd.DataFrame({"time_s": t}),
        on="time_s",
        direction="nearest",
        tolerance=CONTROL_DT_S,
    )

    df = pd.DataFrame({
        "time_s": t,
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
        # Legacy names retained so plotting/comparison scripts keep working. These are direct applied magnetic torque values.
        "mtb_torque_B_x_Nm": applied_torque_B[:, 0],
        "mtb_torque_B_y_Nm": applied_torque_B[:, 1],
        "mtb_torque_B_z_Nm": applied_torque_B[:, 2],
        "mtb_torque_B_mag_Nm": np.linalg.norm(applied_torque_B, axis=1),
    })

    for col in ["ix_A", "iy_A", "iz_A", "pcoil_x_W", "pcoil_y_W", "pcoil_z_W", "pcoil_total_W", "control_torque_B_x_Nm", "control_torque_B_y_Nm", "control_torque_B_z_Nm", "control_torque_B_mag_Nm", "saturation_x", "saturation_y", "saturation_z", "controller_valid", "using_cpp_core"]:
        if col in diag_sampled.columns:
            df[col] = diag_sampled[col]

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
