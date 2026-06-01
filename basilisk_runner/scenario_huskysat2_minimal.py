"""Minimal HuskySat-style Basilisk spacecraft/orbit/attitude scenario.

No custom C++ and no Basilisk ExternalModules are required.
"""

from __future__ import annotations

from pathlib import Path
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    from Basilisk.utilities import SimulationBaseClass, macros
    from Basilisk.simulation import spacecraft, gravityEffector
except ImportError as exc:
    raise SystemExit(
        "Basilisk is not installed in this Python environment. Run: python -m pip install -r requirements.txt"
    ) from exc

HERE = Path(__file__).resolve().parent
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
TASK_DT_S = 0.1
RECORD_DT_S = 1.0
RUN_TIME_S = 600.0


def mrp_to_dcm(sigma):
    s = np.asarray(sigma, dtype=float)
    s2 = float(np.dot(s, s))
    sx = np.array([[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]])
    return np.eye(3) + (8.0 * sx @ sx - 4.0 * (1.0 - s2) * sx) / (1.0 + s2) ** 2


def dcm_to_euler321(C):
    C = np.asarray(C, dtype=float)
    phi = math.atan2(C[1, 2], C[2, 2])
    theta = -math.asin(float(np.clip(C[0, 2], -1.0, 1.0)))
    psi = math.atan2(C[0, 1], C[0, 0])
    return np.array([phi, theta, psi])


def configure_spacecraft():
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "HuskySat2_minimal"
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


def run(run_time_s: float = RUN_TIME_S):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("DynamicsProcess")
    proc.addTask(sim.CreateNewTask("DynamicsTask", macros.sec2nano(TASK_DT_S)))

    sc = configure_spacecraft()
    sim.AddModelToTask("DynamicsTask", sc)

    rec = sc.scStateOutMsg.recorder(macros.sec2nano(RECORD_DT_S))
    sim.AddModelToTask("DynamicsTask", rec)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(run_time_s))
    sim.ExecuteSimulation()

    t = rec.times() * macros.NANO2SEC
    r = np.asarray(rec.r_BN_N, dtype=float)
    v = np.asarray(rec.v_BN_N, dtype=float)
    sigma = np.asarray(rec.sigma_BN, dtype=float)
    omega = np.asarray(rec.omega_BN_B, dtype=float)
    omega_mag = np.linalg.norm(omega, axis=1)
    euler = np.array([dcm_to_euler321(mrp_to_dcm(s)) for s in sigma])

    df = pd.DataFrame({
        "time_s": t,
        "r_N_x_m": r[:, 0], "r_N_y_m": r[:, 1], "r_N_z_m": r[:, 2],
        "v_N_x_m_s": v[:, 0], "v_N_y_m_s": v[:, 1], "v_N_z_m_s": v[:, 2],
        "sigma_BN_1": sigma[:, 0], "sigma_BN_2": sigma[:, 1], "sigma_BN_3": sigma[:, 2],
        "omega_B_x_rad_s": omega[:, 0], "omega_B_y_rad_s": omega[:, 1], "omega_B_z_rad_s": omega[:, 2],
        "omega_mag_rad_s": omega_mag,
        "euler321_phi_rad": euler[:, 0], "euler321_theta_rad": euler[:, 1], "euler321_psi_rad": euler[:, 2],
    })
    out_csv = OUT_DATA / "minimal_output.csv"
    df.to_csv(out_csv, index=False)

    plt.figure(figsize=(9, 4))
    plt.plot(t, omega[:, 0], label="p")
    plt.plot(t, omega[:, 1], label="q")
    plt.plot(t, omega[:, 2], label="r")
    plt.plot(t, omega_mag, label="|omega|", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Rate [rad/s]")
    plt.title("Minimal Basilisk Body Rates")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUT_PLOTS / "body_rates.png", dpi=180)
    plt.close()

    plt.figure(figsize=(9, 4))
    plt.plot(t, euler[:, 0], label="phi")
    plt.plot(t, euler[:, 1], label="theta")
    plt.plot(t, euler[:, 2], label="psi")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")
    plt.title("Minimal Basilisk Euler 321 Angles")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUT_PLOTS / "nav_euler.png", dpi=180)
    plt.close()

    print(f"Wrote {out_csv}")
    print(f"Initial |omega| [rad/s]: {omega_mag[0]:.12g}")
    print(f"Final   |omega| [rad/s]: {omega_mag[-1]:.12g}")
    return df


if __name__ == "__main__":
    run()
