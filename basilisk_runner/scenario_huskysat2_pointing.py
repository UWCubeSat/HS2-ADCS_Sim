"""HuskySat-style Basilisk pointing proof-of-concept.

This is intentionally a proof-of-concept, not the final magnetorquer-only
pointing controller.  It uses Basilisk for the spacecraft/orbit/attitude plant
and applies a direct body torque through extForceTorque.  The controller is a
small Python MRP PD law so we can validate pointing-error reduction before
adding magnetorquer-only torque allocation.

Fast workflow properties:
- pip-installed Basilisk
- no Basilisk source rebuild
- no Basilisk.ExternalModules import
- no custom Basilisk C++ module
"""

from __future__ import annotations

from pathlib import Path
import json
import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    from Basilisk.utilities import SimulationBaseClass, macros
    from Basilisk.simulation import simpleNav, extForceTorque
    from Basilisk.architecture import sysModel, messaging, bskLogging
except ImportError as exc:
    raise SystemExit(
        "Basilisk is not installed in this Python environment. Run: python -m pip install -r requirements.txt"
    ) from exc

from scenario_huskysat2_detumble import (
    configure_spacecraft,
    mrp_to_dcm,
    dcm_to_euler321,
    IXX,
    IYY,
    IZZ,
)

HERE = Path(__file__).resolve().parent
OUT_DATA = HERE / "output_data"
OUT_PLOTS = HERE / "output_plots"
OUT_DATA.mkdir(parents=True, exist_ok=True)
OUT_PLOTS.mkdir(parents=True, exist_ok=True)

CONTROL_DT_S = 0.1
RECORD_DT_S = 1.0
SIM_DURATION_S = 1800.0

# A nonzero initial attitude error and modest initial body rates make this a
# real pointing test instead of merely holding an already aligned attitude.
POINTING_INITIAL_SIGMA_BN = [0.22, -0.12, 0.16]
POINTING_INITIAL_RATES_RAD_S = [0.03, -0.02, 0.015]

# Direct-torque MRP PD gains.  These are intentionally conservative for the
# small CubeSat inertia used in the detumble baseline.
K_MRP_NM = 8.0e-5
P_RATE_NM_PER_RAD_S = 1.8e-3
MAX_TORQUE_NM = 2.0e-4


def mrp_shadow_if_needed(sigma):
    """Return the MRP set with norm <= 1."""
    s = np.asarray(sigma, dtype=float)
    s2 = float(np.dot(s, s))
    if s2 > 1.0:
        return -s / s2
    return s


def mrp_angle_deg(sigma):
    """Return equivalent principal rotation angle for an MRP vector."""
    s = mrp_shadow_if_needed(sigma)
    angle_rad = 4.0 * math.atan(float(np.linalg.norm(s)))
    if angle_rad > math.pi:
        angle_rad = 2.0 * math.pi - angle_rad
    return math.degrees(angle_rad)


class PythonMRPDirectTorqueController(sysModel.SysModel):
    """Direct body-torque MRP PD controller for pointing bring-up.

    Reference attitude: inertial identity frame, sigma_RN = 0.
    Error attitude: because the reference is identity and non-rotating, the
    body-to-reference error is represented by sigma_BN.

    Output: CmdTorqueBodyMsgPayload.torqueRequestBody [N m].
    """

    def __init__(self, *args):
        super().__init__(*args)
        self.navAttInMsg = messaging.NavAttMsgReader()
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()
        self.history = []

    def Reset(self, CurrentSimNanos):
        if not self.navAttInMsg.isLinked():
            self.bskLogger.bskLog(bskLogging.BSK_ERROR, "PythonMRPDirectTorqueController.navAttInMsg is not linked.")
        payload = self.cmdTorqueOutMsg.zeroMsgPayload
        payload.torqueRequestBody = [0.0, 0.0, 0.0]
        self.cmdTorqueOutMsg.write(payload, CurrentSimNanos, self.moduleID)
        self.history.clear()

    def UpdateState(self, CurrentSimNanos):
        nav = self.navAttInMsg()
        time_s = CurrentSimNanos * 1.0e-9

        sigma_BR = mrp_shadow_if_needed(np.asarray(nav.sigma_BN, dtype=float)[:3])
        omega_BR_B = np.asarray(nav.omega_BN_B, dtype=float)[:3]

        raw_torque = -K_MRP_NM * sigma_BR - P_RATE_NM_PER_RAD_S * omega_BR_B
        torque_mag = float(np.linalg.norm(raw_torque))
        if torque_mag > MAX_TORQUE_NM:
            torque = raw_torque * (MAX_TORQUE_NM / torque_mag)
            saturated = True
        else:
            torque = raw_torque
            saturated = False

        payload = self.cmdTorqueOutMsg.zeroMsgPayload
        payload.torqueRequestBody = torque.tolist()
        self.cmdTorqueOutMsg.write(payload, CurrentSimNanos, self.moduleID)

        self.history.append({
            "time_s": time_s,
            "sigma_BR_1": float(sigma_BR[0]),
            "sigma_BR_2": float(sigma_BR[1]),
            "sigma_BR_3": float(sigma_BR[2]),
            "sigma_BR_mag": float(np.linalg.norm(sigma_BR)),
            "pointing_error_deg": mrp_angle_deg(sigma_BR),
            "omega_B_x_rad_s": float(omega_BR_B[0]),
            "omega_B_y_rad_s": float(omega_BR_B[1]),
            "omega_B_z_rad_s": float(omega_BR_B[2]),
            "omega_mag_rad_s": float(np.linalg.norm(omega_BR_B)),
            "torque_B_x_Nm": float(torque[0]),
            "torque_B_y_Nm": float(torque[1]),
            "torque_B_z_Nm": float(torque[2]),
            "torque_B_mag_Nm": float(np.linalg.norm(torque)),
            "raw_torque_B_mag_Nm": torque_mag,
            "saturated": saturated,
        })


def configure_pointing_spacecraft():
    sc = configure_spacecraft()
    sc.ModelTag = "HuskySat2_pointing_poc"
    sc.hub.sigma_BNInit = POINTING_INITIAL_SIGMA_BN
    sc.hub.omega_BN_BInit = POINTING_INITIAL_RATES_RAD_S
    return sc


def plot_pointing(df: pd.DataFrame):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(df["time_s"], df["pointing_error_deg"], label="Pointing error")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Pointing error [deg]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    path = OUT_PLOTS / "pointing_error.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"Saved {path}")

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(df["time_s"], df["omega_B_x_rad_s"], label="omega_x")
    ax.plot(df["time_s"], df["omega_B_y_rad_s"], label="omega_y")
    ax.plot(df["time_s"], df["omega_B_z_rad_s"], label="omega_z")
    ax.plot(df["time_s"], df["omega_mag_rad_s"], label="|omega|", linewidth=2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Body rate [rad/s]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    path = OUT_PLOTS / "pointing_body_rates.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"Saved {path}")

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(df["time_s"], df["torque_B_x_Nm"], label="tau_x")
    ax.plot(df["time_s"], df["torque_B_y_Nm"], label="tau_y")
    ax.plot(df["time_s"], df["torque_B_z_Nm"], label="tau_z")
    ax.plot(df["time_s"], df["torque_B_mag_Nm"], label="|tau|", linewidth=2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Direct body torque [N m]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    path = OUT_PLOTS / "pointing_torque.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"Saved {path}")

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(df["time_s"], df["euler321_phi_deg"], label="phi")
    ax.plot(df["time_s"], df["euler321_theta_deg"], label="theta")
    ax.plot(df["time_s"], df["euler321_psi_deg"], label="psi")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Euler 3-2-1 angle [deg]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    path = OUT_PLOTS / "pointing_euler.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"Saved {path}")

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(df["time_s"], df["pointing_error_deg"], label="Pointing error [deg]")
    ax2 = ax.twinx()
    ax2.plot(df["time_s"], df["omega_mag_rad_s"], label="|omega| [rad/s]", linestyle="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Pointing error [deg]")
    ax2.set_ylabel("Angular speed [rad/s]")
    ax.grid(True)
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines + lines2, labels + labels2, loc="best")
    fig.tight_layout()
    path = OUT_PLOTS / "pointing_summary.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"Saved {path}")


def _check(name: str, passed: bool, value=None, limit=None):
    return name, {"passed": bool(passed), "value": value, "limit": limit}


def write_pointing_metrics(df: pd.DataFrame) -> dict:
    """Write pass/fail validation metrics for the pointing proof-of-concept.

    These are engineering sanity checks for the direct-torque pointing bring-up.
    They are not a claim that the final magnetorquer-limited controller is done.
    """
    numeric = df.select_dtypes(include=[np.number])
    finite_numeric = bool(np.isfinite(numeric.to_numpy()).all())

    initial_error = float(df["pointing_error_deg"].iloc[0])
    final_error = float(df["pointing_error_deg"].iloc[-1])
    initial_rate = float(df["omega_mag_rad_s"].iloc[0])
    final_rate = float(df["omega_mag_rad_s"].iloc[-1])
    peak_torque = float(df["torque_B_mag_Nm"].max())
    saturation_fraction = float(df["saturated"].astype(bool).mean()) if "saturated" in df else float("nan")

    omega = df[["omega_B_x_rad_s", "omega_B_y_rad_s", "omega_B_z_rad_s"]].to_numpy(dtype=float)
    inertia_diag = np.array([IXX, IYY, IZZ], dtype=float)
    rotational_energy = 0.5 * np.sum((omega ** 2) * inertia_diag[None, :], axis=1)
    initial_energy = float(rotational_energy[0])
    final_energy = float(rotational_energy[-1])

    checks = dict([
        _check("numeric_finite", finite_numeric, None, None),
        _check("final_pointing_error_less_than_initial", final_error < initial_error, final_error, f"< {initial_error}"),
        _check("final_pointing_error_below_1_deg", final_error < 1.0, final_error, "< 1 deg"),
        _check("final_omega_less_than_initial", final_rate < initial_rate, final_rate, f"< {initial_rate}"),
        _check("final_omega_below_1e_minus_4_rad_s", final_rate < 1.0e-4, final_rate, "< 1e-4 rad/s"),
        _check("final_rotational_energy_less_than_initial", final_energy < initial_energy, final_energy, f"< {initial_energy}"),
        _check("peak_torque_within_limit", peak_torque <= MAX_TORQUE_NM * 1.000001, peak_torque, f"<= {MAX_TORQUE_NM} N m"),
        _check("saturation_fraction_small", saturation_fraction <= 0.05, saturation_fraction, "<= 0.05"),
    ])

    passed = bool(all(item["passed"] for item in checks.values()))
    metrics = {
        "pointing_file": str(OUT_DATA / "pointing_output.csv"),
        "scenario": "direct_torque_pointing_proof_of_concept",
        "duration_s": float(df["time_s"].iloc[-1] - df["time_s"].iloc[0]),
        "initial_pointing_error_deg": initial_error,
        "final_pointing_error_deg": final_error,
        "pointing_error_reduction_deg": initial_error - final_error,
        "initial_omega_mag_rad_s": initial_rate,
        "final_omega_mag_rad_s": final_rate,
        "initial_rotational_energy_J": initial_energy,
        "final_rotational_energy_J": final_energy,
        "peak_direct_torque_Nm": peak_torque,
        "torque_saturation_fraction": saturation_fraction,
        "validation": {
            "passed": passed,
            "checks": checks,
        },
        "notes": [
            "This validates the direct-torque pointing proof-of-concept only.",
            "It does not validate final magnetorquer-only pointing or native MtbEffector integration.",
        ],
    }

    out_json = OUT_DATA / "pointing_metrics.json"
    out_json.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(f"Wrote {out_json}")
    print(f"Pointing validation passed?: {passed}")
    if not passed:
        print("Failed pointing checks:")
        for name, item in checks.items():
            if not item["passed"]:
                print(f"  - {name}")
    return metrics


def run(duration_s: float = SIM_DURATION_S):
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("PointingProcess")
    proc.addTask(sim.CreateNewTask("PointingTask", macros.sec2nano(CONTROL_DT_S)))

    sc = configure_pointing_spacecraft()

    nav = simpleNav.SimpleNav()
    nav.ModelTag = "SimpleNav_pointing"
    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)

    ctrl = PythonMRPDirectTorqueController()
    ctrl.ModelTag = "PythonMRPDirectTorqueController"
    ctrl.navAttInMsg.subscribeTo(nav.attOutMsg)

    direct_torque = extForceTorque.ExtForceTorque()
    direct_torque.ModelTag = "DirectPointingTorque"
    direct_torque.cmdTorqueInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)
    sc.addDynamicEffector(direct_torque)

    sim.AddModelToTask("PointingTask", nav, ModelPriority=800)
    sim.AddModelToTask("PointingTask", ctrl, ModelPriority=700)
    sim.AddModelToTask("PointingTask", direct_torque, ModelPriority=600)
    sim.AddModelToTask("PointingTask", sc, ModelPriority=100)

    rec_dt = macros.sec2nano(RECORD_DT_S)
    sc_log = sc.scStateOutMsg.recorder(rec_dt)
    nav_log = nav.attOutMsg.recorder(rec_dt)
    sim.AddModelToTask("PointingTask", sc_log)
    sim.AddModelToTask("PointingTask", nav_log)

    print(f"Running pointing proof-of-concept for {duration_s:.1f} s")
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration_s))
    sim.ExecuteSimulation()

    t = sc_log.times() * macros.NANO2SEC
    r = np.asarray(sc_log.r_BN_N, dtype=float)
    v = np.asarray(sc_log.v_BN_N, dtype=float)
    sigma = np.asarray(sc_log.sigma_BN, dtype=float)
    omega = np.asarray(sc_log.omega_BN_B, dtype=float)
    euler = np.array([dcm_to_euler321(mrp_to_dcm(s)) for s in sigma])

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
        "omega_B_x_rad_s": omega[:, 0],
        "omega_B_y_rad_s": omega[:, 1],
        "omega_B_z_rad_s": omega[:, 2],
        "omega_mag_rad_s": np.linalg.norm(omega, axis=1),
        "rotational_energy_J": 0.5 * (IXX * omega[:, 0]**2 + IYY * omega[:, 1]**2 + IZZ * omega[:, 2]**2),
    })

    for col in [
        "sigma_BR_1", "sigma_BR_2", "sigma_BR_3", "sigma_BR_mag", "pointing_error_deg",
        "torque_B_x_Nm", "torque_B_y_Nm", "torque_B_z_Nm", "torque_B_mag_Nm",
        "raw_torque_B_mag_Nm", "saturated",
    ]:
        if col in diag_sampled.columns:
            df[col] = diag_sampled[col]

    out_csv = OUT_DATA / "pointing_output.csv"
    df.to_csv(out_csv, index=False)

    initial_error = float(df["pointing_error_deg"].iloc[0])
    final_error = float(df["pointing_error_deg"].iloc[-1])
    initial_rate = float(df["omega_mag_rad_s"].iloc[0])
    final_rate = float(df["omega_mag_rad_s"].iloc[-1])
    max_torque = float(df["torque_B_mag_Nm"].max())
    saturated_fraction = float(df["saturated"].astype(bool).mean()) if "saturated" in df else float("nan")

    print(f"Wrote {out_csv}")
    print(f"Initial pointing error [deg]: {initial_error:.12g}")
    print(f"Final   pointing error [deg]: {final_error:.12g}")
    print(f"Initial |omega| [rad/s]: {initial_rate:.12g}")
    print(f"Final   |omega| [rad/s]: {final_rate:.12g}")
    print(f"Peak direct torque [N m]: {max_torque:.12g}")
    print(f"Torque saturation fraction: {saturated_fraction:.6g}")

    plot_pointing(df)
    write_pointing_metrics(df)
    return df


if __name__ == "__main__":
    run()
