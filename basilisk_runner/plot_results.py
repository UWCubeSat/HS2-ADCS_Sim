from __future__ import annotations

from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
OUT_DATA = HERE / "output_data"
OUT_PLOTS = HERE / "output_plots"
OUT_PLOTS.mkdir(parents=True, exist_ok=True)


def _save(name: str, output_dir: Path):
    path = output_dir / name
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"Saved {path}")
    return path


def _mag_cols(df: pd.DataFrame, cols):
    return np.sqrt(sum(np.asarray(df[c], dtype=float) ** 2 for c in cols))


def plot_all(df: pd.DataFrame | None = None, output_dir: Path = OUT_PLOTS):
    output_dir.mkdir(parents=True, exist_ok=True)
    if df is None:
        preferred = OUT_DATA / "detumble_output.csv"
        fallback = OUT_DATA / "minimal_output.csv"
        source = preferred if preferred.exists() else fallback
        if not source.exists():
            raise SystemExit("No Basilisk output found. Run a scenario first.")
        df = pd.read_csv(source)
        print(f"Loaded {source}")

    t = np.asarray(df["time_s"], dtype=float)
    paths = []

    rate_cols = ["omega_B_x_rad_s", "omega_B_y_rad_s", "omega_B_z_rad_s"]
    if all(c in df.columns for c in rate_cols):
        omega_mag = df["omega_mag_rad_s"].to_numpy() if "omega_mag_rad_s" in df else _mag_cols(df, rate_cols)
        plt.figure(figsize=(9, 4))
        plt.plot(t, df[rate_cols[0]], label="p")
        plt.plot(t, df[rate_cols[1]], label="q")
        plt.plot(t, df[rate_cols[2]], label="r")
        plt.plot(t, omega_mag, label="|omega|", linewidth=2)
        plt.xlabel("Time [s]")
        plt.ylabel("Rate [rad/s]")
        plt.title("Body Rates")
        plt.grid(True)
        plt.legend()
        paths.append(_save("body_rates.png", output_dir))

    euler_cols = ["euler321_phi_rad", "euler321_theta_rad", "euler321_psi_rad"]
    if all(c in df.columns for c in euler_cols):
        plt.figure(figsize=(9, 4))
        plt.plot(t, df[euler_cols[0]], label="phi")
        plt.plot(t, df[euler_cols[1]], label="theta")
        plt.plot(t, df[euler_cols[2]], label="psi")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [rad]")
        plt.title("Euler 321 Angles")
        plt.grid(True)
        plt.legend()
        paths.append(_save("nav_euler.png", output_dir))

    if "B_B_mag_T" in df.columns or all(c in df.columns for c in ["B_B_x_T", "B_B_y_T", "B_B_z_T"]):
        B_body = df["B_B_mag_T"].to_numpy() if "B_B_mag_T" in df else _mag_cols(df, ["B_B_x_T", "B_B_y_T", "B_B_z_T"])
        plt.figure(figsize=(9, 4))
        plt.plot(t, B_body, label="|B| body/sensor")
        if "B_N_mag_T" in df.columns:
            plt.plot(t, df["B_N_mag_T"], label="|B| inertial", alpha=0.75)
        plt.xlabel("Time [s]")
        plt.ylabel("|B| [T]")
        plt.title("Magnetic Field Magnitude")
        plt.grid(True)
        plt.legend()
        paths.append(_save("mag_field.png", output_dir))

    if all(c in df.columns for c in ["ix_A", "iy_A", "iz_A"]):
        plt.figure(figsize=(9, 4))
        plt.plot(t, df["ix_A"], label="ix")
        plt.plot(t, df["iy_A"], label="iy")
        plt.plot(t, df["iz_A"], label="iz")
        plt.xlabel("Time [s]")
        plt.ylabel("Current [A]")
        plt.title("Magnetorquer Currents")
        plt.grid(True)
        plt.legend()
        paths.append(_save("currents.png", output_dir))

    if all(c in df.columns for c in ["mcmd_x_Am2", "mcmd_y_Am2", "mcmd_z_Am2"]):
        plt.figure(figsize=(9, 4))
        plt.plot(t, df["mcmd_x_Am2"], label="mx")
        plt.plot(t, df["mcmd_y_Am2"], label="my")
        plt.plot(t, df["mcmd_z_Am2"], label="mz")
        plt.xlabel("Time [s]")
        plt.ylabel("Dipole [A m^2]")
        plt.title("Magnetorquer Dipole Commands")
        plt.grid(True)
        plt.legend()
        paths.append(_save("dipole.png", output_dir))

    if "pcoil_total_W" in df.columns:
        plt.figure(figsize=(9, 4))
        for c, label in [("pcoil_x_W", "Px"), ("pcoil_y_W", "Py"), ("pcoil_z_W", "Pz")]:
            if c in df.columns:
                plt.plot(t, df[c], label=label)
        plt.plot(t, df["pcoil_total_W"], label="P total", linewidth=2)
        plt.xlabel("Time [s]")
        plt.ylabel("Power [W]")
        plt.title("Coil Power")
        plt.grid(True)
        plt.legend()
        paths.append(_save("coil_power.png", output_dir))

    # Phase 2A detumble records ExtForceTorque readback, not native MTB output.
    # Keep legacy plotting available for existing minimal/archived CSVs.
    torque_prefix = "applied_torque" if "applied_torque_B_x_Nm" in df else "mtb_torque"
    torque_cols = [f"{torque_prefix}_B_{axis}_Nm" for axis in "xyz"]
    if all(c in df.columns for c in torque_cols):
        torque_mag = _mag_cols(df, torque_cols)
        plt.figure(figsize=(9, 4))
        plt.plot(t, df[torque_cols[0]], label="Tx")
        plt.plot(t, df[torque_cols[1]], label="Ty")
        plt.plot(t, df[torque_cols[2]], label="Tz")
        plt.plot(t, torque_mag, label="|T|", linewidth=2)
        plt.xlabel("Time [s]")
        plt.ylabel("Torque [N m]")
        plt.title("Applied External Torque (Body Frame)" if torque_prefix == "applied_torque"
                  else "Magnetorquer Applied Torque")
        plt.grid(True)
        plt.legend()
        paths.append(_save("mtb_torque.png", output_dir))

    if paths:
        ncols = 2
        nrows = int(np.ceil(len(paths) / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=(14, 4.8 * nrows))
        axes = np.atleast_1d(axes).ravel()
        for ax, path in zip(axes, paths):
            ax.imshow(plt.imread(path))
            ax.set_title(path.stem.replace("_", " ").title())
            ax.axis("off")
        for ax in axes[len(paths):]:
            ax.axis("off")
        paths.append(_save("summary_plots.png", output_dir))

    numeric = df.select_dtypes(include=[np.number])
    print("Rows:", len(df), "Cols:", len(df.columns))
    print("All numeric values finite?:", bool(np.isfinite(numeric.to_numpy(dtype=float)).all()))
    if "omega_mag_rad_s" in df.columns:
        print("Initial |omega| [rad/s]:", float(df["omega_mag_rad_s"].iloc[0]))
        print("Final   |omega| [rad/s]:", float(df["omega_mag_rad_s"].iloc[-1]))
    if "pcoil_total_W" in df.columns:
        print("Mean coil power [W]:", float(df["pcoil_total_W"].mean()))
        print("Peak coil power [W]:", float(df["pcoil_total_W"].max()))
    if all(c in df.columns for c in torque_cols):
        print(f"Mean |{torque_prefix}| [N m]:", float(torque_mag.mean()))
        print(f"Peak |{torque_prefix}| [N m]:", float(torque_mag.max()))
    return paths


if __name__ == "__main__":
    plot_all()
