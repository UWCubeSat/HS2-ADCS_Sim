from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


OUTPUT_DIR = Path("output_plots")
OUTPUT_DIR.mkdir(exist_ok=True)


def mag3(x, y, z):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    z = np.asarray(z, dtype=float)
    return np.sqrt(x * x + y * y + z * z)


def save_current_figure(name):
    path = OUTPUT_DIR / name
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()
    return path


df = pd.read_csv("adcs_output.csv")
t = df["t"].to_numpy()

numeric = df.select_dtypes(include=[np.number])
all_finite = np.isfinite(numeric.to_numpy(dtype=float)).all()

wmag = mag3(df["p"], df["q"], df["r"])
print("Rows:", len(df), "Cols:", len(df.columns))
print("All numeric values finite?:", bool(all_finite))
print("Initial |w| [rad/s]:", float(wmag[0]))
print("Final   |w| [rad/s]:", float(wmag[-1]))
print("Mean coil power [W]:", float(df["pcoil_total"].mean()))
print("Peak coil power [W]:", float(df["pcoil_total"].max()))


plt.figure(figsize=(9, 4))
plt.plot(t, df["p"], label="p")
plt.plot(t, df["q"], label="q")
plt.plot(t, df["r"], label="r")
plt.plot(t, wmag, label="|w|", color="black", linewidth=2, alpha=0.8)
plt.xlabel("Time [s]")
plt.ylabel("Rate [rad/s]")
plt.title("Body Rates")
plt.legend()
plt.grid(True)
body_rates_path = save_current_figure("body_rates.png")


if all(c in df.columns for c in ["ptpN_phi", "ptpN_theta", "ptpN_psi"]):
    plt.figure(figsize=(9, 4))
    plt.plot(t, df["ptpN_phi"], label="phi")
    plt.plot(t, df["ptpN_theta"], label="theta")
    plt.plot(t, df["ptpN_psi"], label="psi")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")
    plt.title("Navigation Euler Angles")
    plt.legend()
    plt.grid(True)
    nav_euler_path = save_current_figure("nav_euler.png")
else:
    nav_euler_path = None


B_truth = mag3(df["BBx"], df["BBy"], df["BBz"])
B_meas = mag3(df["Bmx"], df["Bmy"], df["Bmz"])
B_nav = mag3(df["BNx"], df["BNy"], df["BNz"])

plt.figure(figsize=(9, 4))
plt.plot(t, B_truth, label="|B| truth")
plt.plot(t, B_meas, label="|B| meas", alpha=0.6)
plt.plot(t, B_nav, label="|B| nav", linewidth=2)
plt.xlabel("Time [s]")
plt.ylabel("|B| [T]")
plt.title("Magnetic Field Magnitude")
plt.legend()
plt.grid(True)
mag_field_path = save_current_figure("mag_field.png")


plt.figure(figsize=(9, 4))
plt.plot(t, df["ix"], label="ix")
plt.plot(t, df["iy"], label="iy")
plt.plot(t, df["iz"], label="iz")
plt.xlabel("Time [s]")
plt.ylabel("Current [A]")
plt.title("Magnetorquer Currents")
plt.legend()
plt.grid(True)
currents_path = save_current_figure("currents.png")


plt.figure(figsize=(9, 4))
plt.plot(t, df["mcmd_x"], label="mx")
plt.plot(t, df["mcmd_y"], label="my")
plt.plot(t, df["mcmd_z"], label="mz")
plt.xlabel("Time [s]")
plt.ylabel("Dipole [A m^2]")
plt.title("Magnetorquer Dipole Commands")
plt.legend()
plt.grid(True)
dipole_path = save_current_figure("dipole.png")


plt.figure(figsize=(9, 4))
plt.plot(t, df["pcoil_x"], label="Px coil (CR0002)")
plt.plot(t, df["pcoil_y"], label="Py coil (CR0002)")
plt.plot(t, df["pcoil_z"], label="Pz coil (MT01)")
plt.plot(t, df["pcoil_total"], label="Ptotal coil", linewidth=2, color="black")
plt.xlabel("Time [s]")
plt.ylabel("Power [W]")
plt.title("Hybrid Coil Power")
plt.legend()
plt.grid(True)
coil_power_path = save_current_figure("coil_power.png")


if all(
    c in df.columns
    for c in [
        "Fdrag_x",
        "Fdrag_y",
        "Fdrag_z",
        "Fsrp_x",
        "Fsrp_y",
        "Fsrp_z",
        "Tgg_x",
        "Tgg_y",
        "Tgg_z",
        "Tmag_x",
        "Tmag_y",
        "Tmag_z",
        "Tsrp_x",
        "Tsrp_y",
        "Tsrp_z",
        "Tdist_x",
        "Tdist_y",
        "Tdist_z",
    ]
):
    F_drag = mag3(df["Fdrag_x"], df["Fdrag_y"], df["Fdrag_z"])
    F_srp = mag3(df["Fsrp_x"], df["Fsrp_y"], df["Fsrp_z"])
    T_gg = mag3(df["Tgg_x"], df["Tgg_y"], df["Tgg_z"])
    T_mag = mag3(df["Tmag_x"], df["Tmag_y"], df["Tmag_z"])
    T_srp = mag3(df["Tsrp_x"], df["Tsrp_y"], df["Tsrp_z"])
    T_total = mag3(df["Tdist_x"], df["Tdist_y"], df["Tdist_z"])

    print("Mean drag force [N]:", float(F_drag.mean()))
    print("Mean SRP force [N]:", float(F_srp.mean()))
    print("Mean total disturbance torque [N m]:", float(T_total.mean()))

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    axes[0].plot(t, F_drag, label="|Fdrag|")
    axes[0].plot(t, F_srp, label="|Fsrp|")
    axes[0].set_ylabel("Force [N]")
    axes[0].set_title("Environment Disturbance Forces")
    axes[0].grid(True)
    axes[0].legend()

    for series, label, color, linewidth in [
        (T_gg, "|Tgg|", None, None),
        (T_mag, "|Tresidual mag|", None, None),
        (T_srp, "|Tsrp|", None, None),
        (T_total, "|Tdist total|", "black", 2),
    ]:
        if float(np.max(series)) > 0.0:
            axes[1].semilogy(
                t,
                np.maximum(series, 1e-18),
                label=label,
                color=color,
                linewidth=linewidth,
            )
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Torque [N m]")
    axes[1].set_title("Environment Disturbance Torques")
    axes[1].grid(True)
    axes[1].legend()

    disturbance_path = save_current_figure("disturbances.png")
else:
    disturbance_path = None


summary_paths = [
    ("body_rates", body_rates_path),
    ("nav_euler", nav_euler_path),
    ("mag_field", mag_field_path),
    ("currents", currents_path),
    ("dipole", dipole_path),
    ("coil_power", coil_power_path),
    ("disturbances", disturbance_path),
]

print("Saved plots:")
for label, path in summary_paths:
    if path is not None:
        print(f"  {label}: {path}")

existing_paths = [path for _, path in summary_paths if path is not None]
if existing_paths:
    ncols = 2
    nrows = int(np.ceil(len(existing_paths) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(14, 4.8 * nrows))
    axes = np.atleast_1d(axes).ravel()

    for ax, (label, path) in zip(axes, [(label, path) for label, path in summary_paths if path is not None]):
        ax.imshow(plt.imread(path))
        ax.set_title(label.replace("_", " ").title())
        ax.axis("off")

    for ax in axes[len(existing_paths):]:
        ax.axis("off")

    summary_path = OUTPUT_DIR / "summary_plots.png"
    plt.tight_layout()
    plt.savefig(summary_path, dpi=180)
    plt.close()
    print(f"  summary: {summary_path}")
