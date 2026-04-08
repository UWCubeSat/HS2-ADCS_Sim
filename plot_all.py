import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV written by the simulator.
df = pd.read_csv("adcs_output.csv")

# Helper for turning vector components into one magnitude trace.
def mag3(x, y, z):
    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    return np.sqrt(x * x + y * y + z * z)

# Time axis for every plot.
t = df["t"].to_numpy()

# Quick sanity check on the output file.
numeric = df.select_dtypes(include=[np.number])
all_finite = np.isfinite(numeric.to_numpy()).all()
print("Rows:", len(df), "Cols:", len(df.columns))
print("Any NaN/Inf in numeric data?:", not bool(all_finite))

# Plot the body rates.
wmag = mag3(df["p"], df["q"], df["r"])
print("Initial |w| [rad/s]:", float(wmag[0]))
print("Final   |w| [rad/s]:", float(wmag[-1]))
print("Min/Max |w| [rad/s]:", float(wmag.min()), float(wmag.max()))

plt.figure(figsize=(9, 4))
plt.plot(t, df["p"], label="p")
plt.plot(t, df["q"], label="q")
plt.plot(t, df["r"], label="r")
plt.xlabel("Time [s]")
plt.ylabel("Rate [rad/s]")
plt.title("Body Rates")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Plot the navigation Euler angles.
phi = df.get("ptpN_phi", None)
theta = df.get("ptpN_theta", None)
psi = df.get("ptpN_psi", None)

if phi is not None and theta is not None and psi is not None:
    plt.figure(figsize=(9, 4))
    plt.plot(t, phi, label="phi")
    plt.plot(t, theta, label="theta")
    plt.plot(t, psi, label="psi")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")
    plt.title("Euler Angles (Navigation)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Plot magnetic field magnitudes.
B_truth = mag3(df["BBx"], df["BBy"], df["BBz"])
B_meas = mag3(df["Bmx"], df["Bmy"], df["Bmz"])
B_nav = mag3(df["BNx"], df["BNy"], df["BNz"])

plt.figure(figsize=(9, 4))
plt.plot(t, B_truth, label="|B| truth")
plt.plot(t, B_meas, label="|B| meas", alpha=0.6)
plt.plot(t, B_nav, label="|B| nav", linewidth=2)
plt.xlabel("Time [s]")
plt.ylabel("|B| [Tesla]")
plt.title("Magnetic Field Magnitude")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Plot the magnetorquer current commands.
if all(c in df.columns for c in ["ix", "iy", "iz"]):
    plt.figure(figsize=(9, 4))
    plt.plot(t, df["ix"], label="ix")
    plt.plot(t, df["iy"], label="iy")
    plt.plot(t, df["iz"], label="iz")
    plt.xlabel("Time [s]")
    plt.ylabel("Magnetorquer current [A]")
    plt.title("Magnetorquer Currents")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
