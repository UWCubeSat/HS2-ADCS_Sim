import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("build/adcs_output.csv")

# ---------- Helpers ----------
def mag3(x, y, z):
    return np.sqrt(x*x + y*y + z*z)

t = df["t"].to_numpy()

# ---------- 0) Quick sanity metrics ----------
# Check for NaN/Inf in numeric columns
numeric = df.select_dtypes(include=[np.number])
bad = ~np.isfinite(numeric.to_numpy()).all()
print("Rows:", len(df), "Cols:", len(df.columns))
print("Any NaN/Inf in numeric data?:", bool(bad))

# Final body rate magnitude
wmag = mag3(df["p"], df["q"], df["r"]).to_numpy()
print("Initial |w| [rad/s]:", float(wmag[0]))
print("Final   |w| [rad/s]:", float(wmag[-1]))
print("Min/Max |w| [rad/s]:", float(wmag.min()), float(wmag.max()))

# ---------- 1) Body rates ----------
plt.figure(figsize=(9,4))
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

# ---------- 2) Euler angles (NAV) ----------
# These columns exist in the project logger; if yours differ, tell me the header line and I'll adapt.
phi   = df.get("ptpN_phi", None)
theta = df.get("ptpN_theta", None)
psi   = df.get("ptpN_psi", None)

if phi is not None and theta is not None and psi is not None:
    plt.figure(figsize=(9,4))
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
else:
    print("Euler NAV columns not found (ptpN_phi/theta/psi).")

# ---------- 3) Magnetic field magnitude ----------
B_truth = mag3(df["BBx"], df["BBy"], df["BBz"])
B_meas  = mag3(df["Bmx"], df["Bmy"], df["Bmz"])
B_nav   = mag3(df["BNx"], df["BNy"], df["BNz"])

plt.figure(figsize=(9,4))
plt.plot(t, B_truth, label="|B| truth")
plt.plot(t, B_meas,  label="|B| meas", alpha=0.6)
plt.plot(t, B_nav,   label="|B| nav", linewidth=2)
plt.xlabel("Time [s]")
plt.ylabel("|B| [Tesla]")
plt.title("Magnetic Field Magnitude")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# ---------- 4) Reaction wheel commands ----------
# If your column names differ, print(df.columns) and tell me.
if all(c in df.columns for c in ["rwa1","rwa2","rwa3"]):
    plt.figure(figsize=(9,4))
    plt.plot(t, df["rwa1"], label="alpha1")
    plt.plot(t, df["rwa2"], label="alpha2")
    plt.plot(t, df["rwa3"], label="alpha3")
    plt.xlabel("Time [s]")
    plt.ylabel("Wheel accel [rad/s^2]")
    plt.title("Reaction Wheel Commands")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    print("RW command columns not found (rwa1/rwa2/rwa3).")

# ---------- 5) Magnetorquer currents ----------
if all(c in df.columns for c in ["im1","im2","im3"]):
    plt.figure(figsize=(9,4))
    plt.plot(t, df["im1"], label="im1")
    plt.plot(t, df["im2"], label="im2")
    plt.plot(t, df["im3"], label="im3")
    plt.xlabel("Time [s]")
    plt.ylabel("Magnetorquer current [A]")
    plt.title("Magnetorquer Currents")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    print("Magnetorquer current columns not found (im1/im2/im3).")
