import numpy as np
import matplotlib.pyplot as plt

# Load B-field data (time, Bn, Be, Bd)
data = np.loadtxt("../output/bfield.txt", delimiter=",")

t = data[:, 0]              # seconds
Bn = data[:, 1]
Be = data[:, 2]
Bd = data[:, 3]

Bmag = np.linalg.norm(data[:, 1:4], axis=1)

# Magnitude
plt.figure()
plt.plot(t / 60.0, Bmag)
plt.xlabel("Time (min)")
plt.ylabel("|B| (T)")
plt.title("Magnetic Field Magnitude vs Time (WMM2020)")
plt.grid(True)

# Components (optional)
plt.figure()
plt.plot(t / 60.0, Bn, label="Bn")
plt.plot(t / 60.0, Be, label="Be")
plt.plot(t / 60.0, Bd, label="Bd")
plt.xlabel("Time (min)")
plt.ylabel("B (T)")
plt.title("Magnetic Field Components (NED)")
plt.legend()
plt.grid(True)

plt.show()
