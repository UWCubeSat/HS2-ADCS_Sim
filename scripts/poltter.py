import numpy as np
import matplotlib.pyplot as plt

times = []
positions = []
velocities = []

with open("C:/Users/zaids/HS2-ADCS_Sim/output/test.txt", "r") as f:
    for line in f:
        vals = [float(x) for x in line.strip().split(",")]
        times.append(vals[0])
        positions.append(vals[1:4])
        velocities.append(vals[4:])

times = np.array(times)
positions = np.array(positions)
velocities = np.array(velocities)
velocity_mag = np.linalg.norm(velocities, axis=1)

xout = positions[:, 0] / 1000  # x in km
yout = positions[:, 1] / 1000  # y in km
zout = positions[:, 2] / 1000  # z in km
u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
U, V = np.meshgrid(u, v)

# dimensions in km
X = (6371) * np.cos(U) * np.sin(V)
Y = (6371) * np.sin(U) * np.sin(V)
Z = (6371) * np.cos(V)

# Plotting
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('white')
# ax.view_init(elev=0, azim=0)


# Plot the satellite trajectory
ax.plot3D(xout, yout, zout, 'b-', linewidth=2)

# Plot the Earth
ax.plot_surface(X, Y, Z, rstride=4, cstride=4, color='lightblue', edgecolor='none', alpha=0.6)

# Formatting
ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
ax.grid(True)
ax.set_xlabel('X (km)')
ax.set_ylabel('Y (km)')
ax.set_zlabel('Z (km)')
plt.title("Satellite Orbit Around Earth")

# here for debugging:

# fig4, ax4 = plt.subplots(figsize=(10,6))
# fig4.patch.set_facecolor('white')
# ax4.plot(times, velocity_mag, label='Velocity',color='blue')
# ax4.set_title("Velocity")
# ax4.set_xlabel("Time (s)")
# ax4.set_ylabel("Velocity (m/s)")
# ax4.grid(True)
# ax4.legend()


plt.show()