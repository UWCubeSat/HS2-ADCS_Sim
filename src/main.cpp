#include "modules.hpp"
#include "params.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

// Add two state vectors component-by-component.
static State13 add(const State13& a, const State13& b) {
  State13 c{};
  for (int i = 0; i < 13; ++i) {
    c[i] = a[i] + b[i];
  }
  return c;
}

// Multiply a state vector by a scalar.
static State13 scale(const State13& a, double s) {
  State13 c{};
  for (int i = 0; i < 13; ++i) {
    c[i] = a[i] * s;
  }
  return c;
}

int main() {
  // Fixed seed makes debugging repeatable.
  SimContext ctx(/*seed=*/1);

  // Start in a 600 km orbit.
  const double altitude_m = 600.0 * 1000.0;

  // Set the Earth constants here so we can build the starting orbit right away.
  // We do not call the full dynamics function on a zero state because that would
  // divide by zero in the orbit math and pollute the nav state before the run starts.
  ctx.R = 6.371e6;
  ctx.M = 5.972e24;
  ctx.G = 6.67e-11;
  ctx.mu = ctx.G * ctx.M;

  // Circular orbit initial condition in the inertial frame.
  const double x0 = ctx.R + altitude_m;
  const double y0 = 0.0;
  const double z0 = 0.0;
  const double xdot0 = 0.0;

  const double inclination = 56.0 * M_PI / 180.0;
  const double semi_major = std::sqrt(x0 * x0 + y0 * y0 + z0 * z0);
  const double vcircular = std::sqrt(ctx.mu / semi_major);
  const double ydot0 = vcircular * std::cos(inclination);
  const double zdot0 = vcircular * std::sin(inclination);

  // Start with level attitude but some body-rate tumble.
  const Vec3 ptp0{0.0, 0.0, 0.0};
  const Quat q0 = euler321_to_quat(ptp0);
  const double p0 = 0.8;
  const double q00 = -0.2;
  const double r0 = 0.3;

  State13 state{};
  state[0] = x0;      state[1] = y0;      state[2] = z0;
  state[3] = xdot0;   state[4] = ydot0;   state[5] = zdot0;
  state[6] = q0.q0;   state[7] = q0.q1;   state[8] = q0.q2;   state[9] = q0.q3;
  state[10] = p0;     state[11] = q00;    state[12] = r0;

  // Run for one orbit.
  const double period = 2.0 * M_PI / std::sqrt(ctx.mu) * std::pow(semi_major, 1.5);
  const double tfinal = period;
  const double dt = SimParams::timestep;

  // Initial update settings.
  ctx.lastSensorUpdate = 0.0;
  ctx.lastMagUpdate = 0.0;
  ctx.nextMagUpdate = 1.0;
  ctx.nextSensorUpdate = 1.0;

  // Prime the truth/sensor/nav states before the first log line.
  satellite_derivatives(0.0, state, ctx);

  // Controller update period.
  double lastControl = -dt;
  const double nextControl = 0.1;

  std::ofstream csv("adcs_output.csv");
  csv << std::setprecision(17);

  // CSV header.
  csv << "t,"
      << "x,y,z,xdot,ydot,zdot,q0,q1,q2,q3,p,q,r,"
      << "BBx,BBy,BBz,Bmx,Bmy,Bmz,BNx,BNy,BNz,"
      << "pqrm_x,pqrm_y,pqrm_z,pqrN_x,pqrN_y,pqrN_z,"
      << "ptpm_phi,ptpm_theta,ptpm_psi,ptpN_phi,ptpN_theta,ptpN_psi,"
      << "ix,iy,iz\n";

  for (double t = 0.0; t <= tfinal + 1e-12; t += dt) {
    // Log the state and the main internal signals.
    csv << t << ",";
    for (int i = 0; i < 13; ++i) {
      csv << state[i] << ",";
    }

    csv << ctx.BB_truth.x << "," << ctx.BB_truth.y << "," << ctx.BB_truth.z << ","
        << ctx.BfieldMeasured.x << "," << ctx.BfieldMeasured.y << "," << ctx.BfieldMeasured.z << ","
        << ctx.BfieldNav.x << "," << ctx.BfieldNav.y << "," << ctx.BfieldNav.z << ","
        << ctx.pqrMeasured.x << "," << ctx.pqrMeasured.y << "," << ctx.pqrMeasured.z << ","
        << ctx.pqrNav.x << "," << ctx.pqrNav.y << "," << ctx.pqrNav.z << ","
        << ctx.ptpMeasured.x << "," << ctx.ptpMeasured.y << "," << ctx.ptpMeasured.z << ","
        << ctx.ptpNav.x << "," << ctx.ptpNav.y << "," << ctx.ptpNav.z << ","
        << ctx.current.x << "," << ctx.current.y << "," << ctx.current.z
        << "\n";

    // Update the controller on its own schedule.
    if (t > lastControl) {
      Vec3 current{};
      control_compute(ctx.BfieldNav, ctx.pqrNav, ctx.ptpNav, ctx, current);
      ctx.current = current;
      lastControl += nextControl;
    }

    // RK4 integration step.
    const State13 k1 = satellite_derivatives(t, state, ctx);
    const State13 k2 = satellite_derivatives(t + dt / 2.0, add(state, scale(k1, dt / 2.0)), ctx);
    const State13 k3 = satellite_derivatives(t + dt / 2.0, add(state, scale(k2, dt / 2.0)), ctx);
    const State13 k4 = satellite_derivatives(t + dt, add(state, scale(k3, dt)), ctx);

    State13 incr{};
    for (int i = 0; i < 13; ++i) {
      incr[i] = (1.0 / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    for (int i = 0; i < 13; ++i) {
      state[i] += dt * incr[i];
    }

    // Keep the quaternion from drifting numerically.
    Quat q{state[6], state[7], state[8], state[9]};
    q = normalize(q);
    state[6] = q.q0;
    state[7] = q.q1;
    state[8] = q.q2;
    state[9] = q.q3;
  }

  std::cout << "Done. Wrote adcs_output.csv\n";
  return 0;
}
