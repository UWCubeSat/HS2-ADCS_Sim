#include "modules.hpp"
#include "params.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {

State13 add(const State13& a, const State13& b) {
  State13 c{};
  for (int i = 0; i < 13; ++i) {
    c[i] = a[i] + b[i];
  }
  return c;
}

State13 scale(const State13& a, double s) {
  State13 c{};
  for (int i = 0; i < 13; ++i) {
    c[i] = a[i] * s;
  }
  return c;
}

} // namespace

int main() {
  // Fixed seed makes the sensor noise repeatable while debugging.
  SimContext ctx(/*seed=*/1);
  initialize_simulation(ctx);

  // Start in a 600 km circular orbit.
  const double altitude_m = 600.0 * 1000.0;
  const double x0 = ctx.earthRadius_m + altitude_m;
  const double y0 = 0.0;
  const double z0 = 0.0;
  const double xdot0 = 0.0;

  const double inclination_rad = 56.0 * M_PI / 180.0;
  const double semiMajor_m = std::sqrt(x0 * x0 + y0 * y0 + z0 * z0);
  const double circularSpeed_m_s = std::sqrt(ctx.mu_m3_s2 / semiMajor_m);
  const double ydot0 = circularSpeed_m_s * std::cos(inclination_rad);
  const double zdot0 = circularSpeed_m_s * std::sin(inclination_rad);

  // Start with level attitude and a noticeable tumble rate.
  const Quat q0 = euler321_to_quat({0.0, 0.0, 0.0});
  const double p0 = 0.8;
  const double qRate0 = -0.2;
  const double r0 = 0.3;

  State13 state{};
  state[0] = x0;        state[1] = y0;        state[2] = z0;
  state[3] = xdot0;     state[4] = ydot0;     state[5] = zdot0;
  state[6] = q0.q0;     state[7] = q0.q1;     state[8] = q0.q2;     state[9] = q0.q3;
  state[10] = p0;       state[11] = qRate0;   state[12] = r0;

  // Run for one orbit.
  const double orbitalPeriod_s = 2.0 * M_PI / std::sqrt(ctx.mu_m3_s2) * std::pow(semiMajor_m, 1.5);
  const double tfinal_s = orbitalPeriod_s;
  const double dt_s = SimParams::timestep;

  // Discrete update clocks.
  double nextSensorSample_s = 0.0;
  double nextControlUpdate_s = 0.0;
  const double controlPeriod_s = 0.1;

  std::ofstream csv("adcs_output.csv");
  csv << std::setprecision(17);
  csv << "t,"
      << "x,y,z,xdot,ydot,zdot,q0,q1,q2,q3,p,q,r,"
      << "BBx,BBy,BBz,Bmx,Bmy,Bmz,BNx,BNy,BNz,"
      << "pqrm_x,pqrm_y,pqrm_z,pqrN_x,pqrN_y,pqrN_z,"
      << "ptpm_phi,ptpm_theta,ptpm_psi,ptpN_phi,ptpN_theta,ptpN_psi,"
      << "ix,iy,iz\n";

  for (double t_s = 0.0; t_s <= tfinal_s + 1e-12; t_s += dt_s) {
    // ----- Truth environment -----
    ctx.BfieldTruth_body_T = truth_magnetic_field_body_T(t_s, state, ctx);

    // ----- Sensor and nav updates -----
    if (t_s + 1e-12 >= nextSensorSample_s) {
      sensor_update_from_truth(state, ctx.BfieldTruth_body_T, ctx);
      navigation_update(ctx);
      nextSensorSample_s += ctx.sensorPeriod_s;
    }

    // ----- Controller update -----
    if (t_s + 1e-12 >= nextControlUpdate_s) {
      control_compute(ctx, ctx.commandedCurrent_A);
      nextControlUpdate_s += controlPeriod_s;
    }

    // ----- Log current state -----
    csv << t_s << ",";
    for (int i = 0; i < 13; ++i) {
      csv << state[i] << ",";
    }

    csv << ctx.BfieldTruth_body_T.x << "," << ctx.BfieldTruth_body_T.y << "," << ctx.BfieldTruth_body_T.z << ","
        << ctx.BfieldMeasured_body_T.x << "," << ctx.BfieldMeasured_body_T.y << "," << ctx.BfieldMeasured_body_T.z << ","
        << ctx.BfieldNav_T.x << "," << ctx.BfieldNav_T.y << "," << ctx.BfieldNav_T.z << ","
        << ctx.pqrMeasured_rad_s.x << "," << ctx.pqrMeasured_rad_s.y << "," << ctx.pqrMeasured_rad_s.z << ","
        << ctx.pqrNav_rad_s.x << "," << ctx.pqrNav_rad_s.y << "," << ctx.pqrNav_rad_s.z << ","
        << ctx.ptpMeasured_rad.x << "," << ctx.ptpMeasured_rad.y << "," << ctx.ptpMeasured_rad.z << ","
        << ctx.ptpNav_rad.x << "," << ctx.ptpNav_rad.y << "," << ctx.ptpNav_rad.z << ","
        << ctx.commandedCurrent_A.x << "," << ctx.commandedCurrent_A.y << "," << ctx.commandedCurrent_A.z
        << "\n";

    // ----- Integrate one RK4 step using the held actuator command -----
    const State13 k1 = satellite_derivatives(t_s, state, ctx);
    const State13 k2 = satellite_derivatives(t_s + dt_s / 2.0, add(state, scale(k1, dt_s / 2.0)), ctx);
    const State13 k3 = satellite_derivatives(t_s + dt_s / 2.0, add(state, scale(k2, dt_s / 2.0)), ctx);
    const State13 k4 = satellite_derivatives(t_s + dt_s, add(state, scale(k3, dt_s)), ctx);

    State13 incr{};
    for (int i = 0; i < 13; ++i) {
      incr[i] = (1.0 / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    for (int i = 0; i < 13; ++i) {
      state[i] += dt_s * incr[i];
    }

    // Keep the quaternion normalized so numerical drift does not slowly break attitude math.
    Quat q = state_quaternion(state);
    q = normalize(q);
    state[6] = q.q0;
    state[7] = q.q1;
    state[8] = q.q2;
    state[9] = q.q3;
  }

  std::cout << "Done. Wrote adcs_output.csv\n";
  return 0;
}
