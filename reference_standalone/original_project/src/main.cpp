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

bool all_finite(const State13& a) {
  for (double v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
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
      << "ptpN_phi,ptpN_theta,ptpN_psi,qN0,qN1,qN2,qN3,"
      << "ix,iy,iz,"
      << "mcmd_x,mcmd_y,mcmd_z,"
      << "pcoil_x,pcoil_y,pcoil_z,pcoil_total,"
      << "rho_kg_m3,vrel_m_s,sunlit,"
      << "Fdrag_x,Fdrag_y,Fdrag_z,Fsrp_x,Fsrp_y,Fsrp_z,"
      << "Taero_x,Taero_y,Taero_z,Tgg_x,Tgg_y,Tgg_z,"
      << "Tmag_x,Tmag_y,Tmag_z,Tsrp_x,Tsrp_y,Tsrp_z,"
      << "Tdist_x,Tdist_y,Tdist_z\n";

  for (double t_s = 0.0; t_s <= tfinal_s + 1e-12; t_s += dt_s) {
    // ----- Truth environment -----
    ctx.BfieldReference_eci_T = truth_magnetic_field_eci_T(t_s, state, ctx);
    const Mat3 C_IB = TIBquat(state_quaternion(state));
    ctx.BfieldTruth_body_T = mul(transpose(C_IB), ctx.BfieldReference_eci_T);

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
    const Vec3 dipole_body_Am2 = magnetorquer_dipole_body_Am2(ctx);
    const Vec3 pcoil_W = magnetorquer_coil_power_W(ctx);
    const double pcoilTotal_W = magnetorquer_total_coil_power_W(ctx);
    DisturbanceBreakdown disturbanceNow{};
    const Vec3 pos_eci_m{state[0], state[1], state[2]};
    const Vec3 vel_eci_m_s{state[3], state[4], state[5]};
    const Quat qNow = state_quaternion(state);
    const double radius_m = norm(pos_eci_m);
    const double altitude_m = radius_m - ctx.earthRadius_m;
    disturbance_summary(t_s, altitude_m, pos_eci_m, vel_eci_m_s, qNow, ctx.BfieldTruth_body_T, ctx, disturbanceNow);
    const Vec3 totalDisturbanceTorque_body_Nm = disturbanceNow.totalTorque_body_Nm();

    csv << t_s << ",";
    for (int i = 0; i < 13; ++i) {
      csv << state[i] << ",";
    }

    csv << ctx.BfieldTruth_body_T.x << "," << ctx.BfieldTruth_body_T.y << "," << ctx.BfieldTruth_body_T.z << ","
        << ctx.BfieldMeasured_body_T.x << "," << ctx.BfieldMeasured_body_T.y << "," << ctx.BfieldMeasured_body_T.z << ","
        << ctx.BfieldNav_T.x << "," << ctx.BfieldNav_T.y << "," << ctx.BfieldNav_T.z << ","
        << ctx.pqrMeasured_rad_s.x << "," << ctx.pqrMeasured_rad_s.y << "," << ctx.pqrMeasured_rad_s.z << ","
        << ctx.pqrNav_rad_s.x << "," << ctx.pqrNav_rad_s.y << "," << ctx.pqrNav_rad_s.z << ","
        << ctx.ptpNav_rad.x << "," << ctx.ptpNav_rad.y << "," << ctx.ptpNav_rad.z << ","
        << ctx.qNav_IB.q0 << "," << ctx.qNav_IB.q1 << "," << ctx.qNav_IB.q2 << "," << ctx.qNav_IB.q3 << ","
        << ctx.commandedCurrent_A.x << "," << ctx.commandedCurrent_A.y << "," << ctx.commandedCurrent_A.z << ","
        << dipole_body_Am2.x << "," << dipole_body_Am2.y << "," << dipole_body_Am2.z << ","
        << pcoil_W.x << "," << pcoil_W.y << "," << pcoil_W.z << "," << pcoilTotal_W << ","
        << disturbanceNow.density_kg_m3 << "," << disturbanceNow.relativeSpeed_m_s << "," << disturbanceNow.sunlit << ","
        << disturbanceNow.dragForce_eci_N.x << "," << disturbanceNow.dragForce_eci_N.y << "," << disturbanceNow.dragForce_eci_N.z << ","
        << disturbanceNow.solarRadiationForce_eci_N.x << "," << disturbanceNow.solarRadiationForce_eci_N.y << "," << disturbanceNow.solarRadiationForce_eci_N.z << ","
        << disturbanceNow.aerodynamicTorque_body_Nm.x << "," << disturbanceNow.aerodynamicTorque_body_Nm.y << "," << disturbanceNow.aerodynamicTorque_body_Nm.z << ","
        << disturbanceNow.gravityGradientTorque_body_Nm.x << "," << disturbanceNow.gravityGradientTorque_body_Nm.y << "," << disturbanceNow.gravityGradientTorque_body_Nm.z << ","
        << disturbanceNow.residualMagneticTorque_body_Nm.x << "," << disturbanceNow.residualMagneticTorque_body_Nm.y << "," << disturbanceNow.residualMagneticTorque_body_Nm.z << ","
        << disturbanceNow.solarRadiationTorque_body_Nm.x << "," << disturbanceNow.solarRadiationTorque_body_Nm.y << "," << disturbanceNow.solarRadiationTorque_body_Nm.z << ","
        << totalDisturbanceTorque_body_Nm.x << "," << totalDisturbanceTorque_body_Nm.y << "," << totalDisturbanceTorque_body_Nm.z
        << "\n";

    // ----- Integrate one RK4 step using the held actuator command -----
    const State13 k1 = satellite_derivatives(t_s, state, ctx);
    const State13 k2 = satellite_derivatives(t_s + dt_s / 2.0, add(state, scale(k1, dt_s / 2.0)), ctx);
    const State13 k3 = satellite_derivatives(t_s + dt_s / 2.0, add(state, scale(k2, dt_s / 2.0)), ctx);
    const State13 k4 = satellite_derivatives(t_s + dt_s, add(state, scale(k3, dt_s)), ctx);

    if (!all_finite(k1) || !all_finite(k2) || !all_finite(k3) || !all_finite(k4)) {
      std::cerr << "Non-finite RK4 derivative at t = " << t_s << " s\n";
      return 1;
    }

    State13 incr{};
    for (int i = 0; i < 13; ++i) {
      incr[i] = (1.0 / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    for (int i = 0; i < 13; ++i) {
      state[i] += dt_s * incr[i];
    }

    if (!all_finite(state)) {
      std::cerr << "Non-finite propagated state at t = " << t_s << " s\n";
      return 1;
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
