#include "modules.hpp"
#include "frames.hpp"
#include "quaternion.hpp"
#include "wmm.hpp"
#include <cmath>

namespace {

Vec3 state_position_eci_m(const State13& state) {
  return {state[0], state[1], state[2]};
}

Vec3 state_velocity_eci_m_s(const State13& state) {
  return {state[3], state[4], state[5]};
}

double clamp_abs(double value, double limit) {
  if (limit <= 0.0) {
    return 0.0;
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

} // namespace

void initialize_simulation(SimContext& ctx) {
  // Earth constants.
  ctx.earthRadius_m = 6.371e6;
  ctx.earthMass_kg = 5.972e24;
  ctx.gravConst_SI = 6.67e-11;
  ctx.mu_m3_s2 = ctx.gravConst_SI * ctx.earthMass_kg;

  // Spacecraft box geometry.
  ctx.mass_kg = 2.6;
  ctx.lx_m = 0.10;
  ctx.ly_m = 0.10;
  ctx.lz_m = 0.20;
  ctx.maxArea_m2 = ctx.lx_m * ctx.ly_m;
  ctx.maxMomentArm_m = ctx.lz_m / 2.0;
  ctx.dragCoeff = 2.2;
  ctx.solarRadiationPressure_N_m2 = 4.56e-6;
  ctx.solarReflectivityCoeff = 1.3;
  ctx.residualDipole_body_Am2 = {2.0e-4, 2.0e-4, 4.5e-3};
  ctx.aeroCpOffset_body_m = {0.0, 0.0, 0.0};
  ctx.srpCpOffset_body_m = {0.0, 0.0, 0.0};

  // Box inertia in the body frame.
  Mat3 I{};
  I[0][0] = (ctx.mass_kg / 12.0) * (ctx.ly_m * ctx.ly_m + ctx.lz_m * ctx.lz_m);
  I[1][1] = (ctx.mass_kg / 12.0) * (ctx.lx_m * ctx.lx_m + ctx.lz_m * ctx.lz_m);
  I[2][2] = (ctx.mass_kg / 12.0) * (ctx.lx_m * ctx.lx_m + ctx.ly_m * ctx.ly_m);
  ctx.inertia_body_kgm2 = I;
  ctx.inertiaInv_body_kgm2 = inv3(I);

  // HuskySat-2 first-pass actuator realism assumptions:
  // - X/Y axes use CubeSpace CR0002 public datasheet values.
  // - Z axis uses the EXA MT01 public marketplace sheet and keeps only the
  //   quantities we can defend today without driver-specific test data.
  ctx.mtqDipoleGain_Am2_A = {2.3, 2.3, 0.85 / std::sqrt(1.75 / 4.4)};
  ctx.mtqResistance_Ohm = {51.0, 51.0, 4.4};
  ctx.mtqCurrentLimit_A = {5.0 / 51.0, 5.0 / 51.0, std::sqrt(1.75 / 4.4)};
  ctx.mtqDipoleLimit_Am2 = {0.2, 0.2, 0.85};
  ctx.commandedCurrent_A = {0.0, 0.0, 0.0};

  // Sensor/nav defaults.
  ctx.sensorPeriod_s = 1.0;
  ctx.sensorModelInitialized = false;
  ctx.navInitialized = false;
  ctx.navBlend = 0.3;
  ctx.navAttitudeCorrectionGain = 0.8;
  ctx.navBiasCorrectionGain = 0.02;

  ctx.BfieldReference_eci_T = {0.0, 0.0, 0.0};
  ctx.BfieldTruth_body_T = {0.0, 0.0, 0.0};
  ctx.BfieldMeasured_body_T = {0.0, 0.0, 0.0};
  ctx.pqrMeasured_rad_s = {0.0, 0.0, 0.0};
  ctx.qNav_IB = {1.0, 0.0, 0.0, 0.0};
  ctx.gyroBiasNav_rad_s = {0.0, 0.0, 0.0};
}

Quat state_quaternion(const State13& state) {
  return {state[6], state[7], state[8], state[9]};
}

Vec3 state_body_rates_rad_s(const State13& state) {
  return {state[10], state[11], state[12]};
}

Vec3 state_euler321_rad(const State13& state) {
  return quat_to_euler321(state_quaternion(state));
}

Vec3 truth_magnetic_field_eci_T(double t_s, const State13& state, const SimContext& ctx) {
  const Vec3 r_eci_m = state_position_eci_m(state);

  // 1) Move the orbit position from inertial to Earth-fixed coordinates.
  const double earthAngle = ctx.greenwichAngle0_rad + ctx.earthRotationRate_rad_s * t_s;
  const Vec3 r_ecef_m = eci_to_ecef(r_eci_m, earthAngle);

  // 2) Convert Earth-fixed position to geodetic latitude, longitude, and altitude.
  const GeodeticLLA lla = ecef_to_geodetic_wgs84(r_ecef_m);

  // 3) Ask WMM for the local North/East/Down field in nT.
  const double decimalYear = ctx.decimalYear0 + t_s / (365.25 * 86400.0);
  double X_nT = 0.0;
  double Y_nT = 0.0;
  double Z_nT = 0.0;
  wmm2025_geodetic_ned_nT(lla.lat_rad * 180.0 / M_PI,
                          lla.lon_rad * 180.0 / M_PI,
                          lla.alt_m / 1000.0,
                          decimalYear,
                          X_nT,
                          Y_nT,
                          Z_nT);

  // 4) Convert the local NED vector to ECEF, then to ECI, then to body.
  const Mat3 C_ecef_ned = ned_basis_ecef(lla.lat_rad, lla.lon_rad);
  const Vec3 B_ned_nT{X_nT, Y_nT, Z_nT};
  const Vec3 B_ecef_nT = mul(C_ecef_ned, B_ned_nT);
  const Vec3 B_eci_nT = ecef_to_eci(B_ecef_nT, earthAngle);

  // 5) Convert nT to Tesla before using the field in any torque calculation.
  return 1e-9 * B_eci_nT;
}

Vec3 truth_magnetic_field_body_T(double t_s, const State13& state, const SimContext& ctx) {
  const Quat q = state_quaternion(state);
  const Vec3 B_eci_T = truth_magnetic_field_eci_T(t_s, state, ctx);

  const Mat3 C_IB = TIBquat(q);
  return mul(transpose(C_IB), B_eci_T);
}

Vec3 magnetorquer_dipole_body_Am2(const SimContext& ctx) {
  Vec3 dipole{
    ctx.mtqDipoleGain_Am2_A.x * ctx.commandedCurrent_A.x,
    ctx.mtqDipoleGain_Am2_A.y * ctx.commandedCurrent_A.y,
    ctx.mtqDipoleGain_Am2_A.z * ctx.commandedCurrent_A.z
  };

  dipole.x = clamp_abs(dipole.x, ctx.mtqDipoleLimit_Am2.x);
  dipole.y = clamp_abs(dipole.y, ctx.mtqDipoleLimit_Am2.y);
  dipole.z = clamp_abs(dipole.z, ctx.mtqDipoleLimit_Am2.z);
  return dipole;
}

Vec3 magnetorquer_coil_power_W(const SimContext& ctx) {
  // Until we add explicit RL driver dynamics, the held command is the coil current.
  return {
    ctx.mtqResistance_Ohm.x * ctx.commandedCurrent_A.x * ctx.commandedCurrent_A.x,
    ctx.mtqResistance_Ohm.y * ctx.commandedCurrent_A.y * ctx.commandedCurrent_A.y,
    ctx.mtqResistance_Ohm.z * ctx.commandedCurrent_A.z * ctx.commandedCurrent_A.z
  };
}

double magnetorquer_total_coil_power_W(const SimContext& ctx) {
  const Vec3 pcoil_W = magnetorquer_coil_power_W(ctx);
  return pcoil_W.x + pcoil_W.y + pcoil_W.z;
}

State13 satellite_derivatives(double t_s, const State13& state, const SimContext& ctx) {
  const Vec3 r_eci_m = state_position_eci_m(state);
  const Vec3 v_eci_m_s = state_velocity_eci_m_s(state);
  const Quat q = state_quaternion(state);
  const Vec3 w_body_rad_s = state_body_rates_rad_s(state);

  const double radius_m = norm(r_eci_m);
  const Vec3 rhat_eci = (radius_m > 0.0) ? (r_eci_m / radius_m) : Vec3{0.0, 0.0, 0.0};
  const double altitude_m = radius_m - ctx.earthRadius_m;

  // Central gravity in the inertial frame.
  const Vec3 gravityForce_eci_N = (-(ctx.mu_m3_s2 * ctx.mass_kg) / (radius_m * radius_m)) * rhat_eci;

  // Compute the truth field directly from the current truth state.
  const Vec3 B_body_T = truth_magnetic_field_body_T(t_s, state, ctx);

  // Simple disturbance model.
  Vec3 disturbanceForce_eci_N{0.0, 0.0, 0.0};
  Vec3 disturbanceTorque_body_Nm{0.0, 0.0, 0.0};
  disturbance(t_s, altitude_m, r_eci_m, v_eci_m_s, q, B_body_T, ctx, disturbanceForce_eci_N, disturbanceTorque_body_Nm);

  // Magnetorquer torque in the body frame.
  const Vec3 dipole_body_Am2 = magnetorquer_dipole_body_Am2(ctx);
  const Vec3 controlTorque_body_Nm = cross(dipole_body_Am2, B_body_T);

  // Translational dynamics.
  const Vec3 accel_eci_m_s2 = (gravityForce_eci_N + disturbanceForce_eci_N) / ctx.mass_kg;

  // Rotational dynamics.
  const Quat qdot = quat_derivative(q, w_body_rad_s);
  const Vec3 H_body = mul(ctx.inertia_body_kgm2, w_body_rad_s);
  const Vec3 totalTorque_body_Nm = controlTorque_body_Nm + disturbanceTorque_body_Nm;
  const Vec3 wdot_body_rad_s2 = mul(ctx.inertiaInv_body_kgm2, totalTorque_body_Nm - cross(w_body_rad_s, H_body));

  State13 dst{};
  dst[0] = v_eci_m_s.x;
  dst[1] = v_eci_m_s.y;
  dst[2] = v_eci_m_s.z;

  dst[3] = accel_eci_m_s2.x;
  dst[4] = accel_eci_m_s2.y;
  dst[5] = accel_eci_m_s2.z;

  dst[6] = qdot.q0;
  dst[7] = qdot.q1;
  dst[8] = qdot.q2;
  dst[9] = qdot.q3;

  dst[10] = wdot_body_rad_s2.x;
  dst[11] = wdot_body_rad_s2.y;
  dst[12] = wdot_body_rad_s2.z;

  return dst;
}
