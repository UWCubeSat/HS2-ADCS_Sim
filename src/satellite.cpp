#include "modules.hpp"
#include "frames.hpp"
#include "math.hpp"
#include "quaternion.hpp"
#include "wmm.hpp"
#include <cmath>

namespace {

// Earth constants used by the orbit model.
void planet(SimContext& ctx) {
  ctx.R = 6.371e6;
  ctx.M = 5.972e24;
  ctx.G = 6.67e-11;
  ctx.mu = ctx.G * ctx.M;
}

// Basic spacecraft dimensions and mass.
void satellite_params(SimContext& ctx) {
  ctx.ms = 2.6;
  ctx.lx = 0.10;
  ctx.ly = 0.10;
  ctx.lz = 0.20;
  ctx.Amax = ctx.lx * ctx.ly;
  ctx.lmax = ctx.lz / 2.0;
  ctx.CD = 1.0;
  ctx.m = ctx.ms;
}

// Box inertia model for the spacecraft bus.
void inertia_params(SimContext& ctx) {
  Mat3 Is{};
  Is[0][0] = (ctx.ms / 12.0) * (ctx.ly * ctx.ly + ctx.lz * ctx.lz);
  Is[1][1] = (ctx.ms / 12.0) * (ctx.lx * ctx.lx + ctx.lz * ctx.lz);
  Is[2][2] = (ctx.ms / 12.0) * (ctx.lx * ctx.lx + ctx.ly * ctx.ly);

  ctx.Is = Is;
  ctx.I = Is;
  ctx.invI = inv3(ctx.I);
}

// Coil parameters used by the magnetorquer model.
void magtorquer_params(SimContext& ctx) {
  ctx.n_turns = 84.0;
  ctx.A_turn = 0.02;
  ctx.maxCurrent_mA = 120.0;
}

Quat state_to_quat(const State13& s) {
  return {s[6], s[7], s[8], s[9]};
}

Vec3 state_to_pqr(const State13& s) {
  return {s[10], s[11], s[12]};
}

// Convert seconds since t=0 into decimal year.
double decimal_year_from_seconds(double year0, double t_seconds) {
  const double seconds_per_year = 365.25 * 24.0 * 3600.0;
  return year0 + t_seconds / seconds_per_year;
}

} // namespace

State13 satellite_derivatives(double t, const State13& state, SimContext& ctx) {
  planet(ctx);
  satellite_params(ctx);
  inertia_params(ctx);
  magtorquer_params(ctx);

  const Vec3 r_eci{state[0], state[1], state[2]};
  const Vec3 v_eci{state[3], state[4], state[5]};
  const Quat q = state_to_quat(state);
  const Vec3 pqr_body = state_to_pqr(state);

  const double rho = norm(r_eci);
  const Vec3 rhat = (rho > 0.0) ? (r_eci / rho) : Vec3{0.0, 0.0, 0.0};

  // Central gravity only.
  const Vec3 Fg = (-(ctx.mu * ctx.m) / (rho * rho)) * rhat;

  // Simple disturbance model.
  const double altitude_m = rho - ctx.R;
  Vec3 XYZD{0.0, 0.0, 0.0};
  Vec3 LMND{0.0, 0.0, 0.0};
  disturbance(altitude_m, ctx.Amax, ctx.lmax, v_eci, ctx.CD, ctx.BB_truth, XYZD, LMND);

  const Vec3 accel_eci = (1.0 / ctx.m) * (Fg + XYZD);

  // Attitude kinematics and human-readable angles.
  const Quat qdot = quat_derivative(q, pqr_body);
  const Vec3 ptp = quat_to_euler321(q);

  // Update the truth magnetic field at the requested sample rate.
  if (t >= ctx.lastMagUpdate) {
    ctx.lastMagUpdate += ctx.nextMagUpdate;

    // Step 1: move from the inertial orbit frame into Earth-fixed coordinates.
    const double earth_angle = ctx.greenwichAngle0 + ctx.omegaE * t;
    const Vec3 r_ecef = eci_to_ecef(r_eci, earth_angle);

    // Step 2: get the geodetic point needed by WMM.
    const GeodeticLLA lla = ecef_to_geodetic_wgs84(r_ecef);
    const double decimal_year = decimal_year_from_seconds(ctx.decimalYear0, t);

    // Step 3: ask WMM for the local North/East/Down field in nanoTesla.
    double X_nT = 0.0;
    double Y_nT = 0.0;
    double Z_nT = 0.0;
    wmm2025_geodetic_ned_nT(lla.lat_rad * 180.0 / M_PI,
                            lla.lon_rad * 180.0 / M_PI,
                            lla.alt_m / 1000.0,
                            decimal_year,
                            X_nT,
                            Y_nT,
                            Z_nT);

    // Step 4: turn local NED into an ECEF vector.
    const Mat3 C_ecef_ned = ned_basis_ecef(lla.lat_rad, lla.lon_rad);
    const Vec3 B_ned_nT{X_nT, Y_nT, Z_nT};
    const Vec3 B_ecef_nT = mul(C_ecef_ned, B_ned_nT);

    // Step 5: move ECEF -> ECI -> body frame.
    const Vec3 B_eci_nT = ecef_to_eci(B_ecef_nT, earth_angle);
    const Mat3 C_IB = TIBquat(q);
    const Vec3 B_body_nT = mul(transpose(C_IB), B_eci_nT);

    // Store truth field in Tesla, because that keeps the torque math in SI units.
    ctx.BB_truth = 1e-9 * B_body_nT;
  }

  // Push truth through the sensor and nav models.
  if (t >= ctx.lastSensorUpdate) {
    ctx.lastSensorUpdate += ctx.nextSensorUpdate;

    Vec3 BBm = ctx.BB_truth;
    Vec3 pqrm = pqr_body;
    Vec3 ptpm = ptp;

    sensor_update(BBm, pqrm, ptpm, ctx);
    ctx.BfieldMeasured = BBm;
    ctx.pqrMeasured = pqrm;
    ctx.ptpMeasured = ptpm;

    navigation_update(ctx.BfieldMeasured, ctx.pqrMeasured, ctx.ptpMeasured, ctx);
  }

  // Clamp each coil current separately.
  const double Imax_A = ctx.maxCurrent_mA * 1e-3;
  auto clamp = [](double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  };
  ctx.current.x = clamp(ctx.current.x, -Imax_A, Imax_A);
  ctx.current.y = clamp(ctx.current.y, -Imax_A, Imax_A);
  ctx.current.z = clamp(ctx.current.z, -Imax_A, Imax_A);

  // Magnetic dipole and magnetic torque in the body frame.
  const Vec3 dipole_body = (ctx.n_turns * ctx.A_turn) * ctx.current;
  const Vec3 LMN_mag = cross(dipole_body, ctx.BB_truth);

  const Vec3 LMN_total = LMN_mag + LMND;

  // Rigid-body rotational dynamics in the body frame.
  const Vec3 H = mul(ctx.I, pqr_body);
  const Vec3 pqrdot = mul(ctx.invI, (LMN_total - cross(pqr_body, H)));

  State13 dst{};

  // Position derivative is velocity.
  dst[0] = v_eci.x;
  dst[1] = v_eci.y;
  dst[2] = v_eci.z;

  // Velocity derivative is acceleration.
  dst[3] = accel_eci.x;
  dst[4] = accel_eci.y;
  dst[5] = accel_eci.z;

  // Quaternion derivative.
  dst[6] = qdot.q0;
  dst[7] = qdot.q1;
  dst[8] = qdot.q2;
  dst[9] = qdot.q3;

  // Body-rate derivative.
  dst[10] = pqrdot.x;
  dst[11] = pqrdot.y;
  dst[12] = pqrdot.z;

  return dst;
}
