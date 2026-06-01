#include "modules.hpp"
#include <cmath>

// This is intentionally a placeholder controller.
// The real magnetorquer team code should replace this file once their interface is ready.
// For now the controller takes the smoothed body-rate and magnetic-field estimate and returns
// a coil-current command in body axes.
void control_compute(const SimContext& ctx, Vec3& current_cmd_A) {
  const Vec3 B_body_T = ctx.BfieldNav_T;
  const Vec3 w_body_rad_s = ctx.pqrNav_rad_s;

  const double Bmag = norm(B_body_T);
  if (Bmag < 1e-12) {
    current_cmd_A = {0.0, 0.0, 0.0};
    return;
  }

  // This is a simple rate-cross-field damping law.
  // It is useful for closed-loop bring-up, but it is not the final flight controller.
  const double dipoleCommandGain = 67200.0;
  const Vec3 desiredDipole_body_Am2 = dipoleCommandGain * cross(w_body_rad_s, B_body_T);

  // Clamp each axis separately because each coil has its own current limit.
  auto clamp = [](double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  };

  auto allowed_current = [](double dipoleGain_Am2_A, double currentLimit_A, double dipoleLimit_Am2) {
    if (dipoleGain_Am2_A <= 0.0 || currentLimit_A <= 0.0 || dipoleLimit_Am2 <= 0.0) {
      return 0.0;
    }
    const double currentLimitFromDipole_A = dipoleLimit_Am2 / dipoleGain_Am2_A;
    return std::fmin(currentLimit_A, currentLimitFromDipole_A);
  };

  const double ixLimit_A = allowed_current(ctx.mtqDipoleGain_Am2_A.x, ctx.mtqCurrentLimit_A.x, ctx.mtqDipoleLimit_Am2.x);
  const double iyLimit_A = allowed_current(ctx.mtqDipoleGain_Am2_A.y, ctx.mtqCurrentLimit_A.y, ctx.mtqDipoleLimit_Am2.y);
  const double izLimit_A = allowed_current(ctx.mtqDipoleGain_Am2_A.z, ctx.mtqCurrentLimit_A.z, ctx.mtqDipoleLimit_Am2.z);

  const double rawIx_A = desiredDipole_body_Am2.x / ctx.mtqDipoleGain_Am2_A.x;
  const double rawIy_A = desiredDipole_body_Am2.y / ctx.mtqDipoleGain_Am2_A.y;
  const double rawIz_A = desiredDipole_body_Am2.z / ctx.mtqDipoleGain_Am2_A.z;

  current_cmd_A.x = clamp(rawIx_A, -ixLimit_A, ixLimit_A);
  current_cmd_A.y = clamp(rawIy_A, -iyLimit_A, iyLimit_A);
  current_cmd_A.z = clamp(rawIz_A, -izLimit_A, izLimit_A);
}
