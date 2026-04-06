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
  const double gain = 67200.0;
  const Vec3 rawCurrent = (gain / (ctx.coilTurns * ctx.coilArea_m2)) * cross(w_body_rad_s, B_body_T);

  // Clamp each axis separately because each coil has its own current limit.
  auto clamp = [](double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  };

  current_cmd_A.x = clamp(rawCurrent.x, -ctx.maxCurrent_A, ctx.maxCurrent_A);
  current_cmd_A.y = clamp(rawCurrent.y, -ctx.maxCurrent_A, ctx.maxCurrent_A);
  current_cmd_A.z = clamp(rawCurrent.z, -ctx.maxCurrent_A, ctx.maxCurrent_A);
}
