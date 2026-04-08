#include "modules.hpp"
#include <cmath>

// Keep the actuator constants in one place.
static void magtorquer_params(SimContext& ctx) {
  ctx.n_turns = 84.0;
  ctx.A_turn = 0.02;
  ctx.maxCurrent_mA = 120.0;
}

// Simple magnetic detumble control law.
// This is not a full flight-quality controller, but it is a common first closed-loop test.
void control_compute(const Vec3& BfieldNav,
                     const Vec3& pqrNav,
                     const Vec3& ptpNav,
                     SimContext& ctx,
                     Vec3& current_out) {
  (void)ptpNav;

  const double k = 67200.0;
  magtorquer_params(ctx);

  // Command current from the rate-field cross product.
  const Vec3 c = cross(pqrNav, BfieldNav);
  current_out = (k / (ctx.n_turns * ctx.A_turn)) * c;
}
