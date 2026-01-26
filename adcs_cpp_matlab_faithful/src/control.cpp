#include "modules.hpp"
#include <cmath>

static void magtorquer_params(SimContext& ctx){
  ctx.n_turns = 84;
  ctx.A_turn = 0.02;
  ctx.maxCurrent_mA = 120;
}

void control_compute(const Vec3& BfieldNav, const Vec3& pqrNav, const Vec3& ptpNav, SimContext& ctx, Vec3& current_out, Vec3& rwalphas_out){
  // BDOT-ish controller from Control.m
  const double k = 67200.0;
  magtorquer_params(ctx);
  const Vec3 c = cross(pqrNav, BfieldNav);
  current_out = (k / (ctx.n_turns * ctx.A_turn)) * c;

  // Reaction wheel controller
  Vec3 Mdesired{0,0,0};
  if (std::abs(pqrNav.x) + std::abs(pqrNav.y) + std::abs(pqrNav.z) < 0.3){
    // KP = I*1.0*IrR(1,1); KD = I*45*IrR(1,1)
    const double Ir11 = ctx.IrR[0][0];
    const double KP = 1.0 * Ir11;
    const double KD = 45.0 * Ir11;
    const Vec3 ptpcommand{1,0,0};
    const Vec3 pqrcommand{0,0,0};
    // Mdesired = -KD*(pqrcommand - pqrNav) - KP*(ptpcommand-ptpNav);
    Mdesired = (-KD)*(pqrcommand - pqrNav) + (-KP)*(ptpcommand - ptpNav);
  } else {
    Mdesired = {0,0,0};
  }

  // rwalphas = Jinv * Mdesired (treat Jinv as 3x3, Mdesired as vec)
  rwalphas_out = mul(ctx.Jinv, Mdesired);
}
