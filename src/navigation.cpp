#include "modules.hpp"

// This block is only a simple smoothing filter.
// It exists so the controller reads "estimated" values instead of truth directly.
// Once a real estimator exists, this file is the one that should be replaced.
void navigation_update(SimContext& ctx) {
  if (!ctx.navInitialized) {
    ctx.BfieldNav_T = ctx.BfieldMeasured_body_T;
    ctx.pqrNav_rad_s = ctx.pqrMeasured_rad_s;
    ctx.ptpNav_rad = ctx.ptpMeasured_rad;

    ctx.BfieldNavPrev_T = ctx.BfieldNav_T;
    ctx.pqrNavPrev_rad_s = ctx.pqrNav_rad_s;
    ctx.ptpNavPrev_rad = ctx.ptpNav_rad;
    ctx.BdotNav_T_s = {0.0, 0.0, 0.0};
    ctx.navInitialized = true;
    return;
  }

  const double s = ctx.navBlend;

  ctx.BfieldNav_T = (1.0 - s) * ctx.BfieldNavPrev_T + s * ctx.BfieldMeasured_body_T;
  ctx.pqrNav_rad_s = (1.0 - s) * ctx.pqrNavPrev_rad_s + s * ctx.pqrMeasured_rad_s;
  ctx.ptpNav_rad = (1.0 - s) * ctx.ptpNavPrev_rad + s * ctx.ptpMeasured_rad;

  ctx.BdotNav_T_s = (ctx.BfieldNav_T - ctx.BfieldNavPrev_T) / ctx.sensorPeriod_s;

  ctx.BfieldNavPrev_T = ctx.BfieldNav_T;
  ctx.pqrNavPrev_rad_s = ctx.pqrNav_rad_s;
  ctx.ptpNavPrev_rad = ctx.ptpNav_rad;
}
