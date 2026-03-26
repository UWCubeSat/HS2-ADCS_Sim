#include "modules.hpp"

// Very simple first-order smoothing filter.
// This is still not a real estimator, but it is cleaner than the old sentinel/bias-reset logic.
void navigation_update(const Vec3& BfieldMeasured,
                       const Vec3& pqrMeasured,
                       const Vec3& ptpMeasured,
                       SimContext& ctx) {
  const double s = 0.3;

  // First pass: initialize directly from the measurements.
  if (!ctx.navInitialized) {
    ctx.BfieldNav = BfieldMeasured;
    ctx.pqrNav = pqrMeasured;
    ctx.ptpNav = ptpMeasured;

    ctx.BfieldNavPrev = ctx.BfieldNav;
    ctx.pqrNavPrev = ctx.pqrNav;
    ctx.ptpNavPrev = ctx.ptpNav;
    ctx.Bdot = {0.0, 0.0, 0.0};
    ctx.navInitialized = true;
    return;
  }

  const Vec3 zeroBias{0.0, 0.0, 0.0};

  ctx.BfieldNav = (1.0 - s) * ctx.BfieldNavPrev + s * (BfieldMeasured - zeroBias);
  ctx.pqrNav    = (1.0 - s) * ctx.pqrNavPrev    + s * (pqrMeasured - zeroBias);
  ctx.ptpNav    = (1.0 - s) * ctx.ptpNavPrev    + s * (ptpMeasured - zeroBias);

  ctx.Bdot = (ctx.BfieldNav - ctx.BfieldNavPrev) / ctx.nextSensorUpdate;

  ctx.BfieldNavPrev = ctx.BfieldNav;
  ctx.pqrNavPrev = ctx.pqrNav;
  ctx.ptpNavPrev = ctx.ptpNav;
}
