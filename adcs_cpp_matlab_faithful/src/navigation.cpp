#include "modules.hpp"

// from sensor.cpp
void _sensor_params_for_navigation(SimContext& ctx);

void navigation_update(const Vec3& BfieldMeasured, const Vec3& pqrMeasured, const Vec3& ptpMeasured, SimContext& ctx){
  const double s = 0.3;
  if (ctx.BfieldNavPrev.x == -99){
    ctx.BfieldNav = BfieldMeasured;
    ctx.pqrNav = pqrMeasured;
    ctx.ptpNav = ptpMeasured;
  } else {
    const Vec3 BiasEstimate{0,0,0};
    ctx.BfieldNav = (1.0-s)*ctx.BfieldNavPrev + s*(BfieldMeasured - BiasEstimate);

    const Vec3 pqrBiasEstimate{0,0,0};
    ctx.pqrNav = (1.0-s)*ctx.pqrNavPrev + s*(pqrMeasured - pqrBiasEstimate);

    const Vec3 ptpBiasEstimate{0,0,0};
    ctx.ptpNav = (1.0-s)*ctx.ptpNavPrev + s*(ptpMeasured - ptpBiasEstimate);

    // MATLAB calls sensor_params here (re-randomizes biases!)
    _sensor_params_for_navigation(ctx);
    ctx.Bdot = (ctx.BfieldNav - ctx.BfieldNavPrev) / ctx.nextSensorUpdate;
  }
  ctx.BfieldNavPrev = ctx.BfieldNav;
  ctx.pqrNavPrev = ctx.pqrNav;
  ctx.ptpNavPrev = ctx.ptpNav;
}
