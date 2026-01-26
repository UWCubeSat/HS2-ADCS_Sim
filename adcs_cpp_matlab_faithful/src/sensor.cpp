#include "modules.hpp"

// Mirrors Sensor.m + sensor_noise.m
static void sensor_noise(SimContext& ctx){
  const double MagscaleNoise = (1e-5) * ctx.fsensor;
  ctx.MagFieldNoise = MagscaleNoise * ctx.rng.rand_m11();

  const double AngscaleNoise = 0.001 * ctx.fsensor;
  ctx.AngFieldNoise = AngscaleNoise * ctx.rng.rand_m11();

  const double EulerScaleNoise = (1.0*M_PI/180.0) * ctx.fsensor;
  ctx.EulerNoise = EulerScaleNoise * ctx.rng.rand_m11();
}

// Mirrors sensor_params.m (called from main and Navigation.m)
static void sensor_params(SimContext& ctx){
  ctx.nextSensorUpdate = 1.0;
  ctx.fsensor = 1.0;

  const double MagscaleBias = (4e-7) * ctx.fsensor;
  ctx.MagFieldBias = MagscaleBias * ctx.rng.rand_m11();

  const double AngscaleBias = 0.01 * ctx.fsensor;
  ctx.AngFieldBias = AngscaleBias * ctx.rng.rand_m11();

  double EulerBias = (2.0*M_PI/180.0) * ctx.fsensor;
  ctx.EulerBias = EulerBias * ctx.rng.rand_m11();
}

void sensor_update(Vec3& BB, Vec3& pqr, Vec3& ptp, SimContext& ctx){
  // In MATLAB, biases are globals already set by sensor_params.
  // Here we assume sensor_params was called at init and then (oddly) inside navigation_update.
  for(int idx=0; idx<3; ++idx){
    sensor_noise(ctx);
    BB[idx]  = BB[idx]  + ctx.MagFieldBias + ctx.MagFieldNoise;
    pqr[idx] = pqr[idx] + ctx.AngFieldBias + ctx.AngFieldNoise;
    ptp[idx] = ptp[idx] + ctx.EulerBias    + ctx.EulerNoise;
  }
}

// Expose sensor_params for navigation.cpp via a forward declaration workaround.
// (We keep it in this TU to match MATLAB side-effects.)
void _sensor_params_for_navigation(SimContext& ctx){ sensor_params(ctx); }
