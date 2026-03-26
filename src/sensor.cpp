#include "modules.hpp"

// Pick bias values once at the start of the run.
// That is much closer to a real sensor than changing the bias every time the nav filter runs.
static void sensor_params(SimContext& ctx) {
  ctx.nextSensorUpdate = 1.0;
  ctx.fsensor = 1.0;

  const double MagscaleBias = (4e-7) * ctx.fsensor;
  ctx.MagFieldBias = MagscaleBias * ctx.rng.rand_m11();

  const double AngscaleBias = 0.01 * ctx.fsensor;
  ctx.AngFieldBias = AngscaleBias * ctx.rng.rand_m11();

  const double EulerScaleBias = (2.0 * M_PI / 180.0) * ctx.fsensor;
  ctx.EulerBias = EulerScaleBias * ctx.rng.rand_m11();

  ctx.sensorModelInitialized = true;
}

// Noise gets re-drawn every sample, which is what we want.
static void sensor_noise(SimContext& ctx) {
  const double MagscaleNoise = (1e-5) * ctx.fsensor;
  ctx.MagFieldNoise = MagscaleNoise * ctx.rng.rand_m11();

  const double AngscaleNoise = 0.001 * ctx.fsensor;
  ctx.AngFieldNoise = AngscaleNoise * ctx.rng.rand_m11();

  const double EulerScaleNoise = (1.0 * M_PI / 180.0) * ctx.fsensor;
  ctx.EulerNoise = EulerScaleNoise * ctx.rng.rand_m11();
}

// Apply the sensor model to the truth signals.
void sensor_update(Vec3& BB, Vec3& pqr, Vec3& ptp, SimContext& ctx) {
  if (!ctx.sensorModelInitialized) {
    sensor_params(ctx);
  }

  for (int idx = 0; idx < 3; ++idx) {
    sensor_noise(ctx);
    BB[idx]  = BB[idx]  + ctx.MagFieldBias + ctx.MagFieldNoise;
    pqr[idx] = pqr[idx] + ctx.AngFieldBias + ctx.AngFieldNoise;
    ptp[idx] = ptp[idx] + ctx.EulerBias    + ctx.EulerNoise;
  }
}
