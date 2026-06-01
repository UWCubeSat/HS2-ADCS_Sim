#include "modules.hpp"

namespace {

Vec3 draw_uniform_noise(Rng& rng, double scale) {
  return {
    scale * rng.rand_m11(),
    scale * rng.rand_m11(),
    scale * rng.rand_m11()
  };
}

void initialize_sensor_model(SimContext& ctx) {
  // Fixed per-axis biases. These stay constant for the whole run.
  ctx.magBias_T = draw_uniform_noise(ctx.rng, 4e-7);
  ctx.gyroBias_rad_s = draw_uniform_noise(ctx.rng, 0.01);

  ctx.sensorModelInitialized = true;
}

} // namespace

void sensor_update_from_truth(const State13& truth_state, const Vec3& Btruth_body_T, SimContext& ctx) {
  if (!ctx.sensorModelInitialized) {
    initialize_sensor_model(ctx);
  }

  // Draw fresh white-noise samples.
  ctx.magNoise_T = draw_uniform_noise(ctx.rng, 1e-5);
  ctx.gyroNoise_rad_s = draw_uniform_noise(ctx.rng, 0.001);

  const Vec3 truthRates = state_body_rates_rad_s(truth_state);

  ctx.BfieldMeasured_body_T = Btruth_body_T + ctx.magBias_T + ctx.magNoise_T;
  ctx.pqrMeasured_rad_s = truthRates + ctx.gyroBias_rad_s + ctx.gyroNoise_rad_s;
}
