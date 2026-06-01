#include "adcs/adcs_core.hpp"

#include <algorithm>
#include <cmath>

namespace adcs {
namespace {

Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3{
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
  };
}

double norm(const Vec3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool finite_vec(const Vec3& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

}  // namespace

StepOutput step(const StepInput& input, const ControllerConfig& config) {
  StepOutput out{};

  if (!std::isfinite(input.time_s) || !finite_vec(input.omega_B_rad_s) || !finite_vec(input.mag_B_T)) {
    out.valid = false;
    return out;
  }

  const double bmag = norm(input.mag_B_T);
  if (!std::isfinite(bmag) || bmag < config.minMagField_T) {
    out.valid = false;
    return out;
  }

  const Vec3 omega_cross_b = cross(input.omega_B_rad_s, input.mag_B_T);
  Vec3 desiredDipole{
      config.dipoleCommandGain * omega_cross_b[0],
      config.dipoleCommandGain * omega_cross_b[1],
      config.dipoleCommandGain * omega_cross_b[2],
  };

  for (int i = 0; i < 3; ++i) {
    const double gain = config.mtqDipoleGain_Am2_A[i];
    const double currentLimit = config.mtqCurrentLimit_A[i];
    const double dipoleLimit = config.mtqDipoleLimit_Am2[i];
    const double resistance = config.mtqResistance_Ohm[i];

    if (gain <= 0.0 || currentLimit <= 0.0 || dipoleLimit <= 0.0 || resistance < 0.0) {
      out.valid = false;
      out.commandedCurrent_A[i] = 0.0;
      out.commandedDipole_B_Am2[i] = 0.0;
      out.coilPower_W[i] = 0.0;
      out.saturation[i] = true;
      continue;
    }

    const double allowedCurrent = std::min(currentLimit, dipoleLimit / gain);
    const double rawCurrent = desiredDipole[i] / gain;
    const double cmdCurrent = clamp(rawCurrent, -allowedCurrent, allowedCurrent);

    out.commandedCurrent_A[i] = cmdCurrent;
    out.commandedDipole_B_Am2[i] = cmdCurrent * gain;
    out.coilPower_W[i] = resistance * cmdCurrent * cmdCurrent;
    out.coilPowerTotal_W += out.coilPower_W[i];
    out.saturation[i] = std::abs(rawCurrent - cmdCurrent) > 1.0e-15;
  }

  out.commandedTorque_B_Nm = cross(out.commandedDipole_B_Am2, input.mag_B_T);
  return out;
}

}  // namespace adcs
