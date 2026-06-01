#pragma once

#include <array>
#include <cstdint>

namespace adcs {

using Vec3 = std::array<double, 3>;

struct ControllerConfig {
  double dipoleCommandGain = 67200.0;
  Vec3 mtqDipoleGain_Am2_A{2.3, 2.3, 0.85 / 0.6301260378126048};
  Vec3 mtqResistance_Ohm{51.0, 51.0, 4.4};
  Vec3 mtqCurrentLimit_A{5.0 / 51.0, 5.0 / 51.0, 0.6301260378126048};
  Vec3 mtqDipoleLimit_Am2{0.2, 0.2, 0.85};
  double minMagField_T = 1.0e-12;
};

struct StepInput {
  double time_s = 0.0;
  Vec3 omega_B_rad_s{0.0, 0.0, 0.0};
  Vec3 mag_B_T{0.0, 0.0, 0.0};
};

struct StepOutput {
  Vec3 commandedDipole_B_Am2{0.0, 0.0, 0.0};
  Vec3 commandedCurrent_A{0.0, 0.0, 0.0};
  Vec3 commandedTorque_B_Nm{0.0, 0.0, 0.0};
  Vec3 coilPower_W{0.0, 0.0, 0.0};
  double coilPowerTotal_W = 0.0;
  std::array<bool, 3> saturation{false, false, false};
  bool valid = true;
};

StepOutput step(const StepInput& input, const ControllerConfig& config = ControllerConfig{});

}  // namespace adcs
