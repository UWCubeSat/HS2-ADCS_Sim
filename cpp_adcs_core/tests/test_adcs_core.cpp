#include "adcs/adcs_core.hpp"

#include <cmath>
#include <iostream>

int main() {
  adcs::StepInput input{};
  input.time_s = 0.0;
  input.omega_B_rad_s = {0.8, -0.2, 0.3};
  input.mag_B_T = {1.0e-5, -1.6e-6, 2.06e-5};

  const adcs::StepOutput out = adcs::step(input);

  if (!out.valid) {
    std::cerr << "Expected valid controller output\n";
    return 1;
  }
  if (!std::isfinite(out.coilPowerTotal_W)) {
    std::cerr << "Power is not finite\n";
    return 2;
  }

  std::cout << "m_cmd = ["
            << out.commandedDipole_B_Am2[0] << ", "
            << out.commandedDipole_B_Am2[1] << ", "
            << out.commandedDipole_B_Am2[2] << "] A m^2\n";
  std::cout << "i_cmd = ["
            << out.commandedCurrent_A[0] << ", "
            << out.commandedCurrent_A[1] << ", "
            << out.commandedCurrent_A[2] << "] A\n";
  std::cout << "P_total = " << out.coilPowerTotal_W << " W\n";
  return 0;
}
