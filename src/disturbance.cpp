#include "modules.hpp"
#include <cmath>

// This atmosphere model is still crude.
// It is only here to keep a small amount of drag in the orbit dynamics.
// It should not be treated as a flight-quality environment model.
double density_kg_m3(double altitude_m) {
  const double rho0 = 1.225;
  const double sigma = 0.1354;
  return rho0 * std::exp(-sigma * altitude_m / 1000.0);
}

void disturbance(double altitude_m,
                 const Vec3& vel_eci_m_s,
                 const Vec3& B_body_T,
                 const SimContext& ctx,
                 Vec3& disturbanceForce_eci_N,
                 Vec3& disturbanceTorque_body_Nm) {
  (void)B_body_T;

  // Keep only the simplest drag force for now.
  // This is easier to defend than keeping several half-finished torque models.
  const double speed = norm(vel_eci_m_s);
  const double rho = density_kg_m3(altitude_m);
  const Vec3 vhat = (speed > 0.0) ? (vel_eci_m_s / speed) : Vec3{0.0, 0.0, 0.0};

  const double dragMag_N = 0.5 * rho * speed * speed * ctx.maxArea_m2 * ctx.dragCoeff;
  disturbanceForce_eci_N = -dragMag_N * vhat;

  // Environmental torques are intentionally set to zero until they are modeled correctly.
  // That keeps the sim honest instead of pretending we have good torque physics when we do not.
  disturbanceTorque_body_Nm = {0.0, 0.0, 0.0};
}
