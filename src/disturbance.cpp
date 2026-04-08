#include "modules.hpp"
#include <cmath>

// Very rough exponential atmosphere.
// This is still a placeholder, but it gives the sim some drag without needing a full atmosphere model.
double density_kg_m3(double altitude_m) {
  const double rho0 = 1.225;
  const double sigma = 0.1354;
  return rho0 * std::exp(-sigma * altitude_m / 1000.0);
}

// Simple disturbance force and torque model.
// This part is still approximate and should not be treated as a flight-quality environment model.
void disturbance(double altitude_m,
                 double Amax,
                 double lmax,
                 const Vec3& vel,
                 double CD,
                 const Vec3& BI_Tesla,
                 Vec3& XYZD_out,
                 Vec3& LMND_out) {
  const double V = norm(vel);
  const double rho = density_kg_m3(altitude_m);

  const double Fdrag = 0.5 * rho * V * V * Amax * CD;
  const Vec3 vhat = (V > 0.0) ? (vel / V) : Vec3{0.0, 0.0, 0.0};
  const Vec3 F_aero = -Fdrag * vhat;
  const double M_aero = Fdrag * lmax / 2.0;
  const Vec3 T_aero{M_aero, M_aero, M_aero};

  const double solar_pressure = 4.5e-6;
  const double F_srp_mag = solar_pressure * Amax;
  const Vec3 shat = vhat;
  const Vec3 F_srp = -F_srp_mag * shat;
  const double M_srp = F_srp_mag * lmax / 2.0;
  const Vec3 T_srp{M_srp, M_srp, M_srp};

  // Placeholder residual magnetic dipole torque term.
  const double dconstant = 2.64e-3;
  const Vec3 T_residual = dconstant * BI_Tesla;

  XYZD_out = F_srp + F_aero;
  LMND_out = T_srp + T_aero + T_residual;
}
