#include "modules.hpp"
#include <cmath>

double density_kg_m3(double altitude_m){
  const double ds = 1.225;
  const double sigma = 0.1354;
  return ds * std::exp(-sigma * altitude_m / 1000.0);
}

void disturbance(double altitude_m, double Amax, double lmax, const Vec3& vel, double CD, const Vec3& BI_Tesla,
                 Vec3& XYZD_out, Vec3& LMND_out)
{
  const double V = norm(vel);
  const double d = density_kg_m3(altitude_m);
  const double Fdrag = 0.5 * d * V*V * Amax * CD;
  const Vec3 vhat = (V>0) ? (vel / V) : Vec3{0,0,0};
  const Vec3 XYZAERO = -Fdrag * vhat;
  const double Maero = Fdrag * lmax / 2.0;
  const Vec3 LMNAERO{Maero,Maero,Maero};

  const double solar_pressure = 4.5e-6; // Pa
  const double Fpressure = solar_pressure * Amax;
  const Vec3 shat = vhat; // MATLAB placeholder
  const Vec3 XYZSRP = -Fpressure * shat;
  const double Mpressure = Fpressure * lmax / 2.0;
  const Vec3 LMNSRP{Mpressure,Mpressure,Mpressure};

  const double dconstant = 2.64e-3;
  const Vec3 LMNMDM = dconstant * BI_Tesla;

  XYZD_out = XYZSRP + XYZAERO;
  LMND_out = LMNSRP + LMNAERO + LMNMDM;
}
