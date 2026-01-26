#include "igrf.hpp"
#include <cmath>

void igrf_geocentric_nT(double latitude_deg, double /*longitude_deg*/, double radius_km,
                        double& BN_nT, double& BE_nT, double& BD_nT)
{
  // Centered dipole aligned with Earth's spin axis (very rough).
  // Choose dipole moment so that |B| ~ 30-60 microTesla at Earth's surface.
  // This is NOT IGRF; replace for true numerical faithfulness.
  const double lat = latitude_deg * M_PI/180.0;
  const double r   = radius_km * 1000.0;

  const double Re = 6.371e6;
  const double B0 = 3.12e-5; // Tesla at equator surface (approx)
  const double scale = std::pow(Re/r, 3);

  // Dipole field in NED (north,east,down), with east ~ 0 in this simplified model
  const double BN_T =  B0 * scale * std::cos(lat);
  const double BD_T =  2.0 * B0 * scale * std::sin(lat); // down-positive
  BN_nT = BN_T * 1e9;
  BE_nT = 0.0;
  BD_nT = BD_T * 1e9;
}
