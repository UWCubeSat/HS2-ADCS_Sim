#pragma once
#include "math.hpp"

// Compute the WMM2025 magnetic field at a geodetic point.
// Inputs:
//   latitude_deg  - geodetic latitude in degrees
//   longitude_deg - geodetic longitude in degrees
//   altitude_km   - altitude above the WGS84 ellipsoid in kilometers
//   decimal_year  - decimal year, valid from 2025.0 through 2030.0
// Outputs:
//   X_nT          - north component in nanoTesla
//   Y_nT          - east component in nanoTesla
//   Z_nT          - down component in nanoTesla
void wmm2025_geodetic_ned_nT(double latitude_deg,
                             double longitude_deg,
                             double altitude_km,
                             double decimal_year,
                             double& X_nT,
                             double& Y_nT,
                             double& Z_nT);
