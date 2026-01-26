#pragma once
#include "math.hpp"

// Returns NED magnetic field components (BN, BE, BD) in nanoTesla.
// Inputs are geocentric latitude (deg), longitude (deg), radius (km).
// MATLAB used: [BN,BE,BD] = igrf('01-Jan-2020',lat,lon,rhokm,'geocentric')
//
// NOTE: This implementation is a placeholder centered dipole.
// To be numerically faithful to MATLAB + IGRF, replace igrf_geocentric_nT()
// with a full IGRF (e.g., IGRF-13) implementation using official coefficients.
void igrf_geocentric_nT(double latitude_deg, double longitude_deg, double radius_km,
                        double& BN_nT, double& BE_nT, double& BD_nT);
