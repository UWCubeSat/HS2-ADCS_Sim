#pragma once
#include "math.hpp"
#include <cmath>

// 3-2-1 Euler rotation matrix.
// This maps a vector from body coordinates to inertial coordinates.
inline Mat3 TIB(double phi, double theta, double psi) {
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double sp = std::sin(phi);
  const double cp = std::cos(phi);
  const double ss = std::sin(psi);
  const double cs = std::cos(psi);

  Mat3 out{};
  out[0][0] = ct * cs;
  out[0][1] = sp * st * cs - cp * ss;
  out[0][2] = cp * st * cs + sp * ss;

  out[1][0] = ct * ss;
  out[1][1] = sp * st * ss + cp * cs;
  out[1][2] = cp * st * ss - sp * cs;

  out[2][0] = -st;
  out[2][1] = sp * ct;
  out[2][2] = cp * ct;
  return out;
}

// Small helper used by some of the old code.
inline Mat3 Rscrew(const Vec3& nhat) {
  const double x = nhat.x;
  const double y = nhat.y;
  const double z = nhat.z;
  const double psi = std::atan2(y, x);
  const double theta = std::atan2(z, std::sqrt(x * x + y * y));
  const double phi = 0.0;
  return transpose(TIB(phi, theta, psi));
}

// Rotate a vector about the Earth Z axis.
inline Mat3 Rz(double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  Mat3 R{};
  R[0][0] = c;  R[0][1] = -s; R[0][2] = 0.0;
  R[1][0] = s;  R[1][1] =  c; R[1][2] = 0.0;
  R[2][0] = 0.0;R[2][1] = 0.0;R[2][2] = 1.0;
  return R;
}

// Convert an inertial vector into Earth-fixed coordinates.
inline Vec3 eci_to_ecef(const Vec3& r_eci, double earth_angle_rad) {
  return mul(Rz(earth_angle_rad), r_eci);
}

// Convert an Earth-fixed vector back into inertial coordinates.
inline Vec3 ecef_to_eci(const Vec3& v_ecef, double earth_angle_rad) {
  return mul(Rz(-earth_angle_rad), v_ecef);
}

// Simple geodetic container.
struct GeodeticLLA {
  double lat_rad{0.0};
  double lon_rad{0.0};
  double alt_m{0.0};
};

// Convert ECEF position to geodetic latitude, longitude, and height above WGS84.
inline GeodeticLLA ecef_to_geodetic_wgs84(const Vec3& r_ecef) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);

  const double x = r_ecef.x;
  const double y = r_ecef.y;
  const double z = r_ecef.z;

  const double lon = std::atan2(y, x);
  const double p = std::sqrt(x * x + y * y);

  if (p < 1e-9) {
    const double lat = (z >= 0.0) ? (M_PI / 2.0) : (-M_PI / 2.0);
    const double alt = std::abs(z) - a * std::sqrt(1.0 - e2);
    return {lat, lon, alt};
  }

  double lat = std::atan2(z, p * (1.0 - e2));
  double N = 0.0;
  double alt = 0.0;

  for (int k = 0; k < 6; ++k) {
    const double s = std::sin(lat);
    N = a / std::sqrt(1.0 - e2 * s * s);
    alt = p / std::cos(lat) - N;
    lat = std::atan2(z, p * (1.0 - e2 * (N / (N + alt))));
  }

  return {lat, lon, alt};
}

// Local NED basis written in ECEF components.
// Each column of the matrix is one unit vector: [North East Down].
inline Mat3 ned_basis_ecef(double lat_rad, double lon_rad) {
  const double sphi = std::sin(lat_rad);
  const double cphi = std::cos(lat_rad);
  const double slam = std::sin(lon_rad);
  const double clam = std::cos(lon_rad);

  const Vec3 north{-sphi * clam, -sphi * slam,  cphi};
  const Vec3 east {-slam,         clam,         0.0};
  const Vec3 down {-cphi * clam, -cphi * slam, -sphi};

  Mat3 A{};
  A[0][0] = north.x; A[1][0] = north.y; A[2][0] = north.z;
  A[0][1] = east.x;  A[1][1] = east.y;  A[2][1] = east.z;
  A[0][2] = down.x;  A[1][2] = down.y;  A[2][2] = down.z;
  return A;
}
