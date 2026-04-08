#include "wmm.hpp"
#include <cmath>
#include <stdexcept>

namespace {

constexpr int kMaxOrd = 12;
constexpr int kSize = kMaxOrd + 1;
constexpr double kEpoch = 2025.0;

struct WmmCoeff {
  int n;
  int m;
  double g;
  double h;
  double gd;
  double hd;
};

// Embedded WMM2025 coefficients.
// Keeping them in the code means the sim runs without any extra data files.
constexpr WmmCoeff kCoeff[] = {
  {1,0,-29351.8,0.0,12.0,0.0},
  {1,1,-1410.8,4545.4,9.7,-21.5},
  {2,0,-2556.6,0.0,-11.6,0.0},
  {2,1,2951.1,-3133.6,-5.2,-27.7},
  {2,2,1649.3,-815.1,-8.0,-12.1},
  {3,0,1361.0,0.0,-1.3,0.0},
  {3,1,-2404.1,-56.6,-4.2,4.0},
  {3,2,1243.8,237.5,0.4,-0.3},
  {3,3,453.6,-549.5,-15.6,-4.1},
  {4,0,895.0,0.0,-1.6,0.0},
  {4,1,799.5,278.6,-2.4,-1.1},
  {4,2,55.7,-133.9,-6.0,4.1},
  {4,3,-281.1,212.0,5.6,1.6},
  {4,4,12.1,-375.6,-7.0,-4.4},
  {5,0,-233.2,0.0,0.6,0.0},
  {5,1,368.9,45.4,1.4,-0.5},
  {5,2,187.2,220.2,0.0,2.2},
  {5,3,-138.7,-122.9,0.6,0.4},
  {5,4,-142.0,43.0,2.2,1.7},
  {5,5,20.9,106.1,0.9,1.9},
  {6,0,64.4,0.0,-0.2,0.0},
  {6,1,63.8,-18.4,-0.4,0.3},
  {6,2,76.9,16.8,0.9,-1.6},
  {6,3,-115.7,48.8,1.2,-0.4},
  {6,4,-40.9,-59.8,-0.9,0.9},
  {6,5,14.9,10.9,0.3,0.7},
  {6,6,-60.7,72.7,0.9,0.9},
  {7,0,79.5,0.0,-0.0,0.0},
  {7,1,-77.0,-48.9,-0.1,0.6},
  {7,2,-8.8,-14.4,-0.1,0.5},
  {7,3,59.3,-1.0,0.5,-0.8},
  {7,4,15.8,23.4,-0.1,0.0},
  {7,5,2.5,-7.4,-0.8,-1.0},
  {7,6,-11.1,-25.1,-0.8,0.6},
  {7,7,14.2,-2.3,0.8,-0.2},
  {8,0,23.2,0.0,-0.1,0.0},
  {8,1,10.8,7.1,0.2,-0.2},
  {8,2,-17.5,-12.6,0.0,0.5},
  {8,3,2.0,11.4,0.5,-0.4},
  {8,4,-21.7,-9.7,-0.1,0.4},
  {8,5,16.9,12.7,0.3,-0.5},
  {8,6,15.0,0.7,0.2,-0.6},
  {8,7,-16.8,-5.2,-0.0,0.3},
  {8,8,0.9,3.9,0.2,0.2},
  {9,0,4.6,0.0,-0.0,0.0},
  {9,1,7.8,-24.8,-0.1,-0.3},
  {9,2,3.0,12.2,0.1,0.3},
  {9,3,-0.2,8.3,0.3,-0.3},
  {9,4,-2.5,-3.3,-0.3,0.3},
  {9,5,-13.1,-5.2,0.0,0.2},
  {9,6,2.4,7.2,0.3,-0.1},
  {9,7,8.6,-0.6,-0.1,-0.2},
  {9,8,-8.7,0.8,0.1,0.4},
  {9,9,-12.9,10.0,-0.1,0.1},
  {10,0,-1.3,0.0,0.1,0.0},
  {10,1,-6.4,3.3,0.0,0.0},
  {10,2,0.2,0.0,0.1,-0.0},
  {10,3,2.0,2.4,0.1,-0.2},
  {10,4,-1.0,5.3,-0.0,0.1},
  {10,5,-0.6,-9.1,-0.3,-0.1},
  {10,6,-0.9,0.4,0.0,0.1},
  {10,7,1.5,-4.2,-0.1,0.0},
  {10,8,0.9,-3.8,-0.1,-0.1},
  {10,9,-2.7,0.9,-0.0,0.2},
  {10,10,-3.9,-9.1,-0.0,-0.0},
  {11,0,2.9,0.0,0.0,0.0},
  {11,1,-1.5,0.0,-0.0,-0.0},
  {11,2,-2.5,2.9,0.0,0.1},
  {11,3,2.4,-0.6,0.0,-0.0},
  {11,4,-0.6,0.2,0.0,0.1},
  {11,5,-0.1,0.5,-0.1,-0.0},
  {11,6,-0.6,-0.3,0.0,-0.0},
  {11,7,-0.1,-1.2,-0.0,0.1},
  {11,8,1.1,-1.7,-0.1,-0.0},
  {11,9,-1.0,-2.9,-0.1,0.0},
  {11,10,-0.2,-1.8,-0.1,0.0},
  {11,11,2.6,-2.3,-0.1,0.0},
  {12,0,-2.0,0.0,0.0,0.0},
  {12,1,-0.2,-1.3,0.0,-0.0},
  {12,2,0.3,0.7,-0.0,0.0},
  {12,3,1.2,1.0,-0.0,-0.1},
  {12,4,-1.3,-1.4,-0.0,0.1},
  {12,5,0.6,-0.0,-0.0,-0.0},
  {12,6,0.6,0.6,0.1,-0.0},
  {12,7,0.5,-0.1,-0.0,-0.0},
  {12,8,-0.1,0.8,0.0,0.0},
  {12,9,-0.4,0.1,0.0,-0.0},
  {12,10,-0.2,-1.0,-0.1,-0.0},
  {12,11,-1.3,0.1,-0.0,0.0},
  {12,12,-0.7,0.2,-0.1,-0.1}
};

struct WmmTables {
  double c[kSize][kSize]{};
  double cd[kSize][kSize]{};
  double snorm[kSize * kSize]{};
  double fn[kSize]{};
  double fm[kSize]{};
  double k[kSize][kSize]{};
};

const WmmTables& tables() {
  static const WmmTables t = [] {
    WmmTables out{};

    // Read the coefficient list into the legacy matrix layout.
    for (const auto& entry : kCoeff) {
      out.c[entry.m][entry.n] = entry.g;
      out.cd[entry.m][entry.n] = entry.gd;
      if (entry.m != 0) {
        out.c[entry.n][entry.m - 1] = entry.h;
        out.cd[entry.n][entry.m - 1] = entry.hd;
      }
    }

    // Convert Schmidt-normalized coefficients into the unnormalized form used by the old WMM code.
    out.snorm[0] = 1.0;
    out.fm[0] = 0.0;

    for (int n = 1; n <= kMaxOrd; ++n) {
      out.snorm[n] = out.snorm[n - 1] * static_cast<double>(2 * n - 1) / static_cast<double>(n);

      int j = 2;
      int m = 0;
      while (m <= n) {
        if (n > 1) {
          out.k[m][n] = static_cast<double>(((n - 1) * (n - 1)) - (m * m)) /
                        static_cast<double>((2 * n - 1) * (2 * n - 3));
        }

        if (m > 0) {
          const double flnmj = static_cast<double>((n - m + 1) * j) / static_cast<double>(n + m);
          out.snorm[n + m * kSize] = out.snorm[n + (m - 1) * kSize] * std::sqrt(flnmj);
          j = 1;

          out.c[n][m - 1] *= out.snorm[n + m * kSize];
          out.cd[n][m - 1] *= out.snorm[n + m * kSize];
        }

        out.c[m][n] *= out.snorm[n + m * kSize];
        out.cd[m][n] *= out.snorm[n + m * kSize];
        ++m;
      }

      out.fn[n] = static_cast<double>(n + 1);
      out.fm[n] = static_cast<double>(n);
    }

    out.k[1][1] = 0.0;
    return out;
  }();

  return t;
}

} // namespace

void wmm2025_geodetic_ned_nT(double latitude_deg,
                             double longitude_deg,
                             double altitude_km,
                             double decimal_year,
                             double& X_nT,
                             double& Y_nT,
                             double& Z_nT) {
  if (decimal_year < 2025.0 || decimal_year > 2030.0) {
    throw std::runtime_error("WMM2025 is only valid from 2025.0 to 2030.0.");
  }

  const auto& wmm = tables();

  // Working arrays used by the legacy WMM recurrence relations.
  double tc[kSize][kSize]{};
  double dp[kSize][kSize]{};
  double sp[kSize]{};
  double cp[kSize]{};
  double pp[kSize]{};
  double p[kSize * kSize]{};

  for (int i = 0; i < kSize * kSize; ++i) {
    p[i] = wmm.snorm[i];
  }

  // WGS84 ellipsoid constants used by the legacy WMM code.
  const double a = 6378.137;
  const double b = 6356.7523142;
  const double re = 6371.2;
  const double a2 = a * a;
  const double b2 = b * b;
  const double c2 = a2 - b2;
  const double a4 = a2 * a2;
  const double b4 = b2 * b2;
  const double c4 = a4 - b4;

  const double dt = decimal_year - kEpoch;

  const double rlon = longitude_deg * M_PI / 180.0;
  const double rlat = latitude_deg * M_PI / 180.0;
  const double srlon = std::sin(rlon);
  const double srlat = std::sin(rlat);
  const double crlon = std::cos(rlon);
  const double crlat = std::cos(rlat);
  const double srlat2 = srlat * srlat;
  const double crlat2 = crlat * crlat;

  sp[0] = 0.0;
  cp[0] = 1.0;
  pp[0] = 1.0;
  dp[0][0] = 0.0;
  sp[1] = srlon;
  cp[1] = crlon;

  // Convert geodetic input into the spherical quantities used by the field equations.
  const double q = std::sqrt(a2 - c2 * srlat2);
  const double q1 = altitude_km * q;
  const double q2 = ((q1 + a2) / (q1 + b2)) * ((q1 + a2) / (q1 + b2));
  const double ct = srlat / std::sqrt(q2 * crlat2 + srlat2);
  const double st = std::sqrt(1.0 - ct * ct);
  const double r2 = (altitude_km * altitude_km) + 2.0 * q1 + (a4 - c4 * srlat2) / (q * q);
  const double r = std::sqrt(r2);
  const double d = std::sqrt(a2 * crlat2 + b2 * srlat2);
  const double ca = (altitude_km + d) / r;
  const double sa = c2 * crlat * srlat / (r * d);

  for (int m = 2; m <= kMaxOrd; ++m) {
    sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
    cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
  }

  double aor = re / r;
  double ar = aor * aor;
  double br = 0.0;
  double bt = 0.0;
  double bp = 0.0;
  double bpp = 0.0;

  for (int n = 1; n <= kMaxOrd; ++n) {
    ar *= aor;

    int m = 0;
    while (m <= n) {
      // Compute the unnormalized associated Legendre polynomials and their derivatives.
      if (n == m) {
        p[n + m * kSize] = st * p[n - 1 + (m - 1) * kSize];
        dp[m][n] = st * dp[m - 1][n - 1] + ct * p[n - 1 + (m - 1) * kSize];
      } else if (n == 1 && m == 0) {
        p[n + m * kSize] = ct * p[n - 1 + m * kSize];
        dp[m][n] = ct * dp[m][n - 1] - st * p[n - 1 + m * kSize];
      } else if (n > 1 && n != m) {
        if (m > n - 2) {
          p[n - 2 + m * kSize] = 0.0;
          dp[m][n - 2] = 0.0;
        }
        p[n + m * kSize] = ct * p[n - 1 + m * kSize] - wmm.k[m][n] * p[n - 2 + m * kSize];
        dp[m][n] = ct * dp[m][n - 1] - st * p[n - 1 + m * kSize] - wmm.k[m][n] * dp[m][n - 2];
      }

      // Time-shift the coefficients from epoch 2025.0 to the requested decimal year.
      tc[m][n] = wmm.c[m][n] + dt * wmm.cd[m][n];
      if (m != 0) {
        tc[n][m - 1] = wmm.c[n][m - 1] + dt * wmm.cd[n][m - 1];
      }

      // Accumulate the spherical harmonic expansion.
      const double par = ar * p[n + m * kSize];
      double temp1 = 0.0;
      double temp2 = 0.0;
      if (m == 0) {
        temp1 = tc[m][n] * cp[m];
        temp2 = tc[m][n] * sp[m];
      } else {
        temp1 = tc[m][n] * cp[m] + tc[n][m - 1] * sp[m];
        temp2 = tc[m][n] * sp[m] - tc[n][m - 1] * cp[m];
      }

      bt -= ar * temp1 * dp[m][n];
      bp += wmm.fm[m] * temp2 * par;
      br += wmm.fn[n] * temp1 * par;

      // Special handling at the poles.
      if (st == 0.0 && m == 1) {
        if (n == 1) {
          pp[n] = pp[n - 1];
        } else {
          pp[n] = ct * pp[n - 1] - wmm.k[m][n] * pp[n - 2];
        }
        const double parp = ar * pp[n];
        bpp += wmm.fm[m] * temp2 * parp;
      }

      ++m;
    }
  }

  if (st == 0.0) {
    bp = bpp;
  } else {
    bp /= st;
  }

  // Rotate from spherical components into local geodetic north/east/down.
  X_nT = -bt * ca - br * sa;
  Y_nT = bp;
  Z_nT = bt * sa - br * ca;
}
