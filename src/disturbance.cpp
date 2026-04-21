#include "modules.hpp"
#include <cmath>

namespace {

struct AtmosphereLayer {
  double baseAltitude_km;
  double baseDensity_kg_m3;
  double scaleHeight_km;
};

// Piecewise exponential atmosphere based on the standard engineering table that is
// commonly used for quick LEO density estimates out to 1000 km.
constexpr AtmosphereLayer kAtmosphereLayers[] = {
  {0.0,   1.2250000000000000,  7.249},
  {25.0,  3.899e-2,            6.349},
  {30.0,  1.774e-2,            6.682},
  {40.0,  3.972e-3,            7.554},
  {50.0,  1.057e-3,            8.382},
  {60.0,  3.206e-4,            7.714},
  {70.0,  8.770e-5,            6.549},
  {80.0,  1.905e-5,            5.799},
  {90.0,  3.396e-6,            5.382},
  {100.0, 5.297e-7,            5.877},
  {110.0, 9.661e-8,            7.263},
  {120.0, 2.438e-8,            9.473},
  {130.0, 8.484e-9,           12.636},
  {140.0, 3.845e-9,           16.149},
  {150.0, 2.070e-9,           22.523},
  {180.0, 5.464e-10,          29.740},
  {200.0, 2.789e-10,          37.105},
  {250.0, 7.248e-11,          45.546},
  {300.0, 2.418e-11,          53.628},
  {350.0, 9.518e-12,          53.298},
  {400.0, 3.725e-12,          58.515},
  {450.0, 1.585e-12,          60.828},
  {500.0, 6.967e-13,          63.822},
  {600.0, 1.454e-13,          71.835},
  {700.0, 3.614e-14,          88.667},
  {800.0, 1.170e-14,         124.640},
  {900.0, 5.245e-15,         181.050},
  {1000.0, 3.019e-15,        268.000}
};

constexpr int kNumAtmosphereLayers = static_cast<int>(sizeof(kAtmosphereLayers) / sizeof(kAtmosphereLayers[0]));

double wrap_degrees_360(double angle_deg) {
  double wrapped = std::fmod(angle_deg, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped;
}

Vec3 earth_rotation_eci_rad_s(const SimContext& ctx) {
  return {0.0, 0.0, ctx.earthRotationRate_rad_s};
}

double projected_box_area_m2(const Vec3& look_body_hat, const SimContext& ctx) {
  const double areaYZ_m2 = ctx.ly_m * ctx.lz_m;
  const double areaXZ_m2 = ctx.lx_m * ctx.lz_m;
  const double areaXY_m2 = ctx.lx_m * ctx.ly_m;

  return std::abs(look_body_hat.x) * areaYZ_m2 +
         std::abs(look_body_hat.y) * areaXZ_m2 +
         std::abs(look_body_hat.z) * areaXY_m2;
}

Vec3 sun_direction_eci(double t_s, const SimContext& ctx) {
  const double daysSinceJ2000 = 365.25 * (ctx.decimalYear0 - 2000.0) + t_s / 86400.0;
  const double meanLongitude_deg = wrap_degrees_360(280.460 + 0.9856474 * daysSinceJ2000);
  const double meanAnomaly_deg = wrap_degrees_360(357.528 + 0.9856003 * daysSinceJ2000);

  const double meanLongitude_rad = meanLongitude_deg * M_PI / 180.0;
  const double meanAnomaly_rad = meanAnomaly_deg * M_PI / 180.0;
  const double eclipticLongitude_rad =
    meanLongitude_rad +
    (1.915 * M_PI / 180.0) * std::sin(meanAnomaly_rad) +
    (0.020 * M_PI / 180.0) * std::sin(2.0 * meanAnomaly_rad);
  const double obliquity_rad = (23.439 - 0.0000004 * daysSinceJ2000) * M_PI / 180.0;

  const Vec3 sunVecEci{
    std::cos(eclipticLongitude_rad),
    std::cos(obliquity_rad) * std::sin(eclipticLongitude_rad),
    std::sin(obliquity_rad) * std::sin(eclipticLongitude_rad)
  };
  return normalize(sunVecEci);
}

bool in_cylindrical_eclipse(const Vec3& pos_eci_m, const Vec3& sunHat_eci, const SimContext& ctx) {
  const double alongSun_m = dot(pos_eci_m, sunHat_eci);
  if (alongSun_m >= 0.0) {
    return false;
  }

  const Vec3 perpendicular_m = pos_eci_m - alongSun_m * sunHat_eci;
  return norm(perpendicular_m) < ctx.earthRadius_m;
}

} // namespace

double density_kg_m3(double altitude_m) {
  const double altitude_km = altitude_m / 1000.0;

  if (altitude_km <= kAtmosphereLayers[0].baseAltitude_km) {
    return kAtmosphereLayers[0].baseDensity_kg_m3;
  }

  for (int i = 0; i < kNumAtmosphereLayers - 1; ++i) {
    const AtmosphereLayer& layer = kAtmosphereLayers[i];
    const AtmosphereLayer& nextLayer = kAtmosphereLayers[i + 1];
    if (altitude_km < nextLayer.baseAltitude_km) {
      return layer.baseDensity_kg_m3 *
             std::exp(-(altitude_km - layer.baseAltitude_km) / layer.scaleHeight_km);
    }
  }

  const AtmosphereLayer& topLayer = kAtmosphereLayers[kNumAtmosphereLayers - 1];
  return topLayer.baseDensity_kg_m3 *
         std::exp(-(altitude_km - topLayer.baseAltitude_km) / topLayer.scaleHeight_km);
}

void disturbance_summary(double t_s,
                         double altitude_m,
                         const Vec3& pos_eci_m,
                         const Vec3& vel_eci_m_s,
                         const Quat& q_ib,
                         const Vec3& B_body_T,
                         const SimContext& ctx,
                         DisturbanceBreakdown& summary) {
  summary = DisturbanceBreakdown{};

  summary.density_kg_m3 = density_kg_m3(altitude_m);
  const Mat3 C_IB = TIBquat(q_ib);

  // The upper atmosphere corotates with the Earth to first order, so drag depends
  // on velocity relative to that rotating frame rather than inertial speed.
  const Vec3 atmosphereVel_eci_m_s = cross(earth_rotation_eci_rad_s(ctx), pos_eci_m);
  const Vec3 relVel_eci_m_s = vel_eci_m_s - atmosphereVel_eci_m_s;
  summary.relativeSpeed_m_s = norm(relVel_eci_m_s);

  if (summary.relativeSpeed_m_s > 1e-9 && summary.density_kg_m3 > 0.0) {
    const Vec3 relVelHat_eci = relVel_eci_m_s / summary.relativeSpeed_m_s;
    const Vec3 relVelHat_body = mul(transpose(C_IB), relVelHat_eci);
    const double projectedArea_m2 = projected_box_area_m2(relVelHat_body, ctx);

    const double dragMag_N =
      0.5 * summary.density_kg_m3 *
      summary.relativeSpeed_m_s *
      summary.relativeSpeed_m_s *
      ctx.dragCoeff *
      projectedArea_m2;

    summary.dragForce_eci_N = -dragMag_N * relVelHat_eci;
    const Vec3 dragForce_body_N = mul(transpose(C_IB), summary.dragForce_eci_N);
    summary.aerodynamicTorque_body_Nm = cross(ctx.aeroCpOffset_body_m, dragForce_body_N);
  }

  const double radius_m = norm(pos_eci_m);
  if (radius_m > 1e-9) {
    const Vec3 rhat_eci = pos_eci_m / radius_m;
    const Vec3 rhat_body = mul(transpose(C_IB), rhat_eci);
    const Vec3 Irhat_body = mul(ctx.inertia_body_kgm2, rhat_body);

    const double ggGain = 3.0 * ctx.mu_m3_s2 / (radius_m * radius_m * radius_m);
    summary.gravityGradientTorque_body_Nm = ggGain * cross(rhat_body, Irhat_body);
  }

  // First-pass residual magnetic disturbance from public magnetorquer remanence
  // data: CR0002 rods are bounded at <0.1% FSR and the EXA MT01 sheet lists
  // <0.0045 A m^2 residual moment.
  summary.residualMagneticTorque_body_Nm = cross(ctx.residualDipole_body_Am2, B_body_T);

  const Vec3 sunHat_eci = sun_direction_eci(t_s, ctx);
  summary.sunlit = !in_cylindrical_eclipse(pos_eci_m, sunHat_eci, ctx);
  if (summary.sunlit) {
    const Vec3 sunHat_body = mul(transpose(C_IB), sunHat_eci);
    const double projectedArea_m2 = projected_box_area_m2(sunHat_body, ctx);
    const double srpForceMag_N =
      ctx.solarRadiationPressure_N_m2 * ctx.solarReflectivityCoeff * projectedArea_m2;

    // Force points away from the Sun because photons transfer momentum to the spacecraft.
    summary.solarRadiationForce_eci_N = srpForceMag_N * sunHat_eci;
    const Vec3 srpForce_body_N = mul(transpose(C_IB), summary.solarRadiationForce_eci_N);
    summary.solarRadiationTorque_body_Nm = cross(ctx.srpCpOffset_body_m, srpForce_body_N);
  }
}

void disturbance(double t_s,
                 double altitude_m,
                 const Vec3& pos_eci_m,
                 const Vec3& vel_eci_m_s,
                 const Quat& q_ib,
                 const Vec3& B_body_T,
                 const SimContext& ctx,
                 Vec3& disturbanceForce_eci_N,
                 Vec3& disturbanceTorque_body_Nm) {
  DisturbanceBreakdown summary{};
  disturbance_summary(t_s, altitude_m, pos_eci_m, vel_eci_m_s, q_ib, B_body_T, ctx, summary);
  disturbanceForce_eci_N = summary.totalForce_eci_N();
  disturbanceTorque_body_Nm = summary.totalTorque_body_Nm();
}
