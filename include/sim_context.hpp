#pragma once
#include "math.hpp"
#include "quaternion.hpp"
#include "rng.hpp"

// This struct holds the long-lived simulation configuration and latched signals.
// The goal is not to mimic flight software structure exactly.
// The goal is to keep one clear place for constants, truth values, measurements,
// nav estimates, and actuator commands so the rest of the code is easy to follow.
struct SimContext {
  // ----- Planet model -----
  // Earth constants used by the orbit and magnetic field models.
  double earthRadius_m{6.371e6};
  double earthMass_kg{5.972e24};
  double gravConst_SI{6.67e-11};
  double mu_m3_s2{0.0};

  // Earth rotation for ECI <-> ECEF conversion.
  double earthRotationRate_rad_s{7.2921150e-5};
  double greenwichAngle0_rad{0.0};

  // Start date for WMM. One orbit is only a few hours, so the decimal year hardly changes,
  // but the code still carries time correctly.
  double decimalYear0{2026.0};

  // ----- Spacecraft geometry and mass -----
  double mass_kg{2.6};
  double lx_m{0.10};
  double ly_m{0.10};
  double lz_m{0.20};
  double maxArea_m2{0.0};
  double maxMomentArm_m{0.0};
  double dragCoeff{1.0};

  // Principal inertia model for the simple box spacecraft.
  Mat3 inertia_body_kgm2{};
  Mat3 inertiaInv_body_kgm2{};

  // ----- Actuator model -----
  // Magnetorquer constants. The placeholder controller commands coil current in amps.
  double coilTurns{84.0};
  double coilArea_m2{0.02};
  double maxCurrent_A{0.120};
  Vec3 commandedCurrent_A{0.0, 0.0, 0.0};

  // ----- Sensor model -----
  // Sensor sample period. Truth is continuous; measurements are discrete.
  double sensorPeriod_s{1.0};
  bool sensorModelInitialized{false};

  // One fixed bias per axis plus fresh white noise each sample.
  Vec3 magBias_T{0.0, 0.0, 0.0};
  Vec3 gyroBias_rad_s{0.0, 0.0, 0.0};
  Vec3 angleBias_rad{0.0, 0.0, 0.0};

  Vec3 magNoise_T{0.0, 0.0, 0.0};
  Vec3 gyroNoise_rad_s{0.0, 0.0, 0.0};
  Vec3 angleNoise_rad{0.0, 0.0, 0.0};

  // ----- Navigation filter -----
  // This is only a smoothing filter for now. It is not a flight estimator.
  bool navInitialized{false};
  double navBlend{0.3};

  Vec3 BfieldNav_T{0.0, 0.0, 0.0};
  Vec3 BfieldNavPrev_T{0.0, 0.0, 0.0};
  Vec3 pqrNav_rad_s{0.0, 0.0, 0.0};
  Vec3 pqrNavPrev_rad_s{0.0, 0.0, 0.0};
  Vec3 ptpNav_rad{0.0, 0.0, 0.0};
  Vec3 ptpNavPrev_rad{0.0, 0.0, 0.0};
  Vec3 BdotNav_T_s{0.0, 0.0, 0.0};

  // ----- Latched truth and measured signals -----
  Vec3 BfieldTruth_body_T{0.0, 0.0, 0.0};
  Vec3 BfieldMeasured_body_T{0.0, 0.0, 0.0};
  Vec3 pqrMeasured_rad_s{0.0, 0.0, 0.0};
  Vec3 ptpMeasured_rad{0.0, 0.0, 0.0};

  // Random number generator used by the sensor noise model.
  Rng rng;

  explicit SimContext(uint64_t seed = 1) : rng(seed) {}
};
