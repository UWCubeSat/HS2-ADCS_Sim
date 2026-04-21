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
  double solarRadiationPressure_N_m2{4.56e-6};
  double solarReflectivityCoeff{1.3};
  Vec3 residualDipole_body_Am2{0.0, 0.0, 0.0};
  Vec3 aeroCpOffset_body_m{0.0, 0.0, 0.0};
  Vec3 srpCpOffset_body_m{0.0, 0.0, 0.0};

  // Principal inertia model for the simple box spacecraft.
  Mat3 inertia_body_kgm2{};
  Mat3 inertiaInv_body_kgm2{};

  // ----- Actuator model -----
  // HuskySat-2 hybrid magnetorquer setup used by the current realism pass:
  // X/Y are CubeSpace CR0002 rods and Z is an EXA MT01 on the long axis.
  Vec3 mtqDipoleGain_Am2_A{0.0, 0.0, 0.0};
  Vec3 mtqResistance_Ohm{0.0, 0.0, 0.0};
  Vec3 mtqCurrentLimit_A{0.0, 0.0, 0.0};
  Vec3 mtqDipoleLimit_Am2{0.0, 0.0, 0.0};
  Vec3 commandedCurrent_A{0.0, 0.0, 0.0};

  // ----- Sensor model -----
  // Sensor sample period. Truth is continuous; measurements are discrete.
  double sensorPeriod_s{1.0};
  bool sensorModelInitialized{false};

  // One fixed bias per axis plus fresh white noise each sample.
  Vec3 magBias_T{0.0, 0.0, 0.0};
  Vec3 gyroBias_rad_s{0.0, 0.0, 0.0};

  Vec3 magNoise_T{0.0, 0.0, 0.0};
  Vec3 gyroNoise_rad_s{0.0, 0.0, 0.0};

  // ----- Navigation filter -----
  // This is a first estimator scaffold, not a flight-ready navigation solution.
  // It propagates attitude with gyro measurements and uses the expected magnetic
  // field direction for a lightweight correction step.
  bool navInitialized{false};
  double navBlend{0.3};
  double navAttitudeCorrectionGain{0.8};
  double navBiasCorrectionGain{0.02};

  Vec3 BfieldNav_T{0.0, 0.0, 0.0};
  Vec3 BfieldNavPrev_T{0.0, 0.0, 0.0};
  Vec3 pqrNav_rad_s{0.0, 0.0, 0.0};
  Vec3 pqrNavPrev_rad_s{0.0, 0.0, 0.0};
  Vec3 ptpNav_rad{0.0, 0.0, 0.0};
  Vec3 ptpNavPrev_rad{0.0, 0.0, 0.0};
  Vec3 BdotNav_T_s{0.0, 0.0, 0.0};
  Quat qNav_IB{};
  Vec3 gyroBiasNav_rad_s{0.0, 0.0, 0.0};

  // ----- Latched truth and measured signals -----
  Vec3 BfieldReference_eci_T{0.0, 0.0, 0.0};
  Vec3 BfieldTruth_body_T{0.0, 0.0, 0.0};
  Vec3 BfieldMeasured_body_T{0.0, 0.0, 0.0};
  Vec3 pqrMeasured_rad_s{0.0, 0.0, 0.0};

  // Random number generator used by the sensor noise model.
  Rng rng;

  explicit SimContext(uint64_t seed = 1) : rng(seed) {}
};
