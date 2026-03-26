#pragma once
#include "math.hpp"
#include "quaternion.hpp"
#include "rng.hpp"

// All long-lived simulation state that needs to be shared between modules lives here.
// It is not the prettiest pattern, but it keeps this small codebase easy to move around.
struct SimContext {
  // Earth constants used by the orbit model.
  double R{6.371e6};
  double M{5.972e24};
  double G{6.67e-11};
  double mu{0.0};

  // Earth rotation used for the ECI <-> ECEF conversion.
  double omegaE{7.2921150e-5};
  double greenwichAngle0{0.0};

  // Start date for the geomagnetic model.
  // One orbit is only a few hours, so the decimal year barely changes, but it is still handled correctly.
  double decimalYear0{2026.0};

  // Spacecraft geometry and mass.
  double ms{2.6};
  double lx{0.10};
  double ly{0.10};
  double lz{0.20};
  double Amax{0.0};
  double lmax{0.0};
  double CD{1.0};
  double m{0.0};

  // Spacecraft inertia.
  Mat3 Is{};
  Mat3 I{};
  Mat3 invI{};

  // Magnetorquer settings.
  double n_turns{84.0};
  double A_turn{0.02};
  double maxCurrent_mA{120.0};
  Vec3 current{0.0, 0.0, 0.0};

  // Update timing for field, sensor, and nav logic.
  double nextMagUpdate{1.0};
  double lastMagUpdate{0.0};
  double nextSensorUpdate{1.0};
  double lastSensorUpdate{0.0};

  // Sensor model settings and one-time bias values.
  bool sensorModelInitialized{false};
  double fsensor{1.0};
  double MagFieldBias{0.0};
  double AngFieldBias{0.0};
  double EulerBias{0.0};
  double MagFieldNoise{0.0};
  double AngFieldNoise{0.0};
  double EulerNoise{0.0};

  // Navigation filter state.
  bool navInitialized{false};
  Vec3 BfieldNav{0.0, 0.0, 0.0};
  Vec3 BfieldNavPrev{0.0, 0.0, 0.0};
  Vec3 pqrNav{0.0, 0.0, 0.0};
  Vec3 pqrNavPrev{0.0, 0.0, 0.0};
  Vec3 ptpNav{0.0, 0.0, 0.0};
  Vec3 ptpNavPrev{0.0, 0.0, 0.0};
  Vec3 Bdot{0.0, 0.0, 0.0};

  // Truth and measured signals.
  Vec3 BB_truth{0.0, 0.0, 0.0};
  Vec3 BfieldMeasured{0.0, 0.0, 0.0};
  Vec3 pqrMeasured{0.0, 0.0, 0.0};
  Vec3 ptpMeasured{0.0, 0.0, 0.0};

  // Random number generator used by the noise model.
  Rng rng;

  explicit SimContext(uint64_t seed = 1) : rng(seed) {}
};
