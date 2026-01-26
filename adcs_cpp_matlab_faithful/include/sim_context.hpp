#pragma once
#include "math.hpp"
#include "quaternion.hpp"
#include "rng.hpp"

// This mirrors MATLAB globals 1:1 (numerically faithful; not "clean").
struct SimContext {
  // Planet + satellite parameters
  double R{6.371e6};
  double M{5.972e24};
  double G{6.67e-11};
  double mu{0};

  // Geometry + mass
  double ms{2.6};
  double lx{0.10}, ly{0.10}, lz{0.20};
  double Amax{0};
  double lmax{0};
  double CD{1.0};
  double m{0}; // total

  // Inertia
  Mat3 Is{};
  Mat3 I{};
  Mat3 invI{};

  // Reaction wheel parameters
  double mr{0.13};
  double rr{42.0/1000.0};
  double hr{19.0/1000.0};
  double rpm{8000.0};
  double maxSpeed{0};
  double maxTorque{0.004};
  double maxAlpha{0};
  Mat3 IrR{};
  Mat3 Ir1B{}, Ir2B{}, Ir3B{};
  Mat3 Ir1Bcg{}, Ir2Bcg{}, Ir3Bcg{};
  Vec3 n1{0,1,0}, n2{0,0,0}, n3{1,0,0};
  Vec3 r1{4.0/1000.0,0,0}, r2{0,4.0/1000.0,0}, r3{0,0,4.0/1000.0};
  Mat3 J{}, Jinv{};

  // Magnetorquer parameters
  double n_turns{84};
  double A_turn{0.02};
  double maxCurrent_mA{120};
  Vec3 current{0,0,0};

  // Timing
  double nextMagUpdate{1.0};
  double lastMagUpdate{0.0};
  double nextSensorUpdate{1.0};
  double lastSensorUpdate{0.0};

  // Sensor globals
  double fsensor{1.0};
  double MagFieldBias{0}, AngFieldBias{0}, EulerBias{0};
  double MagFieldNoise{0}, AngFieldNoise{0}, EulerNoise{0};

  // Measured/Nav signals
  Vec3 BB_truth{0,0,0}; // Tesla in body frame
  Vec3 BfieldMeasured{0,0,0};
  Vec3 pqrMeasured{0,0,0};
  Vec3 ptpMeasured{0,0,0};
  Vec3 BfieldNav{0,0,0};
  Vec3 pqrNav{0,0,0};
  Vec3 ptpNav{0,0,0};
  Vec3 BfieldNavPrev{-99,0,0};
  Vec3 pqrNavPrev{0,0,0};
  Vec3 ptpNavPrev{0,0,0};
  Vec3 Bdot{0,0,0};

  // Control output
  Vec3 rwalphas{0,0,0};

  // RNG (for rand() equivalents)
  Rng rng;

  explicit SimContext(uint64_t seed=1) : rng(seed) {}
};
