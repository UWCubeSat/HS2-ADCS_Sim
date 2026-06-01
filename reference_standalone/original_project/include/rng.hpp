#pragma once
#include <random>

// Small wrapper around the C++ random generator.
// The default fixed seed makes runs repeatable, which is handy for debugging.
struct Rng {
  std::mt19937_64 gen;
  std::uniform_real_distribution<double> unif;

  explicit Rng(uint64_t seed = 1) : gen(seed), unif(0.0, 1.0) {}

  double rand01() {
    return unif(gen);
  }

  // Match the common MATLAB pattern (2*rand()-1).
  double rand_m11() {
    return 2.0 * rand01() - 1.0;
  }
};
