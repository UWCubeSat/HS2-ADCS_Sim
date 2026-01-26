#pragma once
#include <random>

// MATLAB uses rand() with implicit seed unless rng set.
// For deterministic C++ regression, we fix a seed by default.
// Change seed in main if you want MATLAB-like run-to-run randomness.
struct Rng {
  std::mt19937_64 gen;
  std::uniform_real_distribution<double> unif; // [0,1)
  explicit Rng(uint64_t seed=1) : gen(seed), unif(0.0, 1.0) {}
  double rand01(){ return unif(gen); }
  // MATLAB pattern: (2*rand()-1)
  double rand_m11(){ return 2.0*rand01() - 1.0; }
};
