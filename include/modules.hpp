#pragma once
#include "sim_context.hpp"
#include <array>

// State vector layout used everywhere in the sim.
// [x y z xdot ydot zdot q0 q1 q2 q3 p q r]
using State13 = std::array<double, 13>;

// Core rigid-body/orbit dynamics.
State13 satellite_derivatives(double t, const State13& state, SimContext& ctx);

// Sensor and navigation pieces.
void sensor_update(Vec3& BB, Vec3& pqr, Vec3& ptp, SimContext& ctx);
void navigation_update(const Vec3& BfieldMeasured, const Vec3& pqrMeasured, const Vec3& ptpMeasured, SimContext& ctx);

// Magnetorquer controller.
void control_compute(const Vec3& BfieldNav, const Vec3& pqrNav, const Vec3& ptpNav,
                     SimContext& ctx, Vec3& current_out);

// Disturbance model.
void disturbance(double altitude_m, double Amax, double lmax, const Vec3& vel, double CD, const Vec3& BI_Tesla,
                 Vec3& XYZD_out, Vec3& LMND_out);

double density_kg_m3(double altitude_m);
