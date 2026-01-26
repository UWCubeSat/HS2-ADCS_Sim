#pragma once
#include "sim_context.hpp"
#include <array>

// State is length-16 in fixed MATLAB order.
using State16 = std::array<double,16>;

State16 satellite_derivatives(double t, const State16& state, SimContext& ctx);
void sensor_update(Vec3& BB, Vec3& pqr, Vec3& ptp, SimContext& ctx);
void navigation_update(const Vec3& BfieldMeasured, const Vec3& pqrMeasured, const Vec3& ptpMeasured, SimContext& ctx);
void control_compute(const Vec3& BfieldNav, const Vec3& pqrNav, const Vec3& ptpNav, SimContext& ctx, Vec3& current_out, Vec3& rwalphas_out);
void disturbance(double altitude_m, double Amax, double lmax, const Vec3& vel, double CD, const Vec3& BI_Tesla,
                 Vec3& XYZD_out, Vec3& LMND_out);

double density_kg_m3(double altitude_m);
