#pragma once
#include "sim_context.hpp"
#include <array>

// State vector layout used everywhere in the sim.
// [x y z xdot ydot zdot q0 q1 q2 q3 p q r]
using State13 = std::array<double, 13>;

// ----- Setup -----
// Fill the context with constant model data before the first time step.
void initialize_simulation(SimContext& ctx);

// ----- Truth-state helpers -----
Quat state_quaternion(const State13& state);
Vec3 state_body_rates_rad_s(const State13& state);
Vec3 state_euler321_rad(const State13& state);

// ----- Environment and dynamics -----
// Compute the truth magnetic field in the body frame from the current truth state.
Vec3 truth_magnetic_field_body_T(double t_s, const State13& state, const SimContext& ctx);

// Pure orbit/attitude derivative. This function reads state, environment, and actuator command,
// and returns only the time derivative of the 13-state.
State13 satellite_derivatives(double t_s, const State13& state, const SimContext& ctx);

// ----- Sensor and navigation -----
// Take the truth state and truth magnetic field and create measured signals.
void sensor_update_from_truth(const State13& truth_state, const Vec3& Btruth_body_T, SimContext& ctx);

// Turn measured signals into smoothed nav signals.
void navigation_update(SimContext& ctx);

// ----- Controller -----
// Temporary magnetorquer-only detumble controller.
// This will be replaced when the magnetorquer team drops in the real control law.
void control_compute(const SimContext& ctx, Vec3& current_cmd_A);

// ----- Disturbance model -----
// Returns non-control force and torque in the current state.
void disturbance(double altitude_m,
                 const Vec3& vel_eci_m_s,
                 const Vec3& B_body_T,
                 const SimContext& ctx,
                 Vec3& disturbanceForce_eci_N,
                 Vec3& disturbanceTorque_body_Nm);

double density_kg_m3(double altitude_m);
