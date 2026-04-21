#pragma once
#include "sim_context.hpp"
#include <array>

// State vector layout used everywhere in the sim.
// [x y z xdot ydot zdot q0 q1 q2 q3 p q r]
using State13 = std::array<double, 13>;

struct DisturbanceBreakdown {
  double density_kg_m3{0.0};
  double relativeSpeed_m_s{0.0};
  bool sunlit{true};

  Vec3 dragForce_eci_N{0.0, 0.0, 0.0};
  Vec3 solarRadiationForce_eci_N{0.0, 0.0, 0.0};

  Vec3 aerodynamicTorque_body_Nm{0.0, 0.0, 0.0};
  Vec3 gravityGradientTorque_body_Nm{0.0, 0.0, 0.0};
  Vec3 residualMagneticTorque_body_Nm{0.0, 0.0, 0.0};
  Vec3 solarRadiationTorque_body_Nm{0.0, 0.0, 0.0};

  Vec3 totalForce_eci_N() const {
    return dragForce_eci_N + solarRadiationForce_eci_N;
  }

  Vec3 totalTorque_body_Nm() const {
    return aerodynamicTorque_body_Nm +
           gravityGradientTorque_body_Nm +
           residualMagneticTorque_body_Nm +
           solarRadiationTorque_body_Nm;
  }
};

// ----- Setup -----
// Fill the context with constant model data before the first time step.
void initialize_simulation(SimContext& ctx);

// ----- Truth-state helpers -----
Quat state_quaternion(const State13& state);
Vec3 state_body_rates_rad_s(const State13& state);
Vec3 state_euler321_rad(const State13& state);

// ----- Environment and dynamics -----
// Compute the truth magnetic field in the inertial frame from the current truth state.
Vec3 truth_magnetic_field_eci_T(double t_s, const State13& state, const SimContext& ctx);

// Compute the truth magnetic field in the body frame from the current truth state.
Vec3 truth_magnetic_field_body_T(double t_s, const State13& state, const SimContext& ctx);

// Pure orbit/attitude derivative. This function reads state, environment, and actuator command,
// and returns only the time derivative of the 13-state.
State13 satellite_derivatives(double t_s, const State13& state, const SimContext& ctx);

// ----- Sensor and navigation -----
// Take the truth state and truth magnetic field and create measured signals.
void sensor_update_from_truth(const State13& truth_state, const Vec3& Btruth_body_T, SimContext& ctx);

// Turn measured signals into the current navigation estimate.
void navigation_update(SimContext& ctx);

// ----- Controller -----
// Temporary magnetorquer-only detumble controller.
// This will be replaced when the magnetorquer team drops in the real control law.
void control_compute(const SimContext& ctx, Vec3& current_cmd_A);

// ----- Actuator helpers -----
// Convert the held current command into the magnetic dipole and coil power used this step.
Vec3 magnetorquer_dipole_body_Am2(const SimContext& ctx);
Vec3 magnetorquer_coil_power_W(const SimContext& ctx);
double magnetorquer_total_coil_power_W(const SimContext& ctx);

// ----- Disturbance model -----
// Returns the named disturbance contributors at the current truth state.
void disturbance_summary(double t_s,
                         double altitude_m,
                         const Vec3& pos_eci_m,
                         const Vec3& vel_eci_m_s,
                         const Quat& q_ib,
                         const Vec3& B_body_T,
                         const SimContext& ctx,
                         DisturbanceBreakdown& summary);

// Returns the total non-control force and torque in the current state.
void disturbance(double t_s,
                 double altitude_m,
                 const Vec3& pos_eci_m,
                 const Vec3& vel_eci_m_s,
                 const Quat& q_ib,
                 const Vec3& B_body_T,
                 const SimContext& ctx,
                 Vec3& disturbanceForce_eci_N,
                 Vec3& disturbanceTorque_body_Nm);

double density_kg_m3(double altitude_m);
