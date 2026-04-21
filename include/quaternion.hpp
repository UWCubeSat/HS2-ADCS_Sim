#pragma once
#include "math.hpp"
#include <cmath>

// Scalar-first quaternion.
// q0 is the scalar term, q1/q2/q3 are the vector terms.
struct Quat {
  double q0{1.0};
  double q1{0.0};
  double q2{0.0};
  double q3{0.0};
};

// Keep a quaternion on the unit sphere.
inline Quat normalize(const Quat& q) {
  const double n = std::sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
  if (n == 0.0) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {q.q0 / n, q.q1 / n, q.q2 / n, q.q3 / n};
}

// Rotation matrix that maps a vector from body coordinates to inertial coordinates.
inline Mat3 TIBquat(const Quat& q_) {
  const Quat q = q_;
  const double q0 = q.q0;
  const double q1 = q.q1;
  const double q2 = q.q2;
  const double q3 = q.q3;

  const double q0s = q0 * q0;
  const double q1s = q1 * q1;
  const double q2s = q2 * q2;
  const double q3s = q3 * q3;

  Mat3 R{};
  R[0][0] = q0s + q1s - q2s - q3s;
  R[0][1] = 2.0 * (q1 * q2 - q0 * q3);
  R[0][2] = 2.0 * (q0 * q2 + q1 * q3);

  R[1][0] = 2.0 * (q1 * q2 + q0 * q3);
  R[1][1] = q0s - q1s + q2s - q3s;
  R[1][2] = 2.0 * (q2 * q3 - q0 * q1);

  R[2][0] = 2.0 * (q1 * q3 - q0 * q2);
  R[2][1] = 2.0 * (q0 * q1 + q2 * q3);
  R[2][2] = q0s - q1s - q2s + q3s;
  return R;
}

// Convert 3-2-1 Euler angles to a quaternion.
inline Quat euler321_to_quat(const Vec3& ptp) {
  const double phi   = ptp.x;
  const double theta = ptp.y;
  const double psi   = ptp.z;

  const double q0 = std::cos(phi / 2.0) * std::cos(theta / 2.0) * std::cos(psi / 2.0)
                  + std::sin(phi / 2.0) * std::sin(theta / 2.0) * std::sin(psi / 2.0);
  const double q1 = std::sin(phi / 2.0) * std::cos(theta / 2.0) * std::cos(psi / 2.0)
                  - std::cos(phi / 2.0) * std::sin(theta / 2.0) * std::sin(psi / 2.0);
  const double q2 = std::cos(phi / 2.0) * std::sin(theta / 2.0) * std::cos(psi / 2.0)
                  + std::sin(phi / 2.0) * std::cos(theta / 2.0) * std::sin(psi / 2.0);
  const double q3 = std::cos(phi / 2.0) * std::cos(theta / 2.0) * std::sin(psi / 2.0)
                  - std::sin(phi / 2.0) * std::sin(theta / 2.0) * std::cos(psi / 2.0);

  return {q0, q1, q2, q3};
}

// Convert a quaternion back to 3-2-1 Euler angles.
inline Vec3 quat_to_euler321(const Quat& q) {
  const double q0 = q.q0;
  const double q1 = q.q1;
  const double q2 = q.q2;
  const double q3 = q.q3;

  const double phi = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

  double arg = 2.0 * (q0 * q2 - q3 * q1);
  if (arg > 1.0) {
    arg = 1.0;
  }
  if (arg < -1.0) {
    arg = -1.0;
  }

  const double theta = std::asin(arg);
  const double psi = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
  return {phi, theta, psi};
}

// Standard rigid-body quaternion kinematics for body rates [p, q, r].
inline Quat quat_derivative(const Quat& q, const Vec3& omega_body) {
  const double p = omega_body.x;
  const double qv = omega_body.y;
  const double r = omega_body.z;

  Quat qdot{};
  qdot.q0 = -0.5 * ( p * q.q1 + qv * q.q2 + r * q.q3 );
  qdot.q1 =  0.5 * ( p * q.q0 + r * q.q2 - qv * q.q3 );
  qdot.q2 =  0.5 * ( qv * q.q0 - r * q.q1 + p * q.q3 );
  qdot.q3 =  0.5 * ( r * q.q0 + qv * q.q1 - p * q.q2 );
  return qdot;
}
