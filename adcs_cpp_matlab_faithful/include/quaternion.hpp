#pragma once
#include "math.hpp"
#include <cmath>

struct Quat {
  // scalar-first: q = [q0, q1, q2, q3]
  double q0{1}, q1{0}, q2{0}, q3{0};
};

inline Quat normalize(const Quat& q){
  const double n = std::sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
  if(n==0) return {1,0,0,0};
  return {q.q0/n, q.q1/n, q.q2/n, q.q3/n};
}

// MATLAB TIBquat: v_inertial = TIB * v_body
inline Mat3 TIBquat(const Quat& q_){
  const Quat q = q_;
  const double q0=q.q0, q1=q.q1, q2=q.q2, q3=q.q3;
  const double q0s=q0*q0, q1s=q1*q1, q2s=q2*q2, q3s=q3*q3;

  Mat3 R{};
  R[0][0] = (q0s+q1s-q2s-q3s);
  R[0][1] = 2*(q1*q2 - q0*q3);
  R[0][2] = 2*(q0*q2 + q1*q3);

  R[1][0] = 2*(q1*q2 + q0*q3);
  R[1][1] = (q0s-q1s+q2s-q3s);
  R[1][2] = 2*(q2*q3 - q0*q1);

  R[2][0] = 2*(q1*q3 - q0*q2);
  R[2][1] = 2*(q0*q1 + q2*q3);
  R[2][2] = (q0s-q1s-q2s+q3s);
  return R;
}

// Euler 3-2-1 (phi,theta,psi) to quaternion, MATLAB EulerAngles2Quaternions
inline Quat euler321_to_quat(const Vec3& ptp){
  const double phi=ptp.x, theta=ptp.y, psi=ptp.z;
  const double q0 = std::cos(phi/2)*std::cos(theta/2)*std::cos(psi/2) + std::sin(phi/2)*std::sin(theta/2)*std::sin(psi/2);
  const double q1 = std::sin(phi/2)*std::cos(theta/2)*std::cos(psi/2) - std::cos(phi/2)*std::sin(theta/2)*std::sin(psi/2);
  const double q2 = std::cos(phi/2)*std::sin(theta/2)*std::cos(psi/2) + std::sin(phi/2)*std::cos(theta/2)*std::sin(psi/2);
  const double q3 = std::cos(phi/2)*std::cos(theta/2)*std::sin(psi/2) - std::sin(phi/2)*std::sin(theta/2)*std::cos(psi/2);
  return {q0,q1,q2,q3};
}

// MATLAB Quaternions2EulerAngles: output is phi,theta,psi
inline Vec3 quat_to_euler321(const Quat& q){
  const double q0=q.q0, q1=q.q1, q2=q.q2, q3=q.q3;
  const double phi = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
  double arg = 2*(q0*q2 - q3*q1);
  if(arg >  1) arg =  1;
  if(arg < -1) arg = -1;
  const double theta = std::asin(arg);
  const double psi = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  return {phi,theta,psi};
}
