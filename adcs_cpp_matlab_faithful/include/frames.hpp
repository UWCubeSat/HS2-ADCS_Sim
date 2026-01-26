#pragma once
#include "math.hpp"
#include <cmath>

// MATLAB TIB(phi,theta,psi)
inline Mat3 TIB(double phi, double theta, double psi){
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double sp = std::sin(phi);
  const double cp = std::cos(phi);
  const double ss = std::sin(psi);
  const double cs = std::cos(psi);

  Mat3 out{};
  out[0][0]=ct*cs;                 out[0][1]=sp*st*cs-cp*ss;      out[0][2]=cp*st*cs+sp*ss;
  out[1][0]=ct*ss;                 out[1][1]=sp*st*ss+cp*cs;      out[1][2]=cp*st*ss-sp*cs;
  out[2][0]=-st;                   out[2][1]=sp*ct;               out[2][2]=cp*ct;
  return out;
}

// MATLAB Rscrew(nhat)
inline Mat3 Rscrew(const Vec3& nhat){
  const double x=nhat.x, y=nhat.y, z=nhat.z;
  const double ps = std::atan2(y,x);
  const double theta = std::atan2(z, std::sqrt(x*x+y*y));
  const double phi = 0.0;
  return transpose(TIB(phi, theta, ps));
}
