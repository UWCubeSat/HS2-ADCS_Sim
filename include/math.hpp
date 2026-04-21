#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

// Simple 3D vector type used everywhere in the sim.
// This is intentionally tiny so the math stays easy to follow.
struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  // Allow v[0], v[1], v[2] access for loops.
  double& operator[](size_t i) { return (i == 0) ? x : (i == 1) ? y : z; }
  double  operator[](size_t i) const { return (i == 0) ? x : (i == 1) ? y : z; }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(double s, const Vec3& v) { return {s * v.x, s * v.y, s * v.z}; }
inline Vec3 operator*(const Vec3& v, double s) { return s * v; }
inline Vec3 operator/(const Vec3& v, double s) { return {v.x / s, v.y / s, v.z / s}; }

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

inline double norm(const Vec3& v) {
  return std::sqrt(dot(v, v));
}

inline Vec3 normalize(const Vec3& v) {
  const double n = norm(v);
  if (n == 0.0) {
    return {0.0, 0.0, 0.0};
  }
  return v / n;
}

// 3x3 matrix stored row-by-row.
struct Mat3 {
  double m[3][3]{};
  double* operator[](size_t r) { return m[r]; }
  const double* operator[](size_t r) const { return m[r]; }
};

inline Mat3 eye3() {
  Mat3 I{};
  I[0][0] = 1.0;
  I[1][1] = 1.0;
  I[2][2] = 1.0;
  return I;
}

inline Mat3 transpose(const Mat3& A) {
  Mat3 T{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T[r][c] = A[c][r];
    }
  }
  return T;
}

inline Vec3 mul(const Mat3& A, const Vec3& v) {
  return {
    A[0][0] * v.x + A[0][1] * v.y + A[0][2] * v.z,
    A[1][0] * v.x + A[1][1] * v.y + A[1][2] * v.z,
    A[2][0] * v.x + A[2][1] * v.y + A[2][2] * v.z
  };
}

inline Mat3 mul(const Mat3& A, const Mat3& B) {
  Mat3 C{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += A[r][k] * B[k][c];
      }
      C[r][c] = s;
    }
  }
  return C;
}

inline Mat3 add(const Mat3& A, const Mat3& B) {
  Mat3 C{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      C[r][c] = A[r][c] + B[r][c];
    }
  }
  return C;
}

inline Mat3 scale(const Mat3& A, double s) {
  Mat3 C{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      C[r][c] = A[r][c] * s;
    }
  }
  return C;
}

// Skew-symmetric matrix used for cross-product style formulas.
inline Mat3 skew(const Vec3& v) {
  Mat3 S{};
  S[0][1] = -v.z;
  S[0][2] =  v.y;
  S[1][0] =  v.z;
  S[1][2] = -v.x;
  S[2][0] = -v.y;
  S[2][1] =  v.x;
  return S;
}

// Plain 3x3 inverse. This assumes the matrix really is invertible.
inline Mat3 inv3(const Mat3& A) {
  const double a = A[0][0], b = A[0][1], c = A[0][2];
  const double d = A[1][0], e = A[1][1], f = A[1][2];
  const double g = A[2][0], h = A[2][1], i = A[2][2];

  const double A11 =  (e * i - f * h);
  const double A12 = -(d * i - f * g);
  const double A13 =  (d * h - e * g);
  const double A21 = -(b * i - c * h);
  const double A22 =  (a * i - c * g);
  const double A23 = -(a * h - b * g);
  const double A31 =  (b * f - c * e);
  const double A32 = -(a * f - c * d);
  const double A33 =  (a * e - b * d);

  const double det = a * A11 + b * A12 + c * A13;
  if (std::abs(det) < 1e-30) {
    throw std::runtime_error("inv3: singular matrix");
  }

  const double invdet = 1.0 / det;
  Mat3 inv{};
  inv[0][0] = A11 * invdet; inv[0][1] = A21 * invdet; inv[0][2] = A31 * invdet;
  inv[1][0] = A12 * invdet; inv[1][1] = A22 * invdet; inv[1][2] = A32 * invdet;
  inv[2][0] = A13 * invdet; inv[2][1] = A23 * invdet; inv[2][2] = A33 * invdet;
  return inv;
}
