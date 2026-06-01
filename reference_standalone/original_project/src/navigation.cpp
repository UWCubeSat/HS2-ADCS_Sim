#include "modules.hpp"

namespace {

Quat integrate_quat_forward_euler(const Quat& q, const Vec3& omega_body_rad_s, double dt_s) {
  const Quat qdot = quat_derivative(q, omega_body_rad_s);
  return normalize({
    q.q0 + dt_s * qdot.q0,
    q.q1 + dt_s * qdot.q1,
    q.q2 + dt_s * qdot.q2,
    q.q3 + dt_s * qdot.q3
  });
}

Quat quat_from_two_unit_vectors(const Vec3& from_unit, const Vec3& to_unit) {
  const double dotv = dot(from_unit, to_unit);
  const Vec3 axis = cross(from_unit, to_unit);
  const double w = 1.0 + dotv;

  if (w < 1e-9) {
    const Vec3 helper = (std::abs(from_unit.x) < 0.9) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    const Vec3 ortho = normalize(cross(from_unit, helper));
    return {0.0, ortho.x, ortho.y, ortho.z};
  }

  return normalize({w, axis.x, axis.y, axis.z});
}

Vec3 predicted_body_field_T(const Quat& q_ib, const Vec3& referenceField_eci_T) {
  return mul(transpose(TIBquat(q_ib)), referenceField_eci_T);
}

} // namespace

// This is a first realistic estimator scaffold.
// It propagates attitude with measured body rates and uses the expected magnetic
// field direction as a lightweight correction source.
void navigation_update(SimContext& ctx) {
  const Vec3 Bmeas_body_T = ctx.BfieldMeasured_body_T;
  const Vec3 Bref_eci_T = ctx.BfieldReference_eci_T;
  const double BmeasNorm = norm(Bmeas_body_T);
  const double BrefNorm = norm(Bref_eci_T);
  const bool magneticReferenceValid = (BmeasNorm > 1e-12) && (BrefNorm > 1e-12);

  if (!ctx.navInitialized) {
    if (magneticReferenceValid) {
      ctx.qNav_IB = quat_from_two_unit_vectors(Bmeas_body_T / BmeasNorm, Bref_eci_T / BrefNorm);
    } else {
      ctx.qNav_IB = {1.0, 0.0, 0.0, 0.0};
    }

    ctx.BfieldNav_T = Bmeas_body_T;
    ctx.pqrNav_rad_s = ctx.pqrMeasured_rad_s;
    ctx.ptpNav_rad = quat_to_euler321(ctx.qNav_IB);

    ctx.BfieldNavPrev_T = ctx.BfieldNav_T;
    ctx.pqrNavPrev_rad_s = ctx.pqrNav_rad_s;
    ctx.ptpNavPrev_rad = ctx.ptpNav_rad;
    ctx.BdotNav_T_s = {0.0, 0.0, 0.0};
    ctx.navInitialized = true;
    return;
  }

  const double dt_s = ctx.sensorPeriod_s;
  const double s = ctx.navBlend;
  const Vec3 gyroUnbiased_rad_s = ctx.pqrMeasured_rad_s - ctx.gyroBiasNav_rad_s;

  Vec3 correction_body_rad_s{0.0, 0.0, 0.0};
  if (magneticReferenceValid) {
    const Vec3 Bpred_body_T = predicted_body_field_T(ctx.qNav_IB, Bref_eci_T);
    correction_body_rad_s = cross(normalize(Bmeas_body_T), normalize(Bpred_body_T));
  }

  const Vec3 omegaCorrected_rad_s = gyroUnbiased_rad_s + ctx.navAttitudeCorrectionGain * correction_body_rad_s;
  ctx.qNav_IB = integrate_quat_forward_euler(ctx.qNav_IB, omegaCorrected_rad_s, dt_s);

  if (magneticReferenceValid) {
    ctx.gyroBiasNav_rad_s = ctx.gyroBiasNav_rad_s - (ctx.navBiasCorrectionGain * dt_s) * correction_body_rad_s;
  }

  ctx.BfieldNav_T = (1.0 - s) * ctx.BfieldNavPrev_T + s * Bmeas_body_T;
  ctx.pqrNav_rad_s = omegaCorrected_rad_s;
  ctx.ptpNav_rad = quat_to_euler321(ctx.qNav_IB);

  ctx.BdotNav_T_s = (ctx.BfieldNav_T - ctx.BfieldNavPrev_T) / ctx.sensorPeriod_s;

  ctx.BfieldNavPrev_T = ctx.BfieldNav_T;
  ctx.pqrNavPrev_rad_s = ctx.pqrNav_rad_s;
  ctx.ptpNavPrev_rad = ctx.ptpNav_rad;
}
