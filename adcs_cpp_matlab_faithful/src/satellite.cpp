#include "modules.hpp"
#include "frames.hpp"
#include "igrf.hpp"
#include <cmath>

static void planet(SimContext& ctx){
  ctx.R = 6.371e6;
  ctx.M = 5.972e24;
  ctx.G = 6.67e-11;
  ctx.mu = ctx.G * ctx.M;
}

static void reaction_wheel_params(SimContext& ctx){
  ctx.mr = 0.13;
  ctx.rr = 42.0/1000.0;
  ctx.hr = 19.0/1000.0;

  ctx.rpm = 8000.0;
  ctx.maxSpeed = ctx.rpm * 2.0*M_PI/60.0;

  ctx.maxTorque = 0.004;

  // Orientation vectors
  ctx.n3 = {1,0,0};
  ctx.n1 = {0,1,0};
  Vec3 n2tmp{1,1,1};
  ctx.n2 = normalize(n2tmp);

  // offsets
  ctx.r1 = {4.0/1000.0,0,0};
  ctx.r2 = {0,4.0/1000.0,0};
  ctx.r3 = {0,0,4.0/1000.0};

  // Inertia of disk
  const double Idisk = (1.0/12.0)*(3*ctx.rr*ctx.rr + ctx.hr*ctx.hr);
  Mat3 IrR{};
  IrR[0][0] = ctx.mr * (0.5*ctx.rr*ctx.rr);
  IrR[1][1] = ctx.mr * Idisk;
  IrR[2][2] = ctx.mr * Idisk;
  ctx.IrR = IrR;

  ctx.maxAlpha = ctx.maxTorque / ctx.IrR[0][0];

  // Transformations
  const Mat3 T1 = Rscrew(ctx.n1);
  const Mat3 T2 = Rscrew(ctx.n2);
  const Mat3 T3 = Rscrew(ctx.n3);

  ctx.Ir1B = mul(transpose(T1), mul(ctx.IrR, T1));
  ctx.Ir2B = mul(transpose(T2), mul(ctx.IrR, T2));
  ctx.Ir3B = mul(transpose(T3), mul(ctx.IrR, T3));

  // J = [Ir1B*n1, Ir2B*n2, Ir3B*n3]
  const Vec3 c1 = mul(ctx.Ir1B, ctx.n1);
  const Vec3 c2 = mul(ctx.Ir2B, ctx.n2);
  const Vec3 c3 = mul(ctx.Ir3B, ctx.n3);
  Mat3 J{};
  J[0][0]=c1.x; J[1][0]=c1.y; J[2][0]=c1.z;
  J[0][1]=c2.x; J[1][1]=c2.y; J[2][1]=c2.z;
  J[0][2]=c3.x; J[1][2]=c3.y; J[2][2]=c3.z;
  ctx.J = J;

  // Jinv = J' * inv(J*J')
  const Mat3 JJt = mul(J, transpose(J));
  const Mat3 invJJt = inv3(JJt);
  ctx.Jinv = mul(transpose(J), invJJt);

  // Parallel axis theorem
  const Mat3 sr1 = skew(ctx.r1);
  const Mat3 sr2 = skew(ctx.r2);
  const Mat3 sr3 = skew(ctx.r3);
  ctx.Ir1Bcg = add(ctx.Ir1B, scale(mul(transpose(sr1), sr1), ctx.mr));
  ctx.Ir2Bcg = add(ctx.Ir2B, scale(mul(transpose(sr2), sr2), ctx.mr));
  ctx.Ir3Bcg = add(ctx.Ir3B, scale(mul(transpose(sr3), sr3), ctx.mr));
}

static void inertia(SimContext& ctx){
  ctx.ms = 2.6;
  ctx.lx = 10.0/100.0;
  ctx.ly = 10.0/100.0;
  ctx.lz = 20.0/100.0;

  // Amax is product of two largest dims
  const double l[3]{ctx.lx, ctx.ly, ctx.lz};
  double a=l[0], b=l[1], c=l[2];
  // sort
  double s1=a, s2=b, s3=c;
  if(s1>s2) std::swap(s1,s2);
  if(s2>s3) std::swap(s2,s3);
  if(s1>s2) std::swap(s1,s2);
  ctx.Amax = s2*s3;
  ctx.lmax = ctx.lz;
  ctx.CD = 1.0;

  Mat3 Is{};
  Is[0][0] = (ctx.ms/12.0) * (ctx.ly*ctx.ly + ctx.lz*ctx.lz);
  Is[1][1] = (ctx.ms/12.0) * (ctx.lx*ctx.lx + ctx.lz*ctx.lz);
  Is[2][2] = (ctx.ms/12.0) * (ctx.lx*ctx.lx + ctx.ly*ctx.ly);
  ctx.Is = Is;

  reaction_wheel_params(ctx);

  ctx.m = ctx.ms + 3.0*ctx.mr;
  ctx.I = add(ctx.Is, add(ctx.Ir1Bcg, add(ctx.Ir2Bcg, ctx.Ir3Bcg)));
  ctx.invI = inv3(ctx.I);
}

static void magtorquer_params(SimContext& ctx){
  ctx.n_turns = 84;
  ctx.A_turn = 0.02;
  ctx.maxCurrent_mA = 120;
}

static Quat state_to_quat(const State16& s){
  return {s[6], s[7], s[8], s[9]};
}

static void quat_to_state(const Quat& q, State16& s){
  s[6]=q.q0; s[7]=q.q1; s[8]=q.q2; s[9]=q.q3;
}

State16 satellite_derivatives(double t, const State16& state, SimContext& ctx){
  // Ensure parameters are initialized once
  if(ctx.mu == 0){
    planet(ctx);
    inertia(ctx);
  }

  const Vec3 r{state[0], state[1], state[2]};
  const Vec3 vel{state[3], state[4], state[5]};
  Quat q = state_to_quat(state);
  const Vec3 pqr{state[10], state[11], state[12]};
  const Vec3 w123{state[13], state[14], state[15]};

  // ptp from quaternion (truth)
  const Vec3 ptp = quat_to_euler321(q);

  // Quaternion kinematics: qdot = 0.5*PQRMAT*q
  const double p=pqr.x, qq=pqr.y, rr=pqr.z;
  const double q0=q.q0, q1=q.q1, q2=q.q2, q3=q.q3;
  Quat qdot{};
  qdot.q0 = 0.5*( 0    *q0 + (-p)*q1 + (-qq)*q2 + (-rr)*q3);
  qdot.q1 = 0.5*( p    *q0 +  0  *q1 + ( rr)*q2 + (-qq)*q3);
  qdot.q2 = 0.5*( qq   *q0 + (-rr)*q1 + 0   *q2 + ( p )*q3);
  qdot.q3 = 0.5*( rr   *q0 + ( qq)*q1 + (-p)*q2 + 0   *q3);

  // Gravity
  const double rho = norm(r);
  const Vec3 rhat = (rho>0)? (r/rho) : Vec3{0,0,0};
  const Vec3 Fgrav = (-(ctx.mu*ctx.m)/(rho*rho)) * rhat;

  // Magnetic field update schedule (truth)
  if(t >= ctx.lastMagUpdate){
    ctx.lastMagUpdate += ctx.nextMagUpdate;

    // Cartesian to lat/lon/rho (geocentric)
    const double x=r.x, y=r.y, z=r.z;
    const double thetaE = std::acos(z / rho);
    const double psiE = std::atan2(y,x);
    const double latitude = 90.0 - thetaE*180.0/M_PI;
    const double longitude = psiE*180.0/M_PI;
    const double rhokm = rho/1000.0;

    double BN_nT, BE_nT, BD_nT;
    igrf_geocentric_nT(latitude, longitude, rhokm, BN_nT, BE_nT, BD_nT);

    // BNED = [BN;BE;-BD] (PCI has Down as Up in this code)
    const Vec3 BNED{BN_nT, BE_nT, -BD_nT};

    const double phiE = 0.0;
    // MATLAB uses thetaE+pi and psiE
    const Mat3 T = TIB(phiE, thetaE + M_PI, psiE);
    const Vec3 BI_nT = mul(T, BNED);

    // Body field: BB = TIBquat(q)' * BI
    const Mat3 TIBq = TIBquat(q);
    const Vec3 BB_nT = mul(transpose(TIBq), BI_nT);

    // convert to Tesla
    ctx.BB_truth = (1e-9) * BB_nT;
  }

  // Sensor + nav update schedule
  if(t >= ctx.lastSensorUpdate){
    ctx.lastSensorUpdate += ctx.nextSensorUpdate;
    Vec3 BBm = ctx.BB_truth;
    Vec3 pqrm = pqr;
    Vec3 ptpm = ptp;

    sensor_update(BBm, pqrm, ptpm, ctx);
    ctx.BfieldMeasured = BBm;
    ctx.pqrMeasured = pqrm;
    ctx.ptpMeasured = ptpm;

    navigation_update(ctx.BfieldMeasured, ctx.pqrMeasured, ctx.ptpMeasured, ctx);
  }

  // Magtorquer model + saturation
  magtorquer_params(ctx);
  const double maxCurrent_A = ctx.maxCurrent_mA/1000.0;
  if (std::abs(ctx.current.x)+std::abs(ctx.current.y)+std::abs(ctx.current.z) > maxCurrent_A){
    const double s = (std::abs(ctx.current.x)+std::abs(ctx.current.y)+std::abs(ctx.current.z));
    ctx.current = (maxCurrent_A/s) * ctx.current;
  }
  const Vec3 muB = (ctx.n_turns * ctx.A_turn) * ctx.current;
  const Vec3 LMN_mag = cross(muB, ctx.BB_truth);

  // Reaction wheels: saturation on speed + alpha
  Vec3 wdot{0,0,0};
  Vec3 alphas = ctx.rwalphas;
  for(int idx=0; idx<3; ++idx){
    if (std::abs(w123[idx]) > ctx.maxSpeed){
      wdot[idx] = 0.0;
    } else {
      if (std::abs(alphas[idx]) > ctx.maxAlpha){
        alphas[idx] = (alphas[idx]>0?1:-1) * ctx.maxAlpha;
      }
      wdot[idx] = alphas[idx];
    }
  }
  ctx.rwalphas = alphas;

  const Vec3 LMN_RWs =
    mul(ctx.Ir1B, ctx.n1) * wdot.x +
    mul(ctx.Ir2B, ctx.n2) * wdot.y +
    mul(ctx.Ir3B, ctx.n3) * wdot.z;

  // Disturbances
  Vec3 XYZD{0,0,0}, LMND{0,0,0};
  disturbance(rho - ctx.R, ctx.Amax, ctx.lmax, vel, ctx.CD, ctx.BB_truth, XYZD, LMND);

  // Total moments and translational dynamics
  const Vec3 LMN = LMN_mag - LMN_RWs + LMND;

  const Vec3 F = Fgrav + XYZD;
  const Vec3 accel = F / ctx.m;

  // Total angular momentum
  const Vec3 H =
    mul(ctx.Is, pqr) +
    (mul(ctx.Ir1B, ctx.n1) * w123.x) +
    (mul(ctx.Ir2B, ctx.n2) * w123.y) +
    (mul(ctx.Ir3B, ctx.n3) * w123.z);

  // Rotational dynamics
  const Vec3 pqrdot = mul(ctx.invI, (LMN - cross(pqr, H)));

  State16 dst{};
  dst[0]=vel.x; dst[1]=vel.y; dst[2]=vel.z;
  dst[3]=accel.x; dst[4]=accel.y; dst[5]=accel.z;
  dst[6]=qdot.q0; dst[7]=qdot.q1; dst[8]=qdot.q2; dst[9]=qdot.q3;
  dst[10]=pqrdot.x; dst[11]=pqrdot.y; dst[12]=pqrdot.z;
  dst[13]=wdot.x; dst[14]=wdot.y; dst[15]=wdot.z;
  return dst;
}
