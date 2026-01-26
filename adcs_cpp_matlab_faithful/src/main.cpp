#include "modules.hpp"
#include "params.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

static State16 add(const State16& a, const State16& b){
  State16 c{};
  for(int i=0;i<16;i++) c[i]=a[i]+b[i];
  return c;
}
static State16 scale(const State16& a, double s){
  State16 c{};
  for(int i=0;i<16;i++) c[i]=a[i]*s;
  return c;
}

int main(){
  // Deterministic seed; change for randomness
  SimContext ctx(/*seed=*/1);

  // === Mirror main.m initialization ===
  // planet + inertia initialized lazily inside satellite_derivatives

  // Initial orbit
  const double altitude = 600.0*1000.0;
  // We need R, mu, etc, so force init by calling satellite_derivatives once with a temp state.
  // We'll build the real state after ctx.R, ctx.mu exist.
  State16 dummy{};
  auto _ = satellite_derivatives(0.0, dummy, ctx);

  const double x0 = ctx.R + altitude;
  const double y0 = 0.0;
  const double z0 = 0.0;
  const double xdot0 = 0.0;
  const double inclination = 56.0*M_PI/180.0;
  const double semi_major = std::sqrt(x0*x0 + y0*y0 + z0*z0);
  const double vcircular = std::sqrt(ctx.mu/semi_major);
  const double ydot0 = vcircular*std::cos(inclination);
  const double zdot0 = vcircular*std::sin(inclination);

  // Attitude IC
  const Vec3 ptp0{0,0,0};
  Quat q0 = euler321_to_quat(ptp0);

  const double p0 = 0.8;
  const double q00 = -0.2;
  const double r0 = 0.3;

  const double w10 = 0.0, w20 = 0.0, w30 = 0.0;

  State16 state{};
  state[0]=x0; state[1]=y0; state[2]=z0;
  state[3]=xdot0; state[4]=ydot0; state[5]=zdot0;
  state[6]=q0.q0; state[7]=q0.q1; state[8]=q0.q2; state[9]=q0.q3;
  state[10]=p0; state[11]=q00; state[12]=r0;
  state[13]=w10; state[14]=w20; state[15]=w30;

  // Time window: 1 orbit
  const double period = 2.0*M_PI/std::sqrt(ctx.mu) * std::pow(semi_major, 1.5);
  const int number_of_orbits = 1;
  const double tfinal = period * number_of_orbits;

  const double dt = SimParams::timestep;

  // Sensor params (initial biases)
  // mirror sensor_params.m once at init:
  // implemented indirectly by calling navigation_update later; we do it here explicitly by invoking sensor_update with no change? no.
  // Instead call a single sensor update sequence inside satellite_derivatives at t=0.
  ctx.lastSensorUpdate = 0.0;
  ctx.lastMagUpdate = 0.0;
  ctx.nextMagUpdate = 1.0;
  ctx.nextSensorUpdate = 1.0;
  // Initialize nav prev sentinel
  ctx.BfieldNavPrev = {-99,0,0};

  // Kick Satellite once to initialize BB and sensor/nav globals
  satellite_derivatives(0.0, state, ctx);

  // Control timing
  double lastControl = -dt;
  const double nextControl = 0.1;

  std::ofstream csv("adcs_output.csv");
  csv << std::setprecision(17);
  // Header
  csv << "t,"
      << "x,y,z,xdot,ydot,zdot,q0,q1,q2,q3,p,q,r,w1,w2,w3,"
      << "BBx,BBy,BBz,Bmx,Bmy,Bmz,BNx,BNy,BNz,"
      << "pqrm_x,pqrm_y,pqrm_z,pqrN_x,pqrN_y,pqrN_z,"
      << "ptpm_phi,ptpm_theta,ptpm_psi,ptpN_phi,ptpN_theta,ptpN_psi,"
      << "ix,iy,iz,rwa1,rwa2,rwa3\n";

  for(double t=0.0; t<=tfinal + 1e-12; t+=dt){
    // Log
    csv << t << ",";
    for(int i=0;i<16;i++){
      csv << state[i] << (i==15? ",": ",");
    }
    csv << ctx.BB_truth.x << "," << ctx.BB_truth.y << "," << ctx.BB_truth.z << ","
        << ctx.BfieldMeasured.x << "," << ctx.BfieldMeasured.y << "," << ctx.BfieldMeasured.z << ","
        << ctx.BfieldNav.x << "," << ctx.BfieldNav.y << "," << ctx.BfieldNav.z << ","
        << ctx.pqrMeasured.x << "," << ctx.pqrMeasured.y << "," << ctx.pqrMeasured.z << ","
        << ctx.pqrNav.x << "," << ctx.pqrNav.y << "," << ctx.pqrNav.z << ","
        << ctx.ptpMeasured.x << "," << ctx.ptpMeasured.y << "," << ctx.ptpMeasured.z << ","
        << ctx.ptpNav.x << "," << ctx.ptpNav.y << "," << ctx.ptpNav.z << ","
        << ctx.current.x << "," << ctx.current.y << "," << ctx.current.z << ","
        << ctx.rwalphas.x << "," << ctx.rwalphas.y << "," << ctx.rwalphas.z
        << "\n";

    // Control block
    if(t > lastControl){
      Vec3 current, rwalphas;
      control_compute(ctx.BfieldNav, ctx.pqrNav, ctx.ptpNav, ctx, current, rwalphas);
      ctx.current = current;
      ctx.rwalphas = rwalphas;
      lastControl += nextControl;
    }

    // RK4 integration
    const State16 k1 = satellite_derivatives(t, state, ctx);
    const State16 k2 = satellite_derivatives(t + dt/2.0, add(state, scale(k1, dt/2.0)), ctx);
    const State16 k3 = satellite_derivatives(t + dt/2.0, add(state, scale(k2, dt/2.0)), ctx);
    const State16 k4 = satellite_derivatives(t + dt, add(state, scale(k3, dt)), ctx);

    State16 incr{};
    for(int i=0;i<16;i++){
      incr[i] = (1.0/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    }
    for(int i=0;i<16;i++){
      state[i] += dt*incr[i];
    }

    // Match MATLAB practice: keep quaternion from drifting (MATLAB does not explicitly normalize here,
    // but numerical drift exists; we keep it OFF for strictness. Uncomment if needed.)
    // Quat q = {state[6],state[7],state[8],state[9]};
    // q = normalize(q);
    // state[6]=q.q0; state[7]=q.q1; state[8]=q.q2; state[9]=q.q3;
  }

  std::cout << "Done. Wrote adcs_output.csv\n";
  return 0;
}
