#include "../include/satellite.hpp"
#include "params.hpp"
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Constants.hpp>
#include <iostream>

using namespace GeographicLib;

// ------------------------------------------------------------
// 1. Magnetic field helper (ECEF position -> B_NED in tesla)
// ------------------------------------------------------------
static imu::Vector<3> compute_B_ned(const imu::Vector<3>& r_ecef_m,
                                    double decimalYear)
{
    static MagneticModel magModel("wmm2020");   // Change if needed, we will eventually use "wmm2025"
    static Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

    double lat_deg, lon_deg, h_m;
    earth.Reverse(r_ecef_m[0], r_ecef_m[1], r_ecef_m[2],
                  lat_deg, lon_deg, h_m);

    double Bn, Be, Bd;
    magModel(decimalYear, lat_deg, lon_deg, h_m, Bn, Be, Bd); // nT

    imu::Vector<3> B_ned;
    B_ned[0] = Bn;
    B_ned[1] = Be;
    B_ned[2] = Bd;

    // nT to tesla
    B_ned[0] *= 1e-9;
    B_ned[1] *= 1e-9;
    B_ned[2] *= 1e-9;

    return B_ned;
}

// ------------------------------------------------------------
// 2. Dynamics (gravity + magnetic field readout)
// ------------------------------------------------------------
imu::Vector<6> Satellite::dStateDt(const imu::Vector<6> &state)
{
    // Unpack state
    double x    = state[0];
    double y    = state[1];
    double z    = state[2];
    double xdot = state[3];
    double ydot = state[4];
    double zdot = state[5];

    imu::Vector<3> position(x, y, z);      // Treated as ECEF for now
    imu::Vector<3> velocity(xdot, ydot, zdot);

    // Gravity
    double rho = position.magnitude();
    imu::Vector<3> rhat = position / rho;

    imu::Vector<3> acceleration =
        rhat * (-1.0 * (params::G * params::M / (rho * rho)));

    // Magnetic field model ( for testing only)
    double decimalYear = 2025.0;
    imu::Vector<3> B_ned = compute_B_ned(position, decimalYear);

    // remember this B-field inside the Satellite object
    lastBField_NED = B_ned;

    static int counter = 0;
    counter++;
    if (counter % 1000 == 0) {
        double Bmag = B_ned.magnitude();
        std::cout << "[DEBUG] |B| = " << Bmag << " T" << std::endl;
    }
    
    // Pack derivative of state
    imu::Vector<6> stateOut;

    stateOut[0] = velocity[0];
    stateOut[1] = velocity[1];
    stateOut[2] = velocity[2];
    stateOut[3] = acceleration[0];
    stateOut[4] = acceleration[1];
    stateOut[5] = acceleration[2];

    return stateOut;
}