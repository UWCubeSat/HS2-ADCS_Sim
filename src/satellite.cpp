#pragma once
#include "../include/satellite.hpp"
#include "params.hpp"

imu::Vector<6> Satellite::dStateDt(const imu::Vector<6> &state){
    double x = state[0];
    double y = state[1];
    double z = state[2];
    double xdot = state[3];
    double ydot = state[4];
    double zdot = state[5];

    imu::Vector<3> position = imu::Vector<3>(x,y,z);
    imu::Vector<3> velocity = imu::Vector<3>(xdot,ydot,zdot);

    double rho = position.magnitude();
    imu::Vector<3> rhat = position / rho;
    imu::Vector<3> FGrav = rhat * (-1 * (params::G * params::M * mass / (rho * rho)));

    imu::Vector<3> acceleration = FGrav / mass;

    imu::Vector<6> stateOut = imu::Vector<6>();

    stateOut[0] = velocity[0];
    stateOut[1] = velocity[1];
    stateOut[2] = velocity[2];
    stateOut[3] = acceleration[0];
    stateOut[4] = acceleration[1];
    stateOut[5] = acceleration[2];

    return stateOut;
}