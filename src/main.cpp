#pragma once
#include<iostream>
#include<cmath>
#include "params.hpp"
#include<imumaths/imumaths.hpp>


int main(){
    std::cout << "Simulation Started." << '\n';
    int numberOfOrbits = 1;
    

    imu::Vector<6> state = imu::Vector<6>();
    state[0] = params::R + params::altitude;
    state[1] = 0.0f;
    state[2] = 0.0f;

    double semimajor = std::sqrt(state[0]*state[0] + state[1]*state[1] + state[2]*state[3]);
    double vCircular = std::sqrt(params::mu / semimajor);
    state[3] = 0.0f;
    state[4] = vCircular * std::cos(params::inclination);
    state[5] = vCircular * std::sin(params::inclination);


    double period = 2 * params::pi * std::sqrt(semimajor*semimajor*semimajor / params::mu);

    // TODO: Figure out RK4.
    /**
     * current idea: create dStateDt in a similar fashion to the python one
     * use the actual vector (from lib) to be able to actually perform math
     * do as you would with a numpy vector
     * possible issues: at the end the vector will have 13 values. Is that ok?
     */

    std::cout << "Simulation Completed." << '\n';
}