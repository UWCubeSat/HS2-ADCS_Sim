#pragma once
#include<iostream>
#include<fstream>
#include<cmath>
#include "../include/params.hpp"
#include "../include/imumaths/imumaths.hpp"
#include "../include/satellite.hpp"


int main(){
    std::ofstream file("../output/test.txt");
    if (!file) {                       // Check if file opened
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::cout << "Simulation Started." << '\n';

    Satellite sat = Satellite(params::mass);
    int numberOfOrbits = 1;
    

    imu::Vector<6> state = imu::Vector<6>();
    state[0] = params::R + params::altitude;
    state[1] = 0.0;
    state[2] = 0.0;

    double semimajor = std::sqrt(state[0]*state[0] + state[1]*state[1] + state[2]*state[2]);
    double vCircular = std::sqrt(params::mu / semimajor);
    std::cout << "" << vCircular << '\n';
    state[3] = 0.0;
    state[4] = vCircular * std::cos(params::inclination);
    state[5] = vCircular * std::sin(params::inclination);


    double period = 2 * params::pi * std::sqrt(semimajor*semimajor*semimajor / params::mu);

    double tFinal = period * numberOfOrbits;
    double t = 0.0;
    file << "" << t << "," << state[0]<< "," << state[1]<< "," << state[2]<< "," << state[3]<< "," << state[4]<< "," << state[5] <<'\n';
    for(; t < tFinal; t += params::timeStep){
        if(static_cast<int>(t) % 100 == 0){
            std::cout << "Time is " << t << '\n';
        }
        imu::Vector<6> k1 = sat.dStateDt(state);
        imu::Vector<6> k2 = sat.dStateDt(state + k1 * (params::timeStep / 2));
        imu::Vector<6> k3 = sat.dStateDt(state + k2 * (params::timeStep / 2));
        imu::Vector<6> k4 = sat.dStateDt(state + k3 * (params::timeStep));
        imu::Vector<6> k = (k1 + k2*2 + k3*2 + k4).scale(1/6.0);

        state = state + k * params::timeStep;
        file << "" << t << "," << state[0]<< "," << state[1]<< "," << state[2]<< "," << state[3]<< "," << state[4]<< "," << state[5] <<'\n';
    }
    // file << std::endl;
    std::cout << "Simulation Completed." << '\n';
}