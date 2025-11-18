#pragma once
#include "imumaths/imumaths.hpp"

class Satellite{
    const int mass;
    public:
        Satellite(const int m): mass(m) {};

        imu::Vector<6> dStateDt(const imu::Vector<6> &state);
};