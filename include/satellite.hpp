#pragma once
#include<imumaths/imumaths.hpp>

class Satellite {
public:
    Satellite(double m) : mass(m), lastBField_NED(0.0, 0.0, 0.0) {}

    // State derivative
    imu::Vector<6> dStateDt(const imu::Vector<6> &state);

    // access to last computed magnetic field (NED, tesla)
    imu::Vector<3> getLastBFieldNED() const { return lastBField_NED; }

private:
    double mass;

    // store last B-field (NED, tesla)
    imu::Vector<3> lastBField_NED;
};
