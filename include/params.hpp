#pragma once

namespace params{
    // general parameters
    constexpr double pi = 3.141592653589793;
    constexpr int timeStep = 1;

    // satellite/orbit parameters
    constexpr int mass = 3; // kg
    constexpr int altitude = 500000; // meters
    constexpr double inclination = 51.6 * (pi)/ 180; // radians

    // planet parameters
    constexpr int R = 6371000; // meters
    constexpr double M = 5.972e24; // kg
    constexpr double G = 6.67e-11; //some SI units
    constexpr double mu = M * G;
}