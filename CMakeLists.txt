cmake_minimum_required(VERSION 3.16)

# Small C++ project for the ADCS simulation.
project(adcs_faithful LANGUAGES CXX)

# Keep the compiler settings simple and explicit.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build one executable from the core simulation modules.
add_executable(adcs
  src/main.cpp
  src/satellite.cpp
  src/control.cpp
  src/sensor.cpp
  src/navigation.cpp
  src/disturbance.cpp
  src/wmm.cpp
)

# All headers live in the local include folder.
target_include_directories(adcs PRIVATE include)
