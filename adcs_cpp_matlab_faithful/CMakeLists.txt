cmake_minimum_required(VERSION 3.16)
project(adcs_faithful LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(adcs
  src/main.cpp
  src/satellite.cpp
  src/control.cpp
  src/sensor.cpp
  src/navigation.cpp
  src/disturbance.cpp
  src/igrf.cpp
)

target_include_directories(adcs PRIVATE include)
