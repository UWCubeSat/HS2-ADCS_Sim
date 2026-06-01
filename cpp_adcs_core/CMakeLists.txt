cmake_minimum_required(VERSION 3.16)
project(cpp_adcs_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(adcs_core_lib STATIC
    src/adcs_core.cpp
)
target_include_directories(adcs_core_lib PUBLIC include)

add_executable(adcs_core_smoke
    tests/test_adcs_core.cpp
)
target_link_libraries(adcs_core_smoke PRIVATE adcs_core_lib)

option(ADCS_BUILD_PYBIND "Build optional pybind11 Python module" ON)
if(ADCS_BUILD_PYBIND)
    find_package(pybind11 CONFIG QUIET)
    if(pybind11_FOUND)
        pybind11_add_module(adcs_core bindings/pybind11_module.cpp)
        target_link_libraries(adcs_core PRIVATE adcs_core_lib)
        target_include_directories(adcs_core PRIVATE include)
    else()
        message(WARNING "pybind11 was not found. Building C++ library and smoke test only. To build Python bindings, install pybind11 and pass -Dpybind11_DIR=$(python -m pybind11 --cmakedir).")
    endif()
endif()
