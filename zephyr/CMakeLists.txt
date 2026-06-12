# Invoke ONLY via: west build -b nucleo_f446re
cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(infusion_pump_fw VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/VolumeTracker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/OcclusionMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/InfusionMode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/ConstantRateMode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/LinearRampMode.cpp
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
