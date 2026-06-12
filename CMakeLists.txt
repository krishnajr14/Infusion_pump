cmake_minimum_required(VERSION 3.20)
project(infusion_pump_host_tests CXX)

set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

option(ENABLE_COVERAGE "Enable gcov coverage instrumentation" OFF)
if(ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(--coverage -O0 -g -fno-inline)
        add_link_options(--coverage)
    endif()
endif()

find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

set(SUT_SOURCES
    src/VolumeTracker.cpp
    src/OcclusionMonitor.cpp
    src/InfusionMode.cpp
    src/ConstantRateMode.cpp
    src/LinearRampMode.cpp
)

add_executable(infusion_pump_tests
    ${SUT_SOURCES}
    tests/test_VolumeTracker.cpp
    tests/test_OcclusionMonitor.cpp
    tests/test_InfusionMode.cpp
)

target_include_directories(infusion_pump_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/stubs
)

target_link_libraries(infusion_pump_tests
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(infusion_pump_tests)

add_custom_target(coverage
    DEPENDS infusion_pump_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND lcov
            --capture
            --directory ${CMAKE_BINARY_DIR}
            --output-file ${CMAKE_BINARY_DIR}/coverage_raw.info
            --rc lcov_branch_coverage=1
            --ignore-errors mismatch
    COMMAND lcov
            --remove ${CMAKE_BINARY_DIR}/coverage_raw.info
            "/usr/*"
            "*/tests/*"
            "*/gtest/*"
            "*/zephyr/*"
            --output-file ${CMAKE_BINARY_DIR}/coverage.info
            --rc lcov_branch_coverage=1
            --ignore-errors unused
    COMMAND genhtml
            ${CMAKE_BINARY_DIR}/coverage.info
            --output-directory ${CMAKE_BINARY_DIR}/coverage_html
            --branch-coverage
    COMMAND lcov --list ${CMAKE_BINARY_DIR}/coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating coverage report → build/coverage_html/index.html"
)
