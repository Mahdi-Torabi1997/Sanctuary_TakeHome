cmake_minimum_required(VERSION 3.14)
project(Sanctuary)

set(CMAKE_CXX_STANDARD 17)

# Disable OSQP's tests in order to avoid mixing with my unit tests
set(OSQP_BUILD_TESTS OFF CACHE BOOL "Disable OSQP tests")
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_TESTS OFF CACHE BOOL "" FORCE)

# Dependencies 
add_subdirectory(osqp-cpp)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
set(QUIK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/QuIK/include")
include_directories(${QUIK_DIR})

# Executables 
add_executable(main_exec
  src/main.cpp
  src/RRRManipulator.cpp
  src/ForwardKinematics.cpp
  src/InverseKinematics.cpp
  src/GradientDescentIK.cpp
  src/LevenbergMarquardtIK.cpp
  src/Jacobian.cpp
  src/QuadraticProgrammingIK.cpp
  src/GradientDescentIKElbowUp.cpp
)

# source files
set(QUIK_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/QuIK/src)
target_sources(main_exec PRIVATE
  ${QUIK_SRC_DIR}/geometry.cpp
)

target_link_libraries(main_exec PRIVATE osqp-cpp Eigen3::Eigen)

# Add include/Sanctuary to search path for both targets
target_include_directories(main_exec PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Tests
add_executable(unit_tests
  tests/test_forward_kinematics.cpp
  tests/test_inverse_kinematics.cpp
  tests/test_ik_fk_integration.cpp
  src/ForwardKinematics.cpp
  src/InverseKinematics.cpp
  src/RRRManipulator.cpp
)

target_link_libraries(unit_tests
  GTest::GTest
  GTest::Main
  Eigen3::Eigen
)

target_include_directories(unit_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

enable_testing()
include(GoogleTest)
gtest_discover_tests(unit_tests)
