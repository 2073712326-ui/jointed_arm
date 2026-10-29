cmake_minimum_required(VERSION 3.16)
project(arm_trajectory_tests LANGUAGES CXX)
enable_testing()

add_executable(arm_trajectory_tests
  arm_trajectory_test.cpp
  ../app/arm.cpp
  ../app/control.cpp
  ../app/arm_kinematics.cpp
  ../app/arm_trajectory.cpp)
target_compile_features(arm_trajectory_tests PRIVATE cxx_std_11)
target_include_directories(arm_trajectory_tests PRIVATE stubs ..)
target_compile_definitions(arm_trajectory_tests PRIVATE DR16)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(arm_trajectory_tests PRIVATE
    -Wall -Wextra -Werror -Ofast)
endif()
add_test(NAME arm_trajectory COMMAND arm_trajectory_tests)
