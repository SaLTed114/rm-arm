cmake_minimum_required(VERSION 3.20)

project(ArmSim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(ARMSIM_BUILD_SIM "Build MuJoCo simulation targets when dependencies are available" ON)
option(ARMSIM_BUILD_TESTS "Build host-side arm_core smoke tests" ON)

if(ARMSIM_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(arm_core)

if(ARMSIM_BUILD_SIM)
  add_subdirectory(sim_mujoco)
endif()

if(ARMSIM_BUILD_TESTS)
  add_executable(test_core_types tests/test_core_types.c)
  target_link_libraries(test_core_types PRIVATE arm_core)
  add_test(NAME test_core_types COMMAND test_core_types)
endif()
