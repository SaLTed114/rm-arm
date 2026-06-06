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

add_subdirectory(arm_common)
add_subdirectory(arm_core)
add_subdirectory(arm_motion)

if(ARMSIM_BUILD_SIM)
  add_subdirectory(sim_mujoco)
endif()

if(ARMSIM_BUILD_TESTS)
  add_executable(test_core_types tests/test_core_types.c)
  target_link_libraries(test_core_types PRIVATE arm_core)
  add_test(NAME test_core_types COMMAND test_core_types)

  add_executable(test_motion_types tests/test_motion_types.c)
  target_link_libraries(test_motion_types PRIVATE arm_motion)
  add_test(NAME test_motion_types COMMAND test_motion_types)

  add_executable(test_core_contract tests/test_core_contract.c)
  target_link_libraries(test_core_contract PRIVATE arm_core)
  add_test(NAME test_core_contract COMMAND test_core_contract)

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import yaml"
      RESULT_VARIABLE ARMSIM_PYYAML_RESULT
      OUTPUT_QUIET
      ERROR_QUIET
    )
    if(ARMSIM_PYYAML_RESULT EQUAL 0)
      add_test(
        NAME generated_arm6_config
        COMMAND ${Python3_EXECUTABLE} tools/generate_arm_model.py --config configs/arm6_placeholder.yaml --check
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
      )
    else()
      message(STATUS "PyYAML not found; generated_arm6_config test is skipped.")
    endif()
  endif()
endif()
