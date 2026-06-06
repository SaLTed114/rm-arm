if(NOT TARGET arm_core)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../arm_core ${CMAKE_CURRENT_BINARY_DIR}/arm_core)
endif()
if(NOT TARGET arm_motion)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../arm_motion ${CMAKE_CURRENT_BINARY_DIR}/arm_motion)
endif()

get_filename_component(ARMSIM_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(MUJOCO_ROOT "${ARMSIM_ROOT}/third_party/mujoco" CACHE PATH "Path to the MuJoCo distribution")
set(GLFW_ROOT "${ARMSIM_ROOT}/third_party/glfw" CACHE PATH "Path to the GLFW distribution")

if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  message(WARNING "MuJoCo headers not found at ${MUJOCO_ROOT}; skipping sim_mujoco targets.")
  return()
endif()

find_library(MUJOCO_LIBRARY
  NAMES mujoco mujoco.dll
  PATHS "${MUJOCO_ROOT}"
  PATH_SUFFIXES lib bin
  NO_DEFAULT_PATH
)

if(NOT MUJOCO_LIBRARY)
  message(WARNING "MuJoCo library not found under ${MUJOCO_ROOT}; skipping sim_mujoco targets.")
  return()
endif()

find_file(MUJOCO_DLL
  NAMES mujoco.dll
  PATHS "${MUJOCO_ROOT}"
  PATH_SUFFIXES bin
  NO_DEFAULT_PATH
)

add_library(mujoco::mujoco UNKNOWN IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include"
)

function(copy_mujoco_runtime target_name)
  if(MUJOCO_DLL)
    add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_DLL}"
        "$<TARGET_FILE_DIR:${target_name}>"
      VERBATIM
    )
  endif()
endfunction()

add_library(armsim_mujoco STATIC
  src/control_log.c
  src/csv_logger.c
  src/default_arm_config.c
  src/mujoco_arm.c
  src/mujoco_gravity_oracle.c
  src/sim_impairment.c
  src/sim_loop.c
)

target_include_directories(armsim_mujoco
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(armsim_mujoco
  PUBLIC
    arm_core
    arm_motion
    mujoco::mujoco
)

add_executable(armsim_verify_joints apps/armsim_verify_joints.c)
target_link_libraries(armsim_verify_joints PRIVATE armsim_mujoco)
copy_mujoco_runtime(armsim_verify_joints)

add_executable(armsim_verify_gravity_comp apps/armsim_verify_gravity_comp.c)
target_link_libraries(armsim_verify_gravity_comp PRIVATE armsim_mujoco)
copy_mujoco_runtime(armsim_verify_gravity_comp)

add_executable(armsim_verify_harsh_hold apps/armsim_verify_harsh_hold.c)
target_link_libraries(armsim_verify_harsh_hold PRIVATE armsim_mujoco)
copy_mujoco_runtime(armsim_verify_harsh_hold)

add_executable(armsim_control_benchmark apps/armsim_control_benchmark.c)
target_link_libraries(armsim_control_benchmark PRIVATE armsim_mujoco)
copy_mujoco_runtime(armsim_control_benchmark)

if(ARMSIM_BUILD_TESTS)
  add_test(NAME armsim_verify_joints COMMAND armsim_verify_joints WORKING_DIRECTORY "${ARMSIM_ROOT}")
  add_test(NAME armsim_verify_gravity_comp COMMAND armsim_verify_gravity_comp WORKING_DIRECTORY "${ARMSIM_ROOT}")
  add_test(NAME armsim_verify_harsh_hold COMMAND armsim_verify_harsh_hold WORKING_DIRECTORY "${ARMSIM_ROOT}")
  add_test(
    NAME armsim_control_benchmark_hold_zero
    COMMAND armsim_control_benchmark hold_zero_harsh logs/control_benchmark_hold_zero_harsh.csv
    WORKING_DIRECTORY "${ARMSIM_ROOT}"
  )

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(
      NAME analyze_control_benchmark
      COMMAND ${Python3_EXECUTABLE}
        tools/analyze_control_log.py
        logs/control_benchmark_hold_zero_harsh.csv
        --config configs/arm6_placeholder.yaml
        --json-out logs/control_benchmark_hold_zero_harsh_metrics.json
      WORKING_DIRECTORY "${ARMSIM_ROOT}"
    )
    set_tests_properties(analyze_control_benchmark PROPERTIES DEPENDS armsim_control_benchmark_hold_zero)
    add_test(
      NAME analyze_control_minimal_csv
      COMMAND ${Python3_EXECUTABLE}
        tools/analyze_control_log.py
        tests/fixtures/minimal_control_log.csv
        --json-out logs/minimal_control_log_metrics.json
      WORKING_DIRECTORY "${ARMSIM_ROOT}"
    )
  endif()
endif()

set(GLFW_TARGET "")
if(EXISTS "${GLFW_ROOT}/CMakeLists.txt")
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory("${GLFW_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/glfw")
  set(GLFW_TARGET glfw)
else()
  find_package(glfw3 CONFIG QUIET)
  if(TARGET glfw)
    set(GLFW_TARGET glfw)
  elseif(TARGET glfw3)
    set(GLFW_TARGET glfw3)
  endif()
endif()

if(GLFW_TARGET)
  find_package(OpenGL REQUIRED)
  add_executable(armsim_viewer apps/armsim_viewer.c)
  target_link_libraries(armsim_viewer PRIVATE armsim_mujoco ${GLFW_TARGET} OpenGL::GL)
  copy_mujoco_runtime(armsim_viewer)
else()
  message(WARNING "GLFW not found; armsim_viewer will not be built.")
endif()
