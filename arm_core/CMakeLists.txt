add_library(arm_core STATIC
  src/arm.c
  src/arm_control.c
  src/arm_math.c
  src/arm_safety.c
  src/joint_ref_planner.c
  src/joint_pvi.c
  src/joint_sweep.c
)

option(ARM_CORE_USE_FLOAT "Use float as arm_real_t instead of double" OFF)

target_include_directories(arm_core
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_features(arm_core PUBLIC c_std_11)

if(ARM_CORE_USE_FLOAT)
  target_compile_definitions(arm_core PUBLIC ARM_CORE_USE_FLOAT)
endif()

if(MSVC)
  target_compile_options(arm_core PRIVATE /W4)
else()
  target_compile_options(arm_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
