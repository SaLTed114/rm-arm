add_library(arm_core STATIC
  src/core/arm.c
  src/core/arm_control.c
  src/core/arm_math.c
  src/controllers/joint_pvi.c
  src/controllers/joint_sweep.c
  src/estimation/joint_state_filter.c
  src/reference/joint_ref_shaper.c
  src/safety/arm_safety.c
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
