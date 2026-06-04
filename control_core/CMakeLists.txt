add_library(arm_control_core STATIC
  src/arm_control.c
  src/arm_math.c
  src/joint_sweep.c
)

target_include_directories(arm_control_core
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_features(arm_control_core PUBLIC c_std_11)

if(MSVC)
  target_compile_options(arm_control_core PRIVATE /W4)
else()
  target_compile_options(arm_control_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
