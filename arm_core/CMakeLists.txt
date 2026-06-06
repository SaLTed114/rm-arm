add_library(arm_core STATIC
  src/core/arm.c
  src/core/arm_control.c
  src/controllers/joint_pd.c
  src/controllers/joint_sweep.c
  src/estimation/joint_state_filter.c
  src/feedforward/joint_gravity_ff.c
  src/feedforward/joint_id_fit_ff.c
  src/safety/arm_safety.c
)

target_include_directories(arm_core
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_features(arm_core PUBLIC c_std_11)

target_link_libraries(arm_core PUBLIC arm_common)

if(MSVC)
  target_compile_options(arm_core PRIVATE /W4)
else()
  target_compile_options(arm_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
