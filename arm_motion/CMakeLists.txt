add_library(arm_motion STATIC
  src/kinematics/joint_kinematics.c
  src/reference/joint_ref_shaper.c
  src/trajectory/joint_trajectory.c
)

target_include_directories(arm_motion
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(arm_motion
  PUBLIC
    arm_core
)

target_compile_features(arm_motion PUBLIC c_std_11)

if(MSVC)
  target_compile_options(arm_motion PRIVATE /W4)
else()
  target_compile_options(arm_motion PRIVATE -Wall -Wextra -Wpedantic)
endif()
