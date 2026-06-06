add_library(arm_common STATIC
  src/arm_math.c
)

option(ARM_COMMON_USE_FLOAT "Use float as arm_real_t instead of double" OFF)
option(ARM_CORE_USE_FLOAT "Compatibility alias for ARM_COMMON_USE_FLOAT" OFF)

target_include_directories(arm_common
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_features(arm_common PUBLIC c_std_11)

if(ARM_COMMON_USE_FLOAT OR ARM_CORE_USE_FLOAT)
  target_compile_definitions(arm_common PUBLIC ARM_COMMON_USE_FLOAT)
endif()

if(UNIX)
  target_link_libraries(arm_common PUBLIC m)
endif()

if(MSVC)
  target_compile_options(arm_common PRIVATE /W4)
else()
  target_compile_options(arm_common PRIVATE -Wall -Wextra -Wpedantic)
endif()
