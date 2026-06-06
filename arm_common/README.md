# arm_common

`arm_common` is the portable shared numeric layer used by `arm_core`,
`arm_motion`, and PC simulation code.

Rules for this directory:

- No MuJoCo, GLFW, HAL, FreeRTOS, stdio, filesystem, or dynamic allocation.
- Public headers live under `include/arm_common/`.
- Keep this layer limited to scalar types, DOF/status constants, and generic
  math helpers.
- Do not add controller, reference generation, kinematics policy, simulation,
  or platform adapter logic here.

`arm_real_t` is `double` by default. Configure with `ARM_COMMON_USE_FLOAT=ON`
to use `float`; the older `ARM_CORE_USE_FLOAT=ON` option is still accepted as a
compatibility alias.
