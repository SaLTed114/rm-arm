# control_core

`control_core` is the portable part intended for both MuJoCo simulation and
STM32F4xx/Keil firmware.

Rules for this directory:

- No MuJoCo, GLFW, HAL, FreeRTOS, stdio, or filesystem dependencies.
- No dynamic allocation.
- Public headers live under `include/arm_core/`.
- Platform code should keep one `arm_t`, update `arm.state` each control tick,
  call `arm_control_step()`, and read torque commands from `arm.command`.

The active robot DOF is runtime-configured through `arm_config_t.dof`, with
fixed arrays sized by `ARM_DOF_MAX`.
