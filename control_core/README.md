# control_core

`control_core` is the portable part intended for both MuJoCo simulation and
STM32F4xx/Keil firmware.

Rules for this directory:

- No MuJoCo, GLFW, HAL, FreeRTOS, stdio, or filesystem dependencies.
- No dynamic allocation.
- Public headers live under `include/arm_core/`.
- Platform code should keep one `arm_t`, update `arm.state` each control tick,
  prepare an `arm_reference_t`, call `arm_control_step()`, and send
  `arm.command` through the platform actuator adapter.
- Controllers are plain C contexts adapted by `arm_controller_t`. `joint_pvi`
  is the current joint-space PVI/PD controller; `joint_sweep` is kept for
  channel verification.

The active robot DOF is runtime-configured through `arm_config_t.dof`, with
fixed arrays sized by `ARM_DOF_MAX`.

Joint sign convention:

- `q_rad > 0` follows the joint axis right-hand rule.
- `dq_rad_s > 0` is velocity in the same direction.
- `command.tau_ff_nm > 0` pushes the joint toward positive acceleration in that
  direction.
- `tau_est_nm > 0` uses the same torque direction.

`arm_reference_t` is the per-cycle target consumed by controllers. `arm_command_t`
is the final actuator command abstraction; a pure torque controller writes
`tau_ff_nm` with zero `kp/kd/q_d/dq_d`, while a future MIT-mode adapter can use
the full command fields.
