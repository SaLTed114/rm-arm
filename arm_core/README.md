# arm_core

`arm_core` is the portable part intended for both MuJoCo simulation and
STM32F4xx/Keil firmware.

Rules for this directory:

- No MuJoCo, GLFW, HAL, FreeRTOS, stdio, or filesystem dependencies.
- No dynamic allocation.
- Public headers live under `include/arm_core/`.
- Source files are grouped internally under `src/core`, `src/controllers`,
  `src/estimation`, `src/reference`, and `src/safety`; this
  does not change public include paths.
- Host-side tests live outside this directory. A future Keil sync script should
  run the tests first, then copy only the approved `arm_core/include` and
  `arm_core/src` files.
- Platform code should keep one `arm_t`, update `arm.state` each control tick,
  shape a goal into an `arm_reference_t`, call `arm_control_step()`, apply
  final safety limits, and send `arm.command` through the platform actuator
  adapter.
- Controllers are plain C contexts adapted by `arm_controller_t`. `joint_pvi`
  is the current joint-space PVI/PD controller; `joint_sweep` is kept for
  channel verification.
- `joint_state_filter` is a simple estimation layer for normalized measured
  state. Platform adapters should not hide heavy filtering outside the core.
- `joint_ref_shaper` is a simple joint-space reference conditioner for
  manual/teleop-style targets, with position, velocity, and acceleration
  limits.
- `arm_safety` is a final command-side guard for invalid state, torque limits,
  soft position limits, and overspeed protection.

The active robot DOF is runtime-configured through `arm_config_t.dof`, with
fixed arrays sized by `ARM_DOF_MAX`.

Joint sign convention:

- `q_rad > 0` follows the joint axis right-hand rule.
- `dq_rad_s > 0` is velocity in the same direction.
- `command.tau_ff_nm > 0` pushes the joint toward positive acceleration in that
  direction.
- `tau_est_nm > 0` uses the same torque direction.

`arm_reference_t` is the per-cycle target consumed by controllers. Reference
sources should feed this common shape: manual/teleop targets can pass through
`joint_ref_shaper`, while a future trajectory generator can output
`arm_reference_t` directly through a future `joint_trajectory_step()` path.
These reference sources are alternatives by default; the trajectory path should
not be automatically fed through `joint_ref_shaper` unless that behavior is
explicitly desired. `arm_safety` remains a final command-side guard; it does not
replace reference source logic.

`arm_command_t` is the final actuator command abstraction; a pure torque
controller writes `tau_ff_nm` with zero `kp/kd/q_d/dq_d`, while a future MIT-mode
adapter can use the full command fields.

## Core Contract

`arm_config_t` is the static contract for one robot instance:

- `dof` is the active joint count. It must be greater than zero and no larger
  than `ARM_DOF_MAX`.
- Joint array order is the canonical channel order used by `state`,
  `reference`, and `command`.
- `joint_name` and `actuator_name` are stable binding names for platform or
  simulation adapters.
- `sign` maps raw platform direction into the core right-hand-rule direction.
  It must be nonzero; use `+1` or `-1` unless there is a deliberate calibrated
  scale factor.
- `q_offset_rad` is the raw joint position that corresponds to core zero.
- `torque_limit_nm` is the final per-joint torque limit used by core command
  limiting. It must not be negative.

`arm_state_t` is the per-cycle measured input. Platform adapters must fill it
before calling core logic:

- `q_rad`, `dq_rad_s`, and `tau_est_nm` are already in core convention.
- `time_s` and `dt_s` are seconds. `dt_s` should be the current control period.
- Validity flags state which measurements are usable. Safety requires position
  and velocity valid before allowing torque output.
- Platform adapters should do unit conversion, direction mapping, zero offset,
  timestamps, and validity checks. Filtering or estimation should be explicit in
  `arm_core`, for example through `joint_state_filter`.

`arm_reference_t` is the per-cycle controller target. It is produced by exactly
one active reference source:

- Manual or teleop targets should pass through `joint_ref_shaper`.
- A future trajectory source may output `arm_reference_t` directly through a
  `joint_trajectory_step()` path.
- These sources are alternatives by default; do not feed trajectory output
  through `joint_ref_shaper` unless that behavior is explicitly requested.

`arm_command_t` is the controller output and final actuator target abstraction:

- `joint_pvi` currently writes only `tau_ff_nm`.
- `q_d_rad`, `dq_d_rad_s`, `kp`, and `kd` are reserved for future command modes
  such as MIT-style actuator targets.
- `arm_limit_command()` and `arm_safety_apply()` protect the final command side;
  they do not replace reference shaping or trajectory generation.

Adapter mapping formulas:

```text
q_core       = sign * (q_raw - q_offset_rad)
dq_core      = sign * dq_raw
tau_est_core = sign * tau_est_raw

tau_raw      = sign * tau_ff_core
q_raw_target = q_offset_rad + sign * q_d_core
dq_raw_target = sign * dq_d_core
```

For a pure torque controller, the platform adapter should send `tau_raw` and
leave actuator-side position and velocity gains disabled unless a future command
mode explicitly enables them.

## Bring-Up Principle

Real-arm bring-up should unlock capability in stages instead of starting with
the full controller:

1. Read only: confirm feedback, timing, joint order, and encoder stability.
2. Input mapping: verify `q/dq/tau_est` signs, zero offsets, and ranges in core
   convention.
3. Low torque probe: command one joint at a time with small, short torque pulses
   and confirm response direction and channel isolation.
4. Low gain hold: hold the current pose with small limits and safety enabled.
5. Slow tracking: move through shaped low-speed joint targets.
6. Full controller: enable stronger controllers, compensation, trajectory, or
   teleoperation only after the earlier stages are reliable.

This repository does not implement real-arm bring-up procedures yet. Those
should be added after the real adapter exists, because driver enable states,
emergency stop, brakes, fault codes, and communication behavior are hardware
facts that should shape the final procedure.
