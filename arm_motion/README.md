# arm_motion

`arm_motion` is the portable reference-generation layer that sits next to
`arm_core`.

Rules for this directory:

- No MuJoCo, GLFW, HAL, FreeRTOS, stdio, or filesystem dependencies.
- No dynamic allocation.
- Public headers live under `include/arm_motion/`.
- `arm_motion` may depend on `arm_core` semantic types and `arm_common` math
  helpers; `arm_core` must not depend on `arm_motion`.

Current modules:

- `joint_ref_shaper` conditions manual or teleop-style joint goals into
  bounded `arm_reference_t` output.
- `joint_kinematics` provides lightweight FK, position/spatial Jacobians,
  position IK, and pose IK for Cartesian target generation.

Trajectory playback, teleoperation source selection, and socket/runtime
integration are intentionally not implemented yet. Those should build on this
module once the reference source interface is designed.
