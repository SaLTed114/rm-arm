#ifndef ARM_CORE_JOINT_KINEMATICS_H_
#define ARM_CORE_JOINT_KINEMATICS_H_

#include "arm_core/arm.h"

#define JOINT_KINEMATICS_BODY_MAX 16u
#define JOINT_KINEMATICS_NO_BODY (-1)
#define JOINT_KINEMATICS_NO_JOINT (-1)

typedef struct {
  int8_t parent;
  int8_t joint;
  arm_real_t pos[3];
  arm_real_t axis[3];
  arm_real_t q_min_rad;
  arm_real_t q_max_rad;
} joint_kinematics_body_t;

typedef struct {
  uint8_t dof;
  uint8_t body_count;
  int8_t tool_body;
  arm_real_t tool_pos[3];
  joint_kinematics_body_t bodies[JOINT_KINEMATICS_BODY_MAX];
} joint_kinematics_params_t;

typedef struct {
  uint8_t max_iterations;
  arm_real_t damping;
  arm_real_t step_limit_rad;
  arm_real_t tolerance_m;
  const arm_real_t *posture_ref_rad;
  arm_real_t posture_gain;
} joint_ik_position_options_t;

typedef struct {
  uint8_t max_iterations;
  arm_real_t damping;
  arm_real_t step_limit_rad;
  arm_real_t pos_tolerance_m;
  arm_real_t rot_tolerance_rad;
  arm_real_t rot_weight;
  const arm_real_t *posture_ref_rad;
  arm_real_t posture_gain;
} joint_ik_pose_options_t;

int joint_kinematics_fk_position(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3]);
int joint_kinematics_fk_pose(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3],
    arm_real_t tool_rot_world[9]);
int joint_kinematics_position_jacobian(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3],
    arm_real_t jacobian[3 * ARM_DOF_MAX]);
int joint_kinematics_spatial_jacobian(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3],
    arm_real_t tool_rot_world[9],
    arm_real_t jacobian[6 * ARM_DOF_MAX]);
int joint_ik_position_solve(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    const arm_real_t target_pos_world[3],
    const joint_ik_position_options_t *options,
    arm_reference_t *ref);
int joint_ik_pose_solve(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    const arm_real_t target_pos_world[3],
    const arm_real_t target_rot_world[9],
    const joint_ik_pose_options_t *options,
    arm_reference_t *ref);

#endif
