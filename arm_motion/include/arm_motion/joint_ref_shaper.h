#ifndef ARM_MOTION_JOINT_REF_SHAPER_H_
#define ARM_MOTION_JOINT_REF_SHAPER_H_

#include <stdbool.h>

#include "arm_core/arm.h"

typedef struct {
  arm_real_t q_min_rad[ARM_DOF_MAX];
  arm_real_t q_max_rad[ARM_DOF_MAX];
  arm_real_t dq_limit_rad_s[ARM_DOF_MAX];
  arm_real_t ddq_limit_rad_s2[ARM_DOF_MAX];
  arm_real_t dddq_limit_rad_s3[ARM_DOF_MAX];
} joint_ref_shaper_params_t;

typedef struct {
  uint8_t dof;
  joint_ref_shaper_params_t params;
  arm_real_t q_goal_rad[ARM_DOF_MAX];
  arm_real_t q_ref_rad[ARM_DOF_MAX];
  arm_real_t dq_ref_rad_s[ARM_DOF_MAX];
  arm_real_t ddq_ref_rad_s2[ARM_DOF_MAX];
  bool initialized;
} joint_ref_shaper_t;

void joint_ref_shaper_init(
    joint_ref_shaper_t *shaper,
    uint8_t dof,
    const joint_ref_shaper_params_t *params);
void joint_ref_shaper_reset_to_state(joint_ref_shaper_t *shaper, const arm_state_t *state);
void joint_ref_shaper_set_params(joint_ref_shaper_t *shaper, const joint_ref_shaper_params_t *params);
int joint_ref_shaper_step(
    joint_ref_shaper_t *shaper,
    const arm_state_t *state,
    const arm_reference_t *goal_ref,
    arm_reference_t *shaped_ref);

#endif
