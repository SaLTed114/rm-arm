#ifndef ARM_CORE_JOINT_REF_PLANNER_H_
#define ARM_CORE_JOINT_REF_PLANNER_H_

#include <stdbool.h>

#include "arm_core/arm.h"

typedef struct {
  arm_real_t q_min_rad[ARM_DOF_MAX];
  arm_real_t q_max_rad[ARM_DOF_MAX];
  arm_real_t dq_limit_rad_s[ARM_DOF_MAX];
  arm_real_t ddq_limit_rad_s2[ARM_DOF_MAX];
} joint_ref_planner_params_t;

typedef struct {
  uint8_t dof;
  joint_ref_planner_params_t params;
  arm_real_t q_goal_rad[ARM_DOF_MAX];
  arm_real_t q_ref_rad[ARM_DOF_MAX];
  arm_real_t dq_ref_rad_s[ARM_DOF_MAX];
  bool initialized;
} joint_ref_planner_t;

void joint_ref_planner_init(
    joint_ref_planner_t *planner,
    uint8_t dof,
    const joint_ref_planner_params_t *params);
void joint_ref_planner_reset_to_state(joint_ref_planner_t *planner, const arm_state_t *state);
void joint_ref_planner_set_params(joint_ref_planner_t *planner, const joint_ref_planner_params_t *params);
int joint_ref_planner_step(
    joint_ref_planner_t *planner,
    const arm_state_t *state,
    const arm_reference_t *goal_ref,
    arm_reference_t *planned_ref);

#endif
