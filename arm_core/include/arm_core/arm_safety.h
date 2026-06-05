#ifndef ARM_CORE_ARM_SAFETY_H_
#define ARM_CORE_ARM_SAFETY_H_

#include "arm_core/arm.h"

typedef struct {
  arm_real_t q_min_rad;
  arm_real_t q_max_rad;
  arm_real_t q_margin_rad;
  arm_real_t dq_limit_rad_s;
  arm_real_t torque_limit_nm;
} arm_safety_joint_params_t;

typedef struct {
  uint8_t dof;
  arm_safety_joint_params_t joints[ARM_DOF_MAX];
} arm_safety_t;

void arm_safety_init(arm_safety_t *safety, uint8_t dof);
void arm_safety_set_joint_params(arm_safety_t *safety, uint8_t joint, arm_safety_joint_params_t params);
int arm_safety_apply(const arm_safety_t *safety, const arm_state_t *state, arm_command_t *command);

#endif
