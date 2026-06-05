#ifndef ARM_CORE_JOINT_STATE_FILTER_H_
#define ARM_CORE_JOINT_STATE_FILTER_H_

#include "arm_core/arm.h"

typedef struct {
  arm_real_t q_time_constant_s;
  arm_real_t dq_time_constant_s;
  bool use_dq_from_q_diff;
} joint_state_filter_params_t;

typedef struct {
  uint8_t dof;
  joint_state_filter_params_t params[ARM_DOF_MAX];
  arm_real_t q_filtered_rad[ARM_DOF_MAX];
  arm_real_t dq_filtered_rad_s[ARM_DOF_MAX];
  arm_real_t last_q_rad[ARM_DOF_MAX];
  bool has_last_q[ARM_DOF_MAX];
  bool initialized;
} joint_state_filter_t;

void joint_state_filter_init(
    joint_state_filter_t *filter,
    uint8_t dof,
    const joint_state_filter_params_t *params);
void joint_state_filter_reset(joint_state_filter_t *filter);
void joint_state_filter_reset_to_state(joint_state_filter_t *filter, const arm_state_t *measured_state);
void joint_state_filter_set_params(
    joint_state_filter_t *filter,
    uint8_t joint,
    joint_state_filter_params_t params);
int joint_state_filter_step(
    joint_state_filter_t *filter,
    const arm_state_t *measured_state,
    arm_state_t *filtered_state);

#endif
