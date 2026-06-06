#ifndef ARM_CORE_ARM_OUTPUT_LIMITER_H_
#define ARM_CORE_ARM_OUTPUT_LIMITER_H_

#include <stdbool.h>

#include "arm_core/arm_types.h"

typedef struct {
  arm_real_t tau_rate_limit_nm_s;
} arm_output_limiter_joint_params_t;

typedef struct {
  uint8_t dof;
  arm_output_limiter_joint_params_t joints[ARM_DOF_MAX];
  arm_real_t last_tau_ff_nm[ARM_DOF_MAX];
  bool initialized;
} arm_output_limiter_t;

void arm_output_limiter_init(arm_output_limiter_t *limiter, uint8_t dof);
void arm_output_limiter_reset(arm_output_limiter_t *limiter);
void arm_output_limiter_reset_to_command(arm_output_limiter_t *limiter, const arm_command_t *command);
void arm_output_limiter_set_joint_params(
    arm_output_limiter_t *limiter,
    uint8_t joint,
    arm_output_limiter_joint_params_t params);
int arm_output_limiter_apply(
    arm_output_limiter_t *limiter,
    const arm_state_t *state,
    arm_command_t *command);

#endif
