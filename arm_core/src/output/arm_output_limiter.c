#include "arm_core/arm_output_limiter.h"

#include "arm_common/arm_math.h"

void arm_output_limiter_init(arm_output_limiter_t *limiter, uint8_t dof) {
  if (!limiter) return;

  limiter->dof = arm_sanitize_dof(dof);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    limiter->joints[i].tau_rate_limit_nm_s = ARM_REAL_ZERO;
    limiter->last_tau_ff_nm[i] = ARM_REAL_ZERO;
  }
  limiter->initialized = true;
}

void arm_output_limiter_reset(arm_output_limiter_t *limiter) {
  if (!limiter) return;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    limiter->last_tau_ff_nm[i] = ARM_REAL_ZERO;
  }
  limiter->initialized = true;
}

void arm_output_limiter_reset_to_command(arm_output_limiter_t *limiter, const arm_command_t *command) {
  if (!limiter || !command) return;
  if (!arm_dof_matches(limiter->dof, command->dof)) return;

  for (uint8_t i = 0u; i < limiter->dof; ++i) {
    limiter->last_tau_ff_nm[i] = command->tau_ff_nm[i];
  }
  for (uint8_t i = limiter->dof; i < ARM_DOF_MAX; ++i) {
    limiter->last_tau_ff_nm[i] = ARM_REAL_ZERO;
  }
  limiter->initialized = true;
}

void arm_output_limiter_set_joint_params(
    arm_output_limiter_t *limiter,
    uint8_t joint,
    arm_output_limiter_joint_params_t params) {
  if (!limiter || joint >= limiter->dof) return;
  limiter->joints[joint] = params;
}

int arm_output_limiter_apply(
    arm_output_limiter_t *limiter,
    const arm_state_t *state,
    arm_command_t *command) {
  if (!limiter || !state || !command) return ARM_ERR_NULL;
  if (!arm_dof_matches(limiter->dof, command->dof)) return ARM_ERR_DOF;
  if (!arm_dof_matches(limiter->dof, state->dof)) return ARM_ERR_DOF;

  if (!limiter->initialized) {
    arm_output_limiter_reset(limiter);
  }

  const arm_real_t dt_s = state->dt_s;
  for (uint8_t i = 0u; i < limiter->dof; ++i) {
    arm_real_t tau = command->tau_ff_nm[i];
    const arm_real_t rate_limit = limiter->joints[i].tau_rate_limit_nm_s;
    if (dt_s > ARM_REAL_ZERO && rate_limit > ARM_REAL_ZERO) {
      const arm_real_t max_delta = rate_limit * dt_s;
      tau = limiter->last_tau_ff_nm[i] +
            arm_clamp(tau - limiter->last_tau_ff_nm[i], -max_delta, max_delta);
      command->tau_ff_nm[i] = tau;
    }
    limiter->last_tau_ff_nm[i] = tau;
  }
  return ARM_OK;
}
