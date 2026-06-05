#include "arm_core/arm.h"

#include "arm_core/arm_math.h"

int arm_config_validate(const arm_config_t *config) {
  if (!config) {
    return ARM_ERR_NULL;
  }
  if (config->dof == 0u || config->dof > ARM_DOF_MAX) {
    return ARM_ERR_DOF;
  }

  for (uint8_t i = 0u; i < config->dof; ++i) {
    const arm_joint_config_t *joint = &config->joints[i];
    if (joint->sign == ARM_REAL_ZERO) {
      return ARM_ERR_CONFIG;
    }
    if (joint->torque_limit_nm < ARM_REAL_ZERO) {
      return ARM_ERR_CONFIG;
    }
  }

  return ARM_OK;
}

int arm_init(arm_t *arm, const arm_config_t *config) {
  if (!arm || !config) {
    return ARM_ERR_NULL;
  }

  const int status = arm_config_validate(config);
  if (status != ARM_OK) {
    return status;
  }

  arm->config = *config;
  arm_reset(arm);
  return ARM_OK;
}

void arm_reset(arm_t *arm) {
  if (!arm) {
    return;
  }
  arm_clear_state(arm);
  arm_clear_command(arm);
}

void arm_clear_state(arm_t *arm) {
  if (!arm) {
    return;
  }
  arm_state_zero(&arm->state, arm->config.dof);
}

void arm_clear_command(arm_t *arm) {
  if (!arm) {
    return;
  }
  arm_command_zero(&arm->command, arm->config.dof);
}

void arm_limit_command(arm_t *arm) {
  if (!arm) {
    return;
  }
  arm_command_apply_limits(&arm->config, &arm->command);
}

void arm_state_zero(arm_state_t *state, uint8_t dof) {
  if (!state) {
    return;
  }

  state->dof = arm_sanitize_dof(dof);
  state->time_s = ARM_REAL_ZERO;
  state->dt_s = ARM_REAL_ZERO;
  state->flags = 0u;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    state->q_rad[i] = ARM_REAL_ZERO;
    state->dq_rad_s[i] = ARM_REAL_ZERO;
    state->tau_est_nm[i] = ARM_REAL_ZERO;
  }
}

void arm_command_zero(arm_command_t *command, uint8_t dof) {
  if (!command) {
    return;
  }

  command->dof = arm_sanitize_dof(dof);
  command->flags = 0u;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    command->q_d_rad[i] = ARM_REAL_ZERO;
    command->dq_d_rad_s[i] = ARM_REAL_ZERO;
    command->kp[i] = ARM_REAL_ZERO;
    command->kd[i] = ARM_REAL_ZERO;
    command->tau_ff_nm[i] = ARM_REAL_ZERO;
  }
}

void arm_reference_zero(arm_reference_t *ref, uint8_t dof) {
  if (!ref) {
    return;
  }

  ref->dof = arm_sanitize_dof(dof);
  ref->flags = 0u;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    ref->q_ref_rad[i] = ARM_REAL_ZERO;
    ref->dq_ref_rad_s[i] = ARM_REAL_ZERO;
    ref->tau_ff_nm[i] = ARM_REAL_ZERO;
  }
}

void arm_command_apply_limits(const arm_config_t *config, arm_command_t *command) {
  if (!config || !command) {
    return;
  }

  const uint8_t dof = config->dof < command->dof ? config->dof : command->dof;
  for (uint8_t i = 0u; i < dof; ++i) {
    const arm_real_t limit = config->joints[i].torque_limit_nm;
    if (limit > ARM_REAL_ZERO) {
      command->tau_ff_nm[i] = arm_clamp(command->tau_ff_nm[i], -limit, limit);
    }
  }
}
