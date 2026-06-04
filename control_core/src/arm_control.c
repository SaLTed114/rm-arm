#include "arm_core/arm_control.h"

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
    if (joint->sign == (arm_real_t)0) {
      return ARM_ERR_CONFIG;
    }
    if (joint->torque_limit_nm < (arm_real_t)0) {
      return ARM_ERR_CONFIG;
    }
  }

  return ARM_OK;
}

void arm_state_zero(arm_state_t *state, uint8_t dof) {
  if (!state) {
    return;
  }

  state->dof = arm_sanitize_dof(dof);
  state->time_s = (arm_real_t)0;
  state->dt_s = (arm_real_t)0;
  state->flags = 0u;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    state->q_rad[i] = (arm_real_t)0;
    state->dq_rad_s[i] = (arm_real_t)0;
    state->tau_est_nm[i] = (arm_real_t)0;
  }
}

void arm_command_zero(arm_command_t *command, uint8_t dof) {
  if (!command) {
    return;
  }

  command->dof = arm_sanitize_dof(dof);
  command->flags = ARM_COMMAND_TAU_VALID;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    command->tau_nm[i] = (arm_real_t)0;
  }
}

void arm_command_apply_limits(const arm_config_t *config, arm_command_t *command) {
  if (!config || !command) {
    return;
  }

  const uint8_t dof = config->dof < command->dof ? config->dof : command->dof;
  for (uint8_t i = 0u; i < dof; ++i) {
    const arm_real_t limit = config->joints[i].torque_limit_nm;
    if (limit > (arm_real_t)0) {
      command->tau_nm[i] = arm_clamp(command->tau_nm[i], -limit, limit);
    }
  }
}

int arm_control_step(
    const arm_config_t *config,
    arm_controller_t *controller,
    const arm_state_t *state,
    arm_command_t *command) {
  const int config_status = arm_config_validate(config);
  if (config_status != ARM_OK) {
    return config_status;
  }
  if (!state || !command) {
    return ARM_ERR_NULL;
  }
  if (state->dof != config->dof) {
    return ARM_ERR_DOF;
  }

  arm_command_zero(command, config->dof);

  if (controller && controller->vt && controller->vt->step) {
    const int status = controller->vt->step(controller->ctx, config, state, command);
    if (status != ARM_OK) {
      arm_command_zero(command, config->dof);
      return status;
    }
  }

  command->dof = config->dof;
  command->flags |= ARM_COMMAND_TAU_VALID;
  arm_command_apply_limits(config, command);
  return ARM_OK;
}
