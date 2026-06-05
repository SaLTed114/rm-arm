#include "armsim/mujoco_arm.h"

#include <stdio.h>

#include "arm_core/arm_control.h"
#include "arm_core/arm_math.h"

static int set_error(char *error, size_t error_size, const char *message, const char *name) {
  if (error && error_size > 0u) {
    if (name) {
      (void)snprintf(error, error_size, "%s: %s", message, name);
    } else {
      (void)snprintf(error, error_size, "%s", message);
    }
  }
  return ARM_ERR_CONFIG;
}

int mujoco_arm_bind(
    const mjModel *model,
    const arm_config_t *config,
    mujoco_arm_t *arm,
    char *error,
    size_t error_size) {
  if (!model || !config || !arm) {
    return set_error(error, error_size, "null argument", NULL);
  }

  const int config_status = arm_config_validate(config);
  if (config_status != ARM_OK) {
    return set_error(error, error_size, "invalid arm config", NULL);
  }

  *arm = (mujoco_arm_t){0};
  arm->dof = config->dof;
  arm->config = config;

  for (uint8_t i = 0u; i < config->dof; ++i) {
    const arm_joint_config_t *joint = &config->joints[i];
    if (!joint->joint_name) {
      return set_error(error, error_size, "joint name missing", NULL);
    }
    if (!joint->actuator_name) {
      return set_error(error, error_size, "actuator name missing", NULL);
    }

    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint->joint_name);
    if (joint_id < 0) {
      return set_error(error, error_size, "joint not found", joint->joint_name);
    }

    const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, joint->actuator_name);
    if (actuator_id < 0) {
      return set_error(error, error_size, "actuator not found", joint->actuator_name);
    }

    arm->joint_ids[i] = joint_id;
    arm->joint_qpos_addr[i] = model->jnt_qposadr[joint_id];
    arm->joint_dof_addr[i] = model->jnt_dofadr[joint_id];
    arm->actuator_ids[i] = actuator_id;
  }

  return ARM_OK;
}

void mujoco_arm_read_state(
    const mjData *data,
    const mujoco_arm_t *arm,
    arm_real_t time_s,
    arm_real_t dt_s,
    arm_state_t *state) {
  if (!data || !arm || !arm->config || !state) {
    return;
  }

  arm_state_zero(state, arm->dof);
  state->time_s = time_s;
  state->dt_s = dt_s;
  state->flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;

  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const arm_joint_config_t *joint = &arm->config->joints[i];
    const arm_real_t sign = joint->sign;
    state->q_rad[i] = sign * (ARM_REAL(data->qpos[arm->joint_qpos_addr[i]]) - joint->q_offset_rad);
    state->dq_rad_s[i] = sign * ARM_REAL(data->qvel[arm->joint_dof_addr[i]]);
  }
}

void mujoco_arm_write_command(
    mjData *data,
    const mujoco_arm_t *arm,
    const arm_command_t *command) {
  if (!data || !arm || !arm->config || !command) {
    return;
  }

  const uint8_t dof = arm->dof < command->dof ? arm->dof : command->dof;
  for (uint8_t i = 0u; i < dof; ++i) {
    const arm_joint_config_t *joint = &arm->config->joints[i];
    arm_real_t tau = command->tau_ff_nm[i];
    if (joint->torque_limit_nm > ARM_REAL_ZERO) {
      tau = arm_clamp(tau, -joint->torque_limit_nm, joint->torque_limit_nm);
    }
    data->ctrl[arm->actuator_ids[i]] = (mjtNum)(joint->sign * tau);
  }
}
