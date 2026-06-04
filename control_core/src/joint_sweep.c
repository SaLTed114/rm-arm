#include "arm_core/joint_sweep.h"

#include "arm_core/arm_math.h"

static int joint_sweep_step(
    void *ctx,
    const arm_config_t *config,
    const arm_state_t *state,
    arm_command_t *command) {
  joint_sweep_t *sweep = (joint_sweep_t *)ctx;
  if (!sweep || !config || !state || !command) {
    return ARM_ERR_NULL;
  }
  if (sweep->dof == 0u || sweep->dof > ARM_DOF_MAX || sweep->dof != config->dof) {
    return ARM_ERR_DOF;
  }

  arm_command_zero(command, config->dof);
  sweep->active_joint = -1;
  sweep->active_direction = 0;

  const arm_real_t pulse_s = sweep->params.pulse_s;
  const arm_real_t settle_s = sweep->params.settle_s;
  if (pulse_s <= (arm_real_t)0 || settle_s < (arm_real_t)0) {
    sweep->complete = true;
    return ARM_ERR_CONFIG;
  }

  const arm_real_t per_joint_s = pulse_s + settle_s + pulse_s + settle_s;
  const arm_real_t total_s = per_joint_s * (arm_real_t)sweep->dof;

  if (sweep->elapsed_s >= total_s) {
    sweep->complete = true;
    return ARM_OK;
  }

  uint8_t active_joint = 0u;
  arm_real_t local_t = sweep->elapsed_s;
  while (local_t >= per_joint_s && active_joint + 1u < sweep->dof) {
    local_t -= per_joint_s;
    ++active_joint;
  }

  arm_real_t tau = (arm_real_t)0;
  if (local_t < pulse_s) {
    tau = sweep->params.torque_nm;
    sweep->active_direction = +1;
  } else if (local_t < pulse_s + settle_s) {
    tau = (arm_real_t)0;
  } else if (local_t < pulse_s + settle_s + pulse_s) {
    tau = -sweep->params.torque_nm;
    sweep->active_direction = -1;
  } else {
    tau = (arm_real_t)0;
  }

  sweep->active_joint = (tau == (arm_real_t)0) ? -1 : (int16_t)active_joint;
  command->tau_nm[active_joint] = tau;
  command->flags |= ARM_COMMAND_TAU_VALID;

  sweep->elapsed_s += state->dt_s;
  if (sweep->elapsed_s >= total_s) {
    sweep->complete = true;
  }

  return ARM_OK;
}

static void joint_sweep_vt_reset(void *ctx) {
  joint_sweep_reset((joint_sweep_t *)ctx);
}

static const arm_controller_vtable_t JOINT_SWEEP_VTABLE = {
  joint_sweep_vt_reset,
  joint_sweep_step,
};

void joint_sweep_init(joint_sweep_t *sweep, uint8_t dof, joint_sweep_params_t params) {
  if (!sweep) {
    return;
  }

  sweep->dof = arm_sanitize_dof(dof);
  sweep->params = params;
  joint_sweep_reset(sweep);
}

void joint_sweep_reset(joint_sweep_t *sweep) {
  if (!sweep) {
    return;
  }
  sweep->elapsed_s = (arm_real_t)0;
  sweep->active_joint = -1;
  sweep->active_direction = 0;
  sweep->complete = false;
}

arm_real_t joint_sweep_total_duration(const joint_sweep_t *sweep) {
  if (!sweep) {
    return (arm_real_t)0;
  }
  return ((arm_real_t)2 * sweep->params.pulse_s + (arm_real_t)2 * sweep->params.settle_s) *
         (arm_real_t)sweep->dof;
}

arm_controller_t joint_sweep_as_controller(joint_sweep_t *sweep) {
  arm_controller_t controller;
  controller.vt = &JOINT_SWEEP_VTABLE;
  controller.ctx = sweep;
  return controller;
}
