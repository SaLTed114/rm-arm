#include "arm_core/arm_control.h"

static int validate_step_io(const arm_t *arm, const arm_reference_t *ref) {
  if (!arm) return ARM_ERR_NULL;

  const arm_config_t *config = &arm->config;
  const arm_state_t *state = &arm->state;
  const int config_status = arm_config_validate(config);
  if (config_status != ARM_OK) return config_status;
  if (state->dof != config->dof) return ARM_ERR_DOF;
  if (ref && ref->dof != config->dof) return ARM_ERR_DOF;
  return ARM_OK;
}

static int run_ctrl(arm_t *arm, const arm_reference_t *ref, arm_controller_t *ctrl) {
  if (!ctrl || !ctrl->vt || !ctrl->vt->step) return ARM_OK;
  return ctrl->vt->step(ctrl->ctx, arm, ref);
}

static int run_ff(arm_t *arm, const arm_reference_t *ref, arm_feedforward_t *ff) {
  if (!ff || !ff->vt || !ff->vt->step) return ARM_OK;
  return ff->vt->step(ff->ctx, arm, ref);
}

static void capture_feedback_torque(const arm_t *arm, arm_real_t tau_fb_nm[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    tau_fb_nm[i] = ARM_REAL_ZERO;
  }
  for (uint8_t i = 0u; i < arm->config.dof; ++i) {
    tau_fb_nm[i] = arm->command.tau_ff_nm[i];
  }
}

static void publish_torque_split(arm_t *arm, const arm_real_t tau_fb_nm[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < arm->config.dof; ++i) {
    arm->command.tau_fb_nm[i] = tau_fb_nm[i];
    arm->command.tau_model_ff_nm[i] = arm->command.tau_ff_nm[i] - tau_fb_nm[i];
  }
  arm->command.flags |= ARM_COMMAND_TAU_FB_VALID | ARM_COMMAND_TAU_MODEL_VALID;
}

int arm_control_step_with_feedforward(
    arm_t *arm,
    const arm_reference_t *ref,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff) {
  const int io_status = validate_step_io(arm, ref);
  if (io_status != ARM_OK) return io_status;

  arm_clear_command(arm);

  int status = run_ctrl(arm, ref, ctrl);
  if (status != ARM_OK) {
    arm_clear_command(arm);
    return status;
  }

  arm_real_t tau_fb_nm[ARM_DOF_MAX];
  capture_feedback_torque(arm, tau_fb_nm);

  status = run_ff(arm, ref, ff);
  if (status != ARM_OK) {
    arm_clear_command(arm);
    return status;
  }
  publish_torque_split(arm, tau_fb_nm);

  arm->command.dof = arm->config.dof;
  arm_limit_command(arm);
  return ARM_OK;
}

int arm_control_step(arm_t *arm, const arm_reference_t *ref, arm_controller_t *ctrl) {
  return arm_control_step_with_feedforward(arm, ref, ctrl, NULL);
}
