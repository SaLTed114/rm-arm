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

  status = run_ff(arm, ref, ff);
  if (status != ARM_OK) {
    arm_clear_command(arm);
    return status;
  }

  arm->command.dof = arm->config.dof;
  arm_limit_command(arm);
  return ARM_OK;
}

int arm_control_step(arm_t *arm, const arm_reference_t *ref, arm_controller_t *ctrl) {
  return arm_control_step_with_feedforward(arm, ref, ctrl, NULL);
}
