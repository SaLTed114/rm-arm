#include "arm_core/arm_control.h"

int arm_control_step(arm_t *arm, const arm_reference_t *ref, arm_controller_t *controller) {
  if (!arm) {
    return ARM_ERR_NULL;
  }

  const arm_config_t *config = &arm->config;
  const arm_state_t *state = &arm->state;
  arm_command_t *command = &arm->command;

  const int config_status = arm_config_validate(config);
  if (config_status != ARM_OK) {
    return config_status;
  }
  if (state->dof != config->dof) {
    return ARM_ERR_DOF;
  }
  if (ref && ref->dof != config->dof) {
    return ARM_ERR_DOF;
  }

  arm_clear_command(arm);

  if (controller && controller->vt && controller->vt->step) {
    const int status = controller->vt->step(controller->ctx, arm, ref);
    if (status != ARM_OK) {
      arm_clear_command(arm);
      return status;
    }
  }

  command->dof = config->dof;
  arm_limit_command(arm);
  return ARM_OK;
}
