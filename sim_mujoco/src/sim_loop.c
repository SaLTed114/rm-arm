#include "armsim/sim_loop.h"

int armsim_step_once(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    const arm_config_t *config,
    arm_controller_t *controller,
    arm_state_t *state,
    arm_command_t *command) {
  if (!model || !data || !arm || !config || !state || !command) {
    return ARM_ERR_NULL;
  }

  mujoco_arm_read_state(data, arm, (arm_real_t)data->time, (arm_real_t)model->opt.timestep, state);
  const int status = arm_control_step(config, controller, state, command);
  if (status != ARM_OK) {
    return status;
  }
  mujoco_arm_write_command(data, arm, command);
  mj_step(model, data);
  return ARM_OK;
}
