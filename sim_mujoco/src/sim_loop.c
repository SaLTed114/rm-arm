#include "armsim/sim_loop.h"

int armsim_step_once(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    arm_controller_t *controller) {
  if (!model || !data || !arm || !core) {
    return ARM_ERR_NULL;
  }

  mujoco_arm_read_state(data, arm, (arm_real_t)data->time, (arm_real_t)model->opt.timestep, &core->state);
  const int status = arm_control_step(core, controller);
  if (status != ARM_OK) {
    return status;
  }
  mujoco_arm_write_command(data, arm, &core->command);
  mj_step(model, data);
  return ARM_OK;
}
