#include "armsim/sim_loop.h"

int armsim_step_once(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *controller) {
  if (!model || !data || !arm || !core) {
    return ARM_ERR_NULL;
  }

  mujoco_arm_read_state(data, arm, ARM_REAL(data->time), ARM_REAL(model->opt.timestep), &core->state);
  const int status = arm_control_step(core, ref, controller);
  if (status != ARM_OK) {
    return status;
  }
  if (safety) {
    const int safety_status = arm_safety_apply(safety, &core->state, &core->command);
    if (safety_status != ARM_OK) {
      return safety_status;
    }
  }
  mujoco_arm_write_command(data, arm, &core->command);
  mj_step(model, data);
  return ARM_OK;
}
