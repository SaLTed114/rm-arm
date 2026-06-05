#include "armsim/sim_loop.h"

int armsim_step_once(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl) {
  return armsim_step_once_with_feedforward(model, data, arm, core, ref, safety, ctrl, NULL);
}

int armsim_step_once_with_feedforward(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff) {
  if (!model || !data || !arm || !core) {
    return ARM_ERR_NULL;
  }

  arm_state_t state;
  mujoco_arm_read_state(data, arm, ARM_REAL(data->time), ARM_REAL(model->opt.timestep), &state);
  return armsim_step_once_with_state_and_feedforward(
      model, data, arm, core, &state, ref, safety, ctrl, ff);
}

int armsim_step_once_with_state(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_state_t *state,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl) {
  return armsim_step_once_with_state_and_feedforward(model, data, arm, core, state, ref, safety, ctrl, NULL);
}

int armsim_step_once_with_state_and_feedforward(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_state_t *state,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff) {
  if (!model || !data || !arm || !core || !state) {
    return ARM_ERR_NULL;
  }

  core->state = *state;
  const int status = arm_control_step_with_feedforward(core, ref, ctrl, ff);
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
