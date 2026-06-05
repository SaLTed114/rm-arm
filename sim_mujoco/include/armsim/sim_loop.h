#ifndef ARMSIM_SIM_LOOP_H_
#define ARMSIM_SIM_LOOP_H_

#include <mujoco/mujoco.h>

#include "arm_core/arm_safety.h"
#include "arm_core/arm_control.h"
#include "armsim/mujoco_arm.h"

int armsim_step_once(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl);
int armsim_step_once_with_feedforward(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff);
int armsim_step_once_with_state(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_state_t *state,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl);
int armsim_step_once_with_state_and_feedforward(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_state_t *state,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff);

#endif
