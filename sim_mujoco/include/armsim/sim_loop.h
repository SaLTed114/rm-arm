#ifndef ARMSIM_SIM_LOOP_H_
#define ARMSIM_SIM_LOOP_H_

#include <mujoco/mujoco.h>

#include "arm_core/arm_control.h"
#include "armsim/mujoco_arm.h"

int armsim_step_once(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    const arm_config_t *config,
    arm_controller_t *controller,
    arm_state_t *state,
    arm_command_t *command);

#endif
