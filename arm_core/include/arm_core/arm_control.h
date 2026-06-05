#ifndef ARM_CORE_ARM_CONTROL_H_
#define ARM_CORE_ARM_CONTROL_H_

#include "arm_core/arm_controller.h"
#include "arm_core/arm_feedforward.h"

int arm_control_step(arm_t *arm, const arm_reference_t *ref, arm_controller_t *ctrl);
int arm_control_step_with_feedforward(
    arm_t *arm,
    const arm_reference_t *ref,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff);

#endif
