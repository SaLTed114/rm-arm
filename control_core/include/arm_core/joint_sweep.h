#ifndef ARM_CORE_JOINT_SWEEP_H_
#define ARM_CORE_JOINT_SWEEP_H_

#include <stdbool.h>

#include "arm_core/arm_control.h"

typedef struct {
  arm_real_t torque_nm;
  arm_real_t pulse_s;
  arm_real_t settle_s;
} joint_sweep_params_t;

typedef struct {
  uint8_t dof;
  joint_sweep_params_t params;
  arm_real_t elapsed_s;
  int16_t active_joint;
  int8_t active_direction;
  bool complete;
} joint_sweep_t;

void joint_sweep_init(joint_sweep_t *sweep, uint8_t dof, joint_sweep_params_t params);
void joint_sweep_reset(joint_sweep_t *sweep);
arm_real_t joint_sweep_total_duration(const joint_sweep_t *sweep);
arm_controller_t joint_sweep_as_controller(joint_sweep_t *sweep);

#endif
