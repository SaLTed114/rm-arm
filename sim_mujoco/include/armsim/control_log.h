#ifndef ARMSIM_CONTROL_LOG_H_
#define ARMSIM_CONTROL_LOG_H_

#include <stdbool.h>
#include <stdio.h>

#include "arm_core/arm.h"
#include "armsim/mujoco_arm.h"

typedef struct {
  FILE *file;
  uint8_t dof;
} control_log_t;

typedef struct {
  bool gravity_on;
  bool gravity_ff_on;
  bool contacts_on;
  bool harsh_on;
} control_log_flags_t;

bool control_log_open(control_log_t *log, const char *path, uint8_t dof);
void control_log_close(control_log_t *log);
bool control_log_write_header(control_log_t *log);
bool control_log_write_step(
    control_log_t *log,
    const mjData *data,
    const mujoco_arm_t *arm,
    const control_log_flags_t *flags,
    const arm_state_t *measured_state,
    const arm_state_t *filtered_state,
    const arm_reference_t *ref,
    const arm_real_t tau_ff_gravity[ARM_DOF_MAX],
    const arm_command_t *command);

#endif
