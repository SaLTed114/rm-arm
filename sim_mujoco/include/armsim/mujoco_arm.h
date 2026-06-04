#ifndef ARMSIM_MUJOCO_ARM_H_
#define ARMSIM_MUJOCO_ARM_H_

#include <stddef.h>

#include <mujoco/mujoco.h>

#include "arm_core/arm_types.h"

typedef struct {
  uint8_t dof;
  int joint_ids[ARM_DOF_MAX];
  int joint_qpos_addr[ARM_DOF_MAX];
  int joint_dof_addr[ARM_DOF_MAX];
  int actuator_ids[ARM_DOF_MAX];
  const arm_config_t *config;
} mujoco_arm_t;

int mujoco_arm_bind(
    const mjModel *model,
    const arm_config_t *config,
    mujoco_arm_t *arm,
    char *error,
    size_t error_size);

void mujoco_arm_read_state(
    const mjData *data,
    const mujoco_arm_t *arm,
    arm_real_t time_s,
    arm_real_t dt_s,
    arm_state_t *state);

void mujoco_arm_write_command(
    mjData *data,
    const mujoco_arm_t *arm,
    const arm_command_t *command);

#endif
