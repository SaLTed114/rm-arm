#ifndef ARMSIM_MUJOCO_GRAVITY_ORACLE_H_
#define ARMSIM_MUJOCO_GRAVITY_ORACLE_H_

#include <mujoco/mujoco.h>

#include "arm_core/arm_types.h"
#include "armsim/mujoco_arm.h"

typedef struct {
  mjData *data;
} mujoco_gravity_oracle_t;

int mujoco_gravity_oracle_init(const mjModel *model, mujoco_gravity_oracle_t *oracle);
void mujoco_gravity_oracle_free(mujoco_gravity_oracle_t *oracle);
int mujoco_gravity_oracle_apply_to_ref(
    const mjModel *model,
    const mjData *data,
    const mujoco_arm_t *arm,
    const arm_state_t *state,
    arm_reference_t *ref,
    mujoco_gravity_oracle_t *oracle);

#endif
