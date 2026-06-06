#ifndef ARMSIM_MUJOCO_INVERSE_DYNAMICS_FF_H_
#define ARMSIM_MUJOCO_INVERSE_DYNAMICS_FF_H_

#include <mujoco/mujoco.h>

#include "arm_core/arm_feedforward.h"
#include "armsim/mujoco_arm.h"

typedef struct {
  const mjModel *model;
  const mujoco_arm_t *arm;
  mjData *scratch;
} mujoco_inverse_dynamics_ff_t;

int mujoco_inverse_dynamics_ff_init(
    mujoco_inverse_dynamics_ff_t *ff,
    const mjModel *model,
    const mujoco_arm_t *arm);
void mujoco_inverse_dynamics_ff_free(mujoco_inverse_dynamics_ff_t *ff);
arm_feedforward_t mujoco_inverse_dynamics_ff_as_feedforward(mujoco_inverse_dynamics_ff_t *ff);

int mujoco_inverse_dynamics_compute_tau(
    const mjModel *model,
    mjData *scratch,
    const mujoco_arm_t *arm,
    const arm_state_t *state,
    const arm_reference_t *ref,
    arm_real_t tau_nm[ARM_DOF_MAX]);

#endif
