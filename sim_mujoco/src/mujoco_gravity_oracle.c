#include "armsim/mujoco_gravity_oracle.h"

#include "arm_common/arm_math.h"

int mujoco_gravity_oracle_init(const mjModel *model, mujoco_gravity_oracle_t *oracle) {
  if (!model || !oracle) return ARM_ERR_NULL;

  oracle->data = mj_makeData(model);
  return oracle->data ? ARM_OK : ARM_ERR_CONFIG;
}

void mujoco_gravity_oracle_free(mujoco_gravity_oracle_t *oracle) {
  if (!oracle) return;

  if (oracle->data) {
    mj_deleteData(oracle->data);
    oracle->data = NULL;
  }
}

int mujoco_gravity_oracle_apply_to_ref(
    const mjModel *model,
    const mjData *data,
    const mujoco_arm_t *arm,
    const arm_state_t *state,
    arm_reference_t *ref,
    mujoco_gravity_oracle_t *oracle) {
  if (!model || !data || !arm || !arm->config || !state || !ref || !oracle || !oracle->data) {
    return ARM_ERR_NULL;
  }
  if (ref->dof != arm->dof || state->dof != arm->dof) return ARM_ERR_DOF;

  mjData *scratch = oracle->data;
  mju_copy(scratch->qpos, data->qpos, (int)model->nq);
  mju_zero(scratch->qvel, (int)model->nv);
  mju_zero(scratch->qacc, (int)model->nv);
  mju_zero(scratch->ctrl, (int)model->nu);
  mju_zero(scratch->qfrc_applied, (int)model->nv);
  mju_zero(scratch->xfrc_applied, 6 * (int)model->nbody);

  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const arm_joint_config_t *joint = &arm->config->joints[i];
    const arm_real_t q_core = (ref->flags & ARM_REFERENCE_Q_VALID) ? ref->q_ref_rad[i] : state->q_rad[i];
    scratch->qpos[arm->joint_qpos_addr[i]] = (mjtNum)(joint->q_offset_rad + joint->sign * q_core);
  }

  mj_forward(model, scratch);

  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const arm_joint_config_t *joint = &arm->config->joints[i];
    const arm_real_t tau = joint->sign * ARM_REAL(scratch->qfrc_bias[arm->joint_dof_addr[i]]);
    const arm_real_t existing = (ref->flags & ARM_REFERENCE_TAU_FF_VALID) ? ref->tau_ff_nm[i] : ARM_REAL_ZERO;
    ref->tau_ff_nm[i] = existing + tau;
  }
  ref->flags |= ARM_REFERENCE_TAU_FF_VALID;
  return ARM_OK;
}
