#include "armsim/mujoco_inverse_dynamics_ff.h"

#include "arm_common/arm_math.h"

static void clear_scratch(const mjModel *model, mjData *scratch) {
  mju_zero(scratch->qvel, model->nv);
  mju_zero(scratch->qacc, model->nv);
  mju_zero(scratch->ctrl, model->nu);
  mju_zero(scratch->qfrc_applied, model->nv);
  mju_zero(scratch->xfrc_applied, 6 * model->nbody);
}

int mujoco_inverse_dynamics_compute_tau(
    const mjModel *model,
    mjData *scratch,
    const mujoco_arm_t *arm,
    const arm_state_t *state,
    const arm_reference_t *ref,
    arm_real_t tau_nm[ARM_DOF_MAX]) {
  if (!model || !scratch || !arm || !arm->config || !state || !tau_nm) return ARM_ERR_NULL;
  if (!arm_dof_matches(arm->dof, state->dof)) return ARM_ERR_DOF;
  if (ref && ref->dof != arm->dof) return ARM_ERR_DOF;

  clear_scratch(model, scratch);

  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const arm_joint_config_t *joint = &arm->config->joints[i];
    const arm_real_t q_core = state->q_rad[i];
    const arm_real_t dq_core = state->dq_rad_s[i];
    const arm_real_t ddq_core =
        (ref && (ref->flags & ARM_REFERENCE_DDQ_VALID)) ? ref->ddq_ref_rad_s2[i] : ARM_REAL_ZERO;
    scratch->qpos[arm->joint_qpos_addr[i]] = (mjtNum)(joint->q_offset_rad + joint->sign * q_core);
    scratch->qvel[arm->joint_dof_addr[i]] = (mjtNum)(joint->sign * dq_core);
    scratch->qacc[arm->joint_dof_addr[i]] = (mjtNum)(joint->sign * ddq_core);
  }

  mj_inverse(model, scratch);

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    tau_nm[i] = ARM_REAL_ZERO;
  }
  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const arm_joint_config_t *joint = &arm->config->joints[i];
    tau_nm[i] = joint->sign * ARM_REAL(scratch->qfrc_inverse[arm->joint_dof_addr[i]]);
  }
  return ARM_OK;
}

static int mujoco_inverse_dynamics_ff_step(void *ctx, arm_t *core, const arm_reference_t *ref) {
  mujoco_inverse_dynamics_ff_t *ff = (mujoco_inverse_dynamics_ff_t *)ctx;
  if (!ff || !core) return ARM_ERR_NULL;

  arm_real_t tau_nm[ARM_DOF_MAX];
  const int status = mujoco_inverse_dynamics_compute_tau(ff->model, ff->scratch, ff->arm, &core->state, ref, tau_nm);
  if (status != ARM_OK) return status;

  for (uint8_t i = 0u; i < core->config.dof; ++i) {
    core->command.tau_ff_nm[i] += tau_nm[i];
  }
  core->command.flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static const arm_feedforward_vtable_t MUJOCO_INVERSE_DYNAMICS_FF_VTABLE = {
  NULL,
  mujoco_inverse_dynamics_ff_step,
};

int mujoco_inverse_dynamics_ff_init(
    mujoco_inverse_dynamics_ff_t *ff,
    const mjModel *model,
    const mujoco_arm_t *arm) {
  if (!ff || !model || !arm) return ARM_ERR_NULL;
  ff->model = model;
  ff->arm = arm;
  ff->scratch = mj_makeData(model);
  return ff->scratch ? ARM_OK : ARM_ERR_CONFIG;
}

void mujoco_inverse_dynamics_ff_free(mujoco_inverse_dynamics_ff_t *ff) {
  if (!ff) return;
  if (ff->scratch) {
    mj_deleteData(ff->scratch);
    ff->scratch = NULL;
  }
  ff->model = NULL;
  ff->arm = NULL;
}

arm_feedforward_t mujoco_inverse_dynamics_ff_as_feedforward(mujoco_inverse_dynamics_ff_t *ff) {
  arm_feedforward_t feedforward;
  feedforward.vt = &MUJOCO_INVERSE_DYNAMICS_FF_VTABLE;
  feedforward.ctx = ff;
  return feedforward;
}
