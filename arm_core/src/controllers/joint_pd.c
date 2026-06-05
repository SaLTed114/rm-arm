#include "arm_core/joint_pd.h"

#include "arm_core/arm_math.h"

static int joint_pd_step(void *ctx, arm_t *arm, const arm_reference_t *ref) {
  joint_pd_t *pd = (joint_pd_t *)ctx;
  if (!pd || !arm) return ARM_ERR_NULL;

  const arm_config_t *config = &arm->config;
  const arm_state_t *state = &arm->state;
  arm_command_t *command = &arm->command;

  if (!arm_dof_matches(config->dof, pd->dof) || !arm_dof_matches(config->dof, state->dof)) return ARM_ERR_DOF;
  if (!ref || ref->dof != config->dof) return ARM_ERR_DOF;

  arm_command_zero(command, config->dof);

  for (uint8_t i = 0u; i < pd->dof; ++i) {
    const joint_pd_params_t *params = &pd->params[i];
    const arm_real_t q_ref = (ref->flags & ARM_REFERENCE_Q_VALID) ? ref->q_ref_rad[i] : state->q_rad[i];
    const arm_real_t dq_ref = (ref->flags & ARM_REFERENCE_DQ_VALID) ? ref->dq_ref_rad_s[i] : ARM_REAL_ZERO;
    const arm_real_t q_err = q_ref - state->q_rad[i];
    const arm_real_t dq_err = dq_ref - state->dq_rad_s[i];

    arm_real_t tau = params->kp * q_err + params->kd * dq_err;
    if (params->out_limit > ARM_REAL_ZERO) {
      tau = arm_clamp(tau, -params->out_limit, params->out_limit);
    }
    command->tau_ff_nm[i] = tau;
  }

  command->flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static void joint_pd_vt_reset(void *ctx) {
  joint_pd_reset((joint_pd_t *)ctx);
}

static const arm_controller_vtable_t JOINT_PD_VTABLE = {
  joint_pd_vt_reset,
  joint_pd_step,
};

void joint_pd_init(joint_pd_t *pd, uint8_t dof) {
  if (!pd) return;

  pd->dof = arm_sanitize_dof(dof);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    pd->params[i] = (joint_pd_params_t){0};
  }
}

void joint_pd_reset(joint_pd_t *pd) {
  (void)pd;
}

void joint_pd_set_params(joint_pd_t *pd, uint8_t joint, joint_pd_params_t params) {
  if (!pd || joint >= pd->dof) return;
  pd->params[joint] = params;
}

arm_controller_t joint_pd_as_controller(joint_pd_t *pd) {
  arm_controller_t controller;
  controller.vt = &JOINT_PD_VTABLE;
  controller.ctx = pd;
  return controller;
}
