#include "arm_core/joint_pvi.h"

#include "arm_core/arm_math.h"

static int joint_pvi_step(void *ctx, arm_t *arm, const arm_reference_t *ref) {
  joint_pvi_t *pvi = (joint_pvi_t *)ctx;
  if (!pvi || !arm) {
    return ARM_ERR_NULL;
  }

  const arm_config_t *config = &arm->config;
  const arm_state_t *state = &arm->state;
  arm_command_t *command = &arm->command;

  if (pvi->dof == 0u || pvi->dof > ARM_DOF_MAX || pvi->dof != config->dof || state->dof != config->dof) {
    return ARM_ERR_DOF;
  }
  if (!ref || ref->dof != config->dof) {
    return ARM_ERR_DOF;
  }

  arm_command_zero(command, config->dof);

  for (uint8_t i = 0u; i < pvi->dof; ++i) {
    const joint_pvi_params_t *params = &pvi->params[i];
    const arm_real_t q_ref = (ref->flags & ARM_REFERENCE_Q_VALID) ? ref->q_ref_rad[i] : state->q_rad[i];
    const arm_real_t dq_ref = (ref->flags & ARM_REFERENCE_DQ_VALID) ? ref->dq_ref_rad_s[i] : ARM_REAL_ZERO;
    const arm_real_t tau_ff = (ref->flags & ARM_REFERENCE_TAU_FF_VALID) ? ref->tau_ff_nm[i] : ARM_REAL_ZERO;
    const arm_real_t q_err = q_ref - state->q_rad[i];
    const arm_real_t dq_err = dq_ref - state->dq_rad_s[i];

    if (params->ki != ARM_REAL_ZERO && state->dt_s > ARM_REAL_ZERO) {
      pvi->integral_nm[i] += params->ki * q_err * state->dt_s;
      if (params->integral_limit > ARM_REAL_ZERO) {
        pvi->integral_nm[i] =
            arm_clamp(pvi->integral_nm[i], -params->integral_limit, params->integral_limit);
      }
    }

    arm_real_t tau = params->kp * q_err + params->kv * dq_err + pvi->integral_nm[i] + tau_ff;
    if (params->out_limit > ARM_REAL_ZERO) {
      tau = arm_clamp(tau, -params->out_limit, params->out_limit);
    }
    command->tau_ff_nm[i] = tau;
  }

  command->flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static void joint_pvi_vt_reset(void *ctx) {
  joint_pvi_reset((joint_pvi_t *)ctx);
}

static const arm_controller_vtable_t JOINT_PVI_VTABLE = {
  joint_pvi_vt_reset,
  joint_pvi_step,
};

void joint_pvi_init(joint_pvi_t *pvi, uint8_t dof) {
  if (!pvi) {
    return;
  }

  pvi->dof = arm_sanitize_dof(dof);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    pvi->params[i] = (joint_pvi_params_t){0};
    pvi->integral_nm[i] = ARM_REAL_ZERO;
  }
}

void joint_pvi_reset(joint_pvi_t *pvi) {
  if (!pvi) {
    return;
  }

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    pvi->integral_nm[i] = ARM_REAL_ZERO;
  }
}

void joint_pvi_set_params(joint_pvi_t *pvi, uint8_t joint, joint_pvi_params_t params) {
  if (!pvi || joint >= pvi->dof) {
    return;
  }
  pvi->params[joint] = params;
}

arm_controller_t joint_pvi_as_controller(joint_pvi_t *pvi) {
  arm_controller_t controller;
  controller.vt = &JOINT_PVI_VTABLE;
  controller.ctx = pvi;
  return controller;
}
