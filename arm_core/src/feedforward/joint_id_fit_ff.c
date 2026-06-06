#include "arm_core/joint_id_fit_ff.h"

#include <math.h>

#include "arm_common/arm_math.h"

static arm_real_t ref_or_state_q(const arm_state_t *state, const arm_reference_t *ref, uint8_t joint) {
  return (ref && (ref->flags & ARM_REFERENCE_Q_VALID)) ? ref->q_ref_rad[joint] : state->q_rad[joint];
}

static arm_real_t ref_or_state_dq(const arm_state_t *state, const arm_reference_t *ref, uint8_t joint) {
  return (ref && (ref->flags & ARM_REFERENCE_DQ_VALID)) ? ref->dq_ref_rad_s[joint] : state->dq_rad_s[joint];
}

static arm_real_t ref_ddq(const arm_reference_t *ref, uint8_t joint) {
  return (ref && (ref->flags & ARM_REFERENCE_DDQ_VALID)) ? ref->ddq_ref_rad_s2[joint] : ARM_REAL_ZERO;
}

static arm_real_t feature_value(
    const joint_id_fit_feature_t *feature,
    const arm_state_t *state,
    const arm_reference_t *ref) {
  const uint8_t a = feature->joint_a;
  const uint8_t b = feature->joint_b;
  switch ((joint_id_fit_feature_type_t)feature->type) {
    case JOINT_ID_FIT_FEATURE_BIAS:
      return ARM_REAL_ONE;
    case JOINT_ID_FIT_FEATURE_SIN_Q:
      return ARM_REAL(sin((double)ref_or_state_q(state, ref, a)));
    case JOINT_ID_FIT_FEATURE_COS_Q:
      return ARM_REAL(cos((double)ref_or_state_q(state, ref, a)));
    case JOINT_ID_FIT_FEATURE_DQ:
      return ref_or_state_dq(state, ref, a);
    case JOINT_ID_FIT_FEATURE_DQ_PRODUCT:
      return ref_or_state_dq(state, ref, a) * ref_or_state_dq(state, ref, b);
    case JOINT_ID_FIT_FEATURE_DDQ:
      return ref_ddq(ref, a);
    case JOINT_ID_FIT_FEATURE_DDQ_SIN_Q:
      return ref_ddq(ref, a) * ARM_REAL(sin((double)ref_or_state_q(state, ref, b)));
    case JOINT_ID_FIT_FEATURE_DDQ_COS_Q:
      return ref_ddq(ref, a) * ARM_REAL(cos((double)ref_or_state_q(state, ref, b)));
    default:
      return ARM_REAL_ZERO;
  }
}

static int joint_id_fit_ff_step(void *ctx, arm_t *arm, const arm_reference_t *ref) {
  joint_id_fit_ff_t *fit = (joint_id_fit_ff_t *)ctx;
  if (!fit || !arm) return ARM_ERR_NULL;

  const joint_id_fit_ff_params_t *params = &fit->params;
  if (!arm_dof_matches(params->dof, arm->config.dof) || !arm_dof_matches(params->dof, arm->state.dof)) {
    return ARM_ERR_DOF;
  }
  if (params->feature_count > JOINT_ID_FIT_FF_FEATURE_MAX) return ARM_ERR_CONFIG;

  arm_real_t values[JOINT_ID_FIT_FF_FEATURE_MAX];
  for (uint8_t feature_index = 0u; feature_index < params->feature_count; ++feature_index) {
    const joint_id_fit_feature_t *feature = &params->features[feature_index];
    if (feature->joint_a >= params->dof || feature->joint_b >= params->dof) {
      if (feature->type != JOINT_ID_FIT_FEATURE_BIAS) return ARM_ERR_CONFIG;
    }
    values[feature_index] = feature_value(feature, &arm->state, ref);
  }

  for (uint8_t joint = 0u; joint < params->dof; ++joint) {
    arm_real_t tau = ARM_REAL_ZERO;
    for (uint8_t feature_index = 0u; feature_index < params->feature_count; ++feature_index) {
      tau += params->coeff_nm[joint][feature_index] * values[feature_index];
    }
    arm->command.tau_ff_nm[joint] += tau;
  }
  arm->command.flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static const arm_feedforward_vtable_t JOINT_ID_FIT_FF_VTABLE = {
  NULL,
  joint_id_fit_ff_step,
};

void joint_id_fit_ff_init(joint_id_fit_ff_t *fit, const joint_id_fit_ff_params_t *params) {
  if (!fit) return;

  fit->params = (joint_id_fit_ff_params_t){0};
  if (params) {
    fit->params = *params;
    fit->params.dof = arm_sanitize_dof(fit->params.dof);
    if (fit->params.feature_count > JOINT_ID_FIT_FF_FEATURE_MAX) {
      fit->params.feature_count = JOINT_ID_FIT_FF_FEATURE_MAX;
    }
  }
}

arm_feedforward_t joint_id_fit_ff_as_feedforward(joint_id_fit_ff_t *fit) {
  arm_feedforward_t feedforward;
  feedforward.vt = &JOINT_ID_FIT_FF_VTABLE;
  feedforward.ctx = fit;
  return feedforward;
}
