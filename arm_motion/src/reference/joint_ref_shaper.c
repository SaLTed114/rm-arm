#include "arm_motion/joint_ref_shaper.h"

#include "arm_core/arm_math.h"

static void default_params(joint_ref_shaper_params_t *params) {
  if (!params) return;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    params->q_min_rad[i] = -ARM_REAL_PI;
    params->q_max_rad[i] = +ARM_REAL_PI;
    params->dq_limit_rad_s[i] = ARM_REAL_ONE;
    params->ddq_limit_rad_s2[i] = ARM_REAL(4.0);
  }
}

static arm_real_t sanitize_positive(arm_real_t value) {
  return value > ARM_REAL_ZERO ? value : ARM_REAL_ZERO;
}

void joint_ref_shaper_init(
    joint_ref_shaper_t *shaper,
    uint8_t dof,
    const joint_ref_shaper_params_t *params) {
  if (!shaper) return;

  shaper->dof = arm_sanitize_dof(dof);
  default_params(&shaper->params);
  if (params) {
    shaper->params = *params;
  }

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    shaper->q_goal_rad[i] = ARM_REAL_ZERO;
    shaper->q_ref_rad[i] = ARM_REAL_ZERO;
    shaper->dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }
  shaper->initialized = false;
}

void joint_ref_shaper_reset_to_state(joint_ref_shaper_t *shaper, const arm_state_t *state) {
  if (!shaper) return;

  const uint8_t dof = state && state->dof < shaper->dof ? state->dof : shaper->dof;
  for (uint8_t i = 0u; i < shaper->dof; ++i) {
    const arm_real_t q = i < dof ? state->q_rad[i] : ARM_REAL_ZERO;
    const arm_real_t q_min = shaper->params.q_min_rad[i];
    const arm_real_t q_max = shaper->params.q_max_rad[i];
    const arm_real_t q_clamped = arm_clamp(q, q_min, q_max);
    shaper->q_goal_rad[i] = q_clamped;
    shaper->q_ref_rad[i] = q_clamped;
    shaper->dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }
  shaper->initialized = true;
}

void joint_ref_shaper_set_params(joint_ref_shaper_t *shaper, const joint_ref_shaper_params_t *params) {
  if (!shaper || !params) return;
  shaper->params = *params;
}

int joint_ref_shaper_step(
    joint_ref_shaper_t *shaper,
    const arm_state_t *state,
    const arm_reference_t *goal_ref,
    arm_reference_t *shaped_ref) {
  if (!shaper || !state || !goal_ref || !shaped_ref) return ARM_ERR_NULL;
  const bool dof_ok = arm_dof_matches(shaper->dof, state->dof) && arm_dof_matches(shaper->dof, goal_ref->dof);
  if (!dof_ok) return ARM_ERR_DOF;

  if (!shaper->initialized) {
    joint_ref_shaper_reset_to_state(shaper, state);
  }

  arm_reference_zero(shaped_ref, shaper->dof);
  shaped_ref->flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;

  const arm_real_t dt = state->dt_s > ARM_REAL_ZERO ? state->dt_s : ARM_REAL_ZERO;
  for (uint8_t i = 0u; i < shaper->dof; ++i) {
    const arm_real_t q_min = shaper->params.q_min_rad[i];
    const arm_real_t q_max = shaper->params.q_max_rad[i];
    const arm_real_t goal =
        (goal_ref->flags & ARM_REFERENCE_Q_VALID) ? goal_ref->q_ref_rad[i] : shaper->q_goal_rad[i];
    shaper->q_goal_rad[i] = arm_clamp(goal, q_min, q_max);

    const arm_real_t dq_limit = sanitize_positive(shaper->params.dq_limit_rad_s[i]);
    const arm_real_t ddq_limit = sanitize_positive(shaper->params.ddq_limit_rad_s2[i]);
    const arm_real_t err = shaper->q_goal_rad[i] - shaper->q_ref_rad[i];

    arm_real_t desired_dq = ARM_REAL_ZERO;
    if (dt > ARM_REAL_ZERO) {
      desired_dq = err / dt;
    }
    if (dq_limit > ARM_REAL_ZERO) {
      desired_dq = arm_clamp(desired_dq, -dq_limit, dq_limit);
    }

    arm_real_t next_dq = desired_dq;
    if (ddq_limit > ARM_REAL_ZERO && dt > ARM_REAL_ZERO) {
      const arm_real_t max_delta_dq = ddq_limit * dt;
      next_dq = arm_clamp(next_dq, shaper->dq_ref_rad_s[i] - max_delta_dq,
                          shaper->dq_ref_rad_s[i] + max_delta_dq);
    }
    if (dq_limit > ARM_REAL_ZERO) {
      next_dq = arm_clamp(next_dq, -dq_limit, dq_limit);
    }

    arm_real_t next_q = shaper->q_ref_rad[i] + next_dq * dt;
    if (dt <= ARM_REAL_ZERO || arm_abs(next_q - shaper->q_ref_rad[i]) >= arm_abs(err)) {
      next_q = shaper->q_goal_rad[i];
      next_dq = ARM_REAL_ZERO;
    }
    next_q = arm_clamp(next_q, q_min, q_max);

    shaper->q_ref_rad[i] = next_q;
    shaper->dq_ref_rad_s[i] = next_dq;
    shaped_ref->q_ref_rad[i] = next_q;
    shaped_ref->dq_ref_rad_s[i] = next_dq;
  }

  return ARM_OK;
}
