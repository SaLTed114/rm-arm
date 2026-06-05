#include "arm_core/joint_ref_planner.h"

#include "arm_core/arm_math.h"

static void default_params(joint_ref_planner_params_t *params) {
  if (!params) {
    return;
  }

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    params->q_min_rad[i] = -ARM_REAL_PI;
    params->q_max_rad[i] = +ARM_REAL_PI;
    params->dq_limit_rad_s[i] = ARM_REAL_ONE;
    params->ddq_limit_rad_s2[i] = (arm_real_t)4.0;
  }
}

static arm_real_t sanitize_positive(arm_real_t value) {
  return value > ARM_REAL_ZERO ? value : ARM_REAL_ZERO;
}

void joint_ref_planner_init(
    joint_ref_planner_t *planner,
    uint8_t dof,
    const joint_ref_planner_params_t *params) {
  if (!planner) {
    return;
  }

  planner->dof = arm_sanitize_dof(dof);
  default_params(&planner->params);
  if (params) {
    planner->params = *params;
  }

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    planner->q_goal_rad[i] = ARM_REAL_ZERO;
    planner->q_ref_rad[i] = ARM_REAL_ZERO;
    planner->dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }
  planner->initialized = false;
}

void joint_ref_planner_reset_to_state(joint_ref_planner_t *planner, const arm_state_t *state) {
  if (!planner) {
    return;
  }

  const uint8_t dof = state && state->dof < planner->dof ? state->dof : planner->dof;
  for (uint8_t i = 0u; i < planner->dof; ++i) {
    const arm_real_t q = i < dof ? state->q_rad[i] : ARM_REAL_ZERO;
    const arm_real_t q_min = planner->params.q_min_rad[i];
    const arm_real_t q_max = planner->params.q_max_rad[i];
    const arm_real_t q_clamped = arm_clamp(q, q_min, q_max);
    planner->q_goal_rad[i] = q_clamped;
    planner->q_ref_rad[i] = q_clamped;
    planner->dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }
  planner->initialized = true;
}

void joint_ref_planner_set_params(joint_ref_planner_t *planner, const joint_ref_planner_params_t *params) {
  if (!planner || !params) {
    return;
  }
  planner->params = *params;
}

int joint_ref_planner_step(
    joint_ref_planner_t *planner,
    const arm_state_t *state,
    const arm_reference_t *goal_ref,
    arm_reference_t *planned_ref) {
  if (!planner || !state || !goal_ref || !planned_ref) {
    return ARM_ERR_NULL;
  }
  if (planner->dof == 0u || planner->dof > ARM_DOF_MAX || state->dof != planner->dof ||
      goal_ref->dof != planner->dof) {
    return ARM_ERR_DOF;
  }

  if (!planner->initialized) {
    joint_ref_planner_reset_to_state(planner, state);
  }

  arm_reference_zero(planned_ref, planner->dof);
  planned_ref->flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  if (goal_ref->flags & ARM_REFERENCE_TAU_FF_VALID) {
    planned_ref->flags |= ARM_REFERENCE_TAU_FF_VALID;
  }

  const arm_real_t dt = state->dt_s > ARM_REAL_ZERO ? state->dt_s : ARM_REAL_ZERO;
  for (uint8_t i = 0u; i < planner->dof; ++i) {
    const arm_real_t q_min = planner->params.q_min_rad[i];
    const arm_real_t q_max = planner->params.q_max_rad[i];
    const arm_real_t goal = (goal_ref->flags & ARM_REFERENCE_Q_VALID) ? goal_ref->q_ref_rad[i] : planner->q_goal_rad[i];
    planner->q_goal_rad[i] = arm_clamp(goal, q_min, q_max);

    const arm_real_t dq_limit = sanitize_positive(planner->params.dq_limit_rad_s[i]);
    const arm_real_t ddq_limit = sanitize_positive(planner->params.ddq_limit_rad_s2[i]);
    const arm_real_t err = planner->q_goal_rad[i] - planner->q_ref_rad[i];

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
      next_dq = arm_clamp(next_dq, planner->dq_ref_rad_s[i] - max_delta_dq,
                          planner->dq_ref_rad_s[i] + max_delta_dq);
    }
    if (dq_limit > ARM_REAL_ZERO) {
      next_dq = arm_clamp(next_dq, -dq_limit, dq_limit);
    }

    arm_real_t next_q = planner->q_ref_rad[i] + next_dq * dt;
    if (dt <= ARM_REAL_ZERO || arm_abs(next_q - planner->q_ref_rad[i]) >= arm_abs(err)) {
      next_q = planner->q_goal_rad[i];
      next_dq = ARM_REAL_ZERO;
    }
    next_q = arm_clamp(next_q, q_min, q_max);

    planner->q_ref_rad[i] = next_q;
    planner->dq_ref_rad_s[i] = next_dq;
    planned_ref->q_ref_rad[i] = next_q;
    planned_ref->dq_ref_rad_s[i] = next_dq;
    planned_ref->tau_ff_nm[i] =
        (goal_ref->flags & ARM_REFERENCE_TAU_FF_VALID) ? goal_ref->tau_ff_nm[i] : ARM_REAL_ZERO;
  }

  return ARM_OK;
}
