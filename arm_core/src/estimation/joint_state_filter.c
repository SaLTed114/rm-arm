#include "arm_core/joint_state_filter.h"

#include "arm_common/arm_math.h"

static arm_real_t positive_or_zero(arm_real_t value) {
  return value > ARM_REAL_ZERO ? value : ARM_REAL_ZERO;
}

static arm_real_t low_pass_step(
    arm_real_t current,
    arm_real_t input,
    arm_real_t dt_s,
    arm_real_t time_constant_s) {
  if (time_constant_s <= ARM_REAL_ZERO || dt_s <= ARM_REAL_ZERO) return input;
  const arm_real_t alpha = arm_clamp(dt_s / (time_constant_s + dt_s), ARM_REAL_ZERO, ARM_REAL_ONE);
  return current + alpha * (input - current);
}

void joint_state_filter_init(
    joint_state_filter_t *filter,
    uint8_t dof,
    const joint_state_filter_params_t *params) {
  if (!filter) return;

  filter->dof = arm_sanitize_dof(dof);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    filter->params[i] = params && i < filter->dof ? params[i] : (joint_state_filter_params_t){0};
  }
  joint_state_filter_reset(filter);
}

void joint_state_filter_reset(joint_state_filter_t *filter) {
  if (!filter) return;

  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    filter->q_filtered_rad[i] = ARM_REAL_ZERO;
    filter->dq_filtered_rad_s[i] = ARM_REAL_ZERO;
    filter->last_q_rad[i] = ARM_REAL_ZERO;
    filter->has_last_q[i] = false;
  }
  filter->initialized = false;
}

void joint_state_filter_reset_to_state(joint_state_filter_t *filter, const arm_state_t *measured_state) {
  if (!filter || !measured_state) return;

  const uint8_t dof = measured_state->dof < filter->dof ? measured_state->dof : filter->dof;
  for (uint8_t i = 0u; i < filter->dof; ++i) {
    filter->q_filtered_rad[i] = i < dof ? measured_state->q_rad[i] : ARM_REAL_ZERO;
    filter->dq_filtered_rad_s[i] = i < dof ? measured_state->dq_rad_s[i] : ARM_REAL_ZERO;
    filter->last_q_rad[i] = filter->q_filtered_rad[i];
    filter->has_last_q[i] = i < dof && (measured_state->flags & ARM_STATE_Q_VALID);
  }
  filter->initialized = true;
}

void joint_state_filter_set_params(
    joint_state_filter_t *filter,
    uint8_t joint,
    joint_state_filter_params_t params) {
  if (!filter || joint >= filter->dof) return;
  params.q_time_constant_s = positive_or_zero(params.q_time_constant_s);
  params.dq_time_constant_s = positive_or_zero(params.dq_time_constant_s);
  filter->params[joint] = params;
}

int joint_state_filter_step(
    joint_state_filter_t *filter,
    const arm_state_t *measured_state,
    arm_state_t *filtered_state) {
  if (!filter || !measured_state || !filtered_state) return ARM_ERR_NULL;
  if (!arm_dof_matches(filter->dof, measured_state->dof)) return ARM_ERR_DOF;

  if (!filter->initialized) {
    joint_state_filter_reset_to_state(filter, measured_state);
  }

  arm_state_zero(filtered_state, filter->dof);
  filtered_state->time_s = measured_state->time_s;
  filtered_state->dt_s = measured_state->dt_s;

  const bool q_valid = (measured_state->flags & ARM_STATE_Q_VALID) != 0u;
  const bool dq_valid = (measured_state->flags & ARM_STATE_DQ_VALID) != 0u;
  const bool tau_valid = (measured_state->flags & ARM_STATE_TAU_EST_VALID) != 0u;

  for (uint8_t i = 0u; i < filter->dof; ++i) {
    const joint_state_filter_params_t *params = &filter->params[i];
    if (q_valid) {
      filter->q_filtered_rad[i] = low_pass_step(
          filter->q_filtered_rad[i],
          measured_state->q_rad[i],
          measured_state->dt_s,
          params->q_time_constant_s);
      filtered_state->q_rad[i] = filter->q_filtered_rad[i];
      filtered_state->flags |= ARM_STATE_Q_VALID;
    }

    bool dq_output_valid = false;
    arm_real_t dq_input = ARM_REAL_ZERO;
    if (params->use_dq_from_q_diff) {
      if (q_valid && filter->has_last_q[i] && measured_state->dt_s > ARM_REAL_ZERO) {
        dq_input = (measured_state->q_rad[i] - filter->last_q_rad[i]) / measured_state->dt_s;
        dq_output_valid = true;
      } else if (dq_valid) {
        dq_input = measured_state->dq_rad_s[i];
        dq_output_valid = true;
      }
    } else if (dq_valid) {
      dq_input = measured_state->dq_rad_s[i];
      dq_output_valid = true;
    }

    if (dq_output_valid) {
      filter->dq_filtered_rad_s[i] = low_pass_step(
          filter->dq_filtered_rad_s[i],
          dq_input,
          measured_state->dt_s,
          params->dq_time_constant_s);
      filtered_state->dq_rad_s[i] = filter->dq_filtered_rad_s[i];
      filtered_state->flags |= ARM_STATE_DQ_VALID;
    }

    if (tau_valid) {
      filtered_state->tau_est_nm[i] = measured_state->tau_est_nm[i];
      filtered_state->flags |= ARM_STATE_TAU_EST_VALID;
    }

    if (q_valid) {
      filter->last_q_rad[i] = measured_state->q_rad[i];
      filter->has_last_q[i] = true;
    }
  }

  return ARM_OK;
}
