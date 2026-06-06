#include "arm_core/arm_safety.h"

#include "arm_common/arm_math.h"

static bool state_has_required_measurements(const arm_state_t *state) {
  return (state->flags & (ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID)) ==
         (ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID);
}

static arm_real_t positive_or_zero(arm_real_t value) {
  return value > ARM_REAL_ZERO ? value : ARM_REAL_ZERO;
}

static arm_real_t clamp_joint_torque(const arm_safety_joint_params_t *joint, arm_real_t tau) {
  if (joint->torque_limit_nm <= ARM_REAL_ZERO) return tau;
  return arm_clamp(tau, -joint->torque_limit_nm, joint->torque_limit_nm);
}

static bool torque_pushes_deeper_into_limit(
    const arm_safety_joint_params_t *joint,
    arm_real_t q_rad,
    arm_real_t tau_nm) {
  const arm_real_t margin = positive_or_zero(joint->q_margin_rad);
  if (q_rad <= joint->q_min_rad + margin && tau_nm < ARM_REAL_ZERO) return true;
  if (q_rad >= joint->q_max_rad - margin && tau_nm > ARM_REAL_ZERO) return true;
  return false;
}

static bool torque_pushes_with_overspeed(
    const arm_safety_joint_params_t *joint,
    arm_real_t dq_rad_s,
    arm_real_t tau_nm) {
  if (joint->dq_limit_rad_s <= ARM_REAL_ZERO) return false;
  if (dq_rad_s >= joint->dq_limit_rad_s && tau_nm > ARM_REAL_ZERO) return true;
  if (dq_rad_s <= -joint->dq_limit_rad_s && tau_nm < ARM_REAL_ZERO) return true;
  return false;
}

void arm_safety_init(arm_safety_t *safety, uint8_t dof) {
  if (!safety) return;

  safety->dof = arm_sanitize_dof(dof);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    safety->joints[i] = (arm_safety_joint_params_t){
      -ARM_REAL_PI,
      +ARM_REAL_PI,
      ARM_REAL(0.02),
      ARM_REAL_ZERO,
      ARM_REAL_ZERO,
    };
  }
}

void arm_safety_set_joint_params(arm_safety_t *safety, uint8_t joint, arm_safety_joint_params_t params) {
  if (!safety || joint >= safety->dof) return;
  safety->joints[joint] = params;
}

int arm_safety_apply(const arm_safety_t *safety, const arm_state_t *state, arm_command_t *command) {
  if (!safety || !state || !command) return ARM_ERR_NULL;
  if (!arm_dof_matches(safety->dof, state->dof) || !arm_dof_matches(safety->dof, command->dof)) {
    arm_command_zero(command, command->dof);
    return ARM_ERR_DOF;
  }

  if (!state_has_required_measurements(state)) {
    arm_command_zero(command, command->dof);
    return ARM_OK;
  }

  for (uint8_t i = 0u; i < safety->dof; ++i) {
    const arm_safety_joint_params_t *joint = &safety->joints[i];
    arm_real_t tau = clamp_joint_torque(joint, command->tau_ff_nm[i]);
    if (torque_pushes_deeper_into_limit(joint, state->q_rad[i], tau) ||
        torque_pushes_with_overspeed(joint, state->dq_rad_s[i], tau)) {
      tau = ARM_REAL_ZERO;
    }

    command->tau_ff_nm[i] = tau;
  }

  return ARM_OK;
}
