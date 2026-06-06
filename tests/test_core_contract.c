#include <assert.h>
#include <math.h>

#include "arm_core/arm.h"
#include "arm_common/arm_math.h"
#include "arm_core/joint_state_filter.h"

static int near(arm_real_t lhs, arm_real_t rhs) {
  return fabs((double)(lhs - rhs)) < 1.0e-9;
}

static arm_config_t make_valid_config(uint8_t dof) {
  arm_config_t config = {0};
  config.dof = dof;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    config.joints[i].joint_name = "joint";
    config.joints[i].actuator_name = "motor";
    config.joints[i].sign = ARM_REAL_ONE;
    config.joints[i].q_offset_rad = ARM_REAL_ZERO;
    config.joints[i].torque_limit_nm = ARM_REAL(2.0);
  }
  return config;
}

static void test_config_contract(void) {
  arm_config_t config = make_valid_config(ARM_DEFAULT_DOF);
  assert(arm_config_validate(&config) == ARM_OK);

  config = make_valid_config(0u);
  assert(arm_config_validate(&config) == ARM_ERR_DOF);

  config = make_valid_config((uint8_t)(ARM_DOF_MAX + 1u));
  assert(arm_config_validate(&config) == ARM_ERR_DOF);

  config = make_valid_config(ARM_DEFAULT_DOF);
  config.joints[2].sign = ARM_REAL_ZERO;
  assert(arm_config_validate(&config) == ARM_ERR_CONFIG);

  config = make_valid_config(ARM_DEFAULT_DOF);
  config.joints[3].torque_limit_nm = ARM_REAL(-0.1);
  assert(arm_config_validate(&config) == ARM_ERR_CONFIG);
}

static void test_zero_helpers_contract(void) {
  arm_state_t state;
  state.dof = 99u;
  state.time_s = ARM_REAL(12.0);
  state.dt_s = ARM_REAL(0.5);
  state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID | ARM_STATE_TAU_EST_VALID;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    state.q_rad[i] = ARM_REAL(1.0);
    state.dq_rad_s[i] = ARM_REAL(2.0);
    state.tau_est_nm[i] = ARM_REAL(3.0);
  }
  arm_state_zero(&state, (uint8_t)(ARM_DOF_MAX + 4u));
  assert(state.dof == ARM_DOF_MAX);
  assert(state.flags == 0u);
  assert(near(state.time_s, ARM_REAL_ZERO));
  assert(near(state.dt_s, ARM_REAL_ZERO));
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    assert(near(state.q_rad[i], ARM_REAL_ZERO));
    assert(near(state.dq_rad_s[i], ARM_REAL_ZERO));
    assert(near(state.tau_est_nm[i], ARM_REAL_ZERO));
  }

  arm_reference_t ref;
  ref.dof = 99u;
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID |
              ARM_REFERENCE_TAU_FF_VALID;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    ref.q_ref_rad[i] = ARM_REAL(1.0);
    ref.dq_ref_rad_s[i] = ARM_REAL(2.0);
    ref.ddq_ref_rad_s2[i] = ARM_REAL(3.0);
    ref.tau_ff_nm[i] = ARM_REAL(4.0);
  }
  arm_reference_zero(&ref, (uint8_t)(ARM_DOF_MAX + 4u));
  assert(ref.dof == ARM_DOF_MAX);
  assert(ref.flags == 0u);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    assert(near(ref.q_ref_rad[i], ARM_REAL_ZERO));
    assert(near(ref.dq_ref_rad_s[i], ARM_REAL_ZERO));
    assert(near(ref.ddq_ref_rad_s2[i], ARM_REAL_ZERO));
    assert(near(ref.tau_ff_nm[i], ARM_REAL_ZERO));
  }

  arm_command_t command;
  command.dof = 99u;
  command.flags = ARM_COMMAND_Q_D_VALID | ARM_COMMAND_DQ_D_VALID | ARM_COMMAND_KP_VALID |
                  ARM_COMMAND_KD_VALID | ARM_COMMAND_TAU_FF_VALID |
                  ARM_COMMAND_TAU_FB_VALID | ARM_COMMAND_TAU_MODEL_VALID;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    command.q_d_rad[i] = ARM_REAL(1.0);
    command.dq_d_rad_s[i] = ARM_REAL(2.0);
    command.kp[i] = ARM_REAL(3.0);
    command.kd[i] = ARM_REAL(4.0);
    command.tau_fb_nm[i] = ARM_REAL(5.0);
    command.tau_model_ff_nm[i] = ARM_REAL(6.0);
    command.tau_ff_nm[i] = ARM_REAL(7.0);
  }
  arm_command_zero(&command, (uint8_t)(ARM_DOF_MAX + 4u));
  assert(command.dof == ARM_DOF_MAX);
  assert(command.flags == 0u);
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    assert(near(command.q_d_rad[i], ARM_REAL_ZERO));
    assert(near(command.dq_d_rad_s[i], ARM_REAL_ZERO));
    assert(near(command.kp[i], ARM_REAL_ZERO));
    assert(near(command.kd[i], ARM_REAL_ZERO));
    assert(near(command.tau_fb_nm[i], ARM_REAL_ZERO));
    assert(near(command.tau_model_ff_nm[i], ARM_REAL_ZERO));
    assert(near(command.tau_ff_nm[i], ARM_REAL_ZERO));
  }
}

static void test_command_limit_contract(void) {
  arm_config_t config = make_valid_config(2u);
  config.joints[0].torque_limit_nm = ARM_REAL(1.0);
  config.joints[1].torque_limit_nm = ARM_REAL(0.5);

  arm_command_t command;
  arm_command_zero(&command, config.dof);
  command.flags = ARM_COMMAND_Q_D_VALID | ARM_COMMAND_DQ_D_VALID | ARM_COMMAND_KP_VALID |
                  ARM_COMMAND_KD_VALID | ARM_COMMAND_TAU_FF_VALID |
                  ARM_COMMAND_TAU_FB_VALID | ARM_COMMAND_TAU_MODEL_VALID;
  command.q_d_rad[0] = ARM_REAL(0.11);
  command.dq_d_rad_s[0] = ARM_REAL(0.22);
  command.kp[0] = ARM_REAL(3.3);
  command.kd[0] = ARM_REAL(4.4);
  command.tau_fb_nm[0] = ARM_REAL(5.5);
  command.tau_model_ff_nm[0] = ARM_REAL(6.6);
  command.tau_ff_nm[0] = ARM_REAL(9.0);
  command.tau_ff_nm[1] = ARM_REAL(-9.0);

  arm_command_apply_limits(&config, &command);
  assert(near(command.tau_ff_nm[0], ARM_REAL(1.0)));
  assert(near(command.tau_ff_nm[1], ARM_REAL(-0.5)));
  assert(near(command.q_d_rad[0], ARM_REAL(0.11)));
  assert(near(command.dq_d_rad_s[0], ARM_REAL(0.22)));
  assert(near(command.kp[0], ARM_REAL(3.3)));
  assert(near(command.kd[0], ARM_REAL(4.4)));
  assert(near(command.tau_fb_nm[0], ARM_REAL(5.5)));
  assert(near(command.tau_model_ff_nm[0], ARM_REAL(6.6)));
  assert(command.flags == (ARM_COMMAND_Q_D_VALID | ARM_COMMAND_DQ_D_VALID | ARM_COMMAND_KP_VALID |
                           ARM_COMMAND_KD_VALID | ARM_COMMAND_TAU_FF_VALID |
                           ARM_COMMAND_TAU_FB_VALID | ARM_COMMAND_TAU_MODEL_VALID));
}

static void test_math_contract(void) {
  assert(arm_dof_is_valid(1u));
  assert(arm_dof_is_valid(ARM_DOF_MAX));
  assert(!arm_dof_is_valid(0u));
  assert(!arm_dof_is_valid((uint8_t)(ARM_DOF_MAX + 1u)));
  assert(arm_dof_matches(ARM_DEFAULT_DOF, ARM_DEFAULT_DOF));
  assert(!arm_dof_matches(ARM_DEFAULT_DOF, (uint8_t)(ARM_DEFAULT_DOF - 1u)));
  assert(!arm_dof_matches(0u, 0u));
  assert(near(ARM_REAL_ONE, ARM_REAL(1.0)));
  assert(near(ARM_REAL_ZERO, ARM_REAL(0.0)));
  assert(ARM_REAL_PI > ARM_REAL(3.0));
}

static void test_state_filter_contract(void) {
  joint_state_filter_params_t params[ARM_DOF_MAX] = {0};
  params[0].q_time_constant_s = ARM_REAL_ZERO;
  params[0].dq_time_constant_s = ARM_REAL(0.1);
  params[0].use_dq_from_q_diff = false;

  joint_state_filter_t filter;
  joint_state_filter_init(&filter, 1u, params);

  arm_state_t measured;
  arm_state_zero(&measured, 1u);
  measured.dt_s = ARM_REAL(0.1);
  measured.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  measured.q_rad[0] = ARM_REAL(1.0);
  measured.dq_rad_s[0] = ARM_REAL_ZERO;
  joint_state_filter_reset_to_state(&filter, &measured);

  arm_state_t filtered;
  assert(joint_state_filter_step(&filter, &measured, &filtered) == ARM_OK);
  assert(near(filtered.q_rad[0], ARM_REAL(1.0)));
  assert(near(filtered.dq_rad_s[0], ARM_REAL_ZERO));
  assert(filtered.flags == (ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID));

  measured.q_rad[0] = ARM_REAL(2.0);
  measured.dq_rad_s[0] = ARM_REAL(10.0);
  assert(joint_state_filter_step(&filter, &measured, &filtered) == ARM_OK);
  assert(near(filtered.q_rad[0], ARM_REAL(2.0)));
  assert(near(filtered.dq_rad_s[0], ARM_REAL(5.0)));

  params[0].dq_time_constant_s = ARM_REAL_ZERO;
  params[0].use_dq_from_q_diff = true;
  joint_state_filter_init(&filter, 1u, params);
  arm_state_zero(&measured, 1u);
  measured.dt_s = ARM_REAL(0.1);
  measured.flags = ARM_STATE_Q_VALID;
  measured.q_rad[0] = ARM_REAL_ZERO;
  joint_state_filter_reset_to_state(&filter, &measured);
  measured.q_rad[0] = ARM_REAL(0.2);
  assert(joint_state_filter_step(&filter, &measured, &filtered) == ARM_OK);
  assert(filtered.flags == (ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID));
  assert(near(filtered.dq_rad_s[0], ARM_REAL(2.0)));

  params[0].use_dq_from_q_diff = false;
  joint_state_filter_init(&filter, 1u, params);
  arm_state_zero(&measured, 1u);
  measured.dt_s = ARM_REAL(0.1);
  measured.flags = ARM_STATE_Q_VALID;
  measured.q_rad[0] = ARM_REAL(0.3);
  assert(joint_state_filter_step(&filter, &measured, &filtered) == ARM_OK);
  assert(filtered.flags == ARM_STATE_Q_VALID);
  assert(near(filtered.q_rad[0], ARM_REAL(0.3)));
  assert(near(filtered.dq_rad_s[0], ARM_REAL_ZERO));
}

int main(void) {
  test_config_contract();
  test_zero_helpers_contract();
  test_command_limit_contract();
  test_math_contract();
  test_state_filter_contract();
  return 0;
}
