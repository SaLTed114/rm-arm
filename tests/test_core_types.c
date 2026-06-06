#include <assert.h>
#include <math.h>

#include "arm_core/arm_safety.h"
#include "arm_core/arm_control.h"
#include "arm_core/arm_feedforward.h"
#include "arm_common/arm_math.h"
#include "arm_core/joint_id_fit_ff.h"
#include "arm_core/arm_output_limiter.h"
#include "arm_core/joint_pd.h"
#include "arm_core/joint_sweep.h"

static arm_config_t make_test_config(void) {
  arm_config_t config = {0};
  config.dof = ARM_DEFAULT_DOF;
  for (uint8_t i = 0u; i < config.dof; ++i) {
    config.joints[i].joint_name = "joint";
    config.joints[i].actuator_name = "motor";
    config.joints[i].sign = 1.0;
    config.joints[i].q_offset_rad = 0.0;
    config.joints[i].torque_limit_nm = 1.5;
  }
  return config;
}

static int near_zero(arm_real_t value) {
  return fabs((double)value) < 1.0e-9;
}

typedef struct {
  uint8_t dof;
  arm_real_t tau_nm[ARM_DOF_MAX];
} test_feedforward_t;

static int test_feedforward_step(void *ctx, arm_t *arm, const arm_reference_t *ref) {
  (void)ref;
  test_feedforward_t *feedforward = (test_feedforward_t *)ctx;
  if (!feedforward || !arm) return ARM_ERR_NULL;
  if (!arm_dof_matches(feedforward->dof, arm->config.dof)) return ARM_ERR_DOF;

  for (uint8_t i = 0u; i < feedforward->dof; ++i) {
    arm->command.tau_ff_nm[i] += feedforward->tau_nm[i];
  }
  arm->command.flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static const arm_feedforward_vtable_t TEST_FEEDFORWARD_VTABLE = {
  NULL,
  test_feedforward_step,
};

int main(void) {
  arm_config_t config = make_test_config();
  assert(arm_config_validate(&config) == ARM_OK);

  arm_t arm;
  assert(arm_init(&arm, &config) == ARM_OK);
  arm.state.dt_s = 0.001;
  arm.state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  arm_reference_t ref;
  arm_reference_zero(&ref, arm.config.dof);

  joint_sweep_t sweep;
  joint_sweep_init(&sweep, arm.config.dof, (joint_sweep_params_t){2.0, 0.1, 0.1});
  arm_controller_t controller = joint_sweep_as_controller(&sweep);

  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.dof == arm.config.dof);
  assert(arm.command.tau_ff_nm[0] == 1.5);
  assert(arm.command.tau_ff_nm[1] == 0.0);

  arm_clear_command(&arm);
  assert(arm.command.flags == 0u);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  joint_pd_t pd;
  joint_pd_init(&pd, arm.config.dof);
  arm_reference_zero(&ref, arm.config.dof);
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    joint_pd_set_params(&pd, i, (joint_pd_params_t){10.0, 1.0, 10.0});
    ref.q_ref_rad[i] = 0.0;
    ref.dq_ref_rad_s[i] = 0.0;
    arm.state.q_rad[i] = 0.0;
    arm.state.dq_rad_s[i] = 0.0;
  }
  controller = joint_pd_as_controller(&pd);
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    assert(near_zero(arm.command.tau_ff_nm[i]));
  }

  ref.q_ref_rad[0] = 0.1;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] > 0.0);

  joint_pd_set_params(&pd, 0u, (joint_pd_params_t){10.0, 0.0, 0.75});
  ref.q_ref_rad[0] = 1.0;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.75);

  test_feedforward_t test_ff = {0};
  test_ff.dof = arm.config.dof;
  test_ff.tau_nm[0] = 0.4;
  arm_feedforward_t feedforward = {&TEST_FEEDFORWARD_VTABLE, &test_ff};
  ref.q_ref_rad[0] = 0.0;
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  assert(arm_control_step_with_feedforward(&arm, &ref, &controller, &feedforward) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0] - 0.4));

  joint_pd_set_params(&pd, 0u, (joint_pd_params_t){10.0, 0.0, 10.0});
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  ref.q_ref_rad[0] = 1.0;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 1.5);

  arm_safety_t safety;
  arm_safety_init(&safety, arm.config.dof);
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    arm_safety_set_joint_params(&safety, i, (arm_safety_joint_params_t){-0.5, 0.5, 0.02, 1.0, 0.7});
  }

  arm_command_zero(&arm.command, arm.config.dof);
  arm.command.tau_ff_nm[0] = 0.5;
  arm.state.flags = 0u;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  arm.state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  arm.state.q_rad[0] = 0.0;
  arm.state.dq_rad_s[0] = 0.0;
  arm.command.tau_ff_nm[0] = 2.0;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.7);

  arm.state.q_rad[0] = 0.49;
  arm.command.tau_ff_nm[0] = 0.5;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  arm.state.q_rad[0] = 0.0;
  arm.state.dq_rad_s[0] = 1.2;
  arm.command.tau_ff_nm[0] = 0.5;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  arm_output_limiter_t output_limiter;
  arm_output_limiter_init(&output_limiter, arm.config.dof);
  arm_output_limiter_set_joint_params(
      &output_limiter, 0u, (arm_output_limiter_joint_params_t){ARM_REAL(100.0)});
  arm_command_zero(&arm.command, arm.config.dof);
  arm.state.dof = arm.config.dof;
  arm.state.dt_s = ARM_REAL(0.01);
  arm.command.tau_ff_nm[0] = ARM_REAL(2.0);
  assert(arm_output_limiter_apply(&output_limiter, &arm.state, &arm.command) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0] - ARM_REAL(1.0)));

  arm.command.tau_ff_nm[0] = ARM_REAL(-2.0);
  assert(arm_output_limiter_apply(&output_limiter, &arm.state, &arm.command) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0]));

  arm.command.tau_ff_nm[0] = ARM_REAL(0.4);
  arm_output_limiter_reset_to_command(&output_limiter, &arm.command);
  arm.command.tau_ff_nm[0] = ARM_REAL(0.5);
  assert(arm_output_limiter_apply(&output_limiter, &arm.state, &arm.command) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0] - ARM_REAL(0.5)));

  joint_id_fit_ff_params_t fit_params = {0};
  fit_params.dof = arm.config.dof;
  fit_params.feature_count = 3u;
  fit_params.features[0] = (joint_id_fit_feature_t){JOINT_ID_FIT_FEATURE_BIAS, 0u, 0u};
  fit_params.features[1] = (joint_id_fit_feature_t){JOINT_ID_FIT_FEATURE_DQ, 0u, 0u};
  fit_params.features[2] = (joint_id_fit_feature_t){JOINT_ID_FIT_FEATURE_DDQ, 0u, 0u};
  fit_params.coeff_nm[0][0] = ARM_REAL(0.2);
  fit_params.coeff_nm[0][1] = ARM_REAL(0.3);
  fit_params.coeff_nm[0][2] = ARM_REAL(0.4);

  joint_id_fit_ff_t fit_ff;
  joint_id_fit_ff_init(&fit_ff, &fit_params);
  arm_feedforward_t fit_feedforward = joint_id_fit_ff_as_feedforward(&fit_ff);
  arm_reference_zero(&ref, arm.config.dof);
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;
  ref.dq_ref_rad_s[0] = ARM_REAL(2.0);
  ref.ddq_ref_rad_s2[0] = ARM_REAL(3.0);
  arm.state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  arm.state.dof = arm.config.dof;
  assert(arm_control_step_with_feedforward(&arm, &ref, NULL, &fit_feedforward) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0] - ARM_REAL(1.5)));

  joint_id_fit_ff_params_t zero_fit_params = {0};
  zero_fit_params.dof = arm.config.dof;
  zero_fit_params.feature_count = 1u;
  zero_fit_params.features[0] = (joint_id_fit_feature_t){JOINT_ID_FIT_FEATURE_BIAS, 0u, 0u};
  joint_id_fit_ff_init(&fit_ff, &zero_fit_params);
  fit_feedforward = joint_id_fit_ff_as_feedforward(&fit_ff);
  assert(arm_control_step_with_feedforward(&arm, &ref, NULL, &fit_feedforward) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0]));

  return 0;
}
