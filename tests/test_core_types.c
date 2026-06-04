#include <assert.h>
#include <math.h>

#include "arm_core/arm_control.h"
#include "arm_core/joint_pvi.h"
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

  joint_pvi_t pvi;
  joint_pvi_init(&pvi, arm.config.dof);
  arm_reference_zero(&ref, arm.config.dof);
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    joint_pvi_set_params(&pvi, i, (joint_pvi_params_t){10.0, 1.0, 0.0, 0.0, 10.0});
    ref.q_ref_rad[i] = 0.0;
    ref.dq_ref_rad_s[i] = 0.0;
    arm.state.q_rad[i] = 0.0;
    arm.state.dq_rad_s[i] = 0.0;
  }
  controller = joint_pvi_as_controller(&pvi);
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    assert(near_zero(arm.command.tau_ff_nm[i]));
  }

  ref.q_ref_rad[0] = 0.1;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] > 0.0);

  joint_pvi_set_params(&pvi, 0u, (joint_pvi_params_t){10.0, 0.0, 0.0, 0.0, 0.75});
  ref.q_ref_rad[0] = 1.0;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.75);

  joint_pvi_set_params(&pvi, 0u, (joint_pvi_params_t){10.0, 0.0, 0.0, 0.0, 10.0});
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 1.5);

  return 0;
}
