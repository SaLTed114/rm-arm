#include <assert.h>

#include "arm_core/arm_control.h"
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

int main(void) {
  arm_config_t config = make_test_config();
  assert(arm_config_validate(&config) == ARM_OK);

  arm_state_t state;
  arm_state_zero(&state, config.dof);
  state.dt_s = 0.001;
  state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;

  joint_sweep_t sweep;
  joint_sweep_init(&sweep, config.dof, (joint_sweep_params_t){2.0, 0.1, 0.1});
  arm_controller_t controller = joint_sweep_as_controller(&sweep);

  arm_command_t command;
  assert(arm_control_step(&config, &controller, &state, &command) == ARM_OK);
  assert(command.dof == config.dof);
  assert(command.tau_nm[0] == 1.5);
  assert(command.tau_nm[1] == 0.0);

  arm_command_zero(&command, config.dof);
  assert(command.flags == ARM_COMMAND_TAU_VALID);
  assert(command.tau_nm[0] == 0.0);

  return 0;
}
