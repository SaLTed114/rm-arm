#include "armsim/default_arm_config.h"

arm_config_t armsim_default_arm6_config(void) {
  arm_config_t config = {0};
  config.dof = ARM_DEFAULT_DOF;

  config.joints[0] = (arm_joint_config_t){"joint_1", "motor_1", 1.0, 0.0, 10.0};
  config.joints[1] = (arm_joint_config_t){"joint_2", "motor_2", 1.0, 0.0, 40.0};
  config.joints[2] = (arm_joint_config_t){"joint_3", "motor_3", 1.0, 0.0, 30.0};
  config.joints[3] = (arm_joint_config_t){"joint_4", "motor_4", 1.0, 0.0, 8.0};
  config.joints[4] = (arm_joint_config_t){"joint_5", "motor_5", 1.0, 0.0, 8.0};
  config.joints[5] = (arm_joint_config_t){"joint_6", "motor_6", 1.0, 0.0, 8.0};

  return config;
}
