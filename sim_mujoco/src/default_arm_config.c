/* Generated from configs/arm6_placeholder.yaml. Do not edit by hand. */
#include "armsim/default_arm_config.h"

#include "armsim/arm6_sim_config.h"

arm_config_t armsim_default_arm6_config(void) {
  arm_config_t config = {0};
  static const arm_real_t torque_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_TORQUE_LIMITS_NM;
  config.dof = ARM_DEFAULT_DOF;

  config.joints[0] = (arm_joint_config_t){"joint_1", "motor_1", ARM_REAL_ONE, ARM_REAL_ZERO, torque_limits[0]};
  config.joints[1] = (arm_joint_config_t){"joint_2", "motor_2", ARM_REAL_ONE, ARM_REAL_ZERO, torque_limits[1]};
  config.joints[2] = (arm_joint_config_t){"joint_3", "motor_3", ARM_REAL_ONE, ARM_REAL_ZERO, torque_limits[2]};
  config.joints[3] = (arm_joint_config_t){"joint_4", "motor_4", ARM_REAL_ONE, ARM_REAL_ZERO, torque_limits[3]};
  config.joints[4] = (arm_joint_config_t){"joint_5", "motor_5", ARM_REAL_ONE, ARM_REAL_ZERO, torque_limits[4]};
  config.joints[5] = (arm_joint_config_t){"joint_6", "motor_6", ARM_REAL_ONE, ARM_REAL_ZERO, torque_limits[5]};

  return config;
}
