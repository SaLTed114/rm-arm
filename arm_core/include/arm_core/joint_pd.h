#ifndef ARM_CORE_JOINT_PD_H_
#define ARM_CORE_JOINT_PD_H_

#include "arm_core/arm_controller.h"

typedef struct {
  arm_real_t kp;
  arm_real_t kd;
  arm_real_t out_limit;
} joint_pd_params_t;

typedef struct {
  uint8_t dof;
  joint_pd_params_t params[ARM_DOF_MAX];
} joint_pd_t;

void joint_pd_init(joint_pd_t *pd, uint8_t dof);
void joint_pd_reset(joint_pd_t *pd);
void joint_pd_set_params(joint_pd_t *pd, uint8_t joint, joint_pd_params_t params);
arm_controller_t joint_pd_as_controller(joint_pd_t *pd);

#endif
