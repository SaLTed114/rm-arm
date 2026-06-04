#ifndef ARM_CORE_JOINT_PVI_H_
#define ARM_CORE_JOINT_PVI_H_

#include "arm_core/arm_control.h"

typedef struct {
  arm_real_t kp;
  arm_real_t kv;
  arm_real_t ki;
  arm_real_t integral_limit;
  arm_real_t out_limit;
} joint_pvi_params_t;

typedef struct {
  uint8_t dof;
  joint_pvi_params_t params[ARM_DOF_MAX];
  arm_real_t integral_nm[ARM_DOF_MAX];
} joint_pvi_t;

void joint_pvi_init(joint_pvi_t *pvi, uint8_t dof);
void joint_pvi_reset(joint_pvi_t *pvi);
void joint_pvi_set_params(joint_pvi_t *pvi, uint8_t joint, joint_pvi_params_t params);
arm_controller_t joint_pvi_as_controller(joint_pvi_t *pvi);

#endif
