#ifndef ARM_CORE_JOINT_ID_FIT_FF_H_
#define ARM_CORE_JOINT_ID_FIT_FF_H_

#include "arm_core/arm_feedforward.h"

#define JOINT_ID_FIT_FF_FEATURE_MAX 128u

typedef enum {
  JOINT_ID_FIT_FEATURE_BIAS = 0,
  JOINT_ID_FIT_FEATURE_SIN_Q = 1,
  JOINT_ID_FIT_FEATURE_COS_Q = 2,
  JOINT_ID_FIT_FEATURE_DQ = 3,
  JOINT_ID_FIT_FEATURE_DQ_PRODUCT = 4,
  JOINT_ID_FIT_FEATURE_DDQ = 5,
  JOINT_ID_FIT_FEATURE_DDQ_SIN_Q = 6,
  JOINT_ID_FIT_FEATURE_DDQ_COS_Q = 7,
} joint_id_fit_feature_type_t;

typedef struct {
  uint8_t type;
  uint8_t joint_a;
  uint8_t joint_b;
} joint_id_fit_feature_t;

typedef struct {
  uint8_t dof;
  uint8_t feature_count;
  joint_id_fit_feature_t features[JOINT_ID_FIT_FF_FEATURE_MAX];
  arm_real_t coeff_nm[ARM_DOF_MAX][JOINT_ID_FIT_FF_FEATURE_MAX];
} joint_id_fit_ff_params_t;

typedef struct {
  joint_id_fit_ff_params_t params;
} joint_id_fit_ff_t;

void joint_id_fit_ff_init(joint_id_fit_ff_t *fit, const joint_id_fit_ff_params_t *params);
arm_feedforward_t joint_id_fit_ff_as_feedforward(joint_id_fit_ff_t *fit);

#endif
