/* Generated from configs/arm6_placeholder.yaml by tools/fit_inverse_dynamics_ff.py. Do not edit by hand. */
#ifndef ARMSIM_ARM6_ID_FIT_PARAMS_H_
#define ARMSIM_ARM6_ID_FIT_PARAMS_H_

#include "arm_core/joint_id_fit_ff.h"

/* fit_mode=zero-placeholder feature_count=1 */
#define ARMSIM_ARM6_ID_FIT_PARAMS \
  { \
    6u, \
    1u, \
    { \
      { JOINT_ID_FIT_FEATURE_BIAS, 0u, 0u }, \
    }, \
    { \
      { ARM_REAL_ZERO }, \
      { ARM_REAL_ZERO }, \
      { ARM_REAL_ZERO }, \
      { ARM_REAL_ZERO }, \
      { ARM_REAL_ZERO }, \
      { ARM_REAL_ZERO }, \
    }, \
  }

#endif
