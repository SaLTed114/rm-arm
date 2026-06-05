#ifndef ARM_CORE_ARM_MATH_H_
#define ARM_CORE_ARM_MATH_H_

#include "arm_core/arm_types.h"

arm_real_t arm_abs(arm_real_t value);
arm_real_t arm_clamp(arm_real_t value, arm_real_t min_value, arm_real_t max_value);
bool arm_dof_is_valid(uint8_t dof);
bool arm_dof_matches(uint8_t expected_dof, uint8_t actual_dof);
uint8_t arm_sanitize_dof(uint8_t dof);

#endif
