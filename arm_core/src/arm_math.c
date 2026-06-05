#include "arm_core/arm_math.h"

arm_real_t arm_abs(arm_real_t value) {
  return value < ARM_REAL_ZERO ? -value : value;
}

arm_real_t arm_clamp(arm_real_t value, arm_real_t min_value, arm_real_t max_value) {
  if (min_value > max_value) {
    const arm_real_t tmp = min_value;
    min_value = max_value;
    max_value = tmp;
  }
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

uint8_t arm_sanitize_dof(uint8_t dof) {
  if (dof > ARM_DOF_MAX) {
    return ARM_DOF_MAX;
  }
  return dof;
}
