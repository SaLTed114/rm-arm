#ifndef ARM_CORE_ARM_MATH_H_
#define ARM_CORE_ARM_MATH_H_

#include "arm_core/arm_types.h"

arm_real_t arm_abs(arm_real_t value);
arm_real_t arm_clamp(arm_real_t value, arm_real_t min_value, arm_real_t max_value);
bool arm_dof_is_valid(uint8_t dof);
bool arm_dof_matches(uint8_t expected_dof, uint8_t actual_dof);
uint8_t arm_sanitize_dof(uint8_t dof);

void arm_vec3_zero(arm_real_t out[3]);
void arm_vec3_add(const arm_real_t a[3], const arm_real_t b[3], arm_real_t out[3]);
void arm_vec3_sub(const arm_real_t a[3], const arm_real_t b[3], arm_real_t out[3]);
void arm_vec3_scale(const arm_real_t a[3], arm_real_t scale, arm_real_t out[3]);
arm_real_t arm_vec3_dot(const arm_real_t a[3], const arm_real_t b[3]);
void arm_vec3_cross(const arm_real_t a[3], const arm_real_t b[3], arm_real_t out[3]);

void arm_mat3_identity(arm_real_t out[9]);
void arm_mat3_mul(const arm_real_t a[9], const arm_real_t b[9], arm_real_t out[9]);
void arm_mat3_mul_vec3(const arm_real_t mat[9], const arm_real_t vec[3], arm_real_t out[3]);
void arm_mat3_from_axis_angle(const arm_real_t axis[3], arm_real_t angle, arm_real_t out[9]);

#endif
