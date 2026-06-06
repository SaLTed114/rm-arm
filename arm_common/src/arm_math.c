#include "arm_common/arm_math.h"

#include <math.h>

arm_real_t arm_abs(arm_real_t value) {
  return value < ARM_REAL_ZERO ? -value : value;
}

arm_real_t arm_clamp(arm_real_t value, arm_real_t min_value, arm_real_t max_value) {
  if (min_value > max_value) {
    const arm_real_t tmp = min_value;
    min_value = max_value;
    max_value = tmp;
  }
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

bool arm_dof_is_valid(uint8_t dof) {
  return dof > 0u && dof <= ARM_DOF_MAX;
}

bool arm_dof_matches(uint8_t expected_dof, uint8_t actual_dof) {
  return arm_dof_is_valid(expected_dof) && actual_dof == expected_dof;
}

uint8_t arm_sanitize_dof(uint8_t dof) {
  if (dof > ARM_DOF_MAX) return ARM_DOF_MAX;
  return dof;
}

void arm_vec3_zero(arm_real_t out[3]) {
  out[0] = ARM_REAL_ZERO;
  out[1] = ARM_REAL_ZERO;
  out[2] = ARM_REAL_ZERO;
}

void arm_vec3_copy(const arm_real_t in[3], arm_real_t out[3]) {
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
}

void arm_vec3_add(const arm_real_t a[3], const arm_real_t b[3], arm_real_t out[3]) {
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
  out[2] = a[2] + b[2];
}

void arm_vec3_sub(const arm_real_t a[3], const arm_real_t b[3], arm_real_t out[3]) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

void arm_vec3_scale(const arm_real_t a[3], arm_real_t scale, arm_real_t out[3]) {
  out[0] = a[0] * scale;
  out[1] = a[1] * scale;
  out[2] = a[2] * scale;
}

arm_real_t arm_vec3_dot(const arm_real_t a[3], const arm_real_t b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void arm_vec3_cross(const arm_real_t a[3], const arm_real_t b[3], arm_real_t out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

arm_real_t arm_vec3_norm(const arm_real_t value[3]) {
  return ARM_REAL(sqrt((double)arm_vec3_dot(value, value)));
}

void arm_mat3_identity(arm_real_t out[9]) {
  out[0] = ARM_REAL_ONE;
  out[1] = ARM_REAL_ZERO;
  out[2] = ARM_REAL_ZERO;
  out[3] = ARM_REAL_ZERO;
  out[4] = ARM_REAL_ONE;
  out[5] = ARM_REAL_ZERO;
  out[6] = ARM_REAL_ZERO;
  out[7] = ARM_REAL_ZERO;
  out[8] = ARM_REAL_ONE;
}

void arm_mat3_copy(const arm_real_t in[9], arm_real_t out[9]) {
  for (uint8_t i = 0u; i < 9u; ++i) {
    out[i] = in[i];
  }
}

void arm_mat3_transpose(const arm_real_t in[9], arm_real_t out[9]) {
  out[0] = in[0];
  out[1] = in[3];
  out[2] = in[6];
  out[3] = in[1];
  out[4] = in[4];
  out[5] = in[7];
  out[6] = in[2];
  out[7] = in[5];
  out[8] = in[8];
}

void arm_mat3_mul(const arm_real_t a[9], const arm_real_t b[9], arm_real_t out[9]) {
  for (uint8_t row = 0u; row < 3u; ++row) {
    for (uint8_t col = 0u; col < 3u; ++col) {
      out[row * 3u + col] = a[row * 3u + 0u] * b[0u * 3u + col] +
                            a[row * 3u + 1u] * b[1u * 3u + col] +
                            a[row * 3u + 2u] * b[2u * 3u + col];
    }
  }
}

void arm_mat3_mul_vec3(const arm_real_t mat[9], const arm_real_t vec[3], arm_real_t out[3]) {
  out[0] = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2];
  out[1] = mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2];
  out[2] = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2];
}

void arm_mat3_from_axis_angle(const arm_real_t axis_in[3], arm_real_t angle, arm_real_t out[9]) {
  arm_real_t axis[3] = {axis_in[0], axis_in[1], axis_in[2]};
  const arm_real_t norm2 = arm_vec3_dot(axis, axis);
  if (norm2 <= ARM_REAL_ZERO) {
    arm_mat3_identity(out);
    return;
  }

  const arm_real_t norm = ARM_REAL(sqrt((double)norm2));
  axis[0] /= norm;
  axis[1] /= norm;
  axis[2] /= norm;

  const arm_real_t c = ARM_REAL(cos((double)angle));
  const arm_real_t s = ARM_REAL(sin((double)angle));
  const arm_real_t one_c = ARM_REAL_ONE - c;
  const arm_real_t x = axis[0];
  const arm_real_t y = axis[1];
  const arm_real_t z = axis[2];

  out[0] = c + x * x * one_c;
  out[1] = x * y * one_c - z * s;
  out[2] = x * z * one_c + y * s;
  out[3] = y * x * one_c + z * s;
  out[4] = c + y * y * one_c;
  out[5] = y * z * one_c - x * s;
  out[6] = z * x * one_c - y * s;
  out[7] = z * y * one_c + x * s;
  out[8] = c + z * z * one_c;
}
