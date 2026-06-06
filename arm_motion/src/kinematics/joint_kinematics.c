#include "arm_motion/joint_kinematics.h"

#include "arm_common/arm_math.h"

#include <math.h>

#define JOINT_KINEMATICS_TASK_MAX 6u

typedef struct {
  arm_real_t rot[9];
  arm_real_t pos[3];
  arm_real_t joint_pos[3];
  arm_real_t joint_axis[3];
} joint_kinematics_pose_t;

static int build_poses(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX]) {
  if (!params || !state || !poses) return ARM_ERR_NULL;
  if (!arm_dof_matches(params->dof, state->dof)) return ARM_ERR_DOF;
  if (params->body_count > JOINT_KINEMATICS_BODY_MAX) return ARM_ERR_CONFIG;

  for (uint8_t i = 0u; i < params->body_count; ++i) {
    const joint_kinematics_body_t *body = &params->bodies[i];
    const joint_kinematics_pose_t *parent = NULL;
    if (body->parent >= 0) {
      if ((uint8_t)body->parent >= i) return ARM_ERR_CONFIG;
      parent = &poses[(uint8_t)body->parent];
    }

    arm_real_t parent_rot[9];
    arm_real_t parent_pos[3];
    if (parent) {
      for (uint8_t j = 0u; j < 9u; ++j) parent_rot[j] = parent->rot[j];
      arm_vec3_copy(parent->pos, parent_pos);
    } else {
      arm_mat3_identity(parent_rot);
      arm_vec3_zero(parent_pos);
    }

    arm_real_t offset_world[3];
    arm_mat3_mul_vec3(parent_rot, body->pos, offset_world);
    arm_vec3_add(parent_pos, offset_world, poses[i].joint_pos);
    arm_vec3_copy(poses[i].joint_pos, poses[i].pos);
    arm_mat3_mul_vec3(parent_rot, body->axis, poses[i].joint_axis);

    if (body->joint >= 0) {
      if ((uint8_t)body->joint >= state->dof) return ARM_ERR_DOF;
      arm_real_t local_rot[9];
      arm_mat3_from_axis_angle(body->axis, state->q_rad[(uint8_t)body->joint], local_rot);
      arm_mat3_mul(parent_rot, local_rot, poses[i].rot);
    } else {
      for (uint8_t j = 0u; j < 9u; ++j) poses[i].rot[j] = parent_rot[j];
    }
  }

  return ARM_OK;
}

static int tool_position_from_poses(
    const joint_kinematics_params_t *params,
    const joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX],
    arm_real_t tool_pos_world[3]) {
  if (!params || !poses || !tool_pos_world) return ARM_ERR_NULL;
  if (params->tool_body < 0 || (uint8_t)params->tool_body >= params->body_count) return ARM_ERR_CONFIG;

  const joint_kinematics_pose_t *tool_body = &poses[(uint8_t)params->tool_body];
  arm_real_t tool_offset_world[3];
  arm_mat3_mul_vec3(tool_body->rot, params->tool_pos, tool_offset_world);
  arm_vec3_add(tool_body->pos, tool_offset_world, tool_pos_world);
  return ARM_OK;
}

static int tool_pose_from_poses(
    const joint_kinematics_params_t *params,
    const joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX],
    arm_real_t tool_pos_world[3],
    arm_real_t tool_rot_world[9]) {
  if (!params || !poses || !tool_pos_world || !tool_rot_world) return ARM_ERR_NULL;
  if (params->tool_body < 0 || (uint8_t)params->tool_body >= params->body_count) return ARM_ERR_CONFIG;

  const joint_kinematics_pose_t *tool_body = &poses[(uint8_t)params->tool_body];
  arm_real_t tool_offset_world[3];
  arm_mat3_mul_vec3(tool_body->rot, params->tool_pos, tool_offset_world);
  arm_vec3_add(tool_body->pos, tool_offset_world, tool_pos_world);
  arm_mat3_copy(tool_body->rot, tool_rot_world);
  return ARM_OK;
}

static bool body_is_ancestor(
    const joint_kinematics_params_t *params,
    uint8_t ancestor_body,
    uint8_t body) {
  int8_t current = (int8_t)body;
  while (current >= 0) {
    if ((uint8_t)current == ancestor_body) return true;
    current = params->bodies[(uint8_t)current].parent;
  }
  return false;
}

static void fill_jacobian(
    const joint_kinematics_params_t *params,
    const joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX],
    const arm_real_t tool_pos_world[3],
    arm_real_t jacobian[3 * ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < 3u * ARM_DOF_MAX; ++i) {
    jacobian[i] = ARM_REAL_ZERO;
  }

  for (uint8_t body_index = 0u; body_index < params->body_count; ++body_index) {
    const joint_kinematics_body_t *body = &params->bodies[body_index];
    if (body->joint < 0) continue;
    if (params->tool_body < 0 || !body_is_ancestor(params, body_index, (uint8_t)params->tool_body)) continue;
    const uint8_t joint = (uint8_t)body->joint;
    if (joint >= params->dof) continue;

    arm_real_t r[3];
    arm_real_t column[3];
    arm_vec3_sub(tool_pos_world, poses[body_index].joint_pos, r);
    arm_vec3_cross(poses[body_index].joint_axis, r, column);
    jacobian[0u * ARM_DOF_MAX + joint] = column[0];
    jacobian[1u * ARM_DOF_MAX + joint] = column[1];
    jacobian[2u * ARM_DOF_MAX + joint] = column[2];
  }
}

static void fill_spatial_jacobian(
    const joint_kinematics_params_t *params,
    const joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX],
    const arm_real_t tool_pos_world[3],
    arm_real_t jacobian[6 * ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < 6u * ARM_DOF_MAX; ++i) {
    jacobian[i] = ARM_REAL_ZERO;
  }

  for (uint8_t body_index = 0u; body_index < params->body_count; ++body_index) {
    const joint_kinematics_body_t *body = &params->bodies[body_index];
    if (body->joint < 0) continue;
    if (params->tool_body < 0 || !body_is_ancestor(params, body_index, (uint8_t)params->tool_body)) continue;
    const uint8_t joint = (uint8_t)body->joint;
    if (joint >= params->dof) continue;

    arm_real_t r[3];
    arm_real_t linear[3];
    arm_vec3_sub(tool_pos_world, poses[body_index].joint_pos, r);
    arm_vec3_cross(poses[body_index].joint_axis, r, linear);
    jacobian[0u * ARM_DOF_MAX + joint] = linear[0];
    jacobian[1u * ARM_DOF_MAX + joint] = linear[1];
    jacobian[2u * ARM_DOF_MAX + joint] = linear[2];
    jacobian[3u * ARM_DOF_MAX + joint] = poses[body_index].joint_axis[0];
    jacobian[4u * ARM_DOF_MAX + joint] = poses[body_index].joint_axis[1];
    jacobian[5u * ARM_DOF_MAX + joint] = poses[body_index].joint_axis[2];
  }
}

static arm_real_t det3(const arm_real_t m[9]) {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) -
         m[1] * (m[3] * m[8] - m[5] * m[6]) +
         m[2] * (m[3] * m[7] - m[4] * m[6]);
}

static int inv3(const arm_real_t m[9], arm_real_t out[9]) {
  const arm_real_t det = det3(m);
  if (arm_abs(det) < ARM_REAL(1e-12)) return ARM_ERR_CONFIG;
  const arm_real_t inv_det = ARM_REAL_ONE / det;
  out[0] = (m[4] * m[8] - m[5] * m[7]) * inv_det;
  out[1] = (m[2] * m[7] - m[1] * m[8]) * inv_det;
  out[2] = (m[1] * m[5] - m[2] * m[4]) * inv_det;
  out[3] = (m[5] * m[6] - m[3] * m[8]) * inv_det;
  out[4] = (m[0] * m[8] - m[2] * m[6]) * inv_det;
  out[5] = (m[2] * m[3] - m[0] * m[5]) * inv_det;
  out[6] = (m[3] * m[7] - m[4] * m[6]) * inv_det;
  out[7] = (m[1] * m[6] - m[0] * m[7]) * inv_det;
  out[8] = (m[0] * m[4] - m[1] * m[3]) * inv_det;
  return ARM_OK;
}

static int compute_dls_inverse(
    uint8_t dof,
    const arm_real_t jacobian[3 * ARM_DOF_MAX],
    arm_real_t damping,
    arm_real_t inv_a[9]) {
  arm_real_t a[9] = {0};
  for (uint8_t row = 0u; row < 3u; ++row) {
    for (uint8_t col = 0u; col < 3u; ++col) {
      arm_real_t value = ARM_REAL_ZERO;
      for (uint8_t joint = 0u; joint < dof; ++joint) {
        value += jacobian[row * ARM_DOF_MAX + joint] * jacobian[col * ARM_DOF_MAX + joint];
      }
      a[row * 3u + col] = value;
    }
  }
  const arm_real_t lambda2 = damping * damping;
  a[0] += lambda2;
  a[4] += lambda2;
  a[8] += lambda2;

  return inv3(a, inv_a);
}

static void solve_dls_delta(
    uint8_t dof,
    const arm_real_t jacobian[3 * ARM_DOF_MAX],
    const arm_real_t inv_a[9],
    const arm_real_t error[3],
    arm_real_t delta[ARM_DOF_MAX]) {
  arm_real_t y[3];
  arm_mat3_mul_vec3(inv_a, error, y);
  for (uint8_t joint = 0u; joint < ARM_DOF_MAX; ++joint) {
    delta[joint] = ARM_REAL_ZERO;
  }
  for (uint8_t joint = 0u; joint < dof; ++joint) {
    delta[joint] = jacobian[0u * ARM_DOF_MAX + joint] * y[0] +
                   jacobian[1u * ARM_DOF_MAX + joint] * y[1] +
                   jacobian[2u * ARM_DOF_MAX + joint] * y[2];
  }
}

static void compute_nullspace_posture_delta(
    uint8_t dof,
    const arm_real_t jacobian[3 * ARM_DOF_MAX],
    const arm_real_t inv_a[9],
    const arm_state_t *work,
    const arm_real_t posture_ref_rad[ARM_DOF_MAX],
    arm_real_t posture_gain,
    arm_real_t delta[ARM_DOF_MAX]) {
  arm_real_t posture_step[ARM_DOF_MAX];
  arm_real_t task_motion[3] = {0};
  for (uint8_t joint = 0u; joint < ARM_DOF_MAX; ++joint) {
    posture_step[joint] = ARM_REAL_ZERO;
    delta[joint] = ARM_REAL_ZERO;
  }

  for (uint8_t joint = 0u; joint < dof; ++joint) {
    posture_step[joint] = (posture_ref_rad[joint] - work->q_rad[joint]) * posture_gain;
  }

  for (uint8_t row = 0u; row < 3u; ++row) {
    for (uint8_t joint = 0u; joint < dof; ++joint) {
      task_motion[row] += jacobian[row * ARM_DOF_MAX + joint] * posture_step[joint];
    }
  }

  arm_real_t projected_task[ARM_DOF_MAX];
  solve_dls_delta(dof, jacobian, inv_a, task_motion, projected_task);
  for (uint8_t joint = 0u; joint < dof; ++joint) {
    delta[joint] = posture_step[joint] - projected_task[joint];
  }
}

static int solve_linear_system(
    uint8_t n,
    arm_real_t a[JOINT_KINEMATICS_TASK_MAX * JOINT_KINEMATICS_TASK_MAX],
    arm_real_t b[JOINT_KINEMATICS_TASK_MAX],
    arm_real_t x[JOINT_KINEMATICS_TASK_MAX]) {
  if (n == 0u || n > JOINT_KINEMATICS_TASK_MAX) return ARM_ERR_DOF;

  for (uint8_t col = 0u; col < n; ++col) {
    uint8_t pivot = col;
    arm_real_t pivot_abs = arm_abs(a[col * JOINT_KINEMATICS_TASK_MAX + col]);
    for (uint8_t row = (uint8_t)(col + 1u); row < n; ++row) {
      const arm_real_t candidate = arm_abs(a[row * JOINT_KINEMATICS_TASK_MAX + col]);
      if (candidate > pivot_abs) {
        pivot = row;
        pivot_abs = candidate;
      }
    }
    if (pivot_abs < ARM_REAL(1e-12)) return ARM_ERR_CONFIG;

    if (pivot != col) {
      for (uint8_t k = col; k < n; ++k) {
        const uint8_t a_col = k;
        const arm_real_t tmp = a[col * JOINT_KINEMATICS_TASK_MAX + a_col];
        a[col * JOINT_KINEMATICS_TASK_MAX + a_col] = a[pivot * JOINT_KINEMATICS_TASK_MAX + a_col];
        a[pivot * JOINT_KINEMATICS_TASK_MAX + a_col] = tmp;
      }
      const arm_real_t tmp_b = b[col];
      b[col] = b[pivot];
      b[pivot] = tmp_b;
    }

    const arm_real_t diag = a[col * JOINT_KINEMATICS_TASK_MAX + col];
    for (uint8_t row = (uint8_t)(col + 1u); row < n; ++row) {
      const arm_real_t factor = a[row * JOINT_KINEMATICS_TASK_MAX + col] / diag;
      a[row * JOINT_KINEMATICS_TASK_MAX + col] = ARM_REAL_ZERO;
      for (uint8_t k = (uint8_t)(col + 1u); k < n; ++k) {
        a[row * JOINT_KINEMATICS_TASK_MAX + k] -= factor * a[col * JOINT_KINEMATICS_TASK_MAX + k];
      }
      b[row] -= factor * b[col];
    }
  }

  for (uint8_t i = 0u; i < JOINT_KINEMATICS_TASK_MAX; ++i) {
    x[i] = ARM_REAL_ZERO;
  }
  for (int row = (int)n - 1; row >= 0; --row) {
    arm_real_t value = b[row];
    for (uint8_t col = (uint8_t)(row + 1); col < n; ++col) {
      value -= a[(uint8_t)row * JOINT_KINEMATICS_TASK_MAX + col] * x[col];
    }
    x[(uint8_t)row] = value / a[(uint8_t)row * JOINT_KINEMATICS_TASK_MAX + (uint8_t)row];
  }
  return ARM_OK;
}

static int solve_dls_delta_task(
    uint8_t dof,
    uint8_t task_dim,
    const arm_real_t jacobian[JOINT_KINEMATICS_TASK_MAX * ARM_DOF_MAX],
    const arm_real_t error[JOINT_KINEMATICS_TASK_MAX],
    arm_real_t damping,
    arm_real_t delta[ARM_DOF_MAX]) {
  arm_real_t a[JOINT_KINEMATICS_TASK_MAX * JOINT_KINEMATICS_TASK_MAX] = {0};
  for (uint8_t row = 0u; row < task_dim; ++row) {
    for (uint8_t col = 0u; col < task_dim; ++col) {
      arm_real_t value = ARM_REAL_ZERO;
      for (uint8_t joint = 0u; joint < dof; ++joint) {
        value += jacobian[row * ARM_DOF_MAX + joint] * jacobian[col * ARM_DOF_MAX + joint];
      }
      a[row * JOINT_KINEMATICS_TASK_MAX + col] = value;
    }
  }
  const arm_real_t lambda2 = damping * damping;
  for (uint8_t i = 0u; i < task_dim; ++i) {
    a[i * JOINT_KINEMATICS_TASK_MAX + i] += lambda2;
  }

  arm_real_t b[JOINT_KINEMATICS_TASK_MAX] = {0};
  arm_real_t y[JOINT_KINEMATICS_TASK_MAX] = {0};
  for (uint8_t i = 0u; i < task_dim; ++i) {
    b[i] = error[i];
  }
  const int status = solve_linear_system(task_dim, a, b, y);
  if (status != ARM_OK) {
    for (uint8_t joint = 0u; joint < ARM_DOF_MAX; ++joint) delta[joint] = ARM_REAL_ZERO;
    return status;
  }

  for (uint8_t joint = 0u; joint < ARM_DOF_MAX; ++joint) {
    delta[joint] = ARM_REAL_ZERO;
  }
  for (uint8_t joint = 0u; joint < dof; ++joint) {
    for (uint8_t row = 0u; row < task_dim; ++row) {
      delta[joint] += jacobian[row * ARM_DOF_MAX + joint] * y[row];
    }
  }
  return ARM_OK;
}

static void compute_nullspace_posture_delta_task(
    uint8_t dof,
    uint8_t task_dim,
    const arm_real_t jacobian[JOINT_KINEMATICS_TASK_MAX * ARM_DOF_MAX],
    const arm_state_t *work,
    const arm_real_t posture_ref_rad[ARM_DOF_MAX],
    arm_real_t posture_gain,
    arm_real_t damping,
    arm_real_t delta[ARM_DOF_MAX]) {
  arm_real_t posture_step[ARM_DOF_MAX];
  arm_real_t task_motion[JOINT_KINEMATICS_TASK_MAX] = {0};
  for (uint8_t joint = 0u; joint < ARM_DOF_MAX; ++joint) {
    posture_step[joint] = ARM_REAL_ZERO;
    delta[joint] = ARM_REAL_ZERO;
  }

  for (uint8_t joint = 0u; joint < dof; ++joint) {
    posture_step[joint] = (posture_ref_rad[joint] - work->q_rad[joint]) * posture_gain;
  }
  for (uint8_t row = 0u; row < task_dim; ++row) {
    for (uint8_t joint = 0u; joint < dof; ++joint) {
      task_motion[row] += jacobian[row * ARM_DOF_MAX + joint] * posture_step[joint];
    }
  }

  arm_real_t projected_task[ARM_DOF_MAX];
  if (solve_dls_delta_task(dof, task_dim, jacobian, task_motion, damping, projected_task) != ARM_OK) return;
  for (uint8_t joint = 0u; joint < dof; ++joint) {
    delta[joint] = posture_step[joint] - projected_task[joint];
  }
}

static void rotation_error_vector(
    const arm_real_t target_rot[9],
    const arm_real_t current_rot[9],
    arm_real_t error[3]) {
  arm_real_t current_rot_t[9];
  arm_real_t rot_err[9];
  arm_mat3_transpose(current_rot, current_rot_t);
  arm_mat3_mul(target_rot, current_rot_t, rot_err);

  const arm_real_t trace = rot_err[0] + rot_err[4] + rot_err[8];
  const arm_real_t cos_angle = arm_clamp((trace - ARM_REAL_ONE) * ARM_REAL(0.5), -ARM_REAL_ONE, ARM_REAL_ONE);
  const arm_real_t angle = ARM_REAL(acos((double)cos_angle));
  const arm_real_t vee[3] = {
      rot_err[7] - rot_err[5],
      rot_err[2] - rot_err[6],
      rot_err[3] - rot_err[1],
  };
  if (angle < ARM_REAL(1e-6)) {
    error[0] = vee[0] * ARM_REAL(0.5);
    error[1] = vee[1] * ARM_REAL(0.5);
    error[2] = vee[2] * ARM_REAL(0.5);
    return;
  }

  const arm_real_t sin_angle = ARM_REAL(sin((double)angle));
  if (arm_abs(sin_angle) < ARM_REAL(1e-6)) {
    error[0] = vee[0] * ARM_REAL(0.5);
    error[1] = vee[1] * ARM_REAL(0.5);
    error[2] = vee[2] * ARM_REAL(0.5);
    return;
  }
  const arm_real_t scale = angle / (ARM_REAL(2.0) * sin_angle);
  error[0] = vee[0] * scale;
  error[1] = vee[1] * scale;
  error[2] = vee[2] * scale;
}

static void clamp_work_to_joint_limits(const joint_kinematics_params_t *params, arm_state_t *work) {
  for (uint8_t body_index = 0u; body_index < params->body_count; ++body_index) {
    const joint_kinematics_body_t *body = &params->bodies[body_index];
    if (body->joint < 0) continue;
    const uint8_t joint = (uint8_t)body->joint;
    if (joint >= params->dof) continue;
    work->q_rad[joint] = arm_clamp(work->q_rad[joint], body->q_min_rad, body->q_max_rad);
  }
}

int joint_kinematics_fk_position(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3]) {
  if (!params || !state || !tool_pos_world) return ARM_ERR_NULL;
  joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX];
  const int status = build_poses(params, state, poses);
  if (status != ARM_OK) return status;
  return tool_position_from_poses(params, poses, tool_pos_world);
}

int joint_kinematics_fk_pose(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3],
    arm_real_t tool_rot_world[9]) {
  if (!params || !state || !tool_pos_world || !tool_rot_world) return ARM_ERR_NULL;
  joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX];
  const int status = build_poses(params, state, poses);
  if (status != ARM_OK) return status;
  return tool_pose_from_poses(params, poses, tool_pos_world, tool_rot_world);
}

int joint_kinematics_position_jacobian(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3],
    arm_real_t jacobian[3 * ARM_DOF_MAX]) {
  if (!params || !state || !tool_pos_world || !jacobian) return ARM_ERR_NULL;
  joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX];
  int status = build_poses(params, state, poses);
  if (status != ARM_OK) return status;
  status = tool_position_from_poses(params, poses, tool_pos_world);
  if (status != ARM_OK) return status;
  fill_jacobian(params, poses, tool_pos_world, jacobian);
  return ARM_OK;
}

int joint_kinematics_spatial_jacobian(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    arm_real_t tool_pos_world[3],
    arm_real_t tool_rot_world[9],
    arm_real_t jacobian[6 * ARM_DOF_MAX]) {
  if (!params || !state || !tool_pos_world || !tool_rot_world || !jacobian) return ARM_ERR_NULL;
  joint_kinematics_pose_t poses[JOINT_KINEMATICS_BODY_MAX];
  int status = build_poses(params, state, poses);
  if (status != ARM_OK) return status;
  status = tool_pose_from_poses(params, poses, tool_pos_world, tool_rot_world);
  if (status != ARM_OK) return status;
  fill_spatial_jacobian(params, poses, tool_pos_world, jacobian);
  return ARM_OK;
}

int joint_ik_position_solve(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    const arm_real_t target_pos_world[3],
    const joint_ik_position_options_t *options,
    arm_reference_t *ref) {
  if (!params || !state || !target_pos_world || !ref) return ARM_ERR_NULL;
  if (!arm_dof_matches(params->dof, state->dof)) return ARM_ERR_DOF;

  joint_ik_position_options_t opt = {16u, ARM_REAL(0.03), ARM_REAL(0.08), ARM_REAL(0.002), NULL, ARM_REAL_ZERO};
  if (options) opt = *options;
  if (opt.max_iterations == 0u) opt.max_iterations = 1u;
  if (opt.damping <= ARM_REAL_ZERO) opt.damping = ARM_REAL(0.03);
  if (opt.step_limit_rad <= ARM_REAL_ZERO) opt.step_limit_rad = ARM_REAL(0.08);
  if (opt.tolerance_m <= ARM_REAL_ZERO) opt.tolerance_m = ARM_REAL(0.002);
  if (!opt.posture_ref_rad) opt.posture_gain = ARM_REAL_ZERO;
  if (opt.posture_gain < ARM_REAL_ZERO) opt.posture_gain = ARM_REAL_ZERO;

  arm_state_t work = *state;
  arm_reference_zero(ref, params->dof);
  ref->flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;

  for (uint8_t iter = 0u; iter < opt.max_iterations; ++iter) {
    arm_real_t tool_pos[3];
    arm_real_t jacobian[3 * ARM_DOF_MAX];
    const int status = joint_kinematics_position_jacobian(params, &work, tool_pos, jacobian);
    if (status != ARM_OK) return status;

    arm_real_t error[3];
    arm_vec3_sub(target_pos_world, tool_pos, error);
    if (arm_vec3_norm(error) <= opt.tolerance_m) break;

    arm_real_t inv_a[9];
    if (compute_dls_inverse(params->dof, jacobian, opt.damping, inv_a) != ARM_OK) break;

    arm_real_t delta[ARM_DOF_MAX];
    solve_dls_delta(params->dof, jacobian, inv_a, error, delta);
    if (opt.posture_ref_rad && opt.posture_gain > ARM_REAL_ZERO) {
      arm_real_t null_delta[ARM_DOF_MAX];
      compute_nullspace_posture_delta(
          params->dof, jacobian, inv_a, &work, opt.posture_ref_rad, opt.posture_gain, null_delta);
      for (uint8_t joint = 0u; joint < params->dof; ++joint) {
        delta[joint] += null_delta[joint];
      }
    }
    for (uint8_t joint = 0u; joint < params->dof; ++joint) {
      const arm_real_t dq = arm_clamp(delta[joint], -opt.step_limit_rad, opt.step_limit_rad);
      work.q_rad[joint] += dq;
    }
    clamp_work_to_joint_limits(params, &work);
  }

  for (uint8_t i = 0u; i < params->dof; ++i) {
    ref->q_ref_rad[i] = work.q_rad[i];
    ref->dq_ref_rad_s[i] = ARM_REAL_ZERO;
    ref->ddq_ref_rad_s2[i] = ARM_REAL_ZERO;
  }
  return ARM_OK;
}

int joint_ik_pose_solve(
    const joint_kinematics_params_t *params,
    const arm_state_t *state,
    const arm_real_t target_pos_world[3],
    const arm_real_t target_rot_world[9],
    const joint_ik_pose_options_t *options,
    arm_reference_t *ref) {
  if (!params || !state || !target_pos_world || !target_rot_world || !ref) return ARM_ERR_NULL;
  if (!arm_dof_matches(params->dof, state->dof)) return ARM_ERR_DOF;

  joint_ik_pose_options_t opt = {
      20u,
      ARM_REAL(0.04),
      ARM_REAL(0.06),
      ARM_REAL(0.002),
      ARM_REAL(0.01),
      ARM_REAL(0.45),
      NULL,
      ARM_REAL_ZERO,
  };
  if (options) opt = *options;
  if (opt.max_iterations == 0u) opt.max_iterations = 1u;
  if (opt.damping <= ARM_REAL_ZERO) opt.damping = ARM_REAL(0.04);
  if (opt.step_limit_rad <= ARM_REAL_ZERO) opt.step_limit_rad = ARM_REAL(0.06);
  if (opt.pos_tolerance_m <= ARM_REAL_ZERO) opt.pos_tolerance_m = ARM_REAL(0.002);
  if (opt.rot_tolerance_rad <= ARM_REAL_ZERO) opt.rot_tolerance_rad = ARM_REAL(0.01);
  if (opt.rot_weight <= ARM_REAL_ZERO) opt.rot_weight = ARM_REAL(0.45);
  if (!opt.posture_ref_rad) opt.posture_gain = ARM_REAL_ZERO;
  if (opt.posture_gain < ARM_REAL_ZERO) opt.posture_gain = ARM_REAL_ZERO;

  arm_state_t work = *state;
  arm_reference_zero(ref, params->dof);
  ref->flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;

  for (uint8_t iter = 0u; iter < opt.max_iterations; ++iter) {
    arm_real_t tool_pos[3];
    arm_real_t tool_rot[9];
    arm_real_t jacobian[6 * ARM_DOF_MAX];
    const int status = joint_kinematics_spatial_jacobian(params, &work, tool_pos, tool_rot, jacobian);
    if (status != ARM_OK) return status;

    arm_real_t pos_error[3];
    arm_real_t rot_error[3];
    arm_vec3_sub(target_pos_world, tool_pos, pos_error);
    rotation_error_vector(target_rot_world, tool_rot, rot_error);
    if (arm_vec3_norm(pos_error) <= opt.pos_tolerance_m &&
        arm_vec3_norm(rot_error) <= opt.rot_tolerance_rad) break;

    arm_real_t task_error[JOINT_KINEMATICS_TASK_MAX] = {0};
    arm_real_t task_jacobian[JOINT_KINEMATICS_TASK_MAX * ARM_DOF_MAX] = {0};
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      task_error[axis] = pos_error[axis];
      task_error[axis + 3u] = rot_error[axis] * opt.rot_weight;
      for (uint8_t joint = 0u; joint < params->dof; ++joint) {
        task_jacobian[axis * ARM_DOF_MAX + joint] = jacobian[axis * ARM_DOF_MAX + joint];
        task_jacobian[(axis + 3u) * ARM_DOF_MAX + joint] =
            jacobian[(axis + 3u) * ARM_DOF_MAX + joint] * opt.rot_weight;
      }
    }

    arm_real_t delta[ARM_DOF_MAX];
    if (solve_dls_delta_task(params->dof, 6u, task_jacobian, task_error, opt.damping, delta) != ARM_OK) break;
    if (opt.posture_ref_rad && opt.posture_gain > ARM_REAL_ZERO) {
      arm_real_t null_delta[ARM_DOF_MAX];
      compute_nullspace_posture_delta_task(
          params->dof, 6u, task_jacobian, &work, opt.posture_ref_rad, opt.posture_gain, opt.damping, null_delta);
      for (uint8_t joint = 0u; joint < params->dof; ++joint) {
        delta[joint] += null_delta[joint];
      }
    }

    for (uint8_t joint = 0u; joint < params->dof; ++joint) {
      const arm_real_t dq = arm_clamp(delta[joint], -opt.step_limit_rad, opt.step_limit_rad);
      work.q_rad[joint] += dq;
    }
    clamp_work_to_joint_limits(params, &work);
  }

  for (uint8_t i = 0u; i < params->dof; ++i) {
    ref->q_ref_rad[i] = work.q_rad[i];
    ref->dq_ref_rad_s[i] = ARM_REAL_ZERO;
    ref->ddq_ref_rad_s2[i] = ARM_REAL_ZERO;
  }
  return ARM_OK;
}
