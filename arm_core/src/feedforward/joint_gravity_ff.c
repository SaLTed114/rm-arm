#include "arm_core/joint_gravity_ff.h"

#include "arm_core/arm_math.h"

typedef struct {
  arm_real_t rot[9];
  arm_real_t pos[3];
  arm_real_t joint_pos[3];
  arm_real_t joint_axis[3];
} body_pose_t;

static bool body_is_descendant(
    const joint_gravity_ff_params_t *params,
    uint8_t body,
    uint8_t ancestor_body) {
  int8_t current = (int8_t)body;
  while (current >= 0) {
    if ((uint8_t)current == ancestor_body) return true;
    current = params->bodies[(uint8_t)current].parent;
  }
  return false;
}

static int build_poses(
    const joint_gravity_ff_params_t *params,
    const arm_state_t *state,
    body_pose_t poses[JOINT_GRAVITY_FF_BODY_MAX]) {
  for (uint8_t i = 0u; i < params->body_count; ++i) {
    const joint_gravity_ff_body_t *body = &params->bodies[i];
    const body_pose_t *parent = NULL;
    if (body->parent >= 0) {
      if ((uint8_t)body->parent >= i) return ARM_ERR_CONFIG;
      parent = &poses[(uint8_t)body->parent];
    }

    arm_real_t parent_rot[9];
    arm_real_t parent_pos[3];
    if (parent) {
      for (uint8_t j = 0u; j < 9u; ++j) parent_rot[j] = parent->rot[j];
      parent_pos[0] = parent->pos[0];
      parent_pos[1] = parent->pos[1];
      parent_pos[2] = parent->pos[2];
    } else {
      arm_mat3_identity(parent_rot);
      arm_vec3_zero(parent_pos);
    }

    arm_real_t offset_world[3];
    arm_mat3_mul_vec3(parent_rot, body->pos, offset_world);
    arm_vec3_add(parent_pos, offset_world, poses[i].joint_pos);
    poses[i].pos[0] = poses[i].joint_pos[0];
    poses[i].pos[1] = poses[i].joint_pos[1];
    poses[i].pos[2] = poses[i].joint_pos[2];

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

static int joint_gravity_ff_step(void *ctx, arm_t *arm, const arm_reference_t *ref) {
  (void)ref;
  joint_gravity_ff_t *gravity = (joint_gravity_ff_t *)ctx;
  if (!gravity || !arm) return ARM_ERR_NULL;

  const joint_gravity_ff_params_t *params = &gravity->params;
  const arm_state_t *state = &arm->state;
  arm_command_t *command = &arm->command;
  if (!arm_dof_matches(params->dof, arm->config.dof) || !arm_dof_matches(params->dof, state->dof)) {
    return ARM_ERR_DOF;
  }
  if (!(state->flags & ARM_STATE_Q_VALID)) return ARM_ERR_CONFIG;

  body_pose_t poses[JOINT_GRAVITY_FF_BODY_MAX];
  const int pose_status = build_poses(params, state, poses);
  if (pose_status != ARM_OK) return pose_status;

  for (uint8_t joint = 0u; joint < params->dof; ++joint) {
    arm_real_t tau = ARM_REAL_ZERO;
    for (uint8_t body_index = 0u; body_index < params->body_count; ++body_index) {
      const joint_gravity_ff_body_t *body = &params->bodies[body_index];
      if (body->joint != (int8_t)joint) continue;

      for (uint8_t mass_body_index = 0u; mass_body_index < params->body_count; ++mass_body_index) {
        const joint_gravity_ff_body_t *mass_body = &params->bodies[mass_body_index];
        if (mass_body->mass_kg <= ARM_REAL_ZERO) continue;
        if (!body_is_descendant(params, mass_body_index, body_index)) continue;

        arm_real_t com_world_offset[3];
        arm_real_t com_world[3];
        arm_mat3_mul_vec3(poses[mass_body_index].rot, mass_body->com, com_world_offset);
        arm_vec3_add(poses[mass_body_index].pos, com_world_offset, com_world);

        arm_real_t force[3];
        arm_vec3_scale(params->gravity_m_s2, mass_body->mass_kg, force);

        arm_real_t r[3];
        arm_real_t moment[3];
        arm_vec3_sub(com_world, poses[body_index].joint_pos, r);
        arm_vec3_cross(r, force, moment);
        tau -= arm_vec3_dot(poses[body_index].joint_axis, moment);
      }
    }
    command->tau_ff_nm[joint] += tau;
  }

  command->flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static const arm_feedforward_vtable_t JOINT_GRAVITY_FF_VTABLE = {
  NULL,
  joint_gravity_ff_step,
};

void joint_gravity_ff_init(joint_gravity_ff_t *gravity, const joint_gravity_ff_params_t *params) {
  if (!gravity) return;

  gravity->params = (joint_gravity_ff_params_t){0};
  if (params) {
    gravity->params = *params;
    gravity->params.dof = arm_sanitize_dof(gravity->params.dof);
    if (gravity->params.body_count > JOINT_GRAVITY_FF_BODY_MAX) {
      gravity->params.body_count = JOINT_GRAVITY_FF_BODY_MAX;
    }
  }
}

arm_feedforward_t joint_gravity_ff_as_feedforward(joint_gravity_ff_t *gravity) {
  arm_feedforward_t feedforward;
  feedforward.vt = &JOINT_GRAVITY_FF_VTABLE;
  feedforward.ctx = gravity;
  return feedforward;
}
