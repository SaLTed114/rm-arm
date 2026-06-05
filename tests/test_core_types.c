#include <assert.h>
#include <math.h>

#include "arm_core/arm_safety.h"
#include "arm_core/arm_control.h"
#include "arm_core/arm_feedforward.h"
#include "arm_core/arm_math.h"
#include "arm_core/joint_kinematics.h"
#include "arm_core/joint_ref_shaper.h"
#include "arm_core/joint_pd.h"
#include "arm_core/joint_sweep.h"

static arm_config_t make_test_config(void) {
  arm_config_t config = {0};
  config.dof = ARM_DEFAULT_DOF;
  for (uint8_t i = 0u; i < config.dof; ++i) {
    config.joints[i].joint_name = "joint";
    config.joints[i].actuator_name = "motor";
    config.joints[i].sign = 1.0;
    config.joints[i].q_offset_rad = 0.0;
    config.joints[i].torque_limit_nm = 1.5;
  }
  return config;
}

static int near_zero(arm_real_t value) {
  return fabs((double)value) < 1.0e-9;
}

static void make_planar_kinematics(joint_kinematics_params_t *params) {
  *params = (joint_kinematics_params_t){0};
  params->dof = 2u;
  params->body_count = 2u;
  params->tool_body = 1;
  params->tool_pos[0] = 1.0;
  params->bodies[0] =
      (joint_kinematics_body_t){JOINT_KINEMATICS_NO_BODY, 0, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -3.14, 3.14};
  params->bodies[1] =
      (joint_kinematics_body_t){0, 1, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -3.14, 3.14};
}

static void make_planar3_kinematics(joint_kinematics_params_t *params) {
  *params = (joint_kinematics_params_t){0};
  params->dof = 3u;
  params->body_count = 3u;
  params->tool_body = 2;
  params->tool_pos[0] = 1.0;
  params->bodies[0] =
      (joint_kinematics_body_t){JOINT_KINEMATICS_NO_BODY, 0, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -3.14, 3.14};
  params->bodies[1] =
      (joint_kinematics_body_t){0, 1, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -3.14, 3.14};
  params->bodies[2] =
      (joint_kinematics_body_t){1, 2, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -3.14, 3.14};
}

static arm_real_t position_error_norm(const arm_real_t a[3], const arm_real_t b[3]) {
  const arm_real_t dx = a[0] - b[0];
  const arm_real_t dy = a[1] - b[1];
  const arm_real_t dz = a[2] - b[2];
  return ARM_REAL(sqrt((double)(dx * dx + dy * dy + dz * dz)));
}

static arm_real_t rotation_matrix_delta_norm(const arm_real_t a[9], const arm_real_t b[9]) {
  arm_real_t total = ARM_REAL_ZERO;
  for (uint8_t i = 0u; i < 9u; ++i) {
    const arm_real_t d = a[i] - b[i];
    total += d * d;
  }
  return ARM_REAL(sqrt((double)total));
}

typedef struct {
  uint8_t dof;
  arm_real_t tau_nm[ARM_DOF_MAX];
} test_feedforward_t;

static int test_feedforward_step(void *ctx, arm_t *arm, const arm_reference_t *ref) {
  (void)ref;
  test_feedforward_t *feedforward = (test_feedforward_t *)ctx;
  if (!feedforward || !arm) return ARM_ERR_NULL;
  if (!arm_dof_matches(feedforward->dof, arm->config.dof)) return ARM_ERR_DOF;

  for (uint8_t i = 0u; i < feedforward->dof; ++i) {
    arm->command.tau_ff_nm[i] += feedforward->tau_nm[i];
  }
  arm->command.flags |= ARM_COMMAND_TAU_FF_VALID;
  return ARM_OK;
}

static const arm_feedforward_vtable_t TEST_FEEDFORWARD_VTABLE = {
  NULL,
  test_feedforward_step,
};

int main(void) {
  arm_config_t config = make_test_config();
  assert(arm_config_validate(&config) == ARM_OK);

  arm_t arm;
  assert(arm_init(&arm, &config) == ARM_OK);
  arm.state.dt_s = 0.001;
  arm.state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  arm_reference_t ref;
  arm_reference_zero(&ref, arm.config.dof);

  joint_kinematics_params_t kin;
  make_planar_kinematics(&kin);
  arm_state_t kin_state;
  arm_state_zero(&kin_state, 2u);
  kin_state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  arm_real_t tool_pos[3];
  assert(joint_kinematics_fk_position(&kin, &kin_state, tool_pos) == ARM_OK);
  assert(fabs((double)(tool_pos[0] - 2.0)) < 1.0e-9);
  assert(near_zero(tool_pos[1]));

  kin_state.q_rad[0] = 0.2;
  kin_state.q_rad[1] = -0.4;
  arm_real_t jacobian[3 * ARM_DOF_MAX];
  assert(joint_kinematics_position_jacobian(&kin, &kin_state, tool_pos, jacobian) == ARM_OK);
  arm_real_t tool_pose_pos[3];
  arm_real_t tool_pose_rot[9];
  assert(joint_kinematics_fk_pose(&kin, &kin_state, tool_pose_pos, tool_pose_rot) == ARM_OK);
  assert(position_error_norm(tool_pose_pos, tool_pos) < ARM_REAL(1e-9));
  arm_real_t spatial_jacobian[6 * ARM_DOF_MAX];
  assert(joint_kinematics_spatial_jacobian(&kin, &kin_state, tool_pose_pos, tool_pose_rot, spatial_jacobian) ==
         ARM_OK);
  assert(fabs((double)(spatial_jacobian[3u * ARM_DOF_MAX + 0u] - 0.0)) < 1.0e-9);
  assert(fabs((double)(spatial_jacobian[4u * ARM_DOF_MAX + 0u] - 0.0)) < 1.0e-9);
  assert(fabs((double)(spatial_jacobian[5u * ARM_DOF_MAX + 0u] - 1.0)) < 1.0e-9);
  const arm_real_t eps = ARM_REAL(1e-5);
  for (uint8_t joint = 0u; joint < kin.dof; ++joint) {
    arm_state_t plus = kin_state;
    plus.q_rad[joint] += eps;
    arm_real_t plus_pos[3];
    assert(joint_kinematics_fk_position(&kin, &plus, plus_pos) == ARM_OK);
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
      const arm_real_t fd = (plus_pos[axis] - tool_pos[axis]) / eps;
      assert(fabs((double)(fd - jacobian[axis * ARM_DOF_MAX + joint])) < 1.0e-4);
    }
  }

  const arm_real_t target[3] = {1.65, 0.45, 0.0};
  arm_real_t before_pos[3];
  assert(joint_kinematics_fk_position(&kin, &kin_state, before_pos) == ARM_OK);
  const arm_real_t before_err = position_error_norm(before_pos, target);
  arm_reference_t ik_ref;
  assert(joint_ik_position_solve(&kin, &kin_state, target, NULL, &ik_ref) == ARM_OK);
  arm_state_t solved = kin_state;
  solved.q_rad[0] = ik_ref.q_ref_rad[0];
  solved.q_rad[1] = ik_ref.q_ref_rad[1];
  arm_real_t after_pos[3];
  assert(joint_kinematics_fk_position(&kin, &solved, after_pos) == ARM_OK);
  assert(position_error_norm(after_pos, target) < before_err);
  assert(ik_ref.q_ref_rad[0] >= kin.bodies[0].q_min_rad && ik_ref.q_ref_rad[0] <= kin.bodies[0].q_max_rad);
  assert(ik_ref.q_ref_rad[1] >= kin.bodies[1].q_min_rad && ik_ref.q_ref_rad[1] <= kin.bodies[1].q_max_rad);

  joint_kinematics_params_t kin3;
  make_planar3_kinematics(&kin3);
  arm_state_t kin3_state;
  arm_state_zero(&kin3_state, 3u);
  kin3_state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  kin3_state.q_rad[0] = ARM_REAL(0.35);
  kin3_state.q_rad[1] = ARM_REAL(-0.55);
  kin3_state.q_rad[2] = ARM_REAL(0.45);
  arm_real_t kin3_seed_q[ARM_DOF_MAX] = {0};
  for (uint8_t i = 0u; i < kin3.dof; ++i) {
    kin3_seed_q[i] = kin3_state.q_rad[i];
  }
  arm_real_t kin3_pos[3];
  assert(joint_kinematics_fk_position(&kin3, &kin3_state, kin3_pos) == ARM_OK);
  arm_real_t kin3_pose_rot[9];
  assert(joint_kinematics_fk_pose(&kin3, &kin3_state, kin3_pos, kin3_pose_rot) == ARM_OK);
  const arm_real_t kin3_pose_target[3] = {kin3_pos[0] - ARM_REAL(0.05), kin3_pos[1] + ARM_REAL(0.03), kin3_pos[2]};
  const joint_ik_pose_options_t pose_options = {
      24u, ARM_REAL(0.04), ARM_REAL(0.05), ARM_REAL(0.002), ARM_REAL(0.02), ARM_REAL(0.7), NULL, ARM_REAL_ZERO};
  arm_reference_t pose_ik_ref;
  assert(joint_ik_pose_solve(&kin3, &kin3_state, kin3_pose_target, kin3_pose_rot, &pose_options, &pose_ik_ref) ==
         ARM_OK);
  arm_state_t pose_solved = kin3_state;
  for (uint8_t i = 0u; i < kin3.dof; ++i) {
    pose_solved.q_rad[i] = pose_ik_ref.q_ref_rad[i];
  }
  arm_real_t pose_after_pos[3];
  arm_real_t pose_after_rot[9];
  assert(joint_kinematics_fk_pose(&kin3, &pose_solved, pose_after_pos, pose_after_rot) == ARM_OK);
  assert(position_error_norm(pose_after_pos, kin3_pose_target) < position_error_norm(kin3_pos, kin3_pose_target));
  assert(rotation_matrix_delta_norm(pose_after_rot, kin3_pose_rot) < ARM_REAL(0.08));

  const arm_real_t kin3_target[3] = {kin3_pos[0] - ARM_REAL(0.08), kin3_pos[1] + ARM_REAL(0.04), kin3_pos[2]};
  arm_reference_t ik_free_ref;
  assert(joint_ik_position_solve(&kin3, &kin3_state, kin3_target, NULL, &ik_free_ref) == ARM_OK);
  const joint_ik_position_options_t posture_options = {
      16u, ARM_REAL(0.03), ARM_REAL(0.08), ARM_REAL(0.002), kin3_seed_q, ARM_REAL(0.25)};
  arm_reference_t ik_posture_ref;
  assert(joint_ik_position_solve(&kin3, &kin3_state, kin3_target, &posture_options, &ik_posture_ref) == ARM_OK);
  arm_real_t free_motion = ARM_REAL_ZERO;
  arm_real_t posture_motion = ARM_REAL_ZERO;
  for (uint8_t i = 0u; i < kin3.dof; ++i) {
    free_motion += arm_abs(ik_free_ref.q_ref_rad[i] - kin3_seed_q[i]);
    posture_motion += arm_abs(ik_posture_ref.q_ref_rad[i] - kin3_seed_q[i]);
  }
  assert(posture_motion <= free_motion + ARM_REAL(1e-6));

  joint_sweep_t sweep;
  joint_sweep_init(&sweep, arm.config.dof, (joint_sweep_params_t){2.0, 0.1, 0.1});
  arm_controller_t controller = joint_sweep_as_controller(&sweep);

  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.dof == arm.config.dof);
  assert(arm.command.tau_ff_nm[0] == 1.5);
  assert(arm.command.tau_ff_nm[1] == 0.0);

  arm_clear_command(&arm);
  assert(arm.command.flags == 0u);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  joint_pd_t pd;
  joint_pd_init(&pd, arm.config.dof);
  arm_reference_zero(&ref, arm.config.dof);
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    joint_pd_set_params(&pd, i, (joint_pd_params_t){10.0, 1.0, 10.0});
    ref.q_ref_rad[i] = 0.0;
    ref.dq_ref_rad_s[i] = 0.0;
    arm.state.q_rad[i] = 0.0;
    arm.state.dq_rad_s[i] = 0.0;
  }
  controller = joint_pd_as_controller(&pd);
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    assert(near_zero(arm.command.tau_ff_nm[i]));
  }

  ref.q_ref_rad[0] = 0.1;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] > 0.0);

  joint_pd_set_params(&pd, 0u, (joint_pd_params_t){10.0, 0.0, 0.75});
  ref.q_ref_rad[0] = 1.0;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.75);

  test_feedforward_t test_ff = {0};
  test_ff.dof = arm.config.dof;
  test_ff.tau_nm[0] = 0.4;
  arm_feedforward_t feedforward = {&TEST_FEEDFORWARD_VTABLE, &test_ff};
  ref.q_ref_rad[0] = 0.0;
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  assert(arm_control_step_with_feedforward(&arm, &ref, &controller, &feedforward) == ARM_OK);
  assert(near_zero(arm.command.tau_ff_nm[0] - 0.4));

  joint_pd_set_params(&pd, 0u, (joint_pd_params_t){10.0, 0.0, 10.0});
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  ref.q_ref_rad[0] = 1.0;
  assert(arm_control_step(&arm, &ref, &controller) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 1.5);

  joint_ref_shaper_params_t shaper_params = {0};
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    shaper_params.q_min_rad[i] = -0.5;
    shaper_params.q_max_rad[i] = 0.5;
    shaper_params.dq_limit_rad_s[i] = 1.0;
    shaper_params.ddq_limit_rad_s2[i] = 2.0;
    arm.state.q_rad[i] = 0.0;
    arm.state.dq_rad_s[i] = 0.0;
  }
  arm.state.dt_s = 0.1;
  arm.state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  joint_ref_shaper_t shaper;
  joint_ref_shaper_init(&shaper, arm.config.dof, &shaper_params);
  joint_ref_shaper_reset_to_state(&shaper, &arm.state);

  arm_reference_t goal_ref;
  arm_reference_t shaped_ref;
  arm_reference_zero(&goal_ref, arm.config.dof);
  goal_ref.flags = ARM_REFERENCE_Q_VALID;
  goal_ref.q_ref_rad[0] = 2.0;
  assert(joint_ref_shaper_step(&shaper, &arm.state, &goal_ref, &shaped_ref) == ARM_OK);
  assert(shaper.q_goal_rad[0] == 0.5);
  assert(shaped_ref.dq_ref_rad_s[0] <= 0.2 + 1.0e-9);
  assert(shaped_ref.dq_ref_rad_s[0] <= 1.0 + 1.0e-9);
  const arm_real_t first_dq = shaped_ref.dq_ref_rad_s[0];
  assert(joint_ref_shaper_step(&shaper, &arm.state, &goal_ref, &shaped_ref) == ARM_OK);
  assert(shaped_ref.dq_ref_rad_s[0] - first_dq <= 0.2 + 1.0e-9);

  arm_safety_t safety;
  arm_safety_init(&safety, arm.config.dof);
  for (uint8_t i = 0u; i < arm.config.dof; ++i) {
    arm_safety_set_joint_params(&safety, i, (arm_safety_joint_params_t){-0.5, 0.5, 0.02, 1.0, 0.7});
  }

  arm_command_zero(&arm.command, arm.config.dof);
  arm.command.tau_ff_nm[0] = 0.5;
  arm.state.flags = 0u;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  arm.state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  arm.state.q_rad[0] = 0.0;
  arm.state.dq_rad_s[0] = 0.0;
  arm.command.tau_ff_nm[0] = 2.0;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.7);

  arm.state.q_rad[0] = 0.49;
  arm.command.tau_ff_nm[0] = 0.5;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  arm.state.q_rad[0] = 0.0;
  arm.state.dq_rad_s[0] = 1.2;
  arm.command.tau_ff_nm[0] = 0.5;
  assert(arm_safety_apply(&safety, &arm.state, &arm.command) == ARM_OK);
  assert(arm.command.tau_ff_nm[0] == 0.0);

  return 0;
}
