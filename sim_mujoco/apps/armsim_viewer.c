#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "arm_core/arm_safety.h"
#include "arm_common/arm_math.h"
#include "arm_core/joint_gravity_ff.h"
#include "arm_core/joint_state_filter.h"
#include "arm_core/joint_pd.h"
#include "arm_motion/joint_kinematics.h"
#include "arm_motion/joint_ref_shaper.h"
#include "armsim/arm6_sim_config.h"
#include "armsim/control_log.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/mujoco_inverse_dynamics_ff.h"
#include "armsim/sim_impairment.h"
#include "armsim/sim_loop.h"

#define PANEL_WIDTH 360
#define SLIDER_X 24
#define SLIDER_W 230
#define SLIDER_H 18
#define ROW_H 58
#define SLIDER_TOP_OFFSET 370
#define TOOL_AXIS_COUNT 3
#define TOOL_AXIS_LENGTH_M 0.16

typedef enum {
  VIEWER_MODE_POSE_EDIT = 0,
  VIEWER_MODE_DYNAMIC = 1,
} viewer_mode_t;

typedef struct {
  mjModel *model;
  mjData *data;
  mujoco_arm_t arm;

  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjrContext context;

  arm_t core;
  arm_reference_t manual_goal_ref;
  arm_reference_t control_ref;
  joint_ref_shaper_t shaper;
  joint_state_filter_t state_filter;
  arm_state_t measured_state;
  arm_safety_t safety;
  arm_output_limiter_t output_limiter;
  joint_pd_t pd;
  arm_controller_t controller;
  joint_gravity_ff_t gravity_ff;
  arm_feedforward_t feedforward;
  mujoco_inverse_dynamics_ff_t inverse_dyn_ff;
  arm_feedforward_t inverse_dyn_feedforward;
  joint_kinematics_params_t kinematics;
  armsim_impairment_t impairment;
  control_log_t debug_log;
  int tool_site_id;

  double slider_angle_rad[ARM_DOF_MAX];
  arm_real_t tool_target_pos[3];
  arm_real_t tool_target_rot[9];
  arm_real_t tool_drag_seed_q_rad[ARM_DOF_MAX];
  viewer_mode_t mode;
  bool gravity_enabled;
  bool gravity_ff_enabled;
  bool inverse_dyn_ff_enabled;
  bool contacts_enabled;
  bool harsh_sim_enabled;
  bool tool_drag_enabled;
  bool tool_drag_active;
  bool tool_target_valid;
  uint8_t tool_drag_axis;
  int active_slider;

  bool button_left;
  bool button_middle;
  bool button_right;
  double last_x;
  double last_y;
} viewer_app_t;

static void sync_sliders_from_target(viewer_app_t *app);

static void capture_tool_drag_seed(viewer_app_t *app) {
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    app->tool_drag_seed_q_rad[i] = ARM_REAL_ZERO;
  }
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->tool_drag_seed_q_rad[i] = app->core.state.q_rad[i];
  }
}

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static double clamp_double(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void cursor_to_framebuffer(GLFWwindow *window, double cursor_x, double cursor_y, double *fb_x, double *fb_y) {
  int win_w = 1;
  int win_h = 1;
  int fb_w = 1;
  int fb_h = 1;
  glfwGetWindowSize(window, &win_w, &win_h);
  glfwGetFramebufferSize(window, &fb_w, &fb_h);

  *fb_x = cursor_x * (double)fb_w / (double)win_w;
  *fb_y = ((double)win_h - cursor_y) * (double)fb_h / (double)win_h;
}

static void apply_physics_options(viewer_app_t *app) {
  if (!app) {
    return;
  }

  if (app->gravity_enabled) {
    app->model->opt.disableflags &= ~mjDSBL_GRAVITY;
  } else {
    app->model->opt.disableflags |= mjDSBL_GRAVITY;
  }

  if (app->contacts_enabled) {
    app->model->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    app->model->opt.disableflags |= mjDSBL_CONTACT;
  }
}

static void joint_limits(const viewer_app_t *app, uint8_t joint, double *low, double *high) {
  const int joint_id = app->arm.joint_ids[joint];
  const arm_joint_config_t *cfg = &app->core.config.joints[joint];

  double lo = -3.141592653589793;
  double hi = +3.141592653589793;
  if (app->model->jnt_limited[joint_id]) {
    lo = app->model->jnt_range[2 * joint_id + 0];
    hi = app->model->jnt_range[2 * joint_id + 1];
  }

  const double core_lo = (double)cfg->sign * (lo - (double)cfg->q_offset_rad);
  const double core_hi = (double)cfg->sign * (hi - (double)cfg->q_offset_rad);
  *low = core_lo < core_hi ? core_lo : core_hi;
  *high = core_lo < core_hi ? core_hi : core_lo;
}

static double clamp_joint_angle(const viewer_app_t *app, uint8_t joint, double angle_rad) {
  double low = 0.0;
  double high = 0.0;
  joint_limits(app, joint, &low, &high);
  return clamp_double(angle_rad, low, high);
}

static void read_core_state(viewer_app_t *app) {
  mujoco_arm_read_state(
      app->data, &app->arm, ARM_REAL(app->data->time), ARM_REAL(app->model->opt.timestep), &app->core.state);
}

static int update_tool_target_from_state(viewer_app_t *app, const arm_state_t *state) {
  const int status = joint_kinematics_fk_pose(&app->kinematics, state, app->tool_target_pos, app->tool_target_rot);
  if (status == ARM_OK) {
    app->tool_target_valid = true;
  }
  return status;
}

static int solve_tool_drag_goal(viewer_app_t *app) {
  if (!app->tool_drag_enabled || !app->tool_drag_active || !app->tool_target_valid) return ARM_OK;

  arm_reference_t ik_ref;
  const joint_ik_pose_options_t options = {
      14u,
      ARM_REAL(0.045),
      ARM_REAL(0.06),
      ARM_REAL(0.002),
      ARM_REAL(0.012),
      ARM_REAL(0.42),
      app->tool_drag_seed_q_rad,
      ARM_REAL(0.12),
  };
  const int status = joint_ik_pose_solve(
      &app->kinematics, &app->core.state, app->tool_target_pos, app->tool_target_rot, &options, &ik_ref);
  if (status != ARM_OK) return status;

  app->manual_goal_ref = ik_ref;
  sync_sliders_from_target(app);
  return ARM_OK;
}

static void camera_drag_axes(const viewer_app_t *app, arm_real_t right[3], arm_real_t up[3]) {
  const mjvGLCamera *camera = &app->scene.camera[0];
  up[0] = ARM_REAL(camera->up[0]);
  up[1] = ARM_REAL(camera->up[1]);
  up[2] = ARM_REAL(camera->up[2]);
  const arm_real_t forward[3] = {
      ARM_REAL(camera->forward[0]),
      ARM_REAL(camera->forward[1]),
      ARM_REAL(camera->forward[2]),
  };
  arm_vec3_cross(forward, up, right);
}

static void move_tool_target_from_mouse(viewer_app_t *app, double dx, double dy, int fb_h) {
  if (!app->tool_target_valid || fb_h <= 0) return;

  arm_real_t right[3];
  arm_real_t up[3];
  camera_drag_axes(app, right, up);
  const arm_real_t scale = ARM_REAL(app->camera.distance * 2.0 / (double)fb_h);
  const arm_real_t drag_plane_delta[3] = {
      right[0] * ARM_REAL(dx) * scale - up[0] * ARM_REAL(dy) * scale,
      right[1] * ARM_REAL(dx) * scale - up[1] * ARM_REAL(dy) * scale,
      right[2] * ARM_REAL(dx) * scale - up[2] * ARM_REAL(dy) * scale,
  };
  const uint8_t drag_axis = app->tool_drag_axis < TOOL_AXIS_COUNT ? app->tool_drag_axis : 0u;
  app->tool_target_pos[drag_axis] += drag_plane_delta[drag_axis];
}

static void compute_gravity_ff_log(const viewer_app_t *app, arm_real_t tau_ff_gravity[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    tau_ff_gravity[i] = ARM_REAL_ZERO;
  }
  if (!app->gravity_enabled || !app->gravity_ff_enabled) return;

  arm_t ff_arm = app->core;
  arm_feedforward_t ff = app->feedforward;
  if (arm_control_step_with_feedforward(&ff_arm, &app->control_ref, NULL, &ff) != ARM_OK) return;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    tau_ff_gravity[i] = ff_arm.command.tau_ff_nm[i];
  }
}

static arm_feedforward_t *active_feedforward(viewer_app_t *app) {
  if (!app->gravity_enabled) return NULL;
  if (app->inverse_dyn_ff_enabled) return &app->inverse_dyn_feedforward;
  if (app->gravity_ff_enabled) return &app->feedforward;
  return NULL;
}

static void compute_model_ff_log(viewer_app_t *app, arm_real_t tau_ff_model[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    tau_ff_model[i] = ARM_REAL_ZERO;
  }
  arm_feedforward_t *ff = active_feedforward(app);
  if (!ff) return;

  arm_t ff_arm = app->core;
  arm_feedforward_t ff_copy = *ff;
  if (arm_control_step_with_feedforward(&ff_arm, &app->control_ref, NULL, &ff_copy) != ARM_OK) return;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    tau_ff_model[i] = ff_arm.command.tau_ff_nm[i];
  }
}

static void debug_log_open(viewer_app_t *app) {
  (void)control_log_open(&app->debug_log, "logs/dynamic_debug.csv", app->core.config.dof);
}

static void debug_log_step(viewer_app_t *app) {
  arm_real_t tau_ff_gravity[ARM_DOF_MAX];
  arm_real_t tau_ff_model[ARM_DOF_MAX];
  compute_gravity_ff_log(app, tau_ff_gravity);
  compute_model_ff_log(app, tau_ff_model);

  const control_log_flags_t flags = {
      app->gravity_enabled,
      app->gravity_ff_enabled,
      app->inverse_dyn_ff_enabled,
      app->contacts_enabled,
      app->harsh_sim_enabled,
  };
  (void)control_log_write_step(
      &app->debug_log,
      app->model,
      app->data,
      &app->arm,
      &flags,
      &app->measured_state,
      &app->core.state,
      &app->control_ref,
      tau_ff_gravity,
      tau_ff_model,
      &app->core.command);
}

static void debug_log_close(viewer_app_t *app) {
  control_log_close(&app->debug_log);
}

static int read_filtered_core_state(viewer_app_t *app) {
  mujoco_arm_read_state(
      app->data, &app->arm, ARM_REAL(app->data->time), ARM_REAL(app->model->opt.timestep), &app->measured_state);
  return joint_state_filter_step(&app->state_filter, &app->measured_state, &app->core.state);
}

static void sync_sliders_from_target(viewer_app_t *app) {
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->slider_angle_rad[i] = (double)app->manual_goal_ref.q_ref_rad[i];
  }
}

static void sync_target_to_current_pose(viewer_app_t *app) {
  read_core_state(app);
  arm_reference_zero(&app->manual_goal_ref, app->core.config.dof);
  app->manual_goal_ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->manual_goal_ref.q_ref_rad[i] = app->core.state.q_rad[i];
    app->manual_goal_ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }
  joint_ref_shaper_reset_to_state(&app->shaper, &app->core.state);
  joint_state_filter_reset_to_state(&app->state_filter, &app->core.state);
  arm_output_limiter_reset(&app->output_limiter);
  (void)joint_ref_shaper_step(&app->shaper, &app->core.state, &app->manual_goal_ref, &app->control_ref);
  joint_pd_reset(&app->pd);
  (void)update_tool_target_from_state(app, &app->core.state);
  sync_sliders_from_target(app);
}

static void set_pose_joint_angle(viewer_app_t *app, uint8_t joint, double angle_rad) {
  const arm_joint_config_t *cfg = &app->core.config.joints[joint];
  const double clamped = clamp_joint_angle(app, joint, angle_rad);
  app->slider_angle_rad[joint] = clamped;
  app->manual_goal_ref.q_ref_rad[joint] = ARM_REAL(clamped);
  app->manual_goal_ref.dq_ref_rad_s[joint] = ARM_REAL_ZERO;
  app->manual_goal_ref.flags |= ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  app->data->qpos[app->arm.joint_qpos_addr[joint]] =
      (mjtNum)((double)cfg->q_offset_rad + (double)cfg->sign * clamped);
}

static void zero_motion(viewer_app_t *app) {
  for (int i = 0; i < app->model->nv; ++i) {
    app->data->qvel[i] = 0.0;
  }
  for (int i = 0; i < app->model->nu; ++i) {
    app->data->ctrl[i] = 0.0;
  }
}

static void apply_pose_edit(viewer_app_t *app) {
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    set_pose_joint_angle(app, i, app->slider_angle_rad[i]);
  }
  zero_motion(app);
  mj_forward(app->model, app->data);
  read_core_state(app);
  (void)update_tool_target_from_state(app, &app->core.state);
}

static void reset_viewer_state(viewer_app_t *app) {
  mj_resetData(app->model, app->data);
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->slider_angle_rad[i] = 0.0;
    app->manual_goal_ref.q_ref_rad[i] = ARM_REAL_ZERO;
    app->manual_goal_ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
    set_pose_joint_angle(app, i, 0.0);
  }
  app->manual_goal_ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  joint_pd_reset(&app->pd);
  armsim_impairment_reset(&app->impairment);
  arm_output_limiter_reset(&app->output_limiter);
  zero_motion(app);
  mj_forward(app->model, app->data);
  read_core_state(app);
  (void)update_tool_target_from_state(app, &app->core.state);
  joint_ref_shaper_reset_to_state(&app->shaper, &app->core.state);
  joint_state_filter_reset_to_state(&app->state_filter, &app->core.state);
  (void)joint_ref_shaper_step(&app->shaper, &app->core.state, &app->manual_goal_ref, &app->control_ref);
}

static void set_viewer_mode(viewer_app_t *app, viewer_mode_t mode) {
  if (app->mode == mode) {
    return;
  }

  app->mode = mode;
  app->active_slider = -1;
  if (mode == VIEWER_MODE_DYNAMIC) {
    sync_target_to_current_pose(app);
    armsim_impairment_reset(&app->impairment);
    arm_output_limiter_reset(&app->output_limiter);
  } else {
    sync_target_to_current_pose(app);
    apply_pose_edit(app);
  }
}

static int slider_hit(const viewer_app_t *app, double fb_x, double fb_y, int fb_h) {
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    const int y = fb_h - SLIDER_TOP_OFFSET - (int)i * ROW_H;
    if (fb_x >= SLIDER_X && fb_x <= SLIDER_X + SLIDER_W && fb_y >= y && fb_y <= y + SLIDER_H) {
      return (int)i;
    }
  }
  return -1;
}

static void update_slider_from_mouse(viewer_app_t *app, int slider, double fb_x) {
  if (slider < 0 || slider >= app->core.config.dof) {
    return;
  }

  double low = 0.0;
  double high = 0.0;
  joint_limits(app, (uint8_t)slider, &low, &high);
  const double t = clamp_double((fb_x - SLIDER_X) / (double)SLIDER_W, 0.0, 1.0);
  const double angle = low + t * (high - low);
  app->slider_angle_rad[slider] = angle;
  app->manual_goal_ref.q_ref_rad[slider] = ARM_REAL(angle);
  app->manual_goal_ref.dq_ref_rad_s[slider] = ARM_REAL_ZERO;
  app->manual_goal_ref.flags |= ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;

  if (app->mode == VIEWER_MODE_POSE_EDIT) {
    set_pose_joint_angle(app, (uint8_t)slider, angle);
    apply_pose_edit(app);
  }
}

static bool button_hit(double fb_x, double fb_y, int x, int y, int w, int h) {
  return fb_x >= x && fb_x <= x + w && fb_y >= y && fb_y <= y + h;
}

static void set_tool_drag_axis(viewer_app_t *app, uint8_t axis) {
  if (axis >= TOOL_AXIS_COUNT) return;
  app->tool_drag_axis = axis;
}

static bool handle_panel_click(viewer_app_t *app, double fb_x, double fb_y, int fb_h) {
  const int y_mode = fb_h - 104;
  const int y_flags = fb_h - 140;
  const int y_actions = fb_h - 176;
  const int y_harsh = fb_h - 212;
  const int y_tool = fb_h - 248;
  const int y_tool_axis = fb_h - 284;

  if (button_hit(fb_x, fb_y, 24, y_mode, 140, 28)) {
    set_viewer_mode(app, VIEWER_MODE_POSE_EDIT);
    return true;
  }
  if (button_hit(fb_x, fb_y, 176, y_mode, 140, 28)) {
    set_viewer_mode(app, VIEWER_MODE_DYNAMIC);
    return true;
  }
  if (button_hit(fb_x, fb_y, 24, y_flags, 140, 28)) {
    app->gravity_enabled = !app->gravity_enabled;
    apply_physics_options(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 176, y_flags, 140, 28)) {
    app->contacts_enabled = !app->contacts_enabled;
    apply_physics_options(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 24, y_actions, 140, 28)) {
    reset_viewer_state(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 176, y_actions, 140, 28)) {
    sync_target_to_current_pose(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 24, y_harsh, 140, 28)) {
    app->harsh_sim_enabled = !app->harsh_sim_enabled;
    armsim_impairment_reset(&app->impairment);
    sync_target_to_current_pose(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 176, y_harsh, 140, 28)) {
    app->gravity_ff_enabled = !app->gravity_ff_enabled;
    if (app->gravity_ff_enabled) app->inverse_dyn_ff_enabled = false;
    sync_target_to_current_pose(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 24, y_tool, 140, 28)) {
    app->tool_drag_enabled = !app->tool_drag_enabled;
    app->tool_drag_active = false;
    sync_target_to_current_pose(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 176, y_tool, 140, 28)) {
    app->inverse_dyn_ff_enabled = !app->inverse_dyn_ff_enabled;
    if (app->inverse_dyn_ff_enabled) app->gravity_ff_enabled = false;
    sync_target_to_current_pose(app);
    return true;
  }
  if (button_hit(fb_x, fb_y, 24, y_tool_axis, 72, 28)) {
    set_tool_drag_axis(app, 0u);
    return true;
  }
  if (button_hit(fb_x, fb_y, 112, y_tool_axis, 72, 28)) {
    set_tool_drag_axis(app, 1u);
    return true;
  }
  if (button_hit(fb_x, fb_y, 200, y_tool_axis, 72, 28)) {
    set_tool_drag_axis(app, 2u);
    return true;
  }

  app->active_slider = slider_hit(app, fb_x, fb_y, fb_h);
  if (app->active_slider >= 0) {
    update_slider_from_mouse(app, app->active_slider, fb_x);
    return true;
  }
  return false;
}

static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
  (void)mods;
  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }

  double cursor_x = 0.0;
  double cursor_y = 0.0;
  double fb_x = 0.0;
  double fb_y = 0.0;
  int fb_w = 1;
  int fb_h = 1;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  cursor_to_framebuffer(window, cursor_x, cursor_y, &fb_x, &fb_y);

  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && fb_x < PANEL_WIDTH) {
    if (handle_panel_click(app, fb_x, fb_y, fb_h)) {
      return;
    }
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && fb_x >= PANEL_WIDTH &&
      app->tool_drag_enabled && app->mode == VIEWER_MODE_DYNAMIC) {
    if (!app->tool_target_valid) {
      (void)update_tool_target_from_state(app, &app->core.state);
    }
    app->tool_drag_active = app->tool_target_valid;
    capture_tool_drag_seed(app);
    app->active_slider = -1;
    app->last_x = cursor_x;
    app->last_y = cursor_y;
    return;
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE && app->tool_drag_active) {
    app->tool_drag_active = false;
    sync_target_to_current_pose(app);
    app->last_x = cursor_x;
    app->last_y = cursor_y;
    return;
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    app->button_left = action == GLFW_PRESS;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    app->button_middle = action == GLFW_PRESS;
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    app->button_right = action == GLFW_PRESS;
  }

  if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT) {
    app->active_slider = -1;
  }

  app->last_x = cursor_x;
  app->last_y = cursor_y;
}

static void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos) {
  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }

  double fb_x = 0.0;
  double fb_y = 0.0;
  cursor_to_framebuffer(window, xpos, ypos, &fb_x, &fb_y);
  if (app->tool_drag_active) {
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window, &width, &height);
    move_tool_target_from_mouse(app, xpos - app->last_x, ypos - app->last_y, height);
    app->last_x = xpos;
    app->last_y = ypos;
    return;
  }
  if (app->active_slider >= 0) {
    update_slider_from_mouse(app, app->active_slider, fb_x);
    return;
  }

  if (!app->button_left && !app->button_middle && !app->button_right) {
    app->last_x = xpos;
    app->last_y = ypos;
    return;
  }

  int width = 1;
  int height = 1;
  glfwGetFramebufferSize(window, &width, &height);
  const double dx = xpos - app->last_x;
  const double dy = ypos - app->last_y;
  app->last_x = xpos;
  app->last_y = ypos;

  int action = mjMOUSE_NONE;
  if (app->button_right) {
    action = mjMOUSE_MOVE_H;
  } else if (app->button_middle) {
    action = mjMOUSE_ZOOM;
  } else if (app->button_left) {
    action = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS
                 ? mjMOUSE_MOVE_V
                 : mjMOUSE_ROTATE_H;
  }

  if (action != mjMOUSE_NONE) {
    mjv_moveCamera(app->model, action, dx / (double)height, dy / (double)height, &app->scene, &app->camera);
  }
}

static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  (void)xoffset;
  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }
  mjv_moveCamera(app->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &app->scene, &app->camera);
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  (void)scancode;
  (void)mods;
  if (action != GLFW_PRESS) {
    return;
  }

  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }

  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  } else if (key == GLFW_KEY_R) {
    reset_viewer_state(app);
  } else if (key == GLFW_KEY_SPACE) {
    set_viewer_mode(app, app->mode == VIEWER_MODE_DYNAMIC ? VIEWER_MODE_POSE_EDIT : VIEWER_MODE_DYNAMIC);
  } else if (key == GLFW_KEY_G) {
    app->gravity_enabled = !app->gravity_enabled;
    apply_physics_options(app);
  } else if (key == GLFW_KEY_C) {
    app->contacts_enabled = !app->contacts_enabled;
    apply_physics_options(app);
  } else if (key == GLFW_KEY_S) {
    sync_target_to_current_pose(app);
  } else if (key == GLFW_KEY_H) {
    app->harsh_sim_enabled = !app->harsh_sim_enabled;
    armsim_impairment_reset(&app->impairment);
    sync_target_to_current_pose(app);
  } else if (key == GLFW_KEY_F) {
    app->gravity_ff_enabled = !app->gravity_ff_enabled;
    if (app->gravity_ff_enabled) app->inverse_dyn_ff_enabled = false;
    sync_target_to_current_pose(app);
  } else if (key == GLFW_KEY_I) {
    app->inverse_dyn_ff_enabled = !app->inverse_dyn_ff_enabled;
    if (app->inverse_dyn_ff_enabled) app->gravity_ff_enabled = false;
    sync_target_to_current_pose(app);
  } else if (key == GLFW_KEY_T) {
    app->tool_drag_enabled = !app->tool_drag_enabled;
    app->tool_drag_active = false;
    sync_target_to_current_pose(app);
  } else if (key == GLFW_KEY_X) {
    set_tool_drag_axis(app, 0u);
  } else if (key == GLFW_KEY_Y) {
    set_tool_drag_axis(app, 1u);
  } else if (key == GLFW_KEY_Z) {
    set_tool_drag_axis(app, 2u);
  }
}

static void draw_button(const mjrContext *context, int x, int y, int w, int h, const char *text, bool active) {
  const mjrRect rect = {x, y, w, h};
  if (active) {
    mjr_label(rect, mjFONT_NORMAL, text, 0.18f, 0.42f, 0.58f, 1.0f, 0.95f, 0.95f, 0.95f, context);
  } else {
    mjr_label(rect, mjFONT_NORMAL, text, 0.28f, 0.31f, 0.35f, 1.0f, 0.90f, 0.90f, 0.90f, context);
  }
}

static void draw_axis_button(
    const mjrContext *context,
    int x,
    int y,
    const char *text,
    const float rgb[3],
    bool active) {
  const mjrRect rect = {x, y, 72, 28};
  const float alpha = active ? 1.0f : 0.42f;
  mjr_label(rect, mjFONT_NORMAL, text, rgb[0], rgb[1], rgb[2], alpha, 0.96f, 0.96f, 0.96f, context);
}

static void init_connector_geom(mjvGeom *geom, const float rgba[4]) {
  mjv_initGeom(geom, mjGEOM_LINE, NULL, NULL, NULL, rgba);
}

static void draw_slider(viewer_app_t *app, const mjrContext *context, int fb_h, uint8_t joint) {
  double low = 0.0;
  double high = 0.0;
  joint_limits(app, joint, &low, &high);

  const int y = fb_h - SLIDER_TOP_OFFSET - (int)joint * ROW_H;
  const double value = app->slider_angle_rad[joint];
  const double denom = high - low;
  const double t = denom > 0.0 ? clamp_double((value - low) / denom, 0.0, 1.0) : 0.0;
  const int knob_x = SLIDER_X + (int)(t * (double)SLIDER_W) - 5;

  char label[80];
  char value_label[48];
  (void)snprintf(label, sizeof(label), "J%u  %.1f deg", (unsigned)(joint + 1u), value * 57.29577951308232);
  (void)snprintf(value_label, sizeof(value_label), "%.0f .. %.0f", low * 57.29577951308232, high * 57.29577951308232);

  const mjrRect label_rect = {SLIDER_X, y + 23, 140, 20};
  const mjrRect range_rect = {SLIDER_X + 154, y + 23, 110, 20};
  const mjrRect bar_rect = {SLIDER_X, y, SLIDER_W, SLIDER_H};
  const mjrRect fill_rect = {SLIDER_X, y, (int)(t * (double)SLIDER_W), SLIDER_H};
  const mjrRect knob_rect = {knob_x, y - 3, 10, SLIDER_H + 6};

  mjr_label(label_rect, mjFONT_NORMAL, label, 0.12f, 0.13f, 0.15f, 0.0f, 0.92f, 0.92f, 0.92f, context);
  mjr_label(range_rect, mjFONT_NORMAL, value_label, 0.12f, 0.13f, 0.15f, 0.0f, 0.65f, 0.68f, 0.72f, context);
  mjr_rectangle(bar_rect, 0.18f, 0.20f, 0.23f, 1.0f);
  mjr_rectangle(fill_rect, 0.25f, 0.52f, 0.68f, 1.0f);
  mjr_rectangle(knob_rect, 0.88f, 0.90f, 0.92f, 1.0f);
}

static void draw_panel(viewer_app_t *app, mjrRect viewport) {
  const mjrRect panel = {0, 0, PANEL_WIDTH, viewport.height};
  mjr_rectangle(panel, 0.08f, 0.09f, 0.10f, 0.94f);

  const mjrRect title = {20, viewport.height - 52, 280, 28};
  mjr_label(title, mjFONT_BIG, "ArmSim Viewer", 0.08f, 0.09f, 0.10f, 0.0f, 0.95f, 0.95f, 0.95f, &app->context);

  draw_button(&app->context, 24, viewport.height - 104, 140, 28, "Pose Edit", app->mode == VIEWER_MODE_POSE_EDIT);
  draw_button(&app->context, 176, viewport.height - 104, 140, 28, "Dynamic", app->mode == VIEWER_MODE_DYNAMIC);
  draw_button(&app->context, 24, viewport.height - 140, 140, 28, "Gravity", app->gravity_enabled);
  draw_button(&app->context, 176, viewport.height - 140, 140, 28, "Contacts", app->contacts_enabled);
  draw_button(&app->context, 24, viewport.height - 176, 140, 28, "Reset", false);
  draw_button(&app->context, 176, viewport.height - 176, 140, 28, "Sync target", false);
  draw_button(&app->context, 24, viewport.height - 212, 140, 28, "Harsh Sim", app->harsh_sim_enabled);
  draw_button(&app->context, 176, viewport.height - 212, 140, 28, "Gravity FF",
              app->gravity_ff_enabled);
  draw_button(&app->context, 24, viewport.height - 248, 140, 28, "Tool Drag", app->tool_drag_enabled);
  draw_button(&app->context, 176, viewport.height - 248, 140, 28, "InverseDyn", app->inverse_dyn_ff_enabled);
  const float x_rgb[3] = {0.82f, 0.18f, 0.16f};
  const float y_rgb[3] = {0.18f, 0.62f, 0.20f};
  const float z_rgb[3] = {0.18f, 0.35f, 0.82f};
  draw_axis_button(&app->context, 24, viewport.height - 284, "X", x_rgb, app->tool_drag_axis == 0u);
  draw_axis_button(&app->context, 112, viewport.height - 284, "Y", y_rgb, app->tool_drag_axis == 1u);
  draw_axis_button(&app->context, 200, viewport.height - 284, "Z", z_rgb, app->tool_drag_axis == 2u);

  const mjrRect hint_a = {24, viewport.height - 314, 128, 18};
  const mjrRect hint_b = {168, viewport.height - 314, 128, 18};
  mjr_label(hint_a, mjFONT_NORMAL, "T: drag", 0.08f, 0.09f, 0.10f, 0.0f, 0.66f, 0.69f, 0.72f,
            &app->context);
  mjr_label(hint_b, mjFONT_NORMAL, "release: hold", 0.08f, 0.09f, 0.10f, 0.0f, 0.66f, 0.69f, 0.72f,
            &app->context);

  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    draw_slider(app, &app->context, viewport.height, i);
  }
}

static void draw_tool_drag_overlay(viewer_app_t *app) {
  if (!app->tool_drag_enabled || !app->tool_target_valid) return;
  if (app->scene.ngeom + 5 >= app->scene.maxgeom) return;

  const mjtNum target_pos[3] = {
      (mjtNum)app->tool_target_pos[0],
      (mjtNum)app->tool_target_pos[1],
      (mjtNum)app->tool_target_pos[2],
  };
  const mjtNum sphere_size[3] = {0.018, 0.018, 0.018};
  const float target_rgba[4] = {0.15f, 0.75f, 0.95f, 1.0f};
  mjvGeom *sphere = &app->scene.geoms[app->scene.ngeom++];
  mjv_initGeom(sphere, mjGEOM_SPHERE, sphere_size, target_pos, NULL, target_rgba);

  static const float axis_rgba[TOOL_AXIS_COUNT][4] = {
      {0.95f, 0.20f, 0.18f, 1.0f},
      {0.25f, 0.85f, 0.28f, 1.0f},
      {0.20f, 0.45f, 0.95f, 1.0f},
  };
  for (uint8_t axis = 0u; axis < TOOL_AXIS_COUNT; ++axis) {
    mjtNum axis_end[3] = {target_pos[0], target_pos[1], target_pos[2]};
    axis_end[axis] += TOOL_AXIS_LENGTH_M;
    mjvGeom *axis_line = &app->scene.geoms[app->scene.ngeom++];
    init_connector_geom(axis_line, axis_rgba[axis]);
    const double width = axis == app->tool_drag_axis ? 5.0 : 2.5;
    mjv_connector(axis_line, mjGEOM_LINE, width, target_pos, axis_end);
    axis_line->rgba[0] = axis_rgba[axis][0];
    axis_line->rgba[1] = axis_rgba[axis][1];
    axis_line->rgba[2] = axis_rgba[axis][2];
    axis_line->rgba[3] = axis == app->tool_drag_axis ? 1.0f : 0.55f;
  }

  if (app->tool_site_id < 0) return;
  const mjtNum *site_pos = &app->data->site_xpos[3 * app->tool_site_id];
  const float error_rgba[4] = {0.15f, 0.75f, 0.95f, 0.85f};
  mjvGeom *line = &app->scene.geoms[app->scene.ngeom++];
  init_connector_geom(line, error_rgba);
  mjv_connector(line, mjGEOM_LINE, 3.0, site_pos, target_pos);
  line->rgba[0] = 0.15f;
  line->rgba[1] = 0.75f;
  line->rgba[2] = 0.95f;
  line->rgba[3] = 0.85f;
}

static void configure_pd_defaults(viewer_app_t *app) {
  static const joint_pd_params_t pd_params[ARM_DEFAULT_DOF] = ARMSIM_ARM6_PD_PARAMS;

  joint_pd_init(&app->pd, app->core.config.dof);
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    joint_pd_set_params(&app->pd, i, pd_params[i]);
  }
  app->controller = joint_pd_as_controller(&app->pd);
}

static void configure_ref_shaper_and_safety(viewer_app_t *app) {
  static const arm_real_t dq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DQ_LIMITS_RAD_S;
  static const arm_real_t ddq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DDQ_LIMITS_RAD_S2;
  static const arm_real_t dddq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DDDQ_LIMITS_RAD_S3;

  joint_ref_shaper_params_t shaper_params = {0};
  arm_safety_init(&app->safety, app->core.config.dof);

  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    double low = 0.0;
    double high = 0.0;
    joint_limits(app, i, &low, &high);
    shaper_params.q_min_rad[i] = ARM_REAL(low);
    shaper_params.q_max_rad[i] = ARM_REAL(high);
    shaper_params.dq_limit_rad_s[i] = dq_limits[i];
    shaper_params.ddq_limit_rad_s2[i] = ddq_limits[i];
    shaper_params.dddq_limit_rad_s3[i] = dddq_limits[i];

    arm_safety_set_joint_params(
        &app->safety,
        i,
        (arm_safety_joint_params_t){
            ARM_REAL(low),
            ARM_REAL(high),
            ARMSIM_ARM6_SAFETY_Q_MARGIN_RAD,
            dq_limits[i] * ARMSIM_ARM6_SAFETY_DQ_LIMIT_SCALE,
            app->core.config.joints[i].torque_limit_nm,
        });
  }

  joint_ref_shaper_init(&app->shaper, app->core.config.dof, &shaper_params);
}

static void configure_state_filter(viewer_app_t *app) {
  static const joint_state_filter_params_t filter_params[ARM_DEFAULT_DOF] = ARMSIM_ARM6_STATE_FILTER_PARAMS;

  joint_state_filter_init(&app->state_filter, app->core.config.dof, filter_params);
  arm_state_zero(&app->measured_state, app->core.config.dof);
}

static void configure_output_limiter(viewer_app_t *app) {
  static const arm_output_limiter_joint_params_t output_limiter_params[ARM_DEFAULT_DOF] =
      ARMSIM_ARM6_OUTPUT_LIMITER_PARAMS;

  arm_output_limiter_init(&app->output_limiter, app->core.config.dof);
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    arm_output_limiter_set_joint_params(&app->output_limiter, i, output_limiter_params[i]);
  }
}

static void configure_gravity_ff(viewer_app_t *app) {
  static const joint_gravity_ff_params_t gravity_params = ARMSIM_ARM6_GRAVITY_FF_PARAMS;

  joint_gravity_ff_init(&app->gravity_ff, &gravity_params);
  app->feedforward = joint_gravity_ff_as_feedforward(&app->gravity_ff);
}

static int configure_inverse_dynamics_ff(viewer_app_t *app) {
  const int status = mujoco_inverse_dynamics_ff_init(&app->inverse_dyn_ff, app->model, &app->arm);
  if (status != ARM_OK) return status;
  app->inverse_dyn_feedforward = mujoco_inverse_dynamics_ff_as_feedforward(&app->inverse_dyn_ff);
  return ARM_OK;
}

static void configure_kinematics(viewer_app_t *app) {
  static const joint_kinematics_params_t kinematics_params = ARMSIM_ARM6_KINEMATICS_PARAMS;

  app->kinematics = kinematics_params;
  app->tool_site_id = mj_name2id(app->model, mjOBJ_SITE, "tool0");
  app->tool_target_valid = false;
  arm_vec3_zero(app->tool_target_pos);
  arm_mat3_identity(app->tool_target_rot);
}

int main(int argc, char **argv) {
  const char *model_path = argc > 1 ? argv[1] : default_model_path();

  char load_error[1024] = {0};
  mjModel *model = mj_loadXML(model_path, NULL, load_error, sizeof(load_error));
  if (!model) {
    fprintf(stderr, "Failed to load model '%s': %s\n", model_path, load_error);
    return EXIT_FAILURE;
  }

  mjData *data = mj_makeData(model);
  if (!data) {
    fprintf(stderr, "Failed to allocate MuJoCo data.\n");
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  viewer_app_t app = {0};
  app.model = model;
  app.data = data;
  app.mode = VIEWER_MODE_POSE_EDIT;
  app.gravity_enabled = true;
  app.gravity_ff_enabled = true;
  app.inverse_dyn_ff_enabled = false;
  app.contacts_enabled = true;
  app.harsh_sim_enabled = false;
  arm_config_t config = armsim_default_arm6_config();
  if (arm_init(&app.core, &config) != ARM_OK) {
    fprintf(stderr, "Failed to initialize arm core.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }
  arm_reference_zero(&app.manual_goal_ref, app.core.config.dof);
  arm_reference_zero(&app.control_ref, app.core.config.dof);
  app.active_slider = -1;

  char bind_error[256] = {0};
  if (mujoco_arm_bind(model, &app.core.config, &app.arm, bind_error, sizeof(bind_error)) != ARM_OK) {
    fprintf(stderr, "Failed to bind arm: %s\n", bind_error);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }
  configure_pd_defaults(&app);
  configure_ref_shaper_and_safety(&app);
  configure_state_filter(&app);
  configure_output_limiter(&app);
  configure_gravity_ff(&app);
  if (configure_inverse_dynamics_ff(&app) != ARM_OK) {
    fprintf(stderr, "Failed to initialize inverse dynamics feedforward.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }
  configure_kinematics(&app);
  armsim_impairment_config_t impairment_config = armsim_impairment_default_config();
  armsim_impairment_init(&app.impairment, &impairment_config, app.core.config.dof);
  apply_physics_options(&app);
  reset_viewer_state(&app);
  debug_log_open(&app);

  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  GLFWwindow *window = glfwCreateWindow(1280, 720, "ArmSim MuJoCo Viewer", NULL, NULL);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window.\n");
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window, &app);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_pos_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetKeyCallback(window, key_callback);

  mjv_defaultCamera(&app.camera);
  mjv_defaultOption(&app.option);
  mjv_defaultScene(&app.scene);
  mjr_defaultContext(&app.context);
  mjv_makeScene(model, &app.scene, 2000);
  mjr_makeContext(model, &app.context, mjFONTSCALE_150);

  app.camera.distance = 1.35;
  app.camera.azimuth = 135.0;
  app.camera.elevation = -22.0;
  app.camera.lookat[0] = 0.30;
  app.camera.lookat[1] = 0.0;
  app.camera.lookat[2] = 0.20;

  while (!glfwWindowShouldClose(window)) {
    apply_physics_options(&app);
    if (app.mode == VIEWER_MODE_DYNAMIC) {
      const mjtNum frame_start = data->time;
      while (data->time - frame_start < 1.0 / 60.0) {
        if (!app.harsh_sim_enabled) {
          const int filter_status = read_filtered_core_state(&app);
          if (filter_status != ARM_OK) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
          }
        }
        const int ik_status = solve_tool_drag_goal(&app);
        if (ik_status != ARM_OK) {
          glfwSetWindowShouldClose(window, GLFW_TRUE);
          break;
        }
        const int shaper_status =
            joint_ref_shaper_step(&app.shaper, &app.core.state, &app.manual_goal_ref, &app.control_ref);
        if (shaper_status != ARM_OK) {
          glfwSetWindowShouldClose(window, GLFW_TRUE);
          break;
        }
        arm_feedforward_t *feedforward = active_feedforward(&app);
        app.impairment.config.enabled = app.harsh_sim_enabled;
        const int step_status = app.harsh_sim_enabled
                                    ? armsim_step_once_impaired_filtered_with_feedforward(
                                          model,
                                          data,
                                          &app.arm,
                                          &app.core,
                                          &app.control_ref,
                                          &app.safety,
                                          &app.controller,
                                          feedforward,
                                          &app.output_limiter,
                                          &app.impairment,
                                          &app.state_filter,
                                          &app.measured_state)
                                    : armsim_step_once_with_state_feedforward_and_output_limiter(
                                          model,
                                          data,
                                          &app.arm,
                                          &app.core,
                                          &app.core.state,
                                          &app.control_ref,
                                          &app.safety,
                                          &app.controller,
                                          feedforward,
                                          &app.output_limiter);
        if (step_status != ARM_OK) {
          glfwSetWindowShouldClose(window, GLFW_TRUE);
          break;
        }
        debug_log_step(&app);
      }
      read_core_state(&app);
    } else {
      apply_pose_edit(&app);
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    const mjrRect viewport = {0, 0, width, height};
    const mjrRect scene_viewport = {PANEL_WIDTH, 0, width - PANEL_WIDTH, height};

    mjv_updateScene(model, data, &app.option, NULL, &app.camera, mjCAT_ALL, &app.scene);
    draw_tool_drag_overlay(&app);
    mjr_render(scene_viewport, &app.scene, &app.context);
    draw_panel(&app, viewport);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjr_freeContext(&app.context);
  mjv_freeScene(&app.scene);
  debug_log_close(&app);
  mujoco_inverse_dynamics_ff_free(&app.inverse_dyn_ff);
  glfwDestroyWindow(window);
  glfwTerminate();
  mj_deleteData(data);
  mj_deleteModel(model);
  return EXIT_SUCCESS;
}
