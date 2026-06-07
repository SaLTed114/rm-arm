#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#if defined(ARMSIM_BENCHMARK_WITH_GUI)
#include <GLFW/glfw3.h>
#endif

#include <mujoco/mujoco.h>

#include "arm_core/arm.h"
#include "arm_core/arm_control.h"
#include "arm_common/arm_math.h"
#include "arm_core/arm_safety.h"
#include "arm_core/joint_gravity_ff.h"
#include "arm_core/joint_pd.h"
#include "arm_core/joint_state_filter.h"
#include "arm_motion/joint_kinematics.h"
#include "arm_motion/joint_ref_shaper.h"
#include "armsim/arm6_sim_config.h"
#include "armsim/control_log.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/mujoco_inverse_dynamics_ff.h"
#include "armsim/sim_impairment.h"
#include "armsim/sim_loop.h"

#define BENCHMARK_TRAIL_MAX 360
#define BENCHMARK_TRAIL_DT_S 0.02
#define BENCHMARK_JOINT_TRAJ_MAX 96

typedef enum {
  SCENARIO_HOLD_ZERO_HARSH = 0,
  SCENARIO_STEP_J2_HARSH = 1,
  SCENARIO_STEP_J3_HARSH = 2,
  SCENARIO_COUPLED_J2J3_HARSH = 3,
  SCENARIO_STEP_J5_HARSH = 4,
  SCENARIO_SINE_J2_HARSH = 5,
  SCENARIO_STRAIGHT_ARM_LIFT_HARSH = 6,
  SCENARIO_NEAR_LIMIT_J2_HARSH = 7,
  SCENARIO_FLOOR_BLOCKED_HARSH = 8,
  SCENARIO_TOOL_CIRCLE_XY_HARSH = 9,
  SCENARIO_TOOL_CIRCLE_XZ_HARSH = 10,
  SCENARIO_TOOL_CIRCLE_YZ_HARSH = 11,
  SCENARIO_TOOL_SQUARE_XY_HARSH = 12,
  SCENARIO_TOOL_SQUARE_XZ_HARSH = 13,
  SCENARIO_TOOL_SQUARE_YZ_HARSH = 14,
  SCENARIO_TOOL_INSERT_LINE_HARSH = 15,
  SCENARIO_JOINT_CIRCLE_J2J3_HARSH = 16,
  SCENARIO_JOINT_SQUARE_J2J3_HARSH = 17,
  SCENARIO_JOINT_INSERT_LINE_HARSH = 18,
} benchmark_scenario_t;

typedef enum {
  BENCHMARK_FF_NONE = 0,
  BENCHMARK_FF_GRAVITY = 1,
  BENCHMARK_FF_INVERSE_DYNAMICS = 2,
} benchmark_ff_mode_t;

typedef struct {
  mjModel *model;
  mjData *data;
  mujoco_arm_t mj_arm;
  arm_t core;
  arm_reference_t manual_goal_ref;
  arm_reference_t active_ref;
  joint_ref_shaper_t shaper;
  joint_state_filter_t state_filter;
  arm_state_t measured_state;
  arm_safety_t safety;
  arm_output_limiter_t output_limiter;
  joint_pd_t pd;
  arm_controller_t ctrl;
  joint_gravity_ff_t gravity_ff;
  arm_feedforward_t ff;
  mujoco_inverse_dynamics_ff_t inverse_dyn_ff;
  arm_feedforward_t inverse_dyn_feedforward;
  joint_kinematics_params_t kinematics;
  arm_real_t initial_q_rad[ARM_DOF_MAX];
  arm_real_t initial_tool_pos[3];
  arm_real_t tool_target_pos[3];
  arm_real_t tool_ik_seed_q_rad[ARM_DOF_MAX];
  arm_real_t joint_traj_waypoints[BENCHMARK_JOINT_TRAJ_MAX][ARM_DOF_MAX];
  uint8_t joint_traj_count;
  arm_real_t joint_traj_segment_s;
  bool joint_traj_loop;
  bool joint_traj_valid;
  bool tool_target_valid;
  bool tool_ik_seed_valid;
  bool use_direct_ref;
  benchmark_ff_mode_t ff_mode;
  armsim_impairment_t impairment;
  control_log_t log;
  bool harsh_enabled;
  bool contacts_enabled;
  const char *param_override_path;
} benchmark_app_t;

typedef struct {
  bool enabled;
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  GLFWwindow *window;
  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjrContext context;
  mjtNum next_render_time;
  mjtNum next_trail_time;
  int trail_count;
  mjtNum actual_trail[BENCHMARK_TRAIL_MAX][3];
  mjtNum target_trail[BENCHMARK_TRAIL_MAX][3];
#endif
} benchmark_gui_t;

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static const char *scenario_name(benchmark_scenario_t scenario) {
  switch (scenario) {
    case SCENARIO_HOLD_ZERO_HARSH:
      return "hold_zero_harsh";
    case SCENARIO_STEP_J2_HARSH:
      return "step_j2_harsh";
    case SCENARIO_STEP_J3_HARSH:
      return "step_j3_harsh";
    case SCENARIO_COUPLED_J2J3_HARSH:
      return "coupled_j2j3_harsh";
    case SCENARIO_STEP_J5_HARSH:
      return "step_j5_harsh";
    case SCENARIO_SINE_J2_HARSH:
      return "sine_j2_harsh";
    case SCENARIO_STRAIGHT_ARM_LIFT_HARSH:
      return "straight_arm_lift_harsh";
    case SCENARIO_NEAR_LIMIT_J2_HARSH:
      return "near_limit_j2_harsh";
    case SCENARIO_FLOOR_BLOCKED_HARSH:
      return "floor_blocked_harsh";
    case SCENARIO_TOOL_CIRCLE_XY_HARSH:
      return "tool_circle_xy_harsh";
    case SCENARIO_TOOL_CIRCLE_XZ_HARSH:
      return "tool_circle_xz_harsh";
    case SCENARIO_TOOL_CIRCLE_YZ_HARSH:
      return "tool_circle_yz_harsh";
    case SCENARIO_TOOL_SQUARE_XY_HARSH:
      return "tool_square_xy_harsh";
    case SCENARIO_TOOL_SQUARE_XZ_HARSH:
      return "tool_square_xz_harsh";
    case SCENARIO_TOOL_SQUARE_YZ_HARSH:
      return "tool_square_yz_harsh";
    case SCENARIO_TOOL_INSERT_LINE_HARSH:
      return "tool_insert_line_harsh";
    case SCENARIO_JOINT_CIRCLE_J2J3_HARSH:
      return "joint_circle_j2j3_harsh";
    case SCENARIO_JOINT_SQUARE_J2J3_HARSH:
      return "joint_square_j2j3_harsh";
    case SCENARIO_JOINT_INSERT_LINE_HARSH:
      return "joint_insert_line_harsh";
    default:
      return "unknown";
  }
}

static int parse_scenario(const char *name, benchmark_scenario_t *scenario) {
  if (!name || !scenario) return ARM_ERR_NULL;
  if (strcmp(name, "hold_zero_harsh") == 0) {
    *scenario = SCENARIO_HOLD_ZERO_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "step_j2_harsh") == 0) {
    *scenario = SCENARIO_STEP_J2_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "step_j3_harsh") == 0) {
    *scenario = SCENARIO_STEP_J3_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "coupled_j2j3_harsh") == 0) {
    *scenario = SCENARIO_COUPLED_J2J3_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "step_j5_harsh") == 0) {
    *scenario = SCENARIO_STEP_J5_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "sine_j2_harsh") == 0) {
    *scenario = SCENARIO_SINE_J2_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "straight_arm_lift_harsh") == 0) {
    *scenario = SCENARIO_STRAIGHT_ARM_LIFT_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "near_limit_j2_harsh") == 0) {
    *scenario = SCENARIO_NEAR_LIMIT_J2_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "floor_blocked_harsh") == 0) {
    *scenario = SCENARIO_FLOOR_BLOCKED_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_circle_xy_harsh") == 0) {
    *scenario = SCENARIO_TOOL_CIRCLE_XY_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_circle_xz_harsh") == 0) {
    *scenario = SCENARIO_TOOL_CIRCLE_XZ_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_circle_yz_harsh") == 0) {
    *scenario = SCENARIO_TOOL_CIRCLE_YZ_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_square_xy_harsh") == 0) {
    *scenario = SCENARIO_TOOL_SQUARE_XY_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_square_xz_harsh") == 0) {
    *scenario = SCENARIO_TOOL_SQUARE_XZ_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_square_yz_harsh") == 0) {
    *scenario = SCENARIO_TOOL_SQUARE_YZ_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "tool_insert_line_harsh") == 0) {
    *scenario = SCENARIO_TOOL_INSERT_LINE_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "joint_circle_j2j3_harsh") == 0) {
    *scenario = SCENARIO_JOINT_CIRCLE_J2J3_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "joint_square_j2j3_harsh") == 0) {
    *scenario = SCENARIO_JOINT_SQUARE_J2J3_HARSH;
    return ARM_OK;
  }
  if (strcmp(name, "joint_insert_line_harsh") == 0) {
    *scenario = SCENARIO_JOINT_INSERT_LINE_HARSH;
    return ARM_OK;
  }
  return ARM_ERR_CONFIG;
}

static int parse_on_off(const char *name, bool *enabled) {
  if (!name || !enabled) return ARM_ERR_NULL;
  if (strcmp(name, "on") == 0 || strcmp(name, "true") == 0 || strcmp(name, "1") == 0) {
    *enabled = true;
    return ARM_OK;
  }
  if (strcmp(name, "off") == 0 || strcmp(name, "false") == 0 || strcmp(name, "0") == 0) {
    *enabled = false;
    return ARM_OK;
  }
  return ARM_ERR_CONFIG;
}

static char *trim_ascii(char *text) {
  if (!text) return NULL;
  while (*text && isspace((unsigned char)*text)) {
    ++text;
  }
  char *end = text + strlen(text);
  while (end > text && isspace((unsigned char)end[-1])) {
    --end;
  }
  *end = '\0';
  return text;
}

static int parse_joint_key(const char *key, const char *prefix, uint8_t dof, uint8_t *joint, const char **field) {
  if (!key || !prefix || !joint || !field) return ARM_ERR_NULL;
  const size_t prefix_len = strlen(prefix);
  if (strncmp(key, prefix, prefix_len) != 0) return ARM_ERR_CONFIG;
  const char *cursor = key + prefix_len;
  char *end = NULL;
  const long joint_one_based = strtol(cursor, &end, 10);
  if (end == cursor || !end || *end != '.') return ARM_ERR_CONFIG;
  if (joint_one_based <= 0 || joint_one_based > dof) return ARM_ERR_CONFIG;
  *joint = (uint8_t)(joint_one_based - 1);
  *field = end + 1;
  return ARM_OK;
}

static bool parse_real_value(const char *text, arm_real_t *value) {
  if (!text || !value) return false;
  char *end = NULL;
  const double parsed = strtod(text, &end);
  if (end == text) return false;
  while (end && *end) {
    if (!isspace((unsigned char)*end)) return false;
    ++end;
  }
  *value = ARM_REAL(parsed);
  return true;
}

static int apply_param_override_line(benchmark_app_t *app, const char *key, arm_real_t value) {
  uint8_t joint = 0u;
  const char *field = NULL;

  if (parse_joint_key(key, "pd.", app->core.config.dof, &joint, &field) == ARM_OK) {
    joint_pd_params_t params = app->pd.params[joint];
    if (strcmp(field, "kp") == 0) {
      params.kp = value;
    } else if (strcmp(field, "kd") == 0) {
      params.kd = value;
    } else if (strcmp(field, "out_limit") == 0) {
      params.out_limit = value;
    } else {
      return ARM_ERR_CONFIG;
    }
    joint_pd_set_params(&app->pd, joint, params);
    return ARM_OK;
  }

  if (parse_joint_key(key, "state_filter.", app->core.config.dof, &joint, &field) == ARM_OK) {
    joint_state_filter_params_t params = app->state_filter.params[joint];
    if (strcmp(field, "q_time_constant_s") == 0) {
      params.q_time_constant_s = value;
    } else if (strcmp(field, "dq_time_constant_s") == 0) {
      params.dq_time_constant_s = value;
    } else {
      return ARM_ERR_CONFIG;
    }
    joint_state_filter_set_params(&app->state_filter, joint, params);
    return ARM_OK;
  }

  if (parse_joint_key(key, "shaper.", app->core.config.dof, &joint, &field) == ARM_OK) {
    if (strcmp(field, "dq_limit_rad_s") == 0) {
      app->shaper.params.dq_limit_rad_s[joint] = value;
      app->safety.joints[joint].dq_limit_rad_s = value * ARMSIM_ARM6_SAFETY_DQ_LIMIT_SCALE;
    } else if (strcmp(field, "ddq_limit_rad_s2") == 0) {
      app->shaper.params.ddq_limit_rad_s2[joint] = value;
    } else if (strcmp(field, "dddq_limit_rad_s3") == 0) {
      app->shaper.params.dddq_limit_rad_s3[joint] = value;
    } else {
      return ARM_ERR_CONFIG;
    }
    return ARM_OK;
  }

  if (parse_joint_key(key, "output_limiter.", app->core.config.dof, &joint, &field) == ARM_OK) {
    arm_output_limiter_joint_params_t params = app->output_limiter.joints[joint];
    if (strcmp(field, "tau_rate_limit_nm_s") != 0) return ARM_ERR_CONFIG;
    params.tau_rate_limit_nm_s = value;
    arm_output_limiter_set_joint_params(&app->output_limiter, joint, params);
    return ARM_OK;
  }

  if (strcmp(key, "output_limiter.tau_rate_limit_nm_s") == 0) {
    for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
      arm_output_limiter_set_joint_params(
          &app->output_limiter,
          i,
          (arm_output_limiter_joint_params_t){value});
    }
    return ARM_OK;
  }

  return ARM_ERR_CONFIG;
}

static int apply_param_overrides(benchmark_app_t *app, const char *path) {
  if (!path || !path[0]) return ARM_OK;
  FILE *file = fopen(path, "r");
  if (!file) {
    fprintf(stderr, "Failed to open parameter override file '%s'.\n", path);
    return ARM_ERR_CONFIG;
  }

  char line[256];
  int line_no = 0;
  while (fgets(line, sizeof(line), file)) {
    ++line_no;
    char *text = trim_ascii(line);
    if (!text || !text[0] || text[0] == '#') continue;

    char *equals = strchr(text, '=');
    if (!equals) {
      fprintf(stderr, "Invalid override %s:%d: expected key=value.\n", path, line_no);
      fclose(file);
      return ARM_ERR_CONFIG;
    }
    *equals = '\0';
    char *key = trim_ascii(text);
    char *value_text = trim_ascii(equals + 1);
    arm_real_t value = ARM_REAL_ZERO;
    if (!parse_real_value(value_text, &value)) {
      fprintf(stderr, "Invalid override %s:%d: bad numeric value '%s'.\n", path, line_no, value_text);
      fclose(file);
      return ARM_ERR_CONFIG;
    }
    if (apply_param_override_line(app, key, value) != ARM_OK) {
      fprintf(stderr, "Invalid override %s:%d: unknown key '%s'.\n", path, line_no, key);
      fclose(file);
      return ARM_ERR_CONFIG;
    }
  }

  fclose(file);
  return ARM_OK;
}

static bool scenario_uses_tool_path(benchmark_scenario_t scenario) {
  return scenario == SCENARIO_TOOL_CIRCLE_XY_HARSH ||
         scenario == SCENARIO_TOOL_CIRCLE_XZ_HARSH ||
         scenario == SCENARIO_TOOL_CIRCLE_YZ_HARSH ||
         scenario == SCENARIO_TOOL_SQUARE_XY_HARSH ||
         scenario == SCENARIO_TOOL_SQUARE_XZ_HARSH ||
         scenario == SCENARIO_TOOL_SQUARE_YZ_HARSH ||
         scenario == SCENARIO_TOOL_INSERT_LINE_HARSH;
}

static bool scenario_uses_joint_trajectory(benchmark_scenario_t scenario) {
  return scenario == SCENARIO_JOINT_CIRCLE_J2J3_HARSH ||
         scenario == SCENARIO_JOINT_SQUARE_J2J3_HARSH ||
         scenario == SCENARIO_JOINT_INSERT_LINE_HARSH;
}

static void set_initial_joint_pose(
    const mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    const arm_real_t q_rad[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < arm->config->dof; ++i) {
    const arm_joint_config_t *cfg = &arm->config->joints[i];
    data->qpos[arm->joint_qpos_addr[i]] = (mjtNum)(cfg->q_offset_rad + cfg->sign * q_rad[i]);
  }
  for (int i = 0; i < model->nv; ++i) {
    data->qvel[i] = 0.0;
  }
  for (int i = 0; i < model->nu; ++i) {
    data->ctrl[i] = 0.0;
  }
}

static void configure_initial_pose_for_scenario(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    benchmark_scenario_t scenario) {
  arm_real_t q_rad[ARM_DOF_MAX] = {0};
  if (scenario_uses_tool_path(scenario)) {
    q_rad[1] = ARM_REAL(-0.65);
    q_rad[2] = ARM_REAL(0.85);
    q_rad[4] = ARM_REAL(-0.20);
  } else if (scenario == SCENARIO_JOINT_CIRCLE_J2J3_HARSH) {
    q_rad[1] = ARM_REAL(-0.32);
    q_rad[2] = ARM_REAL(0.58);
  } else if (scenario == SCENARIO_JOINT_SQUARE_J2J3_HARSH ||
             scenario == SCENARIO_JOINT_INSERT_LINE_HARSH) {
    q_rad[1] = ARM_REAL(-0.65);
    q_rad[2] = ARM_REAL(0.85);
    q_rad[4] = ARM_REAL(-0.20);
  }
  set_initial_joint_pose(model, data, arm, q_rad);
  mj_forward(model, data);
}

static int benchmark_gui_init(benchmark_gui_t *gui, const benchmark_app_t *app, bool enabled) {
  if (!gui) return ARM_ERR_NULL;
  gui->enabled = false;
  if (!enabled) return ARM_OK;

#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!app || !app->model) return ARM_ERR_NULL;
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW for benchmark GUI.\n");
    return ARM_ERR_CONFIG;
  }

  gui->window = glfwCreateWindow(1280, 720, "ArmSim Control Benchmark", NULL, NULL);
  if (!gui->window) {
    glfwTerminate();
    fprintf(stderr, "Failed to create benchmark GUI window.\n");
    return ARM_ERR_CONFIG;
  }

  glfwMakeContextCurrent(gui->window);
  glfwSwapInterval(1);
  mjv_defaultCamera(&gui->camera);
  mjv_defaultOption(&gui->option);
  mjv_defaultScene(&gui->scene);
  mjr_defaultContext(&gui->context);
  mjv_makeScene(app->model, &gui->scene, 2000);
  mjr_makeContext(app->model, &gui->context, mjFONTSCALE_150);

  gui->camera.distance = 1.35;
  gui->camera.azimuth = 135.0;
  gui->camera.elevation = -22.0;
  gui->camera.lookat[0] = 0.30;
  gui->camera.lookat[1] = 0.0;
  gui->camera.lookat[2] = 0.20;
  gui->next_render_time = 0.0;
  gui->next_trail_time = 0.0;
  gui->trail_count = 0;
  gui->enabled = true;
  return ARM_OK;
#else
  (void)app;
  fprintf(stderr, "Benchmark GUI was not built because GLFW/OpenGL was not available.\n");
  return ARM_ERR_CONFIG;
#endif
}

static bool benchmark_gui_should_stop(const benchmark_gui_t *gui) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  return gui && gui->enabled && glfwWindowShouldClose(gui->window);
#else
  (void)gui;
  return false;
#endif
}

static int benchmark_app_tool_ref_position(const benchmark_app_t *app, arm_real_t tool_ref[3]) {
  if (!app || !tool_ref) return ARM_ERR_NULL;
  if (app->tool_target_valid) {
    arm_vec3_copy(app->tool_target_pos, tool_ref);
    return ARM_OK;
  }
  if ((app->active_ref.flags & ARM_REFERENCE_Q_VALID) == 0u) return ARM_ERR_CONFIG;

  arm_state_t ref_state = app->core.state;
  ref_state.dof = app->active_ref.dof;
  ref_state.flags |= ARM_STATE_Q_VALID;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    ref_state.q_rad[i] = app->active_ref.q_ref_rad[i];
  }
  return joint_kinematics_fk_position(&app->kinematics, &ref_state, tool_ref);
}

static void benchmark_gui_push_trail_point(benchmark_gui_t *gui, const arm_real_t actual[3], const arm_real_t target[3]) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!gui) return;
  if (gui->trail_count >= BENCHMARK_TRAIL_MAX) {
    for (int i = 1; i < BENCHMARK_TRAIL_MAX; ++i) {
      for (uint8_t axis = 0u; axis < 3u; ++axis) {
        gui->actual_trail[i - 1][axis] = gui->actual_trail[i][axis];
        gui->target_trail[i - 1][axis] = gui->target_trail[i][axis];
      }
    }
    gui->trail_count = BENCHMARK_TRAIL_MAX - 1;
  }
  const int index = gui->trail_count++;
  for (uint8_t axis = 0u; axis < 3u; ++axis) {
    gui->actual_trail[index][axis] = (mjtNum)actual[axis];
    gui->target_trail[index][axis] = (mjtNum)target[axis];
  }
#else
  (void)gui;
  (void)actual;
  (void)target;
#endif
}

static void benchmark_gui_record_trail(benchmark_gui_t *gui, const benchmark_app_t *app) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!gui || !gui->enabled || !app) return;
  if (app->data->time + 1.0e-12 < gui->next_trail_time) return;

  arm_real_t actual[3];
  arm_real_t target[3];
  if (joint_kinematics_fk_position(&app->kinematics, &app->core.state, actual) != ARM_OK) return;
  if (benchmark_app_tool_ref_position(app, target) != ARM_OK) return;
  benchmark_gui_push_trail_point(gui, actual, target);
  gui->next_trail_time = app->data->time + BENCHMARK_TRAIL_DT_S;
#else
  (void)gui;
  (void)app;
#endif
}

static void benchmark_gui_draw_trail(
    mjvScene *scene,
    const mjtNum trail[BENCHMARK_TRAIL_MAX][3],
    int count,
    const float rgba[4],
    double width) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!scene || count < 2) return;
  for (int i = 1; i < count; ++i) {
    if (scene->ngeom >= scene->maxgeom) return;
    mjvGeom *line = &scene->geoms[scene->ngeom++];
    mjv_initGeom(line, mjGEOM_LINE, NULL, NULL, NULL, rgba);
    mjv_connector(line, mjGEOM_LINE, width, trail[i - 1], trail[i]);
    line->rgba[0] = rgba[0];
    line->rgba[1] = rgba[1];
    line->rgba[2] = rgba[2];
    line->rgba[3] = rgba[3];
  }
#else
  (void)scene;
  (void)trail;
  (void)count;
  (void)rgba;
  (void)width;
#endif
}

static void benchmark_gui_render(benchmark_gui_t *gui, const benchmark_app_t *app) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!gui || !gui->enabled || !app || !app->model || !app->data) return;
  benchmark_gui_record_trail(gui, app);
  if (app->data->time + 1.0e-12 < gui->next_render_time) return;

  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(gui->window, &width, &height);
  const mjrRect viewport = {0, 0, width, height};

  mjv_updateScene(app->model, app->data, &gui->option, NULL, &gui->camera, mjCAT_ALL, &gui->scene);
  const float target_rgba[4] = {0.15f, 0.82f, 0.95f, 0.90f};
  const float actual_rgba[4] = {1.00f, 0.55f, 0.18f, 0.95f};
  benchmark_gui_draw_trail(&gui->scene, gui->target_trail, gui->trail_count, target_rgba, 3.0);
  benchmark_gui_draw_trail(&gui->scene, gui->actual_trail, gui->trail_count, actual_rgba, 4.0);
  mjr_render(viewport, &gui->scene, &gui->context);
  glfwSwapBuffers(gui->window);
  glfwPollEvents();
  gui->next_render_time = app->data->time + 1.0 / 60.0;
#else
  (void)gui;
  (void)app;
#endif
}

static void benchmark_gui_close(benchmark_gui_t *gui) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!gui || !gui->enabled) return;
  mjr_freeContext(&gui->context);
  mjv_freeScene(&gui->scene);
  glfwDestroyWindow(gui->window);
  glfwTerminate();
  gui->window = NULL;
  gui->enabled = false;
#else
  (void)gui;
#endif
}

static int parse_ff_mode(const char *name, benchmark_ff_mode_t *mode) {
  if (!name || !mode) return ARM_ERR_NULL;
  if (strcmp(name, "none") == 0) {
    *mode = BENCHMARK_FF_NONE;
    return ARM_OK;
  }
  if (strcmp(name, "gravity") == 0) {
    *mode = BENCHMARK_FF_GRAVITY;
    return ARM_OK;
  }
  if (strcmp(name, "inverse") == 0 || strcmp(name, "inverse_dyn") == 0) {
    *mode = BENCHMARK_FF_INVERSE_DYNAMICS;
    return ARM_OK;
  }
  return ARM_ERR_CONFIG;
}

static void configure_pd(joint_pd_t *pd, uint8_t dof) {
  static const joint_pd_params_t pd_params[ARM_DEFAULT_DOF] = ARMSIM_ARM6_PD_PARAMS;

  joint_pd_init(pd, dof);
  for (uint8_t i = 0u; i < dof; ++i) {
    joint_pd_set_params(pd, i, pd_params[i]);
  }
}

static void joint_limits(const mjModel *model, const mujoco_arm_t *arm, uint8_t joint, arm_real_t *low, arm_real_t *high) {
  const int joint_id = arm->joint_ids[joint];
  const arm_joint_config_t *cfg = &arm->config->joints[joint];

  arm_real_t lo = -ARM_REAL_PI;
  arm_real_t hi = ARM_REAL_PI;
  if (model->jnt_limited[joint_id]) {
    lo = ARM_REAL(model->jnt_range[2 * joint_id + 0]);
    hi = ARM_REAL(model->jnt_range[2 * joint_id + 1]);
  }

  const arm_real_t core_lo = cfg->sign * (lo - cfg->q_offset_rad);
  const arm_real_t core_hi = cfg->sign * (hi - cfg->q_offset_rad);
  *low = core_lo < core_hi ? core_lo : core_hi;
  *high = core_lo < core_hi ? core_hi : core_lo;
}

static void configure_ref_shaper_and_safety(benchmark_app_t *app) {
  static const arm_real_t dq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DQ_LIMITS_RAD_S;
  static const arm_real_t ddq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DDQ_LIMITS_RAD_S2;
  static const arm_real_t dddq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DDDQ_LIMITS_RAD_S3;

  joint_ref_shaper_params_t shaper_params = {0};
  arm_safety_init(&app->safety, app->core.config.dof);

  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    arm_real_t low = ARM_REAL_ZERO;
    arm_real_t high = ARM_REAL_ZERO;
    joint_limits(app->model, &app->mj_arm, i, &low, &high);
    shaper_params.q_min_rad[i] = low;
    shaper_params.q_max_rad[i] = high;
    shaper_params.dq_limit_rad_s[i] = dq_limits[i];
    shaper_params.ddq_limit_rad_s2[i] = ddq_limits[i];
    shaper_params.dddq_limit_rad_s3[i] = dddq_limits[i];

    arm_safety_set_joint_params(
        &app->safety,
        i,
        (arm_safety_joint_params_t){
            low,
            high,
            ARMSIM_ARM6_SAFETY_Q_MARGIN_RAD,
            dq_limits[i] * ARMSIM_ARM6_SAFETY_DQ_LIMIT_SCALE,
            app->core.config.joints[i].torque_limit_nm,
        });
  }

  joint_ref_shaper_init(&app->shaper, app->core.config.dof, &shaper_params);
}

static void configure_state_filter(benchmark_app_t *app, const arm_state_t *initial_state) {
  static const joint_state_filter_params_t filter_params[ARM_DEFAULT_DOF] = ARMSIM_ARM6_STATE_FILTER_PARAMS;

  joint_state_filter_init(&app->state_filter, app->core.config.dof, filter_params);
  joint_state_filter_reset_to_state(&app->state_filter, initial_state);
  arm_state_zero(&app->measured_state, app->core.config.dof);
}

static void configure_output_limiter(benchmark_app_t *app) {
  static const arm_output_limiter_joint_params_t output_limiter_params[ARM_DEFAULT_DOF] =
      ARMSIM_ARM6_OUTPUT_LIMITER_PARAMS;

  arm_output_limiter_init(&app->output_limiter, app->core.config.dof);
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    arm_output_limiter_set_joint_params(&app->output_limiter, i, output_limiter_params[i]);
  }
}

static void configure_gravity_ff(benchmark_app_t *app) {
  static const joint_gravity_ff_params_t gravity_params = ARMSIM_ARM6_GRAVITY_FF_PARAMS;

  joint_gravity_ff_init(&app->gravity_ff, &gravity_params);
  app->ff = joint_gravity_ff_as_feedforward(&app->gravity_ff);
}

static int configure_inverse_dynamics_ff(benchmark_app_t *app) {
  const int status = mujoco_inverse_dynamics_ff_init(&app->inverse_dyn_ff, app->model, &app->mj_arm);
  if (status != ARM_OK) return status;
  app->inverse_dyn_feedforward = mujoco_inverse_dynamics_ff_as_feedforward(&app->inverse_dyn_ff);
  return ARM_OK;
}

static void configure_kinematics(benchmark_app_t *app) {
  static const joint_kinematics_params_t kinematics_params = ARMSIM_ARM6_KINEMATICS_PARAMS;

  app->kinematics = kinematics_params;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    app->initial_q_rad[i] = ARM_REAL_ZERO;
  }
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->initial_q_rad[i] = app->core.state.q_rad[i];
  }
  if (joint_kinematics_fk_position(&app->kinematics, &app->core.state, app->initial_tool_pos) != ARM_OK) {
    arm_vec3_zero(app->initial_tool_pos);
  }
  arm_vec3_copy(app->initial_tool_pos, app->tool_target_pos);
  app->tool_target_valid = false;
}

static void set_initial_reference(benchmark_app_t *app, const arm_state_t *state) {
  arm_reference_zero(&app->manual_goal_ref, app->core.config.dof);
  arm_reference_zero(&app->active_ref, app->core.config.dof);
  app->manual_goal_ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->manual_goal_ref.q_ref_rad[i] = state->q_rad[i];
    app->manual_goal_ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }
  joint_ref_shaper_reset_to_state(&app->shaper, state);
  (void)joint_ref_shaper_step(&app->shaper, state, &app->manual_goal_ref, &app->active_ref);
}

static bool circle_plane_axes(benchmark_scenario_t scenario, uint8_t *axis_a, uint8_t *axis_b) {
  if (!axis_a || !axis_b) return false;
  if (scenario == SCENARIO_TOOL_CIRCLE_XY_HARSH) {
    *axis_a = 0u;
    *axis_b = 1u;
    return true;
  }
  if (scenario == SCENARIO_TOOL_CIRCLE_XZ_HARSH) {
    *axis_a = 0u;
    *axis_b = 2u;
    return true;
  }
  if (scenario == SCENARIO_TOOL_CIRCLE_YZ_HARSH) {
    *axis_a = 1u;
    *axis_b = 2u;
    return true;
  }
  return false;
}

static bool square_plane_axes(benchmark_scenario_t scenario, uint8_t *axis_a, uint8_t *axis_b) {
  if (!axis_a || !axis_b) return false;
  if (scenario == SCENARIO_TOOL_SQUARE_XY_HARSH) {
    *axis_a = 0u;
    *axis_b = 1u;
    return true;
  }
  if (scenario == SCENARIO_TOOL_SQUARE_XZ_HARSH) {
    *axis_a = 0u;
    *axis_b = 2u;
    return true;
  }
  if (scenario == SCENARIO_TOOL_SQUARE_YZ_HARSH) {
    *axis_a = 1u;
    *axis_b = 2u;
    return true;
  }
  return false;
}

static void square_plane_target(
    const arm_real_t center[3],
    uint8_t axis_a,
    uint8_t axis_b,
    arm_real_t half_size_m,
    arm_real_t phase,
    arm_real_t target[3]) {
  arm_vec3_copy(center, target);
  const arm_real_t p = phase - ARM_REAL(4) * ARM_REAL((int)(phase / ARM_REAL(4)));
  if (p < ARM_REAL_ONE) {
    target[axis_a] += half_size_m * (ARM_REAL_ONE - ARM_REAL(2) * p);
    target[axis_b] += half_size_m;
  } else if (p < ARM_REAL(2)) {
    target[axis_a] -= half_size_m;
    target[axis_b] += half_size_m * (ARM_REAL_ONE - ARM_REAL(2) * (p - ARM_REAL_ONE));
  } else if (p < ARM_REAL(3)) {
    target[axis_a] += half_size_m * (-ARM_REAL_ONE + ARM_REAL(2) * (p - ARM_REAL(2)));
    target[axis_b] -= half_size_m;
  } else {
    target[axis_a] += half_size_m;
    target[axis_b] += half_size_m * (-ARM_REAL_ONE + ARM_REAL(2) * (p - ARM_REAL(3)));
  }
}

static void reset_tool_ik_seed(benchmark_app_t *app) {
  app->tool_ik_seed_valid = false;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    app->tool_ik_seed_q_rad[i] = ARM_REAL_ZERO;
  }
}

static int solve_tool_target(benchmark_app_t *app, const arm_real_t target[3]) {
  joint_ik_position_options_t options = {
      24u,
      ARM_REAL(0.035),
      ARM_REAL(0.10),
      ARM_REAL(0.0025),
      app->initial_q_rad,
      ARM_REAL(0.015),
  };
  arm_state_t seed_state = app->core.state;
  if (app->tool_ik_seed_valid) {
    for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
      seed_state.q_rad[i] = app->tool_ik_seed_q_rad[i];
      seed_state.dq_rad_s[i] = ARM_REAL_ZERO;
    }
  }
  const int status = joint_ik_position_solve(&app->kinematics, &seed_state, target, &options, &app->manual_goal_ref);
  if (status == ARM_OK) {
    arm_vec3_copy(target, app->tool_target_pos);
    app->tool_target_valid = true;
    for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
      app->tool_ik_seed_q_rad[i] = app->manual_goal_ref.q_ref_rad[i];
    }
    app->tool_ik_seed_valid = true;
  }
  return status;
}

static int update_tool_scenario_goal(benchmark_app_t *app, benchmark_scenario_t scenario, arm_real_t time_s) {
  const arm_real_t start_s = ARM_REAL(0.8);
  arm_real_t t = time_s - start_s;
  if (t < ARM_REAL_ZERO) t = ARM_REAL_ZERO;

  arm_real_t target[3];
  arm_vec3_copy(app->initial_tool_pos, target);

  uint8_t axis_a = 0u;
  uint8_t axis_b = 2u;
  if (circle_plane_axes(scenario, &axis_a, &axis_b)) {
    const arm_real_t radius = ARM_REAL(0.080);
    const arm_real_t omega = ARM_REAL(2) * ARM_REAL_PI / ARM_REAL(3.2);
    const arm_real_t theta = omega * t;
    target[axis_a] = app->initial_tool_pos[axis_a] - radius + radius * ARM_REAL(cos((double)theta));
    target[axis_b] = app->initial_tool_pos[axis_b] + radius * ARM_REAL(sin((double)theta));
  } else if (square_plane_axes(scenario, &axis_a, &axis_b)) {
    const arm_real_t half_size = ARM_REAL(0.075);
    arm_real_t center[3];
    arm_vec3_copy(app->initial_tool_pos, center);
    center[axis_a] -= half_size;
    center[axis_b] -= half_size;
    square_plane_target(center, axis_a, axis_b, half_size, t / ARM_REAL(0.85), target);
  } else if (scenario == SCENARIO_TOOL_INSERT_LINE_HARSH) {
    const arm_real_t distance = ARM_REAL(0.080);
    arm_real_t progress = ARM_REAL_ZERO;
    if (t < ARM_REAL(1.6)) {
      progress = t / ARM_REAL(1.6);
    } else if (t < ARM_REAL(2.6)) {
      progress = ARM_REAL_ONE;
    } else if (t < ARM_REAL(4.2)) {
      progress = ARM_REAL_ONE - (t - ARM_REAL(2.6)) / ARM_REAL(1.6);
    }
    progress = arm_clamp(progress, ARM_REAL_ZERO, ARM_REAL_ONE);
    target[0] = app->initial_tool_pos[0] - distance * progress;
  }

  return solve_tool_target(app, target);
}

static benchmark_scenario_t joint_trajectory_source_tool_scenario(benchmark_scenario_t scenario) {
  if (scenario == SCENARIO_JOINT_CIRCLE_J2J3_HARSH) return SCENARIO_TOOL_CIRCLE_XZ_HARSH;
  if (scenario == SCENARIO_JOINT_SQUARE_J2J3_HARSH) return SCENARIO_TOOL_SQUARE_XZ_HARSH;
  if (scenario == SCENARIO_JOINT_INSERT_LINE_HARSH) return SCENARIO_TOOL_INSERT_LINE_HARSH;
  return SCENARIO_HOLD_ZERO_HARSH;
}

static arm_real_t joint_trajectory_period_s(benchmark_scenario_t scenario) {
  if (scenario == SCENARIO_JOINT_CIRCLE_J2J3_HARSH) return ARM_REAL(3.2);
  if (scenario == SCENARIO_JOINT_SQUARE_J2J3_HARSH) return ARM_REAL(3.4);
  if (scenario == SCENARIO_JOINT_INSERT_LINE_HARSH) return ARM_REAL(4.2);
  return ARM_REAL_ZERO;
}

static uint8_t joint_trajectory_sample_count(benchmark_scenario_t scenario) {
  if (scenario == SCENARIO_JOINT_CIRCLE_J2J3_HARSH) return 64u;
  if (scenario == SCENARIO_JOINT_SQUARE_J2J3_HARSH) return 72u;
  if (scenario == SCENARIO_JOINT_INSERT_LINE_HARSH) return 85u;
  return 0u;
}

static int tool_path_target_at_time(
    const benchmark_app_t *app,
    benchmark_scenario_t tool_scenario,
    arm_real_t t,
    arm_real_t target[3]) {
  if (!app || !target) return ARM_ERR_NULL;
  if (t < ARM_REAL_ZERO) t = ARM_REAL_ZERO;
  arm_vec3_copy(app->initial_tool_pos, target);

  uint8_t axis_a = 0u;
  uint8_t axis_b = 2u;
  if (circle_plane_axes(tool_scenario, &axis_a, &axis_b)) {
    const arm_real_t radius = ARM_REAL(0.080);
    const arm_real_t omega = ARM_REAL(2) * ARM_REAL_PI / ARM_REAL(3.2);
    const arm_real_t theta = omega * t;
    target[axis_a] = app->initial_tool_pos[axis_a] - radius + radius * ARM_REAL(cos((double)theta));
    target[axis_b] = app->initial_tool_pos[axis_b] + radius * ARM_REAL(sin((double)theta));
    return ARM_OK;
  }
  if (square_plane_axes(tool_scenario, &axis_a, &axis_b)) {
    const arm_real_t half_size = ARM_REAL(0.075);
    arm_real_t center[3];
    arm_vec3_copy(app->initial_tool_pos, center);
    center[axis_a] -= half_size;
    center[axis_b] -= half_size;
    square_plane_target(center, axis_a, axis_b, half_size, t / ARM_REAL(0.85), target);
    return ARM_OK;
  }
  if (tool_scenario == SCENARIO_TOOL_INSERT_LINE_HARSH) {
    const arm_real_t distance = ARM_REAL(0.080);
    arm_real_t progress = ARM_REAL_ZERO;
    if (t < ARM_REAL(1.6)) {
      progress = t / ARM_REAL(1.6);
    } else if (t < ARM_REAL(2.6)) {
      progress = ARM_REAL_ONE;
    } else if (t < ARM_REAL(4.2)) {
      progress = ARM_REAL_ONE - (t - ARM_REAL(2.6)) / ARM_REAL(1.6);
    }
    progress = arm_clamp(progress, ARM_REAL_ZERO, ARM_REAL_ONE);
    target[0] = app->initial_tool_pos[0] - distance * progress;
    return ARM_OK;
  }
  return ARM_ERR_CONFIG;
}

static int build_joint_trajectory_from_ik(benchmark_app_t *app, benchmark_scenario_t scenario) {
  if (!app || !scenario_uses_joint_trajectory(scenario)) return ARM_ERR_CONFIG;
  const uint8_t sample_count = joint_trajectory_sample_count(scenario);
  const arm_real_t period_s = joint_trajectory_period_s(scenario);
  if (sample_count < 2u || sample_count > BENCHMARK_JOINT_TRAJ_MAX || period_s <= ARM_REAL_ZERO) return ARM_ERR_CONFIG;

  const benchmark_scenario_t tool_scenario = joint_trajectory_source_tool_scenario(scenario);
  const bool loop = scenario != SCENARIO_JOINT_INSERT_LINE_HARSH;
  const arm_real_t divisor = loop ? ARM_REAL(sample_count) : ARM_REAL((uint8_t)(sample_count - 1u));
  arm_state_t seed_state = app->core.state;
  joint_ik_position_options_t options = {
      32u,
      ARM_REAL(0.035),
      ARM_REAL(0.10),
      ARM_REAL(0.0025),
      app->initial_q_rad,
      ARM_REAL(0.015),
  };

  for (uint8_t sample = 0u; sample < sample_count; ++sample) {
    arm_real_t target[3];
    const arm_real_t sample_time_s = period_s * ARM_REAL(sample) / divisor;
    int status = tool_path_target_at_time(app, tool_scenario, sample_time_s, target);
    if (status != ARM_OK) return status;

    arm_reference_t solved_ref;
    status = joint_ik_position_solve(&app->kinematics, &seed_state, target, &options, &solved_ref);
    if (status != ARM_OK) return status;
    for (uint8_t joint = 0u; joint < app->core.config.dof; ++joint) {
      const arm_real_t q = solved_ref.q_ref_rad[joint];
      app->joint_traj_waypoints[sample][joint] = q;
      seed_state.q_rad[joint] = q;
      seed_state.dq_rad_s[joint] = ARM_REAL_ZERO;
    }
  }

  app->joint_traj_count = sample_count;
  app->joint_traj_loop = loop;
  app->joint_traj_segment_s = loop ? (period_s / ARM_REAL(sample_count)) : (period_s / ARM_REAL((uint8_t)(sample_count - 1u)));
  app->joint_traj_valid = true;
  return ARM_OK;
}

static void set_direct_ref_from_q_dq_ddq(
    benchmark_app_t *app,
    const arm_real_t q[ARM_DOF_MAX],
    const arm_real_t dq[ARM_DOF_MAX],
    const arm_real_t ddq[ARM_DOF_MAX]) {
  arm_reference_zero(&app->active_ref, app->core.config.dof);
  app->active_ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->active_ref.q_ref_rad[i] = q[i];
    app->active_ref.dq_ref_rad_s[i] = dq[i];
    app->active_ref.ddq_ref_rad_s2[i] = ddq[i];
    app->manual_goal_ref.q_ref_rad[i] = q[i];
    app->manual_goal_ref.dq_ref_rad_s[i] = dq[i];
    app->manual_goal_ref.ddq_ref_rad_s2[i] = ddq[i];
  }
  app->manual_goal_ref.flags = app->active_ref.flags;
  app->use_direct_ref = true;
  app->tool_target_valid = false;
  reset_tool_ik_seed(app);
}

static void set_direct_ref_hold(benchmark_app_t *app, const arm_real_t q[ARM_DOF_MAX]) {
  arm_real_t dq[ARM_DOF_MAX] = {0};
  arm_real_t ddq[ARM_DOF_MAX] = {0};
  set_direct_ref_from_q_dq_ddq(app, q, dq, ddq);
}

static uint8_t trajectory_prev_index(uint8_t index, uint8_t count, bool loop) {
  if (index > 0u) return (uint8_t)(index - 1u);
  return loop ? (uint8_t)(count - 1u) : 0u;
}

static uint8_t trajectory_next_index(uint8_t index, uint8_t count, bool loop) {
  const uint8_t next = (uint8_t)(index + 1u);
  if (next < count) return next;
  return loop ? 0u : (uint8_t)(count - 1u);
}

static arm_real_t trajectory_waypoint_velocity(
    const arm_real_t waypoints[][ARM_DOF_MAX],
    uint8_t waypoint_count,
    uint8_t index,
    uint8_t joint,
    arm_real_t segment_duration_s,
    bool loop) {
  if (!loop && (index == 0u || index + 1u >= waypoint_count)) return ARM_REAL_ZERO;
  const uint8_t prev = trajectory_prev_index(index, waypoint_count, loop);
  const uint8_t next = trajectory_next_index(index, waypoint_count, loop);
  arm_real_t dt = ARM_REAL(2.0) * segment_duration_s;
  if (!loop && (index == 0u || index + 1u >= waypoint_count)) dt = segment_duration_s;
  if (dt <= ARM_REAL_ZERO) return ARM_REAL_ZERO;
  return (waypoints[next][joint] - waypoints[prev][joint]) / dt;
}

static int update_joint_segment_trajectory(
    benchmark_app_t *app,
    const arm_real_t waypoints[][ARM_DOF_MAX],
    uint8_t waypoint_count,
    arm_real_t segment_duration_s,
    arm_real_t time_s,
    bool loop) {
  if (!app || !waypoints || waypoint_count < 2u || segment_duration_s <= ARM_REAL_ZERO) return ARM_ERR_CONFIG;

  const arm_real_t start_s = ARM_REAL(0.8);
  arm_real_t t = time_s - start_s;
  if (t <= ARM_REAL_ZERO) {
    set_direct_ref_hold(app, waypoints[0]);
    return ARM_OK;
  }

  const uint8_t segment_count = loop ? waypoint_count : (uint8_t)(waypoint_count - 1u);
  const arm_real_t total_s = segment_duration_s * ARM_REAL(segment_count);
  if (loop) {
    while (t >= total_s) {
      t -= total_s;
    }
  } else if (t >= total_s) {
    set_direct_ref_hold(app, waypoints[waypoint_count - 1u]);
    return ARM_OK;
  }

  uint8_t segment = (uint8_t)(t / segment_duration_s);
  if (segment >= segment_count) segment = (uint8_t)(segment_count - 1u);
  const uint8_t next = loop ? (uint8_t)((segment + 1u) % waypoint_count) : (uint8_t)(segment + 1u);
  const arm_real_t local_t = t - segment_duration_s * ARM_REAL(segment);
  const arm_real_t p = arm_clamp(local_t / segment_duration_s, ARM_REAL_ZERO, ARM_REAL_ONE);
  const arm_real_t p2 = p * p;
  const arm_real_t p3 = p2 * p;
  const arm_real_t h00 = ARM_REAL(2.0) * p3 - ARM_REAL(3.0) * p2 + ARM_REAL_ONE;
  const arm_real_t h10 = p3 - ARM_REAL(2.0) * p2 + p;
  const arm_real_t h01 = ARM_REAL(-2.0) * p3 + ARM_REAL(3.0) * p2;
  const arm_real_t h11 = p3 - p2;
  const arm_real_t dh00 = (ARM_REAL(6.0) * p2 - ARM_REAL(6.0) * p) / segment_duration_s;
  const arm_real_t dh10 = ARM_REAL(3.0) * p2 - ARM_REAL(4.0) * p + ARM_REAL_ONE;
  const arm_real_t dh01 = (ARM_REAL(-6.0) * p2 + ARM_REAL(6.0) * p) / segment_duration_s;
  const arm_real_t dh11 = ARM_REAL(3.0) * p2 - ARM_REAL(2.0) * p;
  const arm_real_t ddh00 = (ARM_REAL(12.0) * p - ARM_REAL(6.0)) / (segment_duration_s * segment_duration_s);
  const arm_real_t ddh10 = (ARM_REAL(6.0) * p - ARM_REAL(4.0)) / segment_duration_s;
  const arm_real_t ddh01 = (ARM_REAL(-12.0) * p + ARM_REAL(6.0)) / (segment_duration_s * segment_duration_s);
  const arm_real_t ddh11 = (ARM_REAL(6.0) * p - ARM_REAL(2.0)) / segment_duration_s;

  arm_real_t q[ARM_DOF_MAX] = {0};
  arm_real_t dq[ARM_DOF_MAX] = {0};
  arm_real_t ddq[ARM_DOF_MAX] = {0};
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    const arm_real_t q0 = waypoints[segment][i];
    const arm_real_t q1 = waypoints[next][i];
    const arm_real_t v0 = trajectory_waypoint_velocity(waypoints, waypoint_count, segment, i, segment_duration_s, loop);
    const arm_real_t v1 = trajectory_waypoint_velocity(waypoints, waypoint_count, next, i, segment_duration_s, loop);
    q[i] = h00 * q0 + h10 * segment_duration_s * v0 + h01 * q1 + h11 * segment_duration_s * v1;
    dq[i] = dh00 * q0 + dh10 * v0 + dh01 * q1 + dh11 * v1;
    ddq[i] = ddh00 * q0 + ddh10 * v0 + ddh01 * q1 + ddh11 * v1;
  }
  set_direct_ref_from_q_dq_ddq(app, q, dq, ddq);
  return ARM_OK;
}

static int update_joint_trajectory_goal(benchmark_app_t *app, benchmark_scenario_t scenario, arm_real_t time_s) {
  if (!app || !scenario_uses_joint_trajectory(scenario) || !app->joint_traj_valid) return ARM_ERR_CONFIG;
  return update_joint_segment_trajectory(
      app,
      app->joint_traj_waypoints,
      app->joint_traj_count,
      app->joint_traj_segment_s,
      time_s,
      app->joint_traj_loop);
}

static int update_scenario_goal(benchmark_app_t *app, benchmark_scenario_t scenario, arm_real_t time_s) {
  app->use_direct_ref = false;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->manual_goal_ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }

  if (scenario_uses_tool_path(scenario)) {
    return update_tool_scenario_goal(app, scenario, time_s);
  }
  if (scenario_uses_joint_trajectory(scenario)) {
    return update_joint_trajectory_goal(app, scenario, time_s);
  }

  if (scenario == SCENARIO_STEP_J2_HARSH && time_s >= ARM_REAL(1.0)) {
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-0.45);
  } else if (scenario == SCENARIO_STEP_J3_HARSH && time_s >= ARM_REAL(1.0)) {
    app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(-0.45);
  } else if (scenario == SCENARIO_COUPLED_J2J3_HARSH && time_s >= ARM_REAL(1.0)) {
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-0.55);
    app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(0.42);
  } else if (scenario == SCENARIO_STEP_J5_HARSH && time_s >= ARM_REAL(1.0)) {
    app->manual_goal_ref.q_ref_rad[4] = ARM_REAL(0.65);
  } else if (scenario == SCENARIO_SINE_J2_HARSH && time_s >= ARM_REAL(1.0)) {
    const arm_real_t t = time_s - ARM_REAL(1.0);
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-0.25) + ARM_REAL(0.18) * ARM_REAL(sin((double)(ARM_REAL_PI * t)));
  } else if (scenario == SCENARIO_STRAIGHT_ARM_LIFT_HARSH) {
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-0.85);
    app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(0.10);
    if (time_s >= ARM_REAL(1.0)) {
      app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-0.35);
      app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(0.18);
    }
  } else if (scenario == SCENARIO_NEAR_LIMIT_J2_HARSH) {
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-1.65);
    app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(0.35);
    if (time_s >= ARM_REAL(1.0)) {
      app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-2.25);
      app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(0.55);
    }
  } else if (scenario == SCENARIO_FLOOR_BLOCKED_HARSH) {
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(0.35);
    app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(-0.35);
    if (time_s >= ARM_REAL(1.0)) {
      app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(1.35);
      app->manual_goal_ref.q_ref_rad[2] = ARM_REAL(-1.20);
    }
  }
  app->tool_target_valid = false;
  reset_tool_ik_seed(app);
  return ARM_OK;
}

static void compute_gravity_ff(benchmark_app_t *app, arm_real_t tau_ff_gravity[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    tau_ff_gravity[i] = ARM_REAL_ZERO;
  }
  arm_t ff_arm = app->core;
  arm_feedforward_t ff = app->ff;
  if (arm_control_step_with_feedforward(&ff_arm, &app->active_ref, NULL, &ff) != ARM_OK) return;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    tau_ff_gravity[i] = ff_arm.command.tau_ff_nm[i];
  }
}

static arm_feedforward_t *active_feedforward(benchmark_app_t *app) {
  if (app->ff_mode == BENCHMARK_FF_GRAVITY) return &app->ff;
  if (app->ff_mode == BENCHMARK_FF_INVERSE_DYNAMICS) return &app->inverse_dyn_feedforward;
  return NULL;
}

static void compute_active_model_ff(benchmark_app_t *app, arm_real_t tau_ff_model[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    tau_ff_model[i] = ARM_REAL_ZERO;
  }
  arm_feedforward_t *ff = active_feedforward(app);
  if (!ff) return;

  arm_t ff_arm = app->core;
  arm_feedforward_t ff_copy = *ff;
  if (arm_control_step_with_feedforward(&ff_arm, &app->active_ref, NULL, &ff_copy) != ARM_OK) return;
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    tau_ff_model[i] = ff_arm.command.tau_ff_nm[i];
  }
}

static int load_app(benchmark_app_t *app, const char *model_path, benchmark_scenario_t scenario) {
  char load_error[1024] = {0};
  app->model = mj_loadXML(model_path, NULL, load_error, sizeof(load_error));
  if (!app->model) {
    fprintf(stderr, "Failed to load model '%s': %s\n", model_path, load_error);
    return ARM_ERR_CONFIG;
  }

  app->data = mj_makeData(app->model);
  if (!app->data) {
    fprintf(stderr, "Failed to allocate MuJoCo data.\n");
    return ARM_ERR_CONFIG;
  }

  arm_config_t config = armsim_default_arm6_config();
  int status = arm_init(&app->core, &config);
  if (status != ARM_OK) return status;

  char bind_error[256] = {0};
  status = mujoco_arm_bind(app->model, &app->core.config, &app->mj_arm, bind_error, sizeof(bind_error));
  if (status != ARM_OK) {
    fprintf(stderr, "Failed to bind arm: %s\n", bind_error);
    return status;
  }

  if (app->contacts_enabled) {
    app->model->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    app->model->opt.disableflags |= mjDSBL_CONTACT;
  }

  configure_initial_pose_for_scenario(app->model, app->data, &app->mj_arm, scenario);
  mujoco_arm_read_state(
      app->data, &app->mj_arm, ARM_REAL_ZERO, ARM_REAL(app->model->opt.timestep), &app->core.state);

  configure_pd(&app->pd, app->core.config.dof);
  app->ctrl = joint_pd_as_controller(&app->pd);
  configure_ref_shaper_and_safety(app);
  configure_state_filter(app, &app->core.state);
  configure_output_limiter(app);
  status = apply_param_overrides(app, app->param_override_path);
  if (status != ARM_OK) return status;
  configure_gravity_ff(app);
  status = configure_inverse_dynamics_ff(app);
  if (status != ARM_OK) return status;
  configure_kinematics(app);

  armsim_impairment_config_t impairment_config = armsim_impairment_default_config();
  impairment_config.enabled = app->harsh_enabled;
  armsim_impairment_init(&app->impairment, &impairment_config, app->core.config.dof);
  set_initial_reference(app, &app->core.state);
  if (scenario_uses_joint_trajectory(scenario)) {
    status = build_joint_trajectory_from_ik(app, scenario);
    if (status != ARM_OK) {
      fprintf(stderr, "Failed to build IK-derived joint trajectory for %s: %d\n", scenario_name(scenario), status);
      return status;
    }
  }
  return ARM_OK;
}

static void destroy_app(benchmark_app_t *app) {
  control_log_close(&app->log);
  mujoco_inverse_dynamics_ff_free(&app->inverse_dyn_ff);
  if (app->data) {
    mj_deleteData(app->data);
    app->data = NULL;
  }
  if (app->model) {
    mj_deleteModel(app->model);
    app->model = NULL;
  }
}

int main(int argc, char **argv) {
  benchmark_scenario_t scenario = SCENARIO_HOLD_ZERO_HARSH;
  benchmark_ff_mode_t ff_mode = BENCHMARK_FF_GRAVITY;
  bool harsh_enabled = true;
  bool contacts_enabled = true;
  bool gui_enabled = false;
  const char *param_override_path = NULL;
  if (argc > 1 && parse_scenario(argv[1], &scenario) != ARM_OK) {
    fprintf(stderr, "Unknown scenario '%s'.\n", argv[1]);
    return EXIT_FAILURE;
  }

  const char *log_path_arg = NULL;
  const char *model_path_arg = NULL;
  for (int i = 2; i < argc; ++i) {
    if (strncmp(argv[i], "--ff=", 5) == 0) {
      if (parse_ff_mode(argv[i] + 5, &ff_mode) != ARM_OK) {
        fprintf(stderr, "Unknown feedforward mode '%s'.\n", argv[i] + 5);
        return EXIT_FAILURE;
      }
    } else if (strncmp(argv[i], "--harsh=", 8) == 0) {
      if (parse_on_off(argv[i] + 8, &harsh_enabled) != ARM_OK) {
        fprintf(stderr, "Unknown harsh mode '%s'.\n", argv[i] + 8);
        return EXIT_FAILURE;
      }
    } else if (strncmp(argv[i], "--contacts=", 11) == 0) {
      if (parse_on_off(argv[i] + 11, &contacts_enabled) != ARM_OK) {
        fprintf(stderr, "Unknown contacts mode '%s'.\n", argv[i] + 11);
        return EXIT_FAILURE;
      }
    } else if (strncmp(argv[i], "--gui=", 6) == 0) {
      if (parse_on_off(argv[i] + 6, &gui_enabled) != ARM_OK) {
        fprintf(stderr, "Unknown GUI mode '%s'.\n", argv[i] + 6);
        return EXIT_FAILURE;
      }
    } else if (strncmp(argv[i], "--param-overrides=", 18) == 0) {
      param_override_path = argv[i] + 18;
    } else if (!log_path_arg) {
      log_path_arg = argv[i];
    } else if (!model_path_arg) {
      model_path_arg = argv[i];
    } else {
      fprintf(stderr, "Unexpected argument '%s'.\n", argv[i]);
      return EXIT_FAILURE;
    }
  }

  char default_log_path[128];
  (void)snprintf(
      default_log_path,
      sizeof(default_log_path),
      "logs/control_benchmark_%s_%s_harsh-%s_contacts-%s.csv",
      scenario_name(scenario),
      ff_mode == BENCHMARK_FF_INVERSE_DYNAMICS ? "inverse" :
          (ff_mode == BENCHMARK_FF_NONE ? "none" : "gravity"),
      harsh_enabled ? "on" : "off",
      contacts_enabled ? "on" : "off");
  const char *log_path = log_path_arg ? log_path_arg : default_log_path;
  const char *model_path = model_path_arg ? model_path_arg : default_model_path();

  benchmark_app_t app = {0};
  app.harsh_enabled = harsh_enabled;
  app.contacts_enabled = contacts_enabled;
  app.param_override_path = param_override_path;
  int status = load_app(&app, model_path, scenario);
  if (status != ARM_OK) {
    destroy_app(&app);
    return EXIT_FAILURE;
  }
  app.ff_mode = ff_mode;

  benchmark_gui_t gui = {0};
  status = benchmark_gui_init(&gui, &app, gui_enabled);
  if (status != ARM_OK) {
    destroy_app(&app);
    return EXIT_FAILURE;
  }

  if (!control_log_open(&app.log, log_path, app.core.config.dof)) {
    fprintf(stderr, "Failed to open benchmark log '%s'.\n", log_path);
    benchmark_gui_close(&gui);
    destroy_app(&app);
    return EXIT_FAILURE;
  }

  const control_log_flags_t flags = {
      true,
      app.ff_mode == BENCHMARK_FF_GRAVITY,
      app.ff_mode == BENCHMARK_FF_INVERSE_DYNAMICS,
      app.contacts_enabled,
      app.harsh_enabled,
  };
  const arm_real_t duration_s = ARM_REAL(5.0);
  while (ARM_REAL(app.data->time) < duration_s && !benchmark_gui_should_stop(&gui)) {
    status = update_scenario_goal(&app, scenario, ARM_REAL(app.data->time));
    if (status != ARM_OK) {
      fprintf(stderr, "Scenario update failed: %d\n", status);
      benchmark_gui_close(&gui);
      destroy_app(&app);
      return EXIT_FAILURE;
    }
    if (!app.use_direct_ref) {
      status = joint_ref_shaper_step(&app.shaper, &app.core.state, &app.manual_goal_ref, &app.active_ref);
      if (status != ARM_OK) {
        fprintf(stderr, "Reference shaper failed: %d\n", status);
        benchmark_gui_close(&gui);
        destroy_app(&app);
        return EXIT_FAILURE;
      }
    }

    status = armsim_step_once_impaired_filtered_with_feedforward(
        app.model,
        app.data,
        &app.mj_arm,
        &app.core,
        &app.active_ref,
        &app.safety,
        &app.ctrl,
        active_feedforward(&app),
        &app.output_limiter,
        &app.impairment,
        &app.state_filter,
        &app.measured_state);
    if (status != ARM_OK) {
      fprintf(stderr, "Benchmark step failed: %d\n", status);
      benchmark_gui_close(&gui);
      destroy_app(&app);
      return EXIT_FAILURE;
    }

    arm_real_t tau_ff_gravity[ARM_DOF_MAX];
    arm_real_t tau_ff_model[ARM_DOF_MAX];
    arm_real_t tool_pos[3];
    arm_real_t tool_ref[3];
    compute_gravity_ff(&app, tau_ff_gravity);
    compute_active_model_ff(&app, tau_ff_model);
    if (joint_kinematics_fk_position(&app.kinematics, &app.core.state, tool_pos) != ARM_OK) {
      arm_vec3_zero(tool_pos);
    }
    if (benchmark_app_tool_ref_position(&app, tool_ref) != ARM_OK) {
      arm_vec3_copy(tool_pos, tool_ref);
    }
    (void)control_log_write_step(
        &app.log,
        app.model,
        app.data,
        &app.mj_arm,
        &flags,
        &app.measured_state,
        &app.core.state,
        &app.active_ref,
        tool_pos,
        tool_ref,
        tau_ff_gravity,
        tau_ff_model,
        &app.core.command);
    benchmark_gui_render(&gui, &app);
  }

  printf("Wrote %s benchmark log to %s\n", scenario_name(scenario), log_path);
  benchmark_gui_close(&gui);
  destroy_app(&app);
  return EXIT_SUCCESS;
}
