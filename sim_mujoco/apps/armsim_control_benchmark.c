#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
#include "arm_motion/joint_ref_shaper.h"
#include "armsim/arm6_sim_config.h"
#include "armsim/control_log.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/mujoco_inverse_dynamics_ff.h"
#include "armsim/sim_impairment.h"
#include "armsim/sim_loop.h"

typedef enum {
  SCENARIO_HOLD_ZERO_HARSH = 0,
  SCENARIO_STEP_J2_HARSH = 1,
  SCENARIO_COUPLED_J2J3_HARSH = 2,
  SCENARIO_STEP_J5_HARSH = 3,
  SCENARIO_SINE_J2_HARSH = 4,
  SCENARIO_STRAIGHT_ARM_LIFT_HARSH = 5,
  SCENARIO_NEAR_LIMIT_J2_HARSH = 6,
  SCENARIO_FLOOR_BLOCKED_HARSH = 7,
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
  joint_pd_t pd;
  arm_controller_t ctrl;
  joint_gravity_ff_t gravity_ff;
  arm_feedforward_t ff;
  mujoco_inverse_dynamics_ff_t inverse_dyn_ff;
  arm_feedforward_t inverse_dyn_feedforward;
  benchmark_ff_mode_t ff_mode;
  armsim_impairment_t impairment;
  control_log_t log;
  bool harsh_enabled;
  bool contacts_enabled;
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

static void benchmark_gui_render(benchmark_gui_t *gui, const benchmark_app_t *app) {
#if defined(ARMSIM_BENCHMARK_WITH_GUI)
  if (!gui || !gui->enabled || !app || !app->model || !app->data) return;
  if (app->data->time + 1.0e-12 < gui->next_render_time) return;

  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(gui->window, &width, &height);
  const mjrRect viewport = {0, 0, width, height};

  mjv_updateScene(app->model, app->data, &gui->option, NULL, &gui->camera, mjCAT_ALL, &gui->scene);
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

static void update_scenario_goal(benchmark_app_t *app, benchmark_scenario_t scenario, arm_real_t time_s) {
  for (uint8_t i = 0u; i < app->core.config.dof; ++i) {
    app->manual_goal_ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }

  if (scenario == SCENARIO_STEP_J2_HARSH && time_s >= ARM_REAL(1.0)) {
    app->manual_goal_ref.q_ref_rad[1] = ARM_REAL(-0.45);
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

static int load_app(benchmark_app_t *app, const char *model_path) {
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

  mj_forward(app->model, app->data);
  mujoco_arm_read_state(
      app->data, &app->mj_arm, ARM_REAL_ZERO, ARM_REAL(app->model->opt.timestep), &app->core.state);

  configure_pd(&app->pd, app->core.config.dof);
  app->ctrl = joint_pd_as_controller(&app->pd);
  configure_ref_shaper_and_safety(app);
  configure_state_filter(app, &app->core.state);
  configure_gravity_ff(app);
  status = configure_inverse_dynamics_ff(app);
  if (status != ARM_OK) return status;

  armsim_impairment_config_t impairment_config = armsim_impairment_default_config();
  impairment_config.enabled = app->harsh_enabled;
  armsim_impairment_init(&app->impairment, &impairment_config, app->core.config.dof);
  set_initial_reference(app, &app->core.state);
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
  int status = load_app(&app, model_path);
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
    update_scenario_goal(&app, scenario, ARM_REAL(app.data->time));
    status = joint_ref_shaper_step(&app.shaper, &app.core.state, &app.manual_goal_ref, &app.active_ref);
    if (status != ARM_OK) {
      fprintf(stderr, "Reference shaper failed: %d\n", status);
      benchmark_gui_close(&gui);
      destroy_app(&app);
      return EXIT_FAILURE;
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
    compute_gravity_ff(&app, tau_ff_gravity);
    compute_active_model_ff(&app, tau_ff_model);
    (void)control_log_write_step(
        &app.log,
        app.model,
        app.data,
        &app.mj_arm,
        &flags,
        &app.measured_state,
        &app.core.state,
        &app.active_ref,
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
