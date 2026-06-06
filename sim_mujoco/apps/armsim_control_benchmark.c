#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#include "armsim/sim_impairment.h"
#include "armsim/sim_loop.h"

typedef enum {
  SCENARIO_HOLD_ZERO_HARSH = 0,
  SCENARIO_STEP_J2_HARSH = 1,
  SCENARIO_COUPLED_J2J3_HARSH = 2,
  SCENARIO_STEP_J5_HARSH = 3,
} benchmark_scenario_t;

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
  armsim_impairment_t impairment;
  control_log_t log;
} benchmark_app_t;

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

  mj_forward(app->model, app->data);
  mujoco_arm_read_state(
      app->data, &app->mj_arm, ARM_REAL_ZERO, ARM_REAL(app->model->opt.timestep), &app->core.state);

  configure_pd(&app->pd, app->core.config.dof);
  app->ctrl = joint_pd_as_controller(&app->pd);
  configure_ref_shaper_and_safety(app);
  configure_state_filter(app, &app->core.state);
  configure_gravity_ff(app);

  armsim_impairment_config_t impairment_config = armsim_impairment_default_config();
  armsim_impairment_init(&app->impairment, &impairment_config, app->core.config.dof);
  set_initial_reference(app, &app->core.state);
  return ARM_OK;
}

static void destroy_app(benchmark_app_t *app) {
  control_log_close(&app->log);
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
  if (argc > 1 && parse_scenario(argv[1], &scenario) != ARM_OK) {
    fprintf(stderr, "Unknown scenario '%s'.\n", argv[1]);
    return EXIT_FAILURE;
  }

  char default_log_path[128];
  (void)snprintf(
      default_log_path,
      sizeof(default_log_path),
      "logs/control_benchmark_%s.csv",
      scenario_name(scenario));
  const char *log_path = argc > 2 ? argv[2] : default_log_path;
  const char *model_path = argc > 3 ? argv[3] : default_model_path();

  benchmark_app_t app = {0};
  int status = load_app(&app, model_path);
  if (status != ARM_OK) {
    destroy_app(&app);
    return EXIT_FAILURE;
  }

  if (!control_log_open(&app.log, log_path, app.core.config.dof)) {
    fprintf(stderr, "Failed to open benchmark log '%s'.\n", log_path);
    destroy_app(&app);
    return EXIT_FAILURE;
  }

  const control_log_flags_t flags = {true, true, true, true};
  const arm_real_t duration_s = ARM_REAL(5.0);
  while (ARM_REAL(app.data->time) < duration_s) {
    update_scenario_goal(&app, scenario, ARM_REAL(app.data->time));
    status = joint_ref_shaper_step(&app.shaper, &app.core.state, &app.manual_goal_ref, &app.active_ref);
    if (status != ARM_OK) {
      fprintf(stderr, "Reference shaper failed: %d\n", status);
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
        &app.ff,
        &app.impairment,
        &app.state_filter,
        &app.measured_state);
    if (status != ARM_OK) {
      fprintf(stderr, "Benchmark step failed: %d\n", status);
      destroy_app(&app);
      return EXIT_FAILURE;
    }

    arm_real_t tau_ff_gravity[ARM_DOF_MAX];
    compute_gravity_ff(&app, tau_ff_gravity);
    (void)control_log_write_step(
        &app.log,
        app.data,
        &app.mj_arm,
        &flags,
        &app.measured_state,
        &app.core.state,
        &app.active_ref,
        tau_ff_gravity,
        &app.core.command);
  }

  printf("Wrote %s benchmark log to %s\n", scenario_name(scenario), log_path);
  destroy_app(&app);
  return EXIT_SUCCESS;
}
