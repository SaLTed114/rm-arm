#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mujoco.h>

#include "arm_core/arm_math.h"
#include "arm_core/joint_sweep.h"
#include "armsim/csv_logger.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/sim_loop.h"

typedef struct {
  unsigned ctrl_mismatch_count;
  unsigned response_missing_count;
  arm_real_t max_passive_cmd_abs;
  arm_real_t positive_peak_dq[ARM_DOF_MAX];
  arm_real_t negative_peak_dq[ARM_DOF_MAX];
} verify_stats_t;

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static void collect_mj_ctrl(const mjData *data, const mujoco_arm_t *arm, arm_real_t out[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < arm->dof; ++i) {
    out[i] = (arm_real_t)data->ctrl[arm->actuator_ids[i]];
  }
}

static void update_stats(
    verify_stats_t *stats,
    const mujoco_arm_t *arm,
    const arm_command_t *command,
    const arm_state_t *state,
    const arm_real_t mj_ctrl[ARM_DOF_MAX],
    const joint_sweep_t *sweep) {
  const arm_real_t tolerance = 1e-7;
  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const arm_real_t expected_mj_ctrl = arm->config->joints[i].sign * command->tau_nm[i];
    if (arm_abs(mj_ctrl[i] - expected_mj_ctrl) > tolerance) {
      ++stats->ctrl_mismatch_count;
    }
    if ((int)i != sweep->active_joint) {
      const arm_real_t passive = arm_abs(command->tau_nm[i]);
      if (passive > stats->max_passive_cmd_abs) {
        stats->max_passive_cmd_abs = passive;
      }
    }
  }

  if (sweep->active_joint >= 0 && sweep->active_joint < (int16_t)arm->dof) {
    const uint8_t joint = (uint8_t)sweep->active_joint;
    if (sweep->active_direction > 0 && state->dq_rad_s[joint] > stats->positive_peak_dq[joint]) {
      stats->positive_peak_dq[joint] = state->dq_rad_s[joint];
    } else if (sweep->active_direction < 0 && state->dq_rad_s[joint] < stats->negative_peak_dq[joint]) {
      stats->negative_peak_dq[joint] = state->dq_rad_s[joint];
    }
  }
}

static int summarize_stats(const verify_stats_t *stats, uint8_t dof) {
  int ok = 1;
  if (stats->ctrl_mismatch_count != 0u) {
    printf("FAIL: actuator ctrl mismatches: %u\n", stats->ctrl_mismatch_count);
    ok = 0;
  }
  if (stats->max_passive_cmd_abs > 1e-9) {
    printf("FAIL: passive joint command leakage: %.9f Nm\n", (double)stats->max_passive_cmd_abs);
    ok = 0;
  }

  for (uint8_t i = 0u; i < dof; ++i) {
    if (stats->positive_peak_dq[i] <= 1e-5 || stats->negative_peak_dq[i] >= -1e-5) {
      printf(
          "FAIL: joint_%u weak or wrong signed response: positive_peak=%.9f, negative_peak=%.9f\n",
          (unsigned)(i + 1u),
          (double)stats->positive_peak_dq[i],
          (double)stats->negative_peak_dq[i]);
      ok = 0;
    }
  }

  if (ok) {
    printf("PASS: joint torque I/O channels look consistent.\n");
  }
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main(int argc, char **argv) {
  const char *model_path = argc > 1 ? argv[1] : default_model_path();
  const char *log_path = argc > 2 ? argv[2] : "logs/joint_verify.csv";

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

  arm_config_t config = armsim_default_arm6_config();
  mujoco_arm_t arm;
  char bind_error[256] = {0};
  if (mujoco_arm_bind(model, &config, &arm, bind_error, sizeof(bind_error)) != ARM_OK) {
    fprintf(stderr, "Failed to bind arm: %s\n", bind_error);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  joint_sweep_t sweep;
  joint_sweep_init(&sweep, config.dof, (joint_sweep_params_t){1.0, 0.35, 0.25});
  arm_controller_t controller = joint_sweep_as_controller(&sweep);

  csv_logger_t logger;
  if (!csv_logger_open(&logger, log_path, config.dof) || !csv_logger_write_header(&logger)) {
    fprintf(stderr, "Failed to open CSV log '%s'.\n", log_path);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  verify_stats_t stats;
  memset(&stats, 0, sizeof(stats));

  arm_state_t state;
  arm_command_t command;
  arm_real_t mj_ctrl[ARM_DOF_MAX] = {0};
  const arm_real_t duration_s = joint_sweep_total_duration(&sweep) + 0.1;

  while ((arm_real_t)data->time < duration_s && !sweep.complete) {
    const int step_status = armsim_step_once(model, data, &arm, &config, &controller, &state, &command);
    if (step_status != ARM_OK) {
      fprintf(stderr, "Simulation step failed: %d\n", step_status);
      csv_logger_close(&logger);
      mj_deleteData(data);
      mj_deleteModel(model);
      return EXIT_FAILURE;
    }

    collect_mj_ctrl(data, &arm, mj_ctrl);
    update_stats(&stats, &arm, &command, &state, mj_ctrl, &sweep);
    (void)csv_logger_write_step(&logger, &state, &command, mj_ctrl, sweep.active_joint);
  }

  csv_logger_close(&logger);
  const int result = summarize_stats(&stats, config.dof);

  mj_deleteData(data);
  mj_deleteModel(model);
  return result;
}
