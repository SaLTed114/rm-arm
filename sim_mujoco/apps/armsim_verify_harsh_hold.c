#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <mujoco/mujoco.h>

#include "arm_core/arm.h"
#include "arm_core/arm_control.h"
#include "arm_core/arm_math.h"
#include "arm_core/arm_safety.h"
#include "arm_core/joint_gravity_ff.h"
#include "arm_core/joint_pvi.h"
#include "arm_core/joint_state_filter.h"
#include "armsim/arm6_sim_config.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/sim_impairment.h"

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static void configure_pvi(joint_pvi_t *pvi, uint8_t dof) {
  static const joint_pvi_params_t pvi_params[ARM_DEFAULT_DOF] = ARMSIM_ARM6_PVI_PARAMS;

  joint_pvi_init(pvi, dof);
  for (uint8_t i = 0u; i < dof; ++i) {
    joint_pvi_set_params(pvi, i, pvi_params[i]);
  }
}

static void configure_safety(const mjModel *model, const mujoco_arm_t *arm, arm_safety_t *safety) {
  static const arm_real_t dq_limits[ARM_DEFAULT_DOF] = ARMSIM_ARM6_DQ_LIMITS_RAD_S;

  arm_safety_init(safety, arm->dof);
  for (uint8_t i = 0u; i < arm->dof; ++i) {
    const int joint_id = arm->joint_ids[i];
    arm_real_t low = -ARM_REAL_PI;
    arm_real_t high = ARM_REAL_PI;
    if (model->jnt_limited[joint_id]) {
      const arm_joint_config_t *cfg = &arm->config->joints[i];
      const arm_real_t raw_low = ARM_REAL(model->jnt_range[2 * joint_id + 0]);
      const arm_real_t raw_high = ARM_REAL(model->jnt_range[2 * joint_id + 1]);
      const arm_real_t core_low = cfg->sign * (raw_low - cfg->q_offset_rad);
      const arm_real_t core_high = cfg->sign * (raw_high - cfg->q_offset_rad);
      low = core_low < core_high ? core_low : core_high;
      high = core_low < core_high ? core_high : core_low;
    }

    arm_safety_set_joint_params(
        safety,
        i,
        (arm_safety_joint_params_t){
            low,
            high,
            ARMSIM_ARM6_SAFETY_Q_MARGIN_RAD,
            dq_limits[i] * ARMSIM_ARM6_SAFETY_DQ_LIMIT_SCALE,
            arm->config->joints[i].torque_limit_nm,
        });
  }
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

  arm_config_t config = armsim_default_arm6_config();
  arm_t core;
  if (arm_init(&core, &config) != ARM_OK) {
    fprintf(stderr, "Failed to initialize arm core.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  mujoco_arm_t arm;
  char bind_error[256] = {0};
  if (mujoco_arm_bind(model, &core.config, &arm, bind_error, sizeof(bind_error)) != ARM_OK) {
    fprintf(stderr, "Failed to bind arm: %s\n", bind_error);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  mj_forward(model, data);
  arm_state_t state;
  mujoco_arm_read_state(data, &arm, ARM_REAL_ZERO, ARM_REAL(model->opt.timestep), &state);

  arm_reference_t ref;
  arm_reference_zero(&ref, core.config.dof);
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < core.config.dof; ++i) {
    ref.q_ref_rad[i] = state.q_rad[i];
    ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }

  joint_pvi_t pvi;
  configure_pvi(&pvi, core.config.dof);
  arm_controller_t ctrl = joint_pvi_as_controller(&pvi);

  static const joint_gravity_ff_params_t gravity_params = ARMSIM_ARM6_GRAVITY_FF_PARAMS;
  joint_gravity_ff_t gravity;
  joint_gravity_ff_init(&gravity, &gravity_params);
  arm_feedforward_t ff = joint_gravity_ff_as_feedforward(&gravity);

  static const joint_state_filter_params_t filter_params[ARM_DEFAULT_DOF] = ARMSIM_ARM6_STATE_FILTER_PARAMS;
  joint_state_filter_t filter;
  joint_state_filter_init(&filter, core.config.dof, filter_params);
  joint_state_filter_reset_to_state(&filter, &state);

  arm_safety_t safety;
  configure_safety(model, &arm, &safety);

  armsim_impairment_config_t impairment_config = armsim_impairment_default_config();
  armsim_impairment_t impairment;
  armsim_impairment_init(&impairment, &impairment_config, core.config.dof);

  arm_state_t measured;
  arm_state_zero(&measured, core.config.dof);
  const arm_real_t duration_s = ARM_REAL(5.0);
  arm_real_t max_abs_q23 = ARM_REAL_ZERO;
  arm_real_t max_abs_dq23 = ARM_REAL_ZERO;
  uint32_t saturation_count = 0u;
  uint32_t sample_count = 0u;

  while (ARM_REAL(data->time) < duration_s) {
    const int status = armsim_step_once_impaired_filtered_with_feedforward(
        model, data, &arm, &core, &ref, &safety, &ctrl, &ff, &impairment, &filter, &measured);
    if (status != ARM_OK) {
      fprintf(stderr, "Harsh hold step failed: %d\n", status);
      mj_deleteData(data);
      mj_deleteModel(model);
      return EXIT_FAILURE;
    }

    const arm_real_t q2 = arm_abs(core.state.q_rad[1]);
    const arm_real_t q3 = arm_abs(core.state.q_rad[2]);
    const arm_real_t dq2 = arm_abs(core.state.dq_rad_s[1]);
    const arm_real_t dq3 = arm_abs(core.state.dq_rad_s[2]);
    if (q2 > max_abs_q23) max_abs_q23 = q2;
    if (q3 > max_abs_q23) max_abs_q23 = q3;
    if (dq2 > max_abs_dq23) max_abs_dq23 = dq2;
    if (dq3 > max_abs_dq23) max_abs_dq23 = dq3;

    for (uint8_t i = 1u; i <= 2u; ++i) {
      const arm_real_t limit = core.config.joints[i].torque_limit_nm;
      if (limit > ARM_REAL_ZERO && arm_abs(core.command.tau_ff_nm[i]) > limit - ARM_REAL(1e-6)) {
        ++saturation_count;
      }
    }
    ++sample_count;
  }

  mj_deleteData(data);
  mj_deleteModel(model);

  const double saturation_ratio = sample_count > 0u ? (double)saturation_count / (double)(2u * sample_count) : 1.0;
  if (max_abs_q23 > ARM_REAL(0.12) || max_abs_dq23 > ARM_REAL(1.8) || saturation_ratio > 0.05) {
    fprintf(
        stderr,
        "FAIL: harsh hold unstable: max_abs_q23=%.6f rad max_abs_dq23=%.6f rad/s saturation=%.3f\n",
        (double)max_abs_q23,
        (double)max_abs_dq23,
        saturation_ratio);
    return EXIT_FAILURE;
  }

  printf(
      "PASS: harsh hold stable: max_abs_q23=%.6f rad max_abs_dq23=%.6f rad/s saturation=%.3f\n",
      (double)max_abs_q23,
      (double)max_abs_dq23,
      saturation_ratio);
  return EXIT_SUCCESS;
}
