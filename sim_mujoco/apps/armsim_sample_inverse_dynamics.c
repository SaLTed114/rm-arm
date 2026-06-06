#include <stdio.h>
#include <stdlib.h>

#include <mujoco/mujoco.h>

#include "arm_common/arm_math.h"
#include "arm_core/arm.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/mujoco_inverse_dynamics_ff.h"

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static unsigned lcg_next(unsigned *state) {
  *state = (*state * 1664525u) + 1013904223u;
  return *state;
}

static arm_real_t rand_unit(unsigned *state) {
  return ARM_REAL((double)(lcg_next(state) & 0x00ffffffu) / (double)0x01000000u);
}

static arm_real_t rand_range(unsigned *state, arm_real_t low, arm_real_t high) {
  return low + (high - low) * rand_unit(state);
}

static void joint_limits(
    const mjModel *model,
    const mujoco_arm_t *arm,
    uint8_t joint,
    arm_real_t *low,
    arm_real_t *high) {
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

static void write_header(FILE *file, uint8_t dof) {
  for (uint8_t i = 0u; i < dof; ++i) (void)fprintf(file, "%sq%u", i == 0u ? "" : ",", (unsigned)(i + 1u));
  for (uint8_t i = 0u; i < dof; ++i) (void)fprintf(file, ",dq%u", (unsigned)(i + 1u));
  for (uint8_t i = 0u; i < dof; ++i) (void)fprintf(file, ",ddq%u", (unsigned)(i + 1u));
  for (uint8_t i = 0u; i < dof; ++i) (void)fprintf(file, ",tau%u", (unsigned)(i + 1u));
  (void)fprintf(file, "\n");
}

static void write_values(FILE *file, const arm_real_t values[ARM_DOF_MAX], uint8_t dof, bool first_group) {
  for (uint8_t i = 0u; i < dof; ++i) {
    (void)fprintf(file, "%s%.9f", first_group && i == 0u ? "" : ",", (double)values[i]);
  }
}

int main(int argc, char **argv) {
  const char *csv_path = argc > 1 ? argv[1] : "logs/inverse_dynamics_samples.csv";
  const unsigned samples = argc > 2 ? (unsigned)strtoul(argv[2], NULL, 10) : 800u;
  const char *model_path = argc > 3 ? argv[3] : default_model_path();

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

  FILE *file = fopen(csv_path, "w");
  if (!file) {
    fprintf(stderr, "Failed to open output CSV '%s'.\n", csv_path);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }
  write_header(file, config.dof);

  arm_state_t state;
  arm_reference_t ref;
  arm_state_zero(&state, config.dof);
  arm_reference_zero(&ref, config.dof);
  state.flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID;
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;

  unsigned rng = 0x12345678u;
  arm_real_t tau[ARM_DOF_MAX];
  for (unsigned sample = 0u; sample < samples; ++sample) {
    for (uint8_t joint = 0u; joint < config.dof; ++joint) {
      arm_real_t low;
      arm_real_t high;
      joint_limits(model, &arm, joint, &low, &high);
      ref.q_ref_rad[joint] = rand_range(&rng, low * ARM_REAL(0.85), high * ARM_REAL(0.85));
      ref.dq_ref_rad_s[joint] = rand_range(&rng, ARM_REAL(-1.5), ARM_REAL(1.5));
      ref.ddq_ref_rad_s2[joint] = rand_range(&rng, ARM_REAL(-8.0), ARM_REAL(8.0));
      state.q_rad[joint] = ref.q_ref_rad[joint];
      state.dq_rad_s[joint] = ref.dq_ref_rad_s[joint];
    }
    const int status = mujoco_inverse_dynamics_compute_tau(model, data, &arm, &state, &ref, tau);
    if (status != ARM_OK) {
      fprintf(stderr, "Inverse dynamics failed at sample %u: %d\n", sample, status);
      fclose(file);
      mj_deleteData(data);
      mj_deleteModel(model);
      return EXIT_FAILURE;
    }
    write_values(file, ref.q_ref_rad, config.dof, true);
    write_values(file, ref.dq_ref_rad_s, config.dof, false);
    write_values(file, ref.ddq_ref_rad_s2, config.dof, false);
    write_values(file, tau, config.dof, false);
    (void)fprintf(file, "\n");
  }

  fclose(file);
  mj_deleteData(data);
  mj_deleteModel(model);
  printf("Wrote %u inverse-dynamics samples to %s\n", samples, csv_path);
  return EXIT_SUCCESS;
}
