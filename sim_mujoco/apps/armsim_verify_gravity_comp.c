#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <mujoco/mujoco.h>

#include "arm_core/arm.h"
#include "arm_core/arm_control.h"
#include "arm_common/arm_math.h"
#include "arm_core/joint_gravity_ff.h"
#include "armsim/arm6_sim_config.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/mujoco_gravity_oracle.h"
#include "armsim/mujoco_inverse_dynamics_ff.h"

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static void set_core_joint_pose(mjData *data, const mujoco_arm_t *arm, uint8_t joint, arm_real_t q_core) {
  const arm_joint_config_t *cfg = &arm->config->joints[joint];
  data->qpos[arm->joint_qpos_addr[joint]] = (mjtNum)(cfg->q_offset_rad + cfg->sign * q_core);
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
  if (mujoco_arm_bind(model, &config, &arm, bind_error, sizeof(bind_error)) != ARM_OK) {
    fprintf(stderr, "Failed to bind arm: %s\n", bind_error);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  set_core_joint_pose(data, &arm, 1u, ARM_REAL(-0.8));
  set_core_joint_pose(data, &arm, 2u, ARM_REAL(-0.25));
  mj_forward(model, data);

  arm_state_t state;
  mujoco_arm_read_state(data, &arm, ARM_REAL_ZERO, ARM_REAL(model->opt.timestep), &state);
  core.state = state;

  arm_reference_t ref;
  arm_reference_zero(&ref, config.dof);
  ref.flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID;
  for (uint8_t i = 0u; i < config.dof; ++i) {
    ref.q_ref_rad[i] = state.q_rad[i];
    ref.dq_ref_rad_s[i] = ARM_REAL_ZERO;
  }

  arm_reference_t oracle_ref = ref;
  mujoco_gravity_oracle_t oracle;
  if (mujoco_gravity_oracle_init(model, &oracle) != ARM_OK) {
    fprintf(stderr, "Failed to initialize gravity oracle.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  const int status = mujoco_gravity_oracle_apply_to_ref(model, data, &arm, &state, &oracle_ref, &oracle);
  mujoco_gravity_oracle_free(&oracle);

  if (status != ARM_OK) {
    fprintf(stderr, "Gravity oracle failed: %d\n", status);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  mujoco_inverse_dynamics_ff_t inverse_ff;
  if (mujoco_inverse_dynamics_ff_init(&inverse_ff, model, &arm) != ARM_OK) {
    fprintf(stderr, "Failed to initialize inverse dynamics oracle.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }
  arm_real_t tau_inverse[ARM_DOF_MAX];
  const int inverse_status =
      mujoco_inverse_dynamics_compute_tau(model, inverse_ff.scratch, &arm, &state, &ref, tau_inverse);
  mujoco_inverse_dynamics_ff_free(&inverse_ff);
  if (inverse_status != ARM_OK) {
    fprintf(stderr, "Inverse dynamics oracle failed: %d\n", inverse_status);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  static const joint_gravity_ff_params_t gravity_params = ARMSIM_ARM6_GRAVITY_FF_PARAMS;
  joint_gravity_ff_t gravity;
  joint_gravity_ff_init(&gravity, &gravity_params);
  arm_feedforward_t feedforward = joint_gravity_ff_as_feedforward(&gravity);
  const int core_status = arm_control_step_with_feedforward(&core, &ref, NULL, &feedforward);
  mj_deleteData(data);
  mj_deleteModel(model);

  if (core_status != ARM_OK) {
    fprintf(stderr, "Core gravity feedforward failed: %d\n", core_status);
    return EXIT_FAILURE;
  }
  if (!(oracle_ref.flags & ARM_REFERENCE_TAU_FF_VALID) || !(core.command.flags & ARM_COMMAND_TAU_FF_VALID)) {
    fprintf(stderr, "Gravity feedforward did not mark torque valid.\n");
    return EXIT_FAILURE;
  }
  if (fabs((double)core.command.tau_ff_nm[1]) < 1.0 || fabs((double)core.command.tau_ff_nm[2]) < 0.2) {
    fprintf(
        stderr,
        "Core gravity feedforward unexpectedly small: tau2=%.9f tau3=%.9f\n",
        (double)core.command.tau_ff_nm[1],
        (double)core.command.tau_ff_nm[2]);
    return EXIT_FAILURE;
  }
  const double shoulder_err = fabs((double)(core.command.tau_ff_nm[1] - oracle_ref.tau_ff_nm[1]));
  const double elbow_err = fabs((double)(core.command.tau_ff_nm[2] - oracle_ref.tau_ff_nm[2]));
  const double inverse_shoulder_err = fabs((double)(tau_inverse[1] - oracle_ref.tau_ff_nm[1]));
  const double inverse_elbow_err = fabs((double)(tau_inverse[2] - oracle_ref.tau_ff_nm[2]));
  if (shoulder_err > 3.0 || elbow_err > 1.0) {
    fprintf(
        stderr,
        "Core gravity feedforward differs from MuJoCo oracle: core=(%.9f, %.9f) oracle=(%.9f, %.9f)\n",
        (double)core.command.tau_ff_nm[1],
        (double)core.command.tau_ff_nm[2],
        (double)oracle_ref.tau_ff_nm[1],
        (double)oracle_ref.tau_ff_nm[2]);
    return EXIT_FAILURE;
  }
  if (inverse_shoulder_err > 1.0e-6 || inverse_elbow_err > 1.0e-6) {
    fprintf(
        stderr,
        "Inverse dynamics static torque differs from MuJoCo gravity oracle: inverse=(%.9f, %.9f) oracle=(%.9f, %.9f)\n",
        (double)tau_inverse[1],
        (double)tau_inverse[2],
        (double)oracle_ref.tau_ff_nm[1],
        (double)oracle_ref.tau_ff_nm[2]);
    return EXIT_FAILURE;
  }

  printf(
      "PASS: core gravity feedforward matches MuJoCo oracle within tolerance: core=(%.6f, %.6f) oracle=(%.6f, %.6f)\n",
      (double)core.command.tau_ff_nm[1],
      (double)core.command.tau_ff_nm[2],
      (double)oracle_ref.tau_ff_nm[1],
      (double)oracle_ref.tau_ff_nm[2]);
  return EXIT_SUCCESS;
}
