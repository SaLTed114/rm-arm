#ifndef ARMSIM_SIM_IMPAIRMENT_H_
#define ARMSIM_SIM_IMPAIRMENT_H_

#include <stdbool.h>
#include <stdint.h>

#include "arm_core/arm_control.h"
#include "arm_core/arm_safety.h"
#include "arm_core/joint_state_filter.h"
#include "armsim/mujoco_arm.h"

#define ARMSIM_IMPAIRMENT_DELAY_MAX 16u

typedef struct {
  bool enabled;
  arm_real_t control_period_s;
  arm_real_t control_jitter_s;
  uint8_t sensor_delay_steps;
  arm_real_t encoder_resolution_rad;
  arm_real_t q_noise_rad;
  arm_real_t dq_noise_rad_s;
  arm_real_t tau_est_noise_nm;
  bool use_dq_from_q_diff;
  arm_real_t dq_filter_alpha;
  arm_real_t actuator_tau_time_constant_s;
  arm_real_t actuator_tau_rate_limit_nm_s;
  arm_real_t actuator_deadband_nm;
  uint32_t random_seed;
} armsim_impairment_config_t;

typedef struct {
  armsim_impairment_config_t config;
  uint8_t dof;
  uint32_t rng_state;
  arm_real_t next_control_time_s;
  arm_real_t last_control_time_s;
  arm_real_t target_tau_nm[ARM_DOF_MAX];
  arm_real_t applied_tau_nm[ARM_DOF_MAX];
  arm_real_t last_q_rad[ARM_DOF_MAX];
  arm_real_t filtered_dq_rad_s[ARM_DOF_MAX];
  arm_state_t delay_states[ARMSIM_IMPAIRMENT_DELAY_MAX + 1u];
  uint8_t delay_write_index;
  uint8_t delay_count;
  bool has_control_time;
  bool has_last_q;
} armsim_impairment_t;

armsim_impairment_config_t armsim_impairment_default_config(void);
void armsim_impairment_init(
    armsim_impairment_t *impairment,
    const armsim_impairment_config_t *config,
    uint8_t dof);
void armsim_impairment_reset(armsim_impairment_t *impairment);
int armsim_step_once_impaired(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    armsim_impairment_t *impairment);
int armsim_step_once_impaired_filtered(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    armsim_impairment_t *impairment,
    joint_state_filter_t *state_filter,
    arm_state_t *measured_state);
int armsim_step_once_impaired_filtered_with_feedforward(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    arm_feedforward_t *ff,
    armsim_impairment_t *impairment,
    joint_state_filter_t *state_filter,
    arm_state_t *measured_state);

#endif
