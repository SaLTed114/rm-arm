#include "armsim/sim_impairment.h"

#include "arm_core/arm_math.h"
#include "armsim/arm6_sim_config.h"
#include "armsim/sim_loop.h"

static uint32_t rng_next(uint32_t *state) {
  uint32_t x = *state;
  if (x == 0u) {
    x = 0x6d2b79f5u;
  }
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static arm_real_t rng_uniform_signed(uint32_t *state, arm_real_t amplitude) {
  if (amplitude <= ARM_REAL_ZERO) {
    return ARM_REAL_ZERO;
  }
  const uint32_t sample = rng_next(state);
  const arm_real_t unit = ARM_REAL(sample & 0x00ffffffu) / ARM_REAL(0x00ffffffu);
  return (ARM_REAL(2) * unit - ARM_REAL_ONE) * amplitude;
}

static arm_real_t quantize(arm_real_t value, arm_real_t resolution) {
  if (resolution <= ARM_REAL_ZERO) {
    return value;
  }
  return ARM_REAL((int)(value / resolution + (value >= ARM_REAL_ZERO ? ARM_REAL(0.5) : ARM_REAL(-0.5)))) *
         resolution;
}

static arm_real_t move_with_rate_limit(arm_real_t current, arm_real_t target, arm_real_t max_delta) {
  if (max_delta <= ARM_REAL_ZERO) {
    return target;
  }
  return current + arm_clamp(target - current, -max_delta, max_delta);
}

static arm_real_t first_order_step(arm_real_t current, arm_real_t target, arm_real_t dt_s, arm_real_t tau_s) {
  if (tau_s <= ARM_REAL_ZERO || dt_s <= ARM_REAL_ZERO) {
    return target;
  }
  const arm_real_t alpha = arm_clamp(dt_s / (tau_s + dt_s), ARM_REAL_ZERO, ARM_REAL_ONE);
  return current + alpha * (target - current);
}

static void push_state(armsim_impairment_t *impairment, const arm_state_t *state) {
  impairment->delay_states[impairment->delay_write_index] = *state;
  impairment->delay_write_index =
      (uint8_t)((impairment->delay_write_index + 1u) % (ARMSIM_IMPAIRMENT_DELAY_MAX + 1u));
  if (impairment->delay_count < ARMSIM_IMPAIRMENT_DELAY_MAX + 1u) {
    ++impairment->delay_count;
  }
}

static const arm_state_t *delayed_state(const armsim_impairment_t *impairment) {
  uint8_t steps = impairment->config.sensor_delay_steps;
  if (steps > ARMSIM_IMPAIRMENT_DELAY_MAX) {
    steps = ARMSIM_IMPAIRMENT_DELAY_MAX;
  }
  if (steps >= impairment->delay_count) {
    steps = (uint8_t)(impairment->delay_count - 1u);
  }

  const uint8_t size = ARMSIM_IMPAIRMENT_DELAY_MAX + 1u;
  const uint8_t index = (uint8_t)((impairment->delay_write_index + size - 1u - steps) % size);
  return &impairment->delay_states[index];
}

static void build_measured_state(
    armsim_impairment_t *impairment,
    const arm_state_t *raw,
    arm_real_t dt_s,
    arm_state_t *state) {
  *state = *raw;
  state->dt_s = dt_s;

  for (uint8_t i = 0u; i < impairment->dof; ++i) {
    arm_real_t q = quantize(raw->q_rad[i], impairment->config.encoder_resolution_rad);
    q += rng_uniform_signed(&impairment->rng_state, impairment->config.q_noise_rad);
    state->q_rad[i] = q;

    arm_real_t dq = raw->dq_rad_s[i];
    if (impairment->config.use_dq_from_q_diff && impairment->has_last_q && dt_s > ARM_REAL_ZERO) {
      dq = (q - impairment->last_q_rad[i]) / dt_s;
    }

    dq += rng_uniform_signed(&impairment->rng_state, impairment->config.dq_noise_rad_s);
    const arm_real_t alpha = arm_clamp(impairment->config.dq_filter_alpha, ARM_REAL_ZERO, ARM_REAL_ONE);
    impairment->filtered_dq_rad_s[i] += alpha * (dq - impairment->filtered_dq_rad_s[i]);
    state->dq_rad_s[i] = impairment->filtered_dq_rad_s[i];
    state->tau_est_nm[i] = impairment->applied_tau_nm[i] +
                           rng_uniform_signed(&impairment->rng_state, impairment->config.tau_est_noise_nm);
    impairment->last_q_rad[i] = q;
  }

  state->flags = ARM_STATE_Q_VALID | ARM_STATE_DQ_VALID | ARM_STATE_TAU_EST_VALID;
  impairment->has_last_q = true;
}

static bool control_due(armsim_impairment_t *impairment, arm_real_t time_s) {
  if (!impairment->has_control_time) {
    impairment->next_control_time_s = time_s;
    impairment->last_control_time_s = time_s;
    impairment->has_control_time = true;
  }
  return time_s + ARM_REAL(1e-12) >= impairment->next_control_time_s;
}

static void schedule_next_control(armsim_impairment_t *impairment) {
  arm_real_t period = impairment->config.control_period_s;
  if (period <= ARM_REAL_ZERO) {
    period = ARM_REAL(0.001);
  }
  period += rng_uniform_signed(&impairment->rng_state, impairment->config.control_jitter_s);
  if (period < ARM_REAL(0.0001)) {
    period = ARM_REAL(0.0001);
  }
  impairment->next_control_time_s += period;
}

static void update_actuator_model(
    mjData *data,
    const mujoco_arm_t *arm,
    armsim_impairment_t *impairment,
    arm_real_t dt_s) {
  for (uint8_t i = 0u; i < impairment->dof; ++i) {
    arm_real_t target = impairment->target_tau_nm[i];
    if (arm_abs(target) < impairment->config.actuator_deadband_nm) {
      target = ARM_REAL_ZERO;
    }

    arm_real_t tau = first_order_step(
        impairment->applied_tau_nm[i], target, dt_s, impairment->config.actuator_tau_time_constant_s);
    if (impairment->config.actuator_tau_rate_limit_nm_s > ARM_REAL_ZERO && dt_s > ARM_REAL_ZERO) {
      tau = move_with_rate_limit(
          impairment->applied_tau_nm[i], tau, impairment->config.actuator_tau_rate_limit_nm_s * dt_s);
    }

    const arm_real_t limit = arm->config->joints[i].torque_limit_nm;
    if (limit > ARM_REAL_ZERO) {
      tau = arm_clamp(tau, -limit, limit);
    }

    impairment->applied_tau_nm[i] = tau;
    data->ctrl[arm->actuator_ids[i]] = (mjtNum)(arm->config->joints[i].sign * tau);
  }
}

armsim_impairment_config_t armsim_impairment_default_config(void) {
  armsim_impairment_config_t config = {0};
  config.enabled = true;
  config.control_period_s = ARMSIM_HARSH_CONTROL_PERIOD_S;
  config.control_jitter_s = ARMSIM_HARSH_CONTROL_JITTER_S;
  config.sensor_delay_steps = ARMSIM_HARSH_SENSOR_DELAY_STEPS;
  config.encoder_resolution_rad = ARMSIM_HARSH_ENCODER_RESOLUTION_RAD;
  config.q_noise_rad = ARMSIM_HARSH_Q_NOISE_RAD;
  config.dq_noise_rad_s = ARMSIM_HARSH_DQ_NOISE_RAD_S;
  config.tau_est_noise_nm = ARMSIM_HARSH_TAU_EST_NOISE_NM;
  config.use_dq_from_q_diff = true;
  config.dq_filter_alpha = ARMSIM_HARSH_DQ_FILTER_ALPHA;
  config.actuator_tau_time_constant_s = ARMSIM_HARSH_ACTUATOR_TAU_TIME_CONSTANT_S;
  config.actuator_tau_rate_limit_nm_s = ARMSIM_HARSH_ACTUATOR_TAU_RATE_LIMIT_NM_S;
  config.actuator_deadband_nm = ARMSIM_HARSH_ACTUATOR_DEADBAND_NM;
  config.random_seed = ARMSIM_HARSH_RANDOM_SEED;
  return config;
}

void armsim_impairment_init(
    armsim_impairment_t *impairment,
    const armsim_impairment_config_t *config,
    uint8_t dof) {
  if (!impairment) {
    return;
  }

  impairment->config = config ? *config : armsim_impairment_default_config();
  impairment->dof = arm_sanitize_dof(dof);
  armsim_impairment_reset(impairment);
}

void armsim_impairment_reset(armsim_impairment_t *impairment) {
  if (!impairment) {
    return;
  }

  impairment->rng_state = impairment->config.random_seed ? impairment->config.random_seed : 0x12345678u;
  impairment->next_control_time_s = ARM_REAL_ZERO;
  impairment->last_control_time_s = ARM_REAL_ZERO;
  impairment->delay_write_index = 0u;
  impairment->delay_count = 0u;
  impairment->has_control_time = false;
  impairment->has_last_q = false;
  for (uint8_t i = 0u; i < ARM_DOF_MAX; ++i) {
    impairment->target_tau_nm[i] = ARM_REAL_ZERO;
    impairment->applied_tau_nm[i] = ARM_REAL_ZERO;
    impairment->last_q_rad[i] = ARM_REAL_ZERO;
    impairment->filtered_dq_rad_s[i] = ARM_REAL_ZERO;
  }
  for (uint8_t i = 0u; i < ARMSIM_IMPAIRMENT_DELAY_MAX + 1u; ++i) {
    arm_state_zero(&impairment->delay_states[i], impairment->dof);
  }
}

int armsim_step_once_impaired(
    mjModel *model,
    mjData *data,
    const mujoco_arm_t *arm,
    arm_t *core,
    const arm_reference_t *ref,
    const arm_safety_t *safety,
    arm_controller_t *ctrl,
    armsim_impairment_t *impairment) {
  return armsim_step_once_impaired_filtered(
      model, data, arm, core, ref, safety, ctrl, impairment, NULL, NULL);
}

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
    arm_state_t *measured_state) {
  return armsim_step_once_impaired_filtered_with_feedforward(
      model, data, arm, core, ref, safety, ctrl, NULL, impairment, state_filter, measured_state);
}

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
    arm_state_t *measured_state) {
  if (!impairment || !impairment->config.enabled) {
    if (!state_filter) {
      return armsim_step_once_with_feedforward(model, data, arm, core, ref, safety, ctrl, ff);
    }
    if (!model || !data || !arm || !core) {
      return ARM_ERR_NULL;
    }
    arm_state_t local_measured;
    arm_state_t filtered;
    arm_state_t *measured = measured_state ? measured_state : &local_measured;
    mujoco_arm_read_state(data, arm, ARM_REAL(data->time), ARM_REAL(model->opt.timestep), measured);
    const int filter_status = joint_state_filter_step(state_filter, measured, &filtered);
    if (filter_status != ARM_OK) {
      return filter_status;
    }
    return armsim_step_once_with_state_and_feedforward(
        model, data, arm, core, &filtered, ref, safety, ctrl, ff);
  }
  if (!model || !data || !arm || !core) {
    return ARM_ERR_NULL;
  }
  if (impairment->dof != core->config.dof) {
    return ARM_ERR_DOF;
  }

  arm_state_t raw_state;
  mujoco_arm_read_state(data, arm, ARM_REAL(data->time), ARM_REAL(model->opt.timestep), &raw_state);
  push_state(impairment, &raw_state);

  const arm_real_t time_s = ARM_REAL(data->time);
  if (control_due(impairment, time_s)) {
    const arm_real_t dt_s = impairment->has_control_time ? time_s - impairment->last_control_time_s
                                                         : impairment->config.control_period_s;
    const arm_state_t *delayed = delayed_state(impairment);
    arm_state_t local_measured;
    arm_state_t filtered;
    arm_state_t *measured = measured_state ? measured_state : &local_measured;
    build_measured_state(
        impairment, delayed, dt_s > ARM_REAL_ZERO ? dt_s : impairment->config.control_period_s, measured);

    if (state_filter) {
      const int filter_status = joint_state_filter_step(state_filter, measured, &filtered);
      if (filter_status != ARM_OK) {
        return filter_status;
      }
      core->state = filtered;
    } else {
      core->state = *measured;
    }

    const int status = arm_control_step_with_feedforward(core, ref, ctrl, ff);
    if (status != ARM_OK) {
      return status;
    }
    if (safety) {
      const int safety_status = arm_safety_apply(safety, &core->state, &core->command);
      if (safety_status != ARM_OK) {
        return safety_status;
      }
    }
    for (uint8_t i = 0u; i < impairment->dof; ++i) {
      impairment->target_tau_nm[i] = core->command.tau_ff_nm[i];
    }
    impairment->last_control_time_s = time_s;
    schedule_next_control(impairment);
  }

  update_actuator_model(data, arm, impairment, ARM_REAL(model->opt.timestep));
  mj_step(model, data);
  return ARM_OK;
}
