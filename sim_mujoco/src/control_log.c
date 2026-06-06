#include "armsim/control_log.h"

#include <string.h>

#if defined(_WIN32)
#include <direct.h>
#define ARMSIM_MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define ARMSIM_MKDIR(path) mkdir((path), 0777)
#endif

static void create_parent_dir_if_needed(const char *path) {
  char dir[256];
  const char *last_sep = NULL;

  for (const char *p = path; *p != '\0'; ++p) {
    if (*p == '/' || *p == '\\') {
      last_sep = p;
    }
  }
  if (!last_sep) return;

  const size_t len = (size_t)(last_sep - path);
  if (len == 0u || len >= sizeof(dir)) return;

  memcpy(dir, path, len);
  dir[len] = '\0';
  (void)ARMSIM_MKDIR(dir);
}

static void write_columns(control_log_t *log, const char *prefix) {
  for (uint8_t i = 0u; i < log->dof; ++i) {
    (void)fprintf(log->file, ",%s%u", prefix, (unsigned)(i + 1u));
  }
}

static void write_values(control_log_t *log, const arm_real_t values[ARM_DOF_MAX]) {
  for (uint8_t i = 0u; i < log->dof; ++i) {
    (void)fprintf(log->file, ",%.9f", (double)values[i]);
  }
}

static void write_zero_values(control_log_t *log) {
  for (uint8_t i = 0u; i < log->dof; ++i) {
    (void)fprintf(log->file, ",0.000000000");
  }
}

bool control_log_open(control_log_t *log, const char *path, uint8_t dof) {
  if (!log || !path || dof == 0u || dof > ARM_DOF_MAX) return false;

  create_parent_dir_if_needed(path);
  log->file = fopen(path, "w");
  log->dof = dof;
  if (!log->file) {
    log->dof = 0u;
    return false;
  }
  return control_log_write_header(log);
}

void control_log_close(control_log_t *log) {
  if (!log || !log->file) return;
  fclose(log->file);
  log->file = NULL;
}

bool control_log_write_header(control_log_t *log) {
  if (!log || !log->file) return false;

  (void)fprintf(log->file, "time_s,gravity_on,gravity_ff_on,inverse_dyn_ff_on,contacts_on,harsh_on");
  write_columns(log, "q_meas");
  write_columns(log, "dq_meas");
  write_columns(log, "q_filt");
  write_columns(log, "dq_filt");
  write_columns(log, "q_ref");
  write_columns(log, "dq_ref");
  write_columns(log, "ddq_ref");
  write_columns(log, "tau_ff_gravity");
  write_columns(log, "tau_ff_model");
  write_columns(log, "tau_fb");
  write_columns(log, "tau_cmd");
  write_columns(log, "mj_ctrl");
  (void)fprintf(log->file, ",state_flags,command_flags\n");
  return true;
}

bool control_log_write_step(
    control_log_t *log,
    const mjData *data,
    const mujoco_arm_t *arm,
    const control_log_flags_t *flags,
    const arm_state_t *measured_state,
    const arm_state_t *filtered_state,
    const arm_reference_t *ref,
    const arm_real_t tau_ff_gravity[ARM_DOF_MAX],
    const arm_real_t tau_ff_model[ARM_DOF_MAX],
    const arm_command_t *command) {
  if (!log || !log->file || !data || !arm || !filtered_state || !ref || !command) return false;

  const control_log_flags_t zero_flags = {0};
  const control_log_flags_t *f = flags ? flags : &zero_flags;
  (void)fprintf(
      log->file,
      "%.9f,%u,%u,%u,%u,%u",
      (double)data->time,
      f->gravity_on ? 1u : 0u,
      f->gravity_ff_on ? 1u : 0u,
      f->inverse_dyn_ff_on ? 1u : 0u,
      f->contacts_on ? 1u : 0u,
      f->harsh_on ? 1u : 0u);

  if (measured_state) {
    write_values(log, measured_state->q_rad);
    write_values(log, measured_state->dq_rad_s);
  } else {
    write_zero_values(log);
    write_zero_values(log);
  }
  write_values(log, filtered_state->q_rad);
  write_values(log, filtered_state->dq_rad_s);
  write_values(log, ref->q_ref_rad);
  write_values(log, ref->dq_ref_rad_s);
  write_values(log, ref->ddq_ref_rad_s2);

  if (tau_ff_gravity) {
    write_values(log, tau_ff_gravity);
  } else {
    write_zero_values(log);
  }
  if (tau_ff_model) {
    write_values(log, tau_ff_model);
  } else {
    write_zero_values(log);
  }

  for (uint8_t i = 0u; i < log->dof; ++i) {
    const arm_real_t model_ff = tau_ff_model ? tau_ff_model[i] : ARM_REAL_ZERO;
    (void)fprintf(log->file, ",%.9f", (double)(command->tau_ff_nm[i] - model_ff));
  }
  write_values(log, command->tau_ff_nm);
  for (uint8_t i = 0u; i < log->dof; ++i) {
    const arm_real_t ctrl_core =
        arm->config->joints[i].sign * ARM_REAL(data->ctrl[arm->actuator_ids[i]]);
    (void)fprintf(log->file, ",%.9f", (double)ctrl_core);
  }
  (void)fprintf(log->file, ",%u,%u\n", filtered_state->flags, command->flags);
  return true;
}
