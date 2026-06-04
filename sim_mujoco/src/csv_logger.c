#include "armsim/csv_logger.h"

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
  if (!last_sep) {
    return;
  }

  const size_t len = (size_t)(last_sep - path);
  if (len == 0u || len >= sizeof(dir)) {
    return;
  }

  memcpy(dir, path, len);
  dir[len] = '\0';
  (void)ARMSIM_MKDIR(dir);
}

bool csv_logger_open(csv_logger_t *logger, const char *path, uint8_t dof) {
  if (!logger || !path || dof == 0u || dof > ARM_DOF_MAX) {
    return false;
  }

  create_parent_dir_if_needed(path);
  logger->file = fopen(path, "w");
  logger->dof = dof;
  return logger->file != NULL;
}

void csv_logger_close(csv_logger_t *logger) {
  if (!logger || !logger->file) {
    return;
  }
  fclose(logger->file);
  logger->file = NULL;
}

bool csv_logger_write_header(csv_logger_t *logger) {
  if (!logger || !logger->file) {
    return false;
  }

  fprintf(logger->file, "time_s,active_joint");
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",q%u", (unsigned)(i + 1u));
  }
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",dq%u", (unsigned)(i + 1u));
  }
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",cmd_tau%u", (unsigned)(i + 1u));
  }
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",mj_ctrl%u", (unsigned)(i + 1u));
  }
  fprintf(logger->file, "\n");
  return true;
}

bool csv_logger_write_step(
    csv_logger_t *logger,
    const arm_state_t *state,
    const arm_command_t *command,
    const arm_real_t *mj_ctrl,
    int active_joint) {
  if (!logger || !logger->file || !state || !command || !mj_ctrl) {
    return false;
  }

  fprintf(logger->file, "%.9f,%d", (double)state->time_s, active_joint);
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",%.9f", (double)state->q_rad[i]);
  }
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",%.9f", (double)state->dq_rad_s[i]);
  }
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",%.9f", (double)command->tau_nm[i]);
  }
  for (uint8_t i = 0u; i < logger->dof; ++i) {
    fprintf(logger->file, ",%.9f", (double)mj_ctrl[i]);
  }
  fprintf(logger->file, "\n");
  return true;
}
