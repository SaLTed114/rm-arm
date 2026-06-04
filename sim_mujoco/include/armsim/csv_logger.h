#ifndef ARMSIM_CSV_LOGGER_H_
#define ARMSIM_CSV_LOGGER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_core/arm_types.h"

typedef struct {
  FILE *file;
  uint8_t dof;
} csv_logger_t;

bool csv_logger_open(csv_logger_t *logger, const char *path, uint8_t dof);
void csv_logger_close(csv_logger_t *logger);
bool csv_logger_write_header(csv_logger_t *logger);
bool csv_logger_write_step(
    csv_logger_t *logger,
    const arm_state_t *state,
    const arm_command_t *command,
    const arm_real_t *mj_ctrl,
    int active_joint);

#endif
