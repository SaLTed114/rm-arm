#ifndef ARM_CORE_ARM_H_
#define ARM_CORE_ARM_H_

#include "arm_core/arm_types.h"

typedef struct {
  arm_config_t config;
  arm_state_t state;
  arm_command_t command;
} arm_t;

int arm_config_validate(const arm_config_t *config);
int arm_init(arm_t *arm, const arm_config_t *config);
void arm_reset(arm_t *arm);
void arm_clear_state(arm_t *arm);
void arm_clear_command(arm_t *arm);
void arm_limit_command(arm_t *arm);

void arm_state_zero(arm_state_t *state, uint8_t dof);
void arm_command_zero(arm_command_t *command, uint8_t dof);
void arm_command_apply_limits(const arm_config_t *config, arm_command_t *command);

#endif
