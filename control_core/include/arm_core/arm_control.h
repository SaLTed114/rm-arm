#ifndef ARM_CORE_ARM_CONTROL_H_
#define ARM_CORE_ARM_CONTROL_H_

#include "arm_core/arm_types.h"

typedef struct arm_controller arm_controller_t;

typedef struct {
  void (*reset)(void *ctx);
  int (*step)(
      void *ctx,
      const arm_config_t *config,
      const arm_state_t *state,
      arm_command_t *command);
} arm_controller_vtable_t;

struct arm_controller {
  const arm_controller_vtable_t *vt;
  void *ctx;
};

int arm_config_validate(const arm_config_t *config);
void arm_state_zero(arm_state_t *state, uint8_t dof);
void arm_command_zero(arm_command_t *command, uint8_t dof);
void arm_command_apply_limits(const arm_config_t *config, arm_command_t *command);

int arm_control_step(
    const arm_config_t *config,
    arm_controller_t *controller,
    const arm_state_t *state,
    arm_command_t *command);

#endif
