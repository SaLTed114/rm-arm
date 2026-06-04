#ifndef ARM_CORE_ARM_CONTROL_H_
#define ARM_CORE_ARM_CONTROL_H_

#include "arm_core/arm.h"

typedef struct arm_controller arm_controller_t;

typedef struct {
  void (*reset)(void *ctx);
  int (*step)(void *ctx, arm_t *arm);
} arm_controller_vtable_t;

struct arm_controller {
  const arm_controller_vtable_t *vt;
  void *ctx;
};

int arm_control_step(arm_t *arm, arm_controller_t *controller);

#endif
