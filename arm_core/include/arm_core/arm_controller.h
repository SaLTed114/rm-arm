#ifndef ARM_CORE_ARM_CONTROLLER_H_
#define ARM_CORE_ARM_CONTROLLER_H_

#include "arm_core/arm.h"

typedef struct arm_controller arm_controller_t;

typedef struct {
  void (*reset)(void *ctx);
  int (*step)(void *ctx, arm_t *arm, const arm_reference_t *ref);
} arm_controller_vtable_t;

struct arm_controller {
  const arm_controller_vtable_t *vt;
  void *ctx;
};

#endif
