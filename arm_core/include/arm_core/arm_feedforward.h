#ifndef ARM_CORE_ARM_FEEDFORWARD_H_
#define ARM_CORE_ARM_FEEDFORWARD_H_

#include "arm_core/arm.h"

typedef struct arm_feedforward arm_feedforward_t;

typedef struct {
  void (*reset)(void *ctx);
  int (*step)(void *ctx, arm_t *arm, const arm_reference_t *ref);
} arm_feedforward_vtable_t;

struct arm_feedforward {
  const arm_feedforward_vtable_t *vt;
  void *ctx;
};

#endif
