#ifndef ARM_CORE_JOINT_GRAVITY_FF_H_
#define ARM_CORE_JOINT_GRAVITY_FF_H_

#include "arm_core/arm_feedforward.h"

#define JOINT_GRAVITY_FF_BODY_MAX 16u
#define JOINT_GRAVITY_FF_NO_BODY (-1)
#define JOINT_GRAVITY_FF_NO_JOINT (-1)

typedef struct {
  int8_t parent;
  int8_t joint;
  arm_real_t pos[3];
  arm_real_t axis[3];
  arm_real_t mass_kg;
  arm_real_t com[3];
} joint_gravity_ff_body_t;

typedef struct {
  uint8_t dof;
  uint8_t body_count;
  arm_real_t gravity_m_s2[3];
  joint_gravity_ff_body_t bodies[JOINT_GRAVITY_FF_BODY_MAX];
} joint_gravity_ff_params_t;

typedef struct {
  joint_gravity_ff_params_t params;
} joint_gravity_ff_t;

void joint_gravity_ff_init(joint_gravity_ff_t *gravity, const joint_gravity_ff_params_t *params);
arm_feedforward_t joint_gravity_ff_as_feedforward(joint_gravity_ff_t *gravity);

#endif
