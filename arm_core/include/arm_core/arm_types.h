#ifndef ARM_CORE_ARM_TYPES_H_
#define ARM_CORE_ARM_TYPES_H_

#include "arm_common/arm_types.h"

typedef enum {
  ARM_STATE_Q_VALID       = 1u << 0,
  ARM_STATE_DQ_VALID      = 1u << 1,
  ARM_STATE_TAU_EST_VALID = 1u << 2,
} arm_state_flags_t;

typedef enum {
  ARM_REFERENCE_Q_VALID      = 1u << 0,
  ARM_REFERENCE_DQ_VALID     = 1u << 1,
  ARM_REFERENCE_DDQ_VALID    = 1u << 2,
  ARM_REFERENCE_TAU_FF_VALID = 1u << 3,
} arm_reference_flags_t;

typedef enum {
  ARM_COMMAND_Q_D_VALID      = 1u << 0,
  ARM_COMMAND_DQ_D_VALID     = 1u << 1,
  ARM_COMMAND_KP_VALID       = 1u << 2,
  ARM_COMMAND_KD_VALID       = 1u << 3,
  ARM_COMMAND_TAU_FF_VALID   = 1u << 4,
  ARM_COMMAND_TAU_FB_VALID   = 1u << 5,
  ARM_COMMAND_TAU_MODEL_VALID = 1u << 6,
} arm_command_flags_t;

typedef struct {
  const char *joint_name;
  const char *actuator_name;
  /* Positive q/dq/tau follow the joint's right-hand-rule axis. Platform adapters
     use sign and q_offset_rad to map raw encoder/actuator conventions into this
     core convention. */
  arm_real_t sign;
  arm_real_t q_offset_rad;
  arm_real_t torque_limit_nm;
} arm_joint_config_t;

typedef struct {
  uint8_t dof;
  arm_joint_config_t joints[ARM_DOF_MAX];
} arm_config_t;

typedef struct {
  uint8_t dof;
  arm_real_t time_s;
  arm_real_t dt_s;
  /* Joint-space state in the normalized core convention:
     q > 0 and dq > 0 follow the right-hand-rule joint axis. */
  arm_real_t q_rad[ARM_DOF_MAX];
  arm_real_t dq_rad_s[ARM_DOF_MAX];
  /* Estimated/applied joint torque. tau_est > 0 pushes the joint in the same
     positive direction as q/dq. */
  arm_real_t tau_est_nm[ARM_DOF_MAX];
  uint32_t flags;
} arm_state_t;

typedef struct {
  uint8_t dof;
  arm_real_t q_ref_rad[ARM_DOF_MAX];
  arm_real_t dq_ref_rad_s[ARM_DOF_MAX];
  arm_real_t ddq_ref_rad_s2[ARM_DOF_MAX];
  arm_real_t tau_ff_nm[ARM_DOF_MAX];
  uint32_t flags;
} arm_reference_t;

typedef struct {
  uint8_t dof;
  arm_real_t q_d_rad[ARM_DOF_MAX];
  arm_real_t dq_d_rad_s[ARM_DOF_MAX];
  arm_real_t kp[ARM_DOF_MAX];
  arm_real_t kd[ARM_DOF_MAX];
  arm_real_t tau_fb_nm[ARM_DOF_MAX];
  arm_real_t tau_model_ff_nm[ARM_DOF_MAX];
  /* Feed-forward or direct torque target. tau_ff > 0 pushes the joint in the
     right-hand-rule positive direction. */
  arm_real_t tau_ff_nm[ARM_DOF_MAX];
  uint32_t flags;
} arm_command_t;

#endif
