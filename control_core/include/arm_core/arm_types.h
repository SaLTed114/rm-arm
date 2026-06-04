#ifndef ARM_CORE_ARM_TYPES_H_
#define ARM_CORE_ARM_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#define ARM_DOF_MAX 8u
#define ARM_DEFAULT_DOF 6u

#ifdef ARM_CORE_USE_FLOAT
typedef float arm_real_t;
#else
typedef double arm_real_t;
#endif

typedef enum {
  ARM_STATE_Q_VALID       = 1u << 0,
  ARM_STATE_DQ_VALID      = 1u << 1,
  ARM_STATE_TAU_EST_VALID = 1u << 2,
} arm_state_flags_t;

typedef enum {
  ARM_COMMAND_TAU_VALID = 1u << 0,
} arm_command_flags_t;

typedef struct {
  const char *joint_name;
  const char *actuator_name;
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
  arm_real_t q_rad[ARM_DOF_MAX];
  arm_real_t dq_rad_s[ARM_DOF_MAX];
  arm_real_t tau_est_nm[ARM_DOF_MAX];
  uint32_t flags;
} arm_state_t;

typedef struct {
  uint8_t dof;
  arm_real_t tau_nm[ARM_DOF_MAX];
  uint32_t flags;
} arm_command_t;

typedef enum {
  ARM_OK = 0,
  ARM_ERR_NULL = -1,
  ARM_ERR_DOF = -2,
  ARM_ERR_CONFIG = -3,
} arm_status_t;

#endif
