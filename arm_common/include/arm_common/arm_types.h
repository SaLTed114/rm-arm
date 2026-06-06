#ifndef ARM_COMMON_ARM_TYPES_H_
#define ARM_COMMON_ARM_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#define ARM_DOF_MAX 8u
#define ARM_DEFAULT_DOF 6u

#if defined(ARM_COMMON_USE_FLOAT) || defined(ARM_CORE_USE_FLOAT)
typedef float arm_real_t;
#define ARM_REAL(value) ((arm_real_t)(value))
#define ARM_REAL_ZERO 0.0f
#define ARM_REAL_ONE 1.0f
#define ARM_REAL_PI 3.14159265358979323846f
#else
typedef double arm_real_t;
#define ARM_REAL(value) ((arm_real_t)(value))
#define ARM_REAL_ZERO 0.0
#define ARM_REAL_ONE 1.0
#define ARM_REAL_PI 3.14159265358979323846
#endif

typedef enum {
  ARM_OK = 0,
  ARM_ERR_NULL = -1,
  ARM_ERR_DOF = -2,
  ARM_ERR_CONFIG = -3,
} arm_status_t;

#endif
