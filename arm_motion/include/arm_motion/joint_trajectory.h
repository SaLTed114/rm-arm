#ifndef ARM_MOTION_JOINT_TRAJECTORY_H_
#define ARM_MOTION_JOINT_TRAJECTORY_H_

#include <stdbool.h>

#include "arm_core/arm.h"

#define JOINT_TRAJECTORY_WAYPOINT_MAX 96u

typedef struct {
  uint8_t dof;
  uint8_t waypoint_count;
  bool loop;
  arm_real_t segment_duration_s;
  arm_real_t q_rad[JOINT_TRAJECTORY_WAYPOINT_MAX][ARM_DOF_MAX];
} joint_trajectory_t;

int joint_trajectory_init(
    joint_trajectory_t *trajectory,
    uint8_t dof,
    uint8_t waypoint_count,
    arm_real_t segment_duration_s,
    bool loop);
int joint_trajectory_set_waypoint(
    joint_trajectory_t *trajectory,
    uint8_t waypoint,
    const arm_real_t q_rad[ARM_DOF_MAX]);
int joint_trajectory_sample(
    const joint_trajectory_t *trajectory,
    arm_real_t time_s,
    arm_reference_t *ref);

#endif
