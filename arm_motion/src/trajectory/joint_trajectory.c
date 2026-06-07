#include "arm_motion/joint_trajectory.h"

#include "arm_common/arm_math.h"

static uint8_t prev_index(uint8_t index, uint8_t count, bool loop) {
  if (index > 0u) return (uint8_t)(index - 1u);
  return loop ? (uint8_t)(count - 1u) : 0u;
}

static uint8_t next_index(uint8_t index, uint8_t count, bool loop) {
  const uint8_t next = (uint8_t)(index + 1u);
  if (next < count) return next;
  return loop ? 0u : (uint8_t)(count - 1u);
}

static arm_real_t waypoint_velocity(
    const joint_trajectory_t *trajectory,
    uint8_t index,
    uint8_t joint) {
  if (!trajectory->loop && (index == 0u || index + 1u >= trajectory->waypoint_count)) {
    return ARM_REAL_ZERO;
  }
  const uint8_t prev = prev_index(index, trajectory->waypoint_count, trajectory->loop);
  const uint8_t next = next_index(index, trajectory->waypoint_count, trajectory->loop);
  const arm_real_t dt = ARM_REAL(2.0) * trajectory->segment_duration_s;
  if (dt <= ARM_REAL_ZERO) return ARM_REAL_ZERO;
  return (trajectory->q_rad[next][joint] - trajectory->q_rad[prev][joint]) / dt;
}

int joint_trajectory_init(
    joint_trajectory_t *trajectory,
    uint8_t dof,
    uint8_t waypoint_count,
    arm_real_t segment_duration_s,
    bool loop) {
  if (!trajectory) return ARM_ERR_NULL;
  if (!arm_dof_is_valid(dof)) return ARM_ERR_DOF;
  if (waypoint_count < 2u || waypoint_count > JOINT_TRAJECTORY_WAYPOINT_MAX) return ARM_ERR_CONFIG;
  if (segment_duration_s <= ARM_REAL_ZERO) return ARM_ERR_CONFIG;

  *trajectory = (joint_trajectory_t){0};
  trajectory->dof = dof;
  trajectory->waypoint_count = waypoint_count;
  trajectory->segment_duration_s = segment_duration_s;
  trajectory->loop = loop;
  return ARM_OK;
}

int joint_trajectory_set_waypoint(
    joint_trajectory_t *trajectory,
    uint8_t waypoint,
    const arm_real_t q_rad[ARM_DOF_MAX]) {
  if (!trajectory || !q_rad) return ARM_ERR_NULL;
  if (waypoint >= trajectory->waypoint_count) return ARM_ERR_CONFIG;
  for (uint8_t joint = 0u; joint < trajectory->dof; ++joint) {
    trajectory->q_rad[waypoint][joint] = q_rad[joint];
  }
  return ARM_OK;
}

int joint_trajectory_sample(
    const joint_trajectory_t *trajectory,
    arm_real_t time_s,
    arm_reference_t *ref) {
  if (!trajectory || !ref) return ARM_ERR_NULL;
  if (!arm_dof_is_valid(trajectory->dof)) return ARM_ERR_DOF;
  if (trajectory->waypoint_count < 2u || trajectory->segment_duration_s <= ARM_REAL_ZERO) return ARM_ERR_CONFIG;

  if (time_s < ARM_REAL_ZERO) time_s = ARM_REAL_ZERO;
  const uint8_t segment_count = trajectory->loop ? trajectory->waypoint_count : (uint8_t)(trajectory->waypoint_count - 1u);
  const arm_real_t total_s = trajectory->segment_duration_s * ARM_REAL(segment_count);
  if (trajectory->loop) {
    while (time_s >= total_s) {
      time_s -= total_s;
    }
  } else if (time_s >= total_s) {
    arm_reference_zero(ref, trajectory->dof);
    ref->flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;
    const uint8_t last = (uint8_t)(trajectory->waypoint_count - 1u);
    for (uint8_t joint = 0u; joint < trajectory->dof; ++joint) {
      ref->q_ref_rad[joint] = trajectory->q_rad[last][joint];
    }
    return ARM_OK;
  }

  uint8_t segment = (uint8_t)(time_s / trajectory->segment_duration_s);
  if (segment >= segment_count) segment = (uint8_t)(segment_count - 1u);
  const uint8_t next = trajectory->loop ? (uint8_t)((segment + 1u) % trajectory->waypoint_count) : (uint8_t)(segment + 1u);
  const arm_real_t local_t = time_s - trajectory->segment_duration_s * ARM_REAL(segment);
  const arm_real_t p = arm_clamp(local_t / trajectory->segment_duration_s, ARM_REAL_ZERO, ARM_REAL_ONE);
  const arm_real_t p2 = p * p;
  const arm_real_t p3 = p2 * p;
  const arm_real_t h00 = ARM_REAL(2.0) * p3 - ARM_REAL(3.0) * p2 + ARM_REAL_ONE;
  const arm_real_t h10 = p3 - ARM_REAL(2.0) * p2 + p;
  const arm_real_t h01 = ARM_REAL(-2.0) * p3 + ARM_REAL(3.0) * p2;
  const arm_real_t h11 = p3 - p2;
  const arm_real_t dh00 = (ARM_REAL(6.0) * p2 - ARM_REAL(6.0) * p) / trajectory->segment_duration_s;
  const arm_real_t dh10 = ARM_REAL(3.0) * p2 - ARM_REAL(4.0) * p + ARM_REAL_ONE;
  const arm_real_t dh01 = (ARM_REAL(-6.0) * p2 + ARM_REAL(6.0) * p) / trajectory->segment_duration_s;
  const arm_real_t dh11 = ARM_REAL(3.0) * p2 - ARM_REAL(2.0) * p;
  const arm_real_t ddh00 =
      (ARM_REAL(12.0) * p - ARM_REAL(6.0)) / (trajectory->segment_duration_s * trajectory->segment_duration_s);
  const arm_real_t ddh10 = (ARM_REAL(6.0) * p - ARM_REAL(4.0)) / trajectory->segment_duration_s;
  const arm_real_t ddh01 =
      (ARM_REAL(-12.0) * p + ARM_REAL(6.0)) / (trajectory->segment_duration_s * trajectory->segment_duration_s);
  const arm_real_t ddh11 = (ARM_REAL(6.0) * p - ARM_REAL(2.0)) / trajectory->segment_duration_s;

  arm_reference_zero(ref, trajectory->dof);
  ref->flags = ARM_REFERENCE_Q_VALID | ARM_REFERENCE_DQ_VALID | ARM_REFERENCE_DDQ_VALID;
  for (uint8_t joint = 0u; joint < trajectory->dof; ++joint) {
    const arm_real_t q0 = trajectory->q_rad[segment][joint];
    const arm_real_t q1 = trajectory->q_rad[next][joint];
    const arm_real_t v0 = waypoint_velocity(trajectory, segment, joint);
    const arm_real_t v1 = waypoint_velocity(trajectory, next, joint);
    ref->q_ref_rad[joint] =
        h00 * q0 + h10 * trajectory->segment_duration_s * v0 + h01 * q1 + h11 * trajectory->segment_duration_s * v1;
    ref->dq_ref_rad_s[joint] = dh00 * q0 + dh10 * v0 + dh01 * q1 + dh11 * v1;
    ref->ddq_ref_rad_s2[joint] = ddh00 * q0 + ddh10 * v0 + ddh01 * q1 + ddh11 * v1;
  }
  return ARM_OK;
}
