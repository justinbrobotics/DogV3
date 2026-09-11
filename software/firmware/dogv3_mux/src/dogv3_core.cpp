#include "dogv3_core.h"

#include <math.h>

namespace dogv3 {

namespace {

float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

int clampi(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

float wrapPhase(float phase) {
  float out = fmodf(phase, 1.0f);
  if (out < 0.0f) out += 1.0f;
  return out;
}

float trotOffset(LegIndex leg) {
  return (leg == LEG_FR || leg == LEG_BL) ? 0.5f : 0.0f;
}

float legSx(LegIndex leg) {
  return (leg == LEG_FL || leg == LEG_BL) ? -1.0f : 1.0f;
}

float legSy(LegIndex leg) {
  return (leg == LEG_FL || leg == LEG_FR) ? 1.0f : -1.0f;
}

float swingHeight(float w, SwingShape shape, float step_height) {
  if (shape == SWING_SINE) {
    return step_height * sinf(PI_F * w);
  }
  if (shape == SWING_CYCLOID) {
    return step_height * (1.0f - cosf(2.0f * PI_F * w)) * 0.5f;
  }
  return step_height * (4.0f * w * (1.0f - w));
}

int roundToInt(float value) {
  return value >= 0.0f ? static_cast<int>(value + 0.5f) : static_cast<int>(value - 0.5f);
}

}  // namespace

PhaseClock::PhaseClock(float period) : cycle_period(period), phase(0.0f) {}

void PhaseClock::reset(float new_phase) {
  phase = wrapPhase(new_phase);
}

float PhaseClock::advance(float dt) {
  if (cycle_period <= 0.0f) return phase;
  phase = wrapPhase(phase + dt / cycle_period);
  return phase;
}

float PhaseClock::legPhase(LegIndex leg) const {
  return wrapPhase(phase + trotOffset(leg));
}

Vec3 forwardKinematics(const LegGeometry &geom, const JointAngles &angles) {
  const float t1 = angles.hip;
  const float t2 = angles.femur;
  const float t3 = angles.tibia;
  // Lateral mirror: the hip L1 link points outward (+X right, -X left), matching
  // the Python reference and the MuJoCo twin (sx*L1).
  const float lx = static_cast<float>(geom.side_sign);
  const float py = geom.L2 * sinf(t2) + geom.L3 * sinf(t2 + t3);
  const float pz = -(geom.L2 * cosf(t2) + geom.L3 * cosf(t2 + t3));

  Vec3 out;
  out.x = lx * geom.L1 * cosf(t1) + pz * sinf(t1);
  out.y = lx * py;
  out.z = -lx * geom.L1 * sinf(t1) + pz * cosf(t1);
  return out;
}

JointAngles inverseKinematics(const LegGeometry &geom, const Vec3 &foot) {
  const float lx = static_cast<float>(geom.side_sign);  // lateral mirror (see forwardKinematics)
  const float y_local = lx * foot.y;

  const float d_sq = foot.x * foot.x + foot.z * foot.z - geom.L1 * geom.L1;
  const float d = sqrtf(d_sq > 0.0f ? d_sq : 0.0f);
  const float t1 = atan2f(-d, lx * geom.L1) - atan2f(foot.z, foot.x);
  const float r = sqrtf(y_local * y_local + d * d);
  float c3 = (r * r - geom.L2 * geom.L2 - geom.L3 * geom.L3) / (2.0f * geom.L2 * geom.L3);
  c3 = clampf(c3, -1.0f, 1.0f);
  const float t3 = static_cast<float>(geom.knee_sign) * acosf(c3);
  const float t2 = atan2f(y_local, d) - atan2f(geom.L3 * sinf(t3), geom.L2 + geom.L3 * cosf(t3));

  JointAngles out;
  out.hip = t1;
  out.femur = t2;
  out.tibia = t3;
  return out;
}

int angleToCount(float theta, bool invert, float offset, int min_raw, int max_raw) {
  const float direction = invert ? -1.0f : 1.0f;
  const int count = roundToInt(static_cast<float>(NEUTRAL_COUNT) + direction * (theta - offset) * COUNTS_PER_RAD);
  return clampi(count, min_raw, max_raw);
}

float countToAngle(int count, bool invert, float offset) {
  const float direction = invert ? -1.0f : 1.0f;
  return offset + direction * (static_cast<float>(count - NEUTRAL_COUNT) / COUNTS_PER_RAD);
}

void standTargets(
    const GaitSeed &seed,
    float phase,
    const GaitCommand &cmd,
    FootTarget out_targets[LEG_COUNT]) {
  for (uint8_t i = 0; i < LEG_COUNT; ++i) {
    const LegIndex leg = static_cast<LegIndex>(i);
    out_targets[i].x = legSx(leg) * seed.stance_width_offset;
    out_targets[i].y = 0.0f;
    out_targets[i].z = -(seed.body_height + cmd.body_z);
    out_targets[i].phase = wrapPhase(phase + trotOffset(leg));
    out_targets[i].in_stance = true;
  }
}

void trotTargets(
    const GaitSeed &seed,
    float phase,
    const GaitCommand &cmd,
    float yaw_radius_mm,
    FootTarget out_targets[LEG_COUNT]) {
  const float duty = seed.duty_factor;
  const float stride_fwd = seed.step_length * clampf(cmd.vy, -1.0f, 1.0f);
  const float stride_lat = seed.step_length * clampf(cmd.vx, -1.0f, 1.0f);
  const float yaw = seed.turn_gain * clampf(cmd.wz, -1.0f, 1.0f);

  for (uint8_t i = 0; i < LEG_COUNT; ++i) {
    const LegIndex leg = static_cast<LegIndex>(i);
    const float phi = wrapPhase(phase + trotOffset(leg));
    const float sx = legSx(leg);
    const float sy = legSy(leg);
    const float yaw_fwd = -yaw * yaw_radius_mm * sx;
    const float yaw_lat = yaw * yaw_radius_mm * sy;
    const float total_fwd = stride_fwd + yaw_fwd;
    const float total_lat = stride_lat + yaw_lat;

    const float nx = sx * seed.stance_width_offset;
    const float z_down = -(seed.body_height + cmd.body_z);
    float frac = 0.0f;

    out_targets[i].phase = phi;
    if (phi < duty) {
      const float u = duty > 0.0f ? phi / duty : 0.0f;
      frac = 0.5f - u;
      out_targets[i].z = z_down;
      out_targets[i].in_stance = true;
    } else {
      const float w = duty < 1.0f ? (phi - duty) / (1.0f - duty) : 0.0f;
      frac = -0.5f + w;
      out_targets[i].z = z_down + swingHeight(w, seed.swing_shape, seed.step_height);
      out_targets[i].in_stance = false;
    }

    out_targets[i].x = nx + total_lat * frac;
    out_targets[i].y = total_fwd * frac;
  }
}

}  // namespace dogv3
