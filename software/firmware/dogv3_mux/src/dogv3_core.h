#ifndef DOGV3_CORE_H
#define DOGV3_CORE_H

#include <stdint.h>

namespace dogv3 {

static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float COUNTS_PER_RAD = 4096.0f / (2.0f * PI_F);
static constexpr int NEUTRAL_COUNT = 2048;

enum LegIndex : uint8_t {
  LEG_FL = 0,
  LEG_FR = 1,
  LEG_BL = 2,
  LEG_BR = 3,
  LEG_COUNT = 4,
};

enum SwingShape : uint8_t {
  SWING_PARABOLA = 0,
  SWING_SINE = 1,
  SWING_CYCLOID = 2,
};

struct Vec3 {
  float x;
  float y;
  float z;
};

struct JointAngles {
  float hip;
  float femur;
  float tibia;
};

struct LegGeometry {
  float L1;
  float L2;
  float L3;
  int8_t side_sign;
  int8_t knee_sign;
};

struct GaitSeed {
  float body_height;
  float step_length;
  float step_height;
  float cycle_period;
  float duty_factor;
  float stance_width_offset;
  float turn_gain;
  SwingShape swing_shape;
};

struct GaitCommand {
  float vx;
  float vy;
  float wz;
  float body_z;
};

struct FootTarget {
  float x;
  float y;
  float z;
  float phase;
  bool in_stance;
};

struct PhaseClock {
  float cycle_period;
  float phase;

  explicit PhaseClock(float period = 0.6f);
  void reset(float new_phase = 0.0f);
  float advance(float dt);
  float legPhase(LegIndex leg) const;
};

Vec3 forwardKinematics(const LegGeometry &geom, const JointAngles &angles);
JointAngles inverseKinematics(const LegGeometry &geom, const Vec3 &foot);

int angleToCount(
    float theta,
    bool invert = false,
    float offset = 0.0f,
    int min_raw = 0,
    int max_raw = 4095);

float countToAngle(int count, bool invert = false, float offset = 0.0f);

void standTargets(
    const GaitSeed &seed,
    float phase,
    const GaitCommand &cmd,
    FootTarget out_targets[LEG_COUNT]);

void trotTargets(
    const GaitSeed &seed,
    float phase,
    const GaitCommand &cmd,
    float yaw_radius_mm,
    FootTarget out_targets[LEG_COUNT]);

}  // namespace dogv3

#endif  // DOGV3_CORE_H
