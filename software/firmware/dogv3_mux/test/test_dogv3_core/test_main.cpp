#include <unity.h>

#include "dogv3_core.h"
#include "dogv3_config_blob.h"
#include "dogv3_state.h"

using namespace dogv3;

namespace {

GaitSeed defaultSeed() {
  GaitSeed seed;
  seed.body_height = 160.0f;
  seed.step_length = 40.0f;
  seed.step_height = 30.0f;
  seed.cycle_period = 0.6f;
  seed.duty_factor = 0.6f;
  seed.stance_width_offset = 10.0f;
  seed.turn_gain = 0.5f;
  seed.swing_shape = SWING_PARABOLA;
  return seed;
}

void assertClose(float expected, float actual, float tolerance = 0.001f) {
  TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_angle_count_matches_python_oracle() {
  TEST_ASSERT_EQUAL_INT(2048, NEUTRAL_COUNT);
  assertClose(651.898647f, COUNTS_PER_RAD, 0.0005f);
  TEST_ASSERT_EQUAL_INT(2048, angleToCount(0.0f));
  TEST_ASSERT_EQUAL_INT(2374, angleToCount(0.5f));
  TEST_ASSERT_EQUAL_INT(1722, angleToCount(0.5f, true));
  TEST_ASSERT_EQUAL_INT(1396, angleToCount(-1.0f));
}

void test_ik_fk_right_mid_matches_python_oracle() {
  LegGeometry geom{50.0f, 110.0f, 130.0f, 1, 1};
  Vec3 foot{40.0f, 30.0f, -200.0f};
  JointAngles angles = inverseKinematics(geom, foot);

  assertClose(0.050274f, angles.hip);
  assertClose(-0.492933f, angles.femur);
  assertClose(1.176005f, angles.tibia);

  Vec3 fk = forwardKinematics(geom, angles);
  assertClose(40.0f, fk.x);
  assertClose(30.0f, fk.y);
  assertClose(-200.0f, fk.z);
}

void test_ik_fk_left_mid_matches_python_oracle() {
  LegGeometry geom{50.0f, 110.0f, 130.0f, -1, 1};
  Vec3 foot{-30.0f, -40.0f, -180.0f};
  JointAngles angles = inverseKinematics(geom, foot);

  assertClose(-0.112399f, angles.hip);
  assertClose(-0.575540f, angles.femur);
  assertClose(1.451633f, angles.tibia);

  Vec3 fk = forwardKinematics(geom, angles);
  assertClose(-30.0f, fk.x);
  assertClose(-40.0f, fk.y);
  assertClose(-180.0f, fk.z);
}

void test_trot_phase_zero_matches_python_oracle() {
  FootTarget targets[LEG_COUNT];
  GaitCommand cmd{0.0f, 1.0f, 0.0f, 0.0f};
  trotTargets(defaultSeed(), 0.0f, cmd, 95.0f, targets);

  assertClose(-10.0f, targets[LEG_FL].x);
  assertClose(20.0f, targets[LEG_FL].y);
  assertClose(-160.0f, targets[LEG_FL].z);
  TEST_ASSERT_TRUE(targets[LEG_FL].in_stance);

  assertClose(10.0f, targets[LEG_FR].x);
  assertClose(-13.333333f, targets[LEG_FR].y);
  assertClose(-160.0f, targets[LEG_FR].z);
  TEST_ASSERT_TRUE(targets[LEG_FR].in_stance);
}

void test_trot_phase_seventy_five_matches_python_oracle() {
  FootTarget targets[LEG_COUNT];
  GaitCommand cmd{0.0f, 1.0f, 0.0f, 0.0f};
  trotTargets(defaultSeed(), 0.75f, cmd, 95.0f, targets);

  assertClose(-10.0f, targets[LEG_FL].x);
  assertClose(-5.0f, targets[LEG_FL].y);
  assertClose(-131.875f, targets[LEG_FL].z);
  TEST_ASSERT_FALSE(targets[LEG_FL].in_stance);

  assertClose(10.0f, targets[LEG_FR].x);
  assertClose(3.333333f, targets[LEG_FR].y);
  assertClose(-160.0f, targets[LEG_FR].z);
  TEST_ASSERT_TRUE(targets[LEG_FR].in_stance);
}

void test_mode_b_requires_disarmed_valid_config() {
  BrainState state = initialBrainState(1000);
  TEST_ASSERT_EQUAL_UINT8(MODE_A_MUX, state.mode);
  TEST_ASSERT_FALSE(enterModeB(state));

  state.config_valid = true;
  state.armed = true;
  TEST_ASSERT_FALSE(enterModeB(state));

  state.armed = false;
  TEST_ASSERT_TRUE(enterModeB(state));
  TEST_ASSERT_EQUAL_UINT8(MODE_B_BRAIN, state.mode);
  TEST_ASSERT_FALSE(state.armed);
}

void test_intent_safety_holds_then_disarms() {
  SafetyConfig config = defaultSafetyConfig();
  BrainState state = initialBrainState(0);
  state.config_valid = true;
  TEST_ASSERT_TRUE(enterModeB(state));
  TEST_ASSERT_TRUE(arm(state, 100));

  TEST_ASSERT_EQUAL_UINT8(INTENT_FRESH, updateSafety(state, config, 399));
  TEST_ASSERT_TRUE(state.armed);
  state.soft_start = 1.0f;
  TEST_ASSERT_EQUAL_UINT8(INTENT_HOLD_STAND, updateSafety(state, config, 401));
  TEST_ASSERT_TRUE(state.armed);
  assertClose(0.0f, state.soft_start);

  TEST_ASSERT_EQUAL_UINT8(INTENT_DISARM_REQUIRED, updateSafety(state, config, 10101));
  TEST_ASSERT_FALSE(state.armed);
}

void test_estop_latches_and_only_clears_disarmed() {
  BrainState state = initialBrainState(0);
  state.config_valid = true;
  TEST_ASSERT_TRUE(enterModeB(state));
  TEST_ASSERT_TRUE(arm(state, 10));
  latchEstop(state);
  TEST_ASSERT_TRUE(state.estop_latched);
  TEST_ASSERT_FALSE(state.armed);
  TEST_ASSERT_TRUE(clearEstop(state));
  TEST_ASSERT_FALSE(state.estop_latched);
}

void test_config_blob_crc32_matches_python_oracle() {
  const uint8_t payload[] = {'a', 'b', 'c'};
  TEST_ASSERT_EQUAL_UINT32(0x352441C2u, configBlobCrc32(payload, sizeof(payload)));
}

void test_config_blob_validator_accepts_valid_header() {
  const uint8_t blob[] = {
      'S', 'D', 'B', 'C', 'F', 'G', '1', '\0',
      0x01, 0x00,  // blob_version
      0x01, 0x00,  // schema_version
      0x03, 0x00, 0x00, 0x00,  // payload_len
      0xC2, 0x41, 0x24, 0x35,  // crc32("abc")
      'a', 'b', 'c',
  };
  ConfigBlobHeader header;
  TEST_ASSERT_EQUAL_UINT8(CONFIG_BLOB_OK, validateConfigBlob(blob, sizeof(blob), 1, &header));
  TEST_ASSERT_EQUAL_UINT16(1, header.blob_version);
  TEST_ASSERT_EQUAL_UINT16(1, header.schema_version);
  TEST_ASSERT_EQUAL_UINT32(3, header.payload_len);
  TEST_ASSERT_EQUAL_UINT32(0x352441C2u, header.crc32);
}

void test_config_blob_validator_rejects_schema_and_crc() {
  uint8_t blob[] = {
      'S', 'D', 'B', 'C', 'F', 'G', '1', '\0',
      0x01, 0x00,  // blob_version
      0x01, 0x00,  // schema_version
      0x03, 0x00, 0x00, 0x00,  // payload_len
      0xC2, 0x41, 0x24, 0x35,  // crc32("abc")
      'a', 'b', 'c',
  };
  TEST_ASSERT_EQUAL_UINT8(CONFIG_BLOB_BAD_SCHEMA, validateConfigBlob(blob, sizeof(blob), 2, nullptr));
  blob[CONFIG_BLOB_HEADER_LEN + 2] = 'd';
  TEST_ASSERT_EQUAL_UINT8(CONFIG_BLOB_BAD_CRC, validateConfigBlob(blob, sizeof(blob), 1, nullptr));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_angle_count_matches_python_oracle);
  RUN_TEST(test_ik_fk_right_mid_matches_python_oracle);
  RUN_TEST(test_ik_fk_left_mid_matches_python_oracle);
  RUN_TEST(test_trot_phase_zero_matches_python_oracle);
  RUN_TEST(test_trot_phase_seventy_five_matches_python_oracle);
  RUN_TEST(test_mode_b_requires_disarmed_valid_config);
  RUN_TEST(test_intent_safety_holds_then_disarms);
  RUN_TEST(test_estop_latches_and_only_clears_disarmed);
  RUN_TEST(test_config_blob_crc32_matches_python_oracle);
  RUN_TEST(test_config_blob_validator_accepts_valid_header);
  RUN_TEST(test_config_blob_validator_rejects_schema_and_crc);
  return UNITY_END();
}
