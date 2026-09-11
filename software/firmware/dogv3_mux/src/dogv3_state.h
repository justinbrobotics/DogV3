#ifndef DOGV3_STATE_H
#define DOGV3_STATE_H

#include <stdint.h>

namespace dogv3 {

static constexpr uint32_t INTENT_HOLD_STAND_MS = 300;
static constexpr uint32_t INTENT_DISARM_MS = 10000;
static constexpr uint32_t SOFT_START_MS = 1200;

enum FirmwareMode : uint8_t {
  MODE_A_MUX = 0,
  MODE_B_BRAIN = 1,
};

enum IntentState : uint8_t {
  INTENT_FRESH = 0,
  INTENT_HOLD_STAND = 1,
  INTENT_DISARM_REQUIRED = 2,
};

struct SafetyConfig {
  uint32_t intent_hold_stand_ms;
  uint32_t intent_disarm_ms;
  uint32_t soft_start_ms;
};

struct BrainState {
  FirmwareMode mode;
  bool armed;
  bool estop_latched;
  bool config_valid;
  uint32_t last_intent_ms;
  float soft_start;
};

SafetyConfig defaultSafetyConfig();
BrainState initialBrainState(uint32_t now_ms = 0);

bool canEnterModeB(const BrainState &state);
bool enterModeB(BrainState &state);
void enterModeA(BrainState &state);

bool arm(BrainState &state, uint32_t now_ms);
void disarm(BrainState &state);
void latchEstop(BrainState &state);
bool clearEstop(BrainState &state);
void noteIntent(BrainState &state, uint32_t now_ms);

IntentState classifyIntent(const BrainState &state, const SafetyConfig &config, uint32_t now_ms);
IntentState updateSafety(BrainState &state, const SafetyConfig &config, uint32_t now_ms);

}  // namespace dogv3

#endif  // DOGV3_STATE_H
