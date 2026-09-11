#include "dogv3_state.h"

namespace dogv3 {

namespace {

uint32_t elapsedMs(uint32_t now_ms, uint32_t then_ms) {
  return now_ms - then_ms;
}

}  // namespace

SafetyConfig defaultSafetyConfig() {
  SafetyConfig config;
  config.intent_hold_stand_ms = INTENT_HOLD_STAND_MS;
  config.intent_disarm_ms = INTENT_DISARM_MS;
  config.soft_start_ms = SOFT_START_MS;
  return config;
}

BrainState initialBrainState(uint32_t now_ms) {
  BrainState state;
  state.mode = MODE_A_MUX;
  state.armed = false;
  state.estop_latched = false;
  state.config_valid = false;
  state.last_intent_ms = now_ms;
  state.soft_start = 0.0f;
  return state;
}

bool canEnterModeB(const BrainState &state) {
  return state.mode == MODE_A_MUX && !state.armed && !state.estop_latched && state.config_valid;
}

bool enterModeB(BrainState &state) {
  if (!canEnterModeB(state)) {
    return false;
  }
  state.mode = MODE_B_BRAIN;
  state.armed = false;
  state.soft_start = 0.0f;
  return true;
}

void enterModeA(BrainState &state) {
  state.mode = MODE_A_MUX;
  state.armed = false;
  state.soft_start = 0.0f;
}

bool arm(BrainState &state, uint32_t now_ms) {
  if (state.mode != MODE_B_BRAIN || state.estop_latched || !state.config_valid) {
    state.armed = false;
    return false;
  }
  state.armed = true;
  state.last_intent_ms = now_ms;
  state.soft_start = 0.0f;
  return true;
}

void disarm(BrainState &state) {
  state.armed = false;
  state.soft_start = 0.0f;
}

void latchEstop(BrainState &state) {
  state.estop_latched = true;
  disarm(state);
}

bool clearEstop(BrainState &state) {
  if (state.armed) {
    return false;
  }
  state.estop_latched = false;
  return true;
}

void noteIntent(BrainState &state, uint32_t now_ms) {
  state.last_intent_ms = now_ms;
}

IntentState classifyIntent(const BrainState &state, const SafetyConfig &config, uint32_t now_ms) {
  const uint32_t stale_ms = elapsedMs(now_ms, state.last_intent_ms);
  if (stale_ms > config.intent_disarm_ms) {
    return INTENT_DISARM_REQUIRED;
  }
  if (stale_ms > config.intent_hold_stand_ms) {
    return INTENT_HOLD_STAND;
  }
  return INTENT_FRESH;
}

IntentState updateSafety(BrainState &state, const SafetyConfig &config, uint32_t now_ms) {
  if (state.estop_latched) {
    disarm(state);
    return INTENT_DISARM_REQUIRED;
  }
  const IntentState intent = classifyIntent(state, config, now_ms);
  if (intent == INTENT_DISARM_REQUIRED) {
    disarm(state);
  } else if (intent == INTENT_HOLD_STAND && state.armed) {
    state.soft_start = 0.0f;
  }
  return intent;
}

}  // namespace dogv3
