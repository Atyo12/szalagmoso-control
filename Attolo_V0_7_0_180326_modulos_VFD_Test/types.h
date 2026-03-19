#pragma once
#include <Arduino.h>

// -------------------- State machine --------------------
enum class State : uint16_t {
  IDLE = 0,
  WAIT_RETURN_FREE = 1,
  PUSH_ACTIVE = 2,
  ERROR = 100
};

// -------------------- Fault codes --------------------
enum class Fault : uint16_t {
  NONE = 0,

  // OUT opto spacing / dirty
  E201_SPACING_COLLAPSE = 201,
  E202_OUT_OPTO_STUCK_OR_DIRTY = 202,
  E203_OUT_OPTO_NO_EDGES_WHILE_RUN = 203,

  // Push mechanism
  E301_PUSH_NO_EXT = 301,
  E302_PUSH_HOME_TIMEOUT = 302,
  E303_PUSH_SENSOR_CONFLICT = 303,
  E304_PUSH_HOME_LOST = 304,

  // ÚJ: tail-clear timeout (raklap vége nem hagyja el a veszélypontot)
  E305_PUSH_TAIL_CLEAR_TIMEOUT = 305,

  // Return conveyor
  E401_RETURN_START_STUCK = 401,
  E403_RETURN_END_BLOCKED_HARD = 403,

  // Safety / sync
  E501_DANGER_HOME_LATE = 501,
  E502_TABLE_WAIT_TIMEOUT = 502,

  // E-stop
  E900_ESTOP = 900
};

// -------------------- Operating mode --------------------
enum class OpMode : uint8_t {
  AUTO = 0,
  HANDLE = 1,
  LOCAL_JOG = 2,
};

// -------------------- Simple timer --------------------
struct Timer {
  uint32_t start_ms = 0;
  uint32_t dur_ms = 0;
  bool running = false;

  void start(uint32_t now, uint32_t duration_ms) {
    start_ms = now;
    dur_ms = duration_ms;
    running = true;
  }

  void stop() { running = false; }

  bool expired(uint32_t now) const {
    if (!running) return false;
    return (uint32_t)(now - start_ms) >= dur_ms;
  }

  uint32_t elapsed(uint32_t now) const {
    if (!running) return 0;
    return (uint32_t)(now - start_ms);
  }
};