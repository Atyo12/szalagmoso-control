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

  // Fan1 damper
  E601_FAN1_DAMPER_NOT_OPEN = 601,
  E602_FAN1_DAMPER_NOT_CLOSED = 602,

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

// -------------------- Runtime állítható gépparaméterek --------------------
struct MachineParams {
  uint16_t debounce_ms = 50;

  uint32_t free_min_ms = 500;
  uint32_t block_max_ms = 6000;

  uint32_t push_ext_max_ms = 9000;
  uint32_t push_cycle_max_ms = 22000;
  uint32_t tail_clear_max_ms = 5000;

  uint32_t ret_start_clear_max_ms = 10000;
  uint32_t ret_end_soft_ms = 5000;
  uint32_t ret_end_hard_ms = 25000;

  uint32_t danger_time_ms = 4500;
  uint32_t danger_margin_ms = 500;

  uint32_t table_wait_max_ms = 30000;
  uint32_t buzzer_max_ms = 30000;

  uint32_t ret_ghost_timeout_ms = 120000;
  uint32_t fan1_damper_move_timeout_ms = 20000;
  uint32_t trh_fresh_timeout_ms = 10000;

  int16_t negative_pressure_offset_x10 = 20; // 2.0 Hz
    uint16_t semafor_pulses = 5;

  // ------------------------------------------------------------
  // Légtechnika AUTO paraméterek (HMI-ről állítható)
  // ------------------------------------------------------------
  uint16_t climate_on_demand_pct_x10 = 80;   // 8.0 %
  uint16_t climate_off_demand_pct_x10 = 30;  // 3.0 %
  uint16_t climate_min_run_sec = 60;         // 60 s

  uint16_t fan2_auto_min_hz_x10 = 190;       // 19.0 Hz
  uint16_t fan2_auto_max_hz_x10 = 450;       // 45.0 Hz
  uint16_t fan1_auto_min_hz_x10 = 100;       // 10.0 Hz

  uint16_t hot_offset_temp_c_x10 = 300;      // 30.0 °C
  uint16_t hot_force_temp_c_x10 = 420;       // 42.0 °C
  uint16_t hot_force_fan2_min_hz_x10 = 400;  // 40.0 Hz
  uint16_t hot_force_fan1_min_hz_x10 = 360;  // 36.0 Hz
};