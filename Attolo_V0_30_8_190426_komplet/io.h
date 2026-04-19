#pragma once
#include <Arduino.h>
#include <Controllino.h>

#define SIM_MODE 0
#define CLI_MODE 0
#define CLI_VFD_DIRECT_TEST 0
#define DEBUG_VFD_MACHINE 0

// ------------------------------------------------------------
// HARDVER PROFIL VÁLASZTÁS
// ------------------------------------------------------------
//#define HW_PROFILE_BENCH 1
#define HW_PROFILE_CABINET 1

#if defined(HW_PROFILE_BENCH) && defined(HW_PROFILE_CABINET)
  #error "Csak egy hardverprofil lehet aktiv!"
#endif

#if !defined(HW_PROFILE_BENCH) && !defined(HW_PROFILE_CABINET)
  #error "Valassz hardverprofilt!"
#endif

#if defined(HW_PROFILE_BENCH)
  #include "io_bench.h"
#elif defined(HW_PROFILE_CABINET)
  #include "io_cabinet.h"
#endif

// ------------------------------------------------------------
// Időzítési paraméterek
// ------------------------------------------------------------
constexpr uint16_t DEBOUNCE_MS = 50;

constexpr uint32_t T_FREE_MIN_MS       = 500;
constexpr uint32_t T_BLOCK_MAX_MS      = 6000;

#if SIM_MODE
constexpr uint32_t T_EXT_MAX_MS        = 15000;
constexpr uint32_t T_CYCLE_MAX_MS      = 35000;
constexpr uint32_t T_TAIL_CLEAR_MAX_MS = 10000;
#else
constexpr uint32_t T_EXT_MAX_MS        = 9000;
constexpr uint32_t T_CYCLE_MAX_MS      = 22000;
constexpr uint32_t T_TAIL_CLEAR_MAX_MS = 5000;
#endif

constexpr uint32_t T_RET_START_CLEAR_MAX_MS = 10000;
constexpr uint32_t T_RET_END_SOFT_MS        = 5000;
constexpr uint32_t T_RET_END_HARD_MS        = 25000;
constexpr uint32_t T_DANGER_TIME_MS         = 4500;
constexpr uint32_t T_DANGER_MARGIN_MS       = 500;
constexpr uint32_t T_TABLE_WAIT_MAX_MS      = 30000;

// ------------------------------------------------------------
// SIM változók
// ------------------------------------------------------------
extern bool g_sim_di[32];
extern bool g_sim_do[32];

// ------------------------------------------------------------
// Biztonságos I/O
// ------------------------------------------------------------
inline bool safeRead(int pin) {
#if SIM_MODE
  if (pin < 0) return false;
  return g_sim_di[pin];
#else
  if (pin < 0) return false;
  return digitalRead(pin) == HIGH;
#endif
}

inline void safeWrite(int pin, bool value) {
#if SIM_MODE
  if (pin < 0) return;
  g_sim_do[pin] = value;
#else
  if (pin < 0) return;
  digitalWrite(pin, value ? HIGH : LOW);
#endif
}