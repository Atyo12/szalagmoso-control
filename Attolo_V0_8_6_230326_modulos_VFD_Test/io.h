#pragma once
#include <Arduino.h>
#include <Controllino.h>

/*
  SIM_MODE = 1:
    - nincs fizikai bekötés
    - DI-ket Serial parancsokkal állítod
    - DO-kat a kód "virtuális kimenetre" írja, és kiírjuk

  CLI_MODE = 1:
    - soros portról kézzel vezérelheted a VFD tesztet
    - valódi VFD próbánál ezt kapcsold be
*/
#define SIM_MODE 0
#define CLI_MODE 0
#define CLI_VFD_DIRECT_TEST 0
#define DEBUG_VFD_MACHINE 0

// ------------------------------------------------------------
// LOGIKAI I/O indexek
// ------------------------------------------------------------

// Inputs
constexpr int PIN_DI_WASH_IN_OPTO    = CONTROLLINO_AI0;
constexpr int PIN_DI_WASH_OUT_OPTO   = 1;
constexpr int PIN_DI_TABLE_SW        = 2;
constexpr int PIN_DI_PUSH_HOME       = CONTROLLINO_AI3;
constexpr int PIN_DI_PUSH_EXT        = 4;
constexpr int PIN_DI_RET_START_OCC   = 5;
constexpr int PIN_DI_RET_END_OCC     = 6;
constexpr int PIN_DI_RET_TAIL_OCC    = 7; // 1=foglalt
constexpr int PIN_DI_RESET           = CONTROLLINO_AI8;

// ÚJ LOCAL_JOG bemenetek
constexpr int PIN_DI_JOG_DIR_FWD     = 9;   // kapcsoló egyik irány
constexpr int PIN_DI_JOG_DIR_REV     = 10;  // kapcsoló másik irány
constexpr int PIN_DI_JOG_BTN         = 11;  // nyomógomb / deadman

// Outputs
constexpr int PIN_DO_STOP_REQ        = 0;
constexpr int PIN_DO_RET_CONV        = 1;
constexpr int PIN_DO_PUSH_MOTOR      = 2;   // üzemi irány
constexpr int PIN_DO_BUZZER          = 3;
constexpr int PIN_DO_ALARM_LAMP      = 4;
constexpr int PIN_DO_PUSH_REV        = 5;   // ÚJ: szerviz visszairány

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

constexpr uint32_t T_DANGER_TIME_MS   = 4500;
constexpr uint32_t T_DANGER_MARGIN_MS = 500;

constexpr uint32_t T_TABLE_WAIT_MAX_MS = 30000;

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