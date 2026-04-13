#pragma once

/* ============================================================
   HMI MAP – MODBUS TCP
   ------------------------------------------------------------
   Tartományok:
   0 – 49   : HMI -> PLC COIL (parancsok)
   50 – 99  : tartalék
   100 – 199: PLC -> HMI STATUS BIT
   200 – 299: PLC -> HMI STATUS NUMERIC (HR)
   300 – 399: HMI -> PLC SETPOINT / PARAM (HR)
   ============================================================ */

/* ============================================================
   1) HMI → PLC PARANCSOK (COIL)
   ============================================================ */
constexpr uint16_t HMI_COIL_HANDLE_MODE        = 0;
constexpr uint16_t HMI_COIL_SERVICE_ENABLE     = 1;
constexpr uint16_t HMI_COIL_RESET_PULSE        = 2;

constexpr uint16_t HMI_COIL_RET_FORCE_EMPTY    = 10;
constexpr uint16_t HMI_COIL_RET_CONTINUOUS     = 11;

constexpr uint16_t HMI_COIL_VENT_MANUAL_MODE   = 12;   // 0=AUTO, 1=MANUAL
constexpr uint16_t HMI_COIL_FAN1_MAN_RUN       = 13;
constexpr uint16_t HMI_COIL_FAN2_MAN_RUN       = 14;

/* ============================================================
   2) PLC → HMI ÁLLAPOT BITEK
   ============================================================ */
constexpr uint16_t HMI_ST_FAULT_ACTIVE         = 100;
constexpr uint16_t HMI_ST_RESET_REQUEST        = 101;

constexpr uint16_t HMI_ST_HANDLE_MODE          = 110;
constexpr uint16_t HMI_ST_SERVICE_ENABLE       = 111;

constexpr uint16_t HMI_ST_WASH_STOP_REQUEST    = 120;

constexpr uint16_t HMI_ST_RET_RUNNING          = 130;
constexpr uint16_t HMI_ST_RET_FORCE_EMPTY      = 131;
constexpr uint16_t HMI_ST_RET_CONTINUOUS       = 132;

constexpr uint16_t HMI_ST_VENT_MANUAL_MODE     = 133;

constexpr uint16_t HMI_ST_PUSH_ACTIVE          = 140;

constexpr uint16_t HMI_ST_FAN1_RUNNING         = 150;
constexpr uint16_t HMI_ST_FAN1_DAMPER_OPEN_CMD = 151;
constexpr uint16_t HMI_ST_FAN1_DAMPER_OPEN_FB  = 152;
constexpr uint16_t HMI_ST_FAN1_RUN_PERMITTED   = 153;
constexpr uint16_t HMI_ST_FAN2_RUNNING         = 154;   // ÚJ

/* ============================================================
   3) PLC → HMI NUMERIKUS (HOLDING REGISTER)
   ============================================================ */
constexpr uint16_t HMI_HR_OP_MODE                  = 200;
constexpr uint16_t HMI_HR_STATE                    = 201;
constexpr uint16_t HMI_HR_FAULT                    = 202;

constexpr uint16_t HMI_HR_RET_FREQ_CMD_X10         = 210;
constexpr uint16_t HMI_HR_RET_FREQ_ACT_X10         = 211;

constexpr uint16_t HMI_HR_FAN1_FREQ_CMD_X10        = 212;
constexpr uint16_t HMI_HR_FAN1_FREQ_ACT_X10        = 213;

constexpr uint16_t HMI_HR_FAN2_FREQ_CMD_X10        = 214;
constexpr uint16_t HMI_HR_FAN2_FREQ_ACT_X10        = 215;

constexpr uint16_t HMI_HR_FAN1_MAN_FREQ_X10        = 216;
constexpr uint16_t HMI_HR_FAN2_MAN_FREQ_X10        = 217;

constexpr uint16_t HMI_HR_FAN1_DAMPER_CMD_PCT_X10  = 220;  // PLC által kiadott %
constexpr uint16_t HMI_HR_FAN1_DAMPER_FB_PCT_X10   = 221;  // visszajelzett %

constexpr uint16_t HMI_HR_PARAM_ACT_RET_GHOST_TIMEOUT_SEC    = 253;
constexpr uint16_t HMI_HR_PARAM_ACT_DAMPER_MOVE_SEC          = 254;
constexpr uint16_t HMI_HR_PARAM_ACT_TRH_FRESH_TIMEOUT_SEC    = 255;
constexpr uint16_t HMI_HR_PARAM_ACT_NEG_PRESSURE_OFFSET_X10  = 256;



/* ============================================================
   4) HMI → PLC PARAMÉTEREK (HOLDING REGISTER)
   ============================================================ */
constexpr uint16_t HMI_HR_RET_SPEED_SET_X10        = 300;
constexpr uint16_t HMI_HR_FAN1_DAMPER_SET_PCT_X10  = 301;  // ÚJ: kézi damper set %

// ------------------------------------------------------------
// 4/B) HMI -> PLC PARAMÉTEREZHETŐ IDŐZÍTŐK (HR)
// FIGYELEM: 300 és 301 már foglalt!
// ------------------------------------------------------------
constexpr uint16_t HMI_HR_PARAM_DEBOUNCE_MS            = 320;
constexpr uint16_t HMI_HR_PARAM_FREE_MIN_MS            = 321;
constexpr uint16_t HMI_HR_PARAM_BLOCK_MAX_MS           = 322;
constexpr uint16_t HMI_HR_PARAM_PUSH_EXT_MAX_MS        = 323;
constexpr uint16_t HMI_HR_PARAM_PUSH_CYCLE_MAX_MS      = 324;
constexpr uint16_t HMI_HR_PARAM_TAIL_CLEAR_MAX_MS      = 325;
constexpr uint16_t HMI_HR_PARAM_RET_START_CLEAR_MAX_MS = 326;
constexpr uint16_t HMI_HR_PARAM_RET_END_SOFT_MS        = 327;
constexpr uint16_t HMI_HR_PARAM_RET_END_HARD_MS        = 328;
constexpr uint16_t HMI_HR_PARAM_DANGER_TIME_MS         = 329;
constexpr uint16_t HMI_HR_PARAM_DANGER_MARGIN_MS       = 330;
constexpr uint16_t HMI_HR_PARAM_TABLE_WAIT_MAX_MS      = 331;
constexpr uint16_t HMI_HR_PARAM_BUZZER_MAX_MS          = 332;

// sec alapúak (HMI sec, PLC ms)
constexpr uint16_t HMI_HR_PARAM_RET_GHOST_TIMEOUT_SEC    = 333;
constexpr uint16_t HMI_HR_PARAM_DAMPER_MOVE_SEC          = 334;
constexpr uint16_t HMI_HR_PARAM_TRH_FRESH_TIMEOUT_SEC    = 335;

// nem idő (Hz x10)
constexpr uint16_t HMI_HR_PARAM_NEG_PRESSURE_OFFSET_X10  = 336;

// APPLY / SAVE / DEFAULT parancsok
constexpr uint16_t HMI_HR_PARAM_APPLY_CMD   = 340;
constexpr uint16_t HMI_HR_PARAM_SAVE_CMD    = 341;
constexpr uint16_t HMI_HR_PARAM_DEFAULT_CMD = 342;

// ------------------------------------------------------------
// PLC -> HMI VISSZATÜKRÖZÖTT / CLAMPELT PARAMÉTEREK (HR)
// ------------------------------------------------------------
constexpr uint16_t HMI_HR_PARAM_ACT_DEBOUNCE_MS            = 240;
constexpr uint16_t HMI_HR_PARAM_ACT_FREE_MIN_MS            = 241;
constexpr uint16_t HMI_HR_PARAM_ACT_BLOCK_MAX_MS           = 242;
constexpr uint16_t HMI_HR_PARAM_ACT_PUSH_EXT_MAX_MS        = 243;
constexpr uint16_t HMI_HR_PARAM_ACT_PUSH_CYCLE_MAX_MS      = 244;
constexpr uint16_t HMI_HR_PARAM_ACT_TAIL_CLEAR_MAX_MS      = 245;
constexpr uint16_t HMI_HR_PARAM_ACT_RET_START_CLEAR_MAX_MS = 246;
constexpr uint16_t HMI_HR_PARAM_ACT_RET_END_SOFT_MS        = 247;
constexpr uint16_t HMI_HR_PARAM_ACT_RET_END_HARD_MS        = 248;
constexpr uint16_t HMI_HR_PARAM_ACT_DANGER_TIME_MS         = 249;
constexpr uint16_t HMI_HR_PARAM_ACT_DANGER_MARGIN_MS       = 250;
constexpr uint16_t HMI_HR_PARAM_ACT_TABLE_WAIT_MAX_MS      = 251;
constexpr uint16_t HMI_HR_PARAM_ACT_BUZZER_MAX_MS          = 252;

// ============================================================
// STÁTUSZOK (PLC -> HMI)
// ============================================================

constexpr uint16_t HMI_HR_RET_LOAD         = 257;

constexpr uint16_t HMI_HR_TRH1_TEMP_X10    = 258;
constexpr uint16_t HMI_HR_TRH1_RH_X10      = 259;

constexpr uint16_t HMI_HR_TRH2_TEMP_X10    = 260;
constexpr uint16_t HMI_HR_TRH2_RH_X10      = 261;

constexpr uint16_t HMI_HR_TRH3_TEMP_X10    = 262;
constexpr uint16_t HMI_HR_TRH3_RH_X10      = 263;
