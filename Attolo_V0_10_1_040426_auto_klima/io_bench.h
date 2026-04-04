#pragma once

// ------------------------------------------------------------
// BENCH / TESZTPAD I/O TÉRKÉP
// Ez a mostani működő logikát tartja meg,
// csak a nyers számok helyett Controllino aliasokkal.
// ------------------------------------------------------------

// Inputs
constexpr int PIN_DI_WASH_IN_OPTO    = CONTROLLINO_AI0;
constexpr int PIN_DI_WASH_OUT_OPTO   = CONTROLLINO_AI1;
constexpr int PIN_DI_TABLE_SW        = CONTROLLINO_AI2;
constexpr int PIN_DI_PUSH_HOME       = CONTROLLINO_AI3;
constexpr int PIN_DI_PUSH_EXT        = CONTROLLINO_AI4;
constexpr int PIN_DI_RET_START_OCC   = CONTROLLINO_AI5;
constexpr int PIN_DI_RET_END_OCC     = CONTROLLINO_AI6;
constexpr int PIN_DI_RET_TAIL_OCC    = CONTROLLINO_AI7;   // 1 = foglalt
constexpr int PIN_DI_RESET_MAIN   = CONTROLLINO_AI8;   // HMI alatti külön reset gomb

// LOCAL_JOG / hátsó kezelő
constexpr int PIN_DI_JOG_DIR_FWD  = CONTROLLINO_AI9;   // kapcsoló egyik irány
constexpr int PIN_DI_JOG_DIR_REV  = CONTROLLINO_AI10;  // kapcsoló másik irány
constexpr int PIN_DI_JOG_BTN      = CONTROLLINO_DI2;  // deadman / reset ha nincs LOCAL_JOG

constexpr int PIN_AO_FAN1_DAMPER_CMD = CONTROLLINO_AO0;
constexpr int PIN_AI_FAN1_DAMPER_FB  = CONTROLLINO_AI11;

// Outputs
constexpr int PIN_STOP_REQ           = CONTROLLINO_R0;
constexpr int PIN_DO_RET_CONV        = CONTROLLINO_DO1;   // bench debug / régi logika szerint még él
constexpr int PIN_DO_PUSH_MOTOR      = CONTROLLINO_DO2;   // üzemi irány
constexpr int PIN_DO_BUZZER          = CONTROLLINO_DO3; 
constexpr int PIN_DO_ALARM_LAMP      = CONTROLLINO_DO4;
constexpr int PIN_DO_PUSH_REV        = CONTROLLINO_DO5;   // szerviz visszairány
constexpr int PIN_DO_GREEN_LIGHT     = CONTROLLINO_DO6;  // HMI alatti zöld lámpa + reset
constexpr int PIN_DO_JOG_LIGHT       = CONTROLLINO_DO7;
