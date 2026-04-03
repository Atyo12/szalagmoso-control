#pragma once

// ------------------------------------------------------------
// CABINET / VÉGLEGES GÉP I/O TÉRKÉP
// ------------------------------------------------------------

// Inputs
constexpr int PIN_DI_WASH_IN_OPTO    = CONTROLLINO_AI0;   // nagy eloszto
constexpr int PIN_DI_WASH_OUT_OPTO   = CONTROLLINO_AI1;   // doboz
constexpr int PIN_DI_TABLE_SW        = CONTROLLINO_AI2;   // doboz
constexpr int PIN_DI_PUSH_HOME       = CONTROLLINO_AI4;   // doboz
constexpr int PIN_DI_PUSH_EXT        = CONTROLLINO_AI3;   // doboz
constexpr int PIN_DI_RET_START_OCC   = CONTROLLINO_AI5;   // doboz
constexpr int PIN_DI_RET_END_OCC     = CONTROLLINO_AI6;   // nagy eloszto
constexpr int PIN_DI_RET_TAIL_OCC    = CONTROLLINO_AI7;   // 1 = foglalt
constexpr int PIN_DI_JOG_DIR_FWD = CONTROLLINO_AI8;
constexpr int PIN_DI_JOG_DIR_REV = CONTROLLINO_AI9;
constexpr int PIN_DI_JOG_BTN     = CONTROLLINO_AI10;   // deadman / reset ha nincs LOCAL_JOG
constexpr int PIN_AI_ENCODER     = CONTROLLINO_DI2;
constexpr int PIN_DI_RESET_MAIN  = CONTROLLINO_DI0;    // HMI alatti külön reset gomb

constexpr int PIN_AO_FAN1_DAMPER_CMD = CONTROLLINO_AO1;
constexpr int PIN_AI_FAN1_DAMPER_FB  = CONTROLLINO_AI12;

// Outputs
constexpr int PIN_STOP_REQ           = CONTROLLINO_R3;
constexpr int PIN_DO_GREEN_LIGHT     = CONTROLLINO_DO1; //HMI alatti zöld
constexpr int PIN_DO_JOG_LIGHT       = CONTROLLINO_DO2;
constexpr int PIN_DO_SEMAFOR_YELLOW  = CONTROLLINO_DO3;
constexpr int PIN_DO_ALARM_LAMP      = CONTROLLINO_DO4;
constexpr int PIN_DO_SEMAFOR_GREEN   = CONTROLLINO_DO5;
constexpr int PIN_DO_PUSH_MOTOR      = CONTROLLINO_DO6;
constexpr int PIN_DO_PUSH_REV        = CONTROLLINO_DO7;
constexpr int PIN_DO_BUZZER          = CONTROLLINO_R3;