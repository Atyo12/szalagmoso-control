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

/* ============================================================
   4) HMI → PLC PARAMÉTEREK (HOLDING REGISTER)
   ============================================================ */
constexpr uint16_t HMI_HR_RET_SPEED_SET_X10        = 300;
constexpr uint16_t HMI_HR_FAN1_DAMPER_SET_PCT_X10  = 301;  // ÚJ: kézi damper set %