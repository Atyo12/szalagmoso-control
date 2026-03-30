#pragma once

/* ============================================================
   HMI MAP – MODBUS TCP
   ------------------------------------------------------------
   Tartományok:

   0 – 49     : HMI -> PLC COIL (parancsok)
   50 – 99    : tartalék

   100 – 199  : PLC -> HMI STATUS BIT
   200 – 299  : PLC -> HMI STATUS NUMERIC (HR)

   300 – 399  : HMI -> PLC SETPOINT / PARAM (HR)
   ============================================================ */


/* ============================================================
   1) HMI → PLC PARANCSOK (COIL)
   ============================================================ */

constexpr uint16_t HMI_COIL_HANDLE_MODE        = 0;
constexpr uint16_t HMI_COIL_SERVICE_ENABLE     = 1;
constexpr uint16_t HMI_COIL_RESET_PULSE        = 2;

constexpr uint16_t HMI_COIL_RET_FORCE_EMPTY    = 10;
constexpr uint16_t HMI_COIL_RET_CONTINUOUS     = 11;

// --- tartalék: 12–49 ---


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

constexpr uint16_t HMI_ST_PUSH_ACTIVE          = 140;

// --- tartalék: 141–199 ---


/* ============================================================
   3) PLC → HMI NUMERIKUS (HOLDING REGISTER)
   ============================================================ */

constexpr uint16_t HMI_HR_OP_MODE              = 200;
constexpr uint16_t HMI_HR_STATE                = 201;
constexpr uint16_t HMI_HR_FAULT                = 202;

constexpr uint16_t HMI_HR_RET_FREQ_CMD_X10     = 210;
constexpr uint16_t HMI_HR_RET_FREQ_ACT_X10     = 211;

// --- tartalék: 212–299 ---


/* ============================================================
   4) HMI → PLC PARAMÉTEREK (HOLDING REGISTER)
   ============================================================ */

constexpr uint16_t HMI_HR_RET_SPEED_SET_X10    = 300;

// --- tartalék: 301–399 ---