#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>

#include "modbus_tcp.h"
#include "hmi.h"
#include "hmi_map.h"
#include "vfd.h"
#include "trh.h"
#include "machine.h"

// ------------------------------------------------------------
// Hálózat beállítás
// ------------------------------------------------------------

static byte g_mac[] = { 0x02, 0x12, 0x34, 0x56, 0x78, 0x9A };
static IPAddress g_ip(192, 168, 1, 180);
static IPAddress g_dns(192, 168, 1, 1);
static IPAddress g_gateway(192, 168, 1, 1);
static IPAddress g_subnet(255, 255, 255, 0);

EthernetServer ethernetServer(502);
ModbusTCPServer modbusTCPServer;

// ===== PARAMÉTER KEZELÉS =====

static uint16_t clampU16ToHr(uint32_t v)
{
  if (v > 65535UL) return 65535;
  return (uint16_t)v;
}

static uint16_t clampI32ToHr(int32_t v)
{
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

static void writeActualParamsToStatusRegs(const MachineParams& p) {

  // ms-ben maradók
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_DEBOUNCE_MS, clampU16ToHr(p.debounce_ms));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_FREE_MIN_MS, clampU16ToHr(p.free_min_ms));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_BLOCK_MAX_MS, clampU16ToHr(p.block_max_ms));

  // HMI-n sec-ben megjelenők
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_PUSH_EXT_MAX_MS,        clampU16ToHr(p.push_ext_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_PUSH_CYCLE_MAX_MS,      clampU16ToHr(p.push_cycle_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_TAIL_CLEAR_MAX_MS,      clampU16ToHr(p.tail_clear_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_RET_START_CLEAR_MAX_MS, clampU16ToHr(p.ret_start_clear_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_RET_END_SOFT_MS,        clampU16ToHr(p.ret_end_soft_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_RET_END_HARD_MS,        clampU16ToHr(p.ret_end_hard_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_DANGER_TIME_MS,         clampU16ToHr(p.danger_time_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_DANGER_MARGIN_MS,       clampU16ToHr(p.danger_margin_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_TABLE_WAIT_MAX_MS,      clampU16ToHr(p.table_wait_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_BUZZER_MAX_MS,          clampU16ToHr(p.buzzer_max_ms / 1000UL));

  // új paraméterek
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_RET_GHOST_TIMEOUT_SEC,
                                       clampU16ToHr(p.ret_ghost_timeout_ms / 1000UL));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_DAMPER_MOVE_SEC,
                                       clampU16ToHr(p.fan1_damper_move_timeout_ms / 1000UL));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_TRH_FRESH_TIMEOUT_SEC,
                                       clampU16ToHr(p.trh_fresh_timeout_ms / 1000UL));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_ACT_NEG_PRESSURE_OFFSET_X10,
                                       clampU16ToHr((uint32_t)p.negative_pressure_offset_x10));
}

static void writeActualParamsBackToEditRegs(const MachineParams& p) {

  // ms-ben maradók
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_DEBOUNCE_MS, clampU16ToHr(p.debounce_ms));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_FREE_MIN_MS, clampU16ToHr(p.free_min_ms));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_BLOCK_MAX_MS, clampU16ToHr(p.block_max_ms));

  // HMI-n sec-ben szerkesztettek
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_PUSH_EXT_MAX_MS,        clampU16ToHr(p.push_ext_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_PUSH_CYCLE_MAX_MS,      clampU16ToHr(p.push_cycle_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_TAIL_CLEAR_MAX_MS,      clampU16ToHr(p.tail_clear_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_RET_START_CLEAR_MAX_MS, clampU16ToHr(p.ret_start_clear_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_RET_END_SOFT_MS,        clampU16ToHr(p.ret_end_soft_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_RET_END_HARD_MS,        clampU16ToHr(p.ret_end_hard_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_DANGER_TIME_MS,         clampU16ToHr(p.danger_time_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_DANGER_MARGIN_MS,       clampU16ToHr(p.danger_margin_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_TABLE_WAIT_MAX_MS,      clampU16ToHr(p.table_wait_max_ms / 1000UL));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_BUZZER_MAX_MS,          clampU16ToHr(p.buzzer_max_ms / 1000UL));

  // új paraméterek
  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_RET_GHOST_TIMEOUT_SEC,
                                       clampU16ToHr(p.ret_ghost_timeout_ms / 1000UL));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_DAMPER_MOVE_SEC,
                                       clampU16ToHr(p.fan1_damper_move_timeout_ms / 1000UL));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_TRH_FRESH_TIMEOUT_SEC,
                                       clampU16ToHr(p.trh_fresh_timeout_ms / 1000UL));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_NEG_PRESSURE_OFFSET_X10,
                                       clampU16ToHr((uint32_t)p.negative_pressure_offset_x10));
}

static void processMachineParamCommands() {

  auto applyEditRegsToMachine = []() {
    MachineParams p = machineGetParams();

    // ms-ben maradók
    p.debounce_ms = (uint16_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_DEBOUNCE_MS);
    p.free_min_ms = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_FREE_MIN_MS);
    p.block_max_ms = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_BLOCK_MAX_MS);

    // HMI-n sec -> PLC-ben ms
    p.push_ext_max_ms        = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_PUSH_EXT_MAX_MS)        * 1000UL;
    p.push_cycle_max_ms      = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_PUSH_CYCLE_MAX_MS)      * 1000UL;
    p.tail_clear_max_ms      = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_TAIL_CLEAR_MAX_MS)      * 1000UL;
    p.ret_start_clear_max_ms = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_RET_START_CLEAR_MAX_MS) * 1000UL;
    p.ret_end_soft_ms        = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_RET_END_SOFT_MS)        * 1000UL;
    p.ret_end_hard_ms        = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_RET_END_HARD_MS)        * 1000UL;
    p.danger_time_ms         = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_DANGER_TIME_MS)         * 1000UL;
    p.danger_margin_ms       = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_DANGER_MARGIN_MS)       * 1000UL;
    p.table_wait_max_ms      = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_TABLE_WAIT_MAX_MS)      * 1000UL;
    p.buzzer_max_ms          = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_BUZZER_MAX_MS)          * 1000UL;

    // új paraméterek
    p.ret_ghost_timeout_ms        = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_RET_GHOST_TIMEOUT_SEC) * 1000UL;
    p.fan1_damper_move_timeout_ms = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_DAMPER_MOVE_SEC)       * 1000UL;
    p.trh_fresh_timeout_ms        = (uint32_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_TRH_FRESH_TIMEOUT_SEC) * 1000UL;
    p.negative_pressure_offset_x10 =
        (int16_t)modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_NEG_PRESSURE_OFFSET_X10);

    machineSetParams(p);

    const MachineParams& act = machineGetParams();
    writeActualParamsToStatusRegs(act);
    writeActualParamsBackToEditRegs(act);
  };

  if (modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_APPLY_CMD) != 0) {
    applyEditRegsToMachine();
    modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_APPLY_CMD, 0);
  }

  if (modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_SAVE_CMD) != 0) {
    applyEditRegsToMachine();

    machineSaveParamsToEeprom();

    modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_SAVE_CMD, 0);
  }

  if (modbusTCPServer.holdingRegisterRead(HMI_HR_PARAM_DEFAULT_CMD) != 0) {
    machineResetParamsToDefaults();

    const MachineParams& act = machineGetParams();
    writeActualParamsToStatusRegs(act);
    writeActualParamsBackToEditRegs(act);

    modbusTCPServer.holdingRegisterWrite(HMI_HR_PARAM_DEFAULT_CMD, 0);
  }
}

// ------------------------------------------------------------
// Belső segédek
// ------------------------------------------------------------

static uint16_t clampX10(float v) {
  if (v < 0.0f) v = 0.0f;
  float x10 = v * 10.0f;
  if (x10 > 65535.0f) x10 = 65535.0f;
  return (uint16_t)(x10 + 0.5f);
}

static void updateModbusOutputsFromStatus() {
  const HMIStatus& st = hmiGetStatus();

  // --- PLC -> HMI státusz bitek
  modbusTCPServer.coilWrite(HMI_ST_FAULT_ACTIVE,         st.fault_active);
  modbusTCPServer.coilWrite(HMI_ST_RESET_REQUEST,        st.reset_request);

  modbusTCPServer.coilWrite(HMI_ST_HANDLE_MODE,          st.handle_mode);
  modbusTCPServer.coilWrite(HMI_ST_SERVICE_ENABLE,       st.service_enable);

  modbusTCPServer.coilWrite(HMI_ST_WASH_STOP_REQUEST,    st.wash_stop_request);

  modbusTCPServer.coilWrite(HMI_ST_RET_RUNNING,          st.return_running);
  modbusTCPServer.coilWrite(HMI_ST_RET_FORCE_EMPTY,      st.return_force_empty);
  modbusTCPServer.coilWrite(HMI_ST_RET_CONTINUOUS,       st.return_continuous);

  modbusTCPServer.coilWrite(HMI_ST_VENT_MANUAL_MODE,     st.vent_manual_mode);

  modbusTCPServer.coilWrite(HMI_ST_PUSH_ACTIVE,          st.push_active);

  modbusTCPServer.coilWrite(HMI_ST_FAN1_RUNNING,         st.fan1_running);
  modbusTCPServer.coilWrite(HMI_ST_FAN1_DAMPER_OPEN_CMD, st.fan1_damper_open_cmd);
  modbusTCPServer.coilWrite(HMI_ST_FAN1_DAMPER_OPEN_FB,  st.fan1_damper_open_fb);
  modbusTCPServer.coilWrite(HMI_ST_FAN1_RUN_PERMITTED,   st.fan1_run_permitted);
  modbusTCPServer.coilWrite(HMI_ST_FAN2_RUNNING,         st.fan2_running);

  // --- PLC -> HMI numerikus
  modbusTCPServer.holdingRegisterWrite(HMI_HR_OP_MODE,                 (uint16_t)st.op_mode);
  modbusTCPServer.holdingRegisterWrite(HMI_HR_STATE,                   (uint16_t)st.state);
  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAULT,                   (uint16_t)st.fault);

  modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_FREQ_CMD_X10,        clampX10(st.return_freq_cmd_hz));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_FREQ_ACT_X10,        clampX10(st.return_freq_act_hz));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAN1_FREQ_CMD_X10,       clampX10(st.fan1_freq_cmd_hz));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAN1_FREQ_ACT_X10,       clampX10(st.fan1_freq_act_hz));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAN2_FREQ_CMD_X10,       clampX10(st.fan2_freq_cmd_hz));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAN2_FREQ_ACT_X10,       clampX10(st.fan2_freq_act_hz));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAN1_DAMPER_CMD_PCT_X10, clampX10(st.fan1_damper_cmd_pct));
  modbusTCPServer.holdingRegisterWrite(HMI_HR_FAN1_DAMPER_FB_PCT_X10, clampX10(st.fan1_damper_fb_pct));

  modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_LOAD,
                                       clampI32ToHr(machineGetReturnLoad()));

  // --- Paraméterek tényleges (clampelt) értékei
  writeActualParamsToStatusRegs(machineGetParams());
}

static void applyModbusInputsToHmi() {
  // --- HMI -> PLC parancs coilok
  hmiSetHandleMode(modbusTCPServer.coilRead(HMI_COIL_HANDLE_MODE));
  hmiSetServiceEnable(modbusTCPServer.coilRead(HMI_COIL_SERVICE_ENABLE));

  hmiSetReturnForceEmpty(modbusTCPServer.coilRead(HMI_COIL_RET_FORCE_EMPTY));
  hmiSetReturnConveyorContinuous(modbusTCPServer.coilRead(HMI_COIL_RET_CONTINUOUS));

  hmiSetVentManualMode(modbusTCPServer.coilRead(HMI_COIL_VENT_MANUAL_MODE));
  hmiSetFan1ManualRun(modbusTCPServer.coilRead(HMI_COIL_FAN1_MAN_RUN));
  hmiSetFan2ManualRun(modbusTCPServer.coilRead(HMI_COIL_FAN2_MAN_RUN));

  // --- HMI -> PLC numerikus értékek
  hmiSetFan1ManualFreqHz(modbusTCPServer.holdingRegisterRead(HMI_HR_FAN1_MAN_FREQ_X10) / 10.0f);
  hmiSetFan2ManualFreqHz(modbusTCPServer.holdingRegisterRead(HMI_HR_FAN2_MAN_FREQ_X10) / 10.0f);
  hmiSetFan1DamperSetPct(modbusTCPServer.holdingRegisterRead(HMI_HR_FAN1_DAMPER_SET_PCT_X10) / 10.0f);

  // reset pulse: élre dolgozzuk fel, majd lenullázzuk
  if (modbusTCPServer.coilRead(HMI_COIL_RESET_PULSE)) {
    hmiPulseReset();
    modbusTCPServer.coilWrite(HMI_COIL_RESET_PULSE, 0);
  }

  // Return speed set
  uint16_t speedX10 = modbusTCPServer.holdingRegisterRead(HMI_HR_RET_SPEED_SET_X10);
  hmiSetReturnConveyorSpeedHz((float)speedX10 / 10.0f);

  // --- Paraméter APPLY / DEFAULT parancsok
  processMachineParamCommands();
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

void modbusTcpSetup() {
  Ethernet.begin(g_mac, g_ip, g_dns, g_gateway, g_subnet);
  ethernetServer.begin();

  if (!modbusTCPServer.begin()) {
    Serial.println("MODBUS TCP server start FAILED");
    return;
  }

  // --- INIT: Return conveyor speed szinkron HMI-vel ---
uint16_t retHzX10 = (uint16_t)(machineGetReturnConveyorSpeedHz() * 10.0f);

// Edit mező (HMI -> PLC)
modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_SPEED_SET_X10, retHzX10);

// Státusz mezők (PLC -> HMI)
modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_FREQ_CMD_X10, retHzX10);
modbusTCPServer.holdingRegisterWrite(HMI_HR_RET_FREQ_ACT_X10, retHzX10);

  modbusTCPServer.configureCoils(0, 200);
  modbusTCPServer.configureHoldingRegisters(0, 400);

  Serial.print("MODBUS TCP server ready at IP: ");
  Serial.println(Ethernet.localIP());

  // --- Paraméterek induló tükrözése HMI felé
  const MachineParams& p = machineGetParams();
  writeActualParamsToStatusRegs(p);
  writeActualParamsBackToEditRegs(p);
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------

void modbusTcpLoop() {
  EthernetClient client = ethernetServer.available();
  if (client) {
    modbusTCPServer.accept(client);
  }

  modbusTCPServer.poll();

  applyModbusInputsToHmi();
  hmiUpdateStatus();
  updateModbusOutputsFromStatus();
}