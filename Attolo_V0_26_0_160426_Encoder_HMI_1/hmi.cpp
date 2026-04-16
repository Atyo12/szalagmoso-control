#include "hmi.h"
#include "machine.h"
#include "vfd.h"

static bool g_hmi_vent_manual_mode = false;
static HMIStatus g_hmi_status;

static bool  g_hmi_fan1_manual_run = false;
static bool  g_hmi_fan2_manual_run = false;
static float g_hmi_fan1_manual_freq_hz = 20.0f;
static float g_hmi_fan2_manual_freq_hz = 20.0f;
static float g_hmi_fan1_damper_set_pct = 0.0f;   // ÚJ

static float clampHmiFanHz(float hz) {
  if (isnan(hz) || isinf(hz)) return 5.0f;
  if (hz < 5.0f) return 5.0f;
  if (hz > 80.0f) return 80.0f;
  return hz;
}

static float clampPct(float v) {
  if (isnan(v) || isinf(v)) return 0.0f;
  if (v < 0.0f) return 0.0f;
  if (v > 100.0f) return 100.0f;
  return v;
}

void hmiSetupDefaults() {
  g_hmi_status = HMIStatus{};
  g_hmi_vent_manual_mode = false;
  g_hmi_fan1_manual_run = false;
  g_hmi_fan2_manual_run = false;
  g_hmi_fan1_manual_freq_hz = 20.0f;
  g_hmi_fan2_manual_freq_hz = 20.0f;
  g_hmi_fan1_damper_set_pct = 0.0f;
}

// --------------------------------------------------
// általános HMI parancsok
// --------------------------------------------------

void hmiSetHandleMode(bool v) {
  machineSetHmiHandleMode(v);
}

void hmiSetServiceEnable(bool v) {
  machineSetHmiServiceEnable(v);
}

void hmiPulseReset() {
  machinePulseHmiReset();
}

// --------------------------------------------------
// return conveyor parancsok
// --------------------------------------------------

void hmiSetReturnForceEmpty(bool v) {
  machineSetReturnForceEmpty(v);
}

bool hmiGetReturnForceEmpty() {
  return machineGetReturnForceEmpty();
}

void hmiSetReturnConveyorContinuous(bool v) {
  machineSetReturnConveyorContinuous(v);
}

bool hmiGetReturnConveyorContinuous() {
  return machineGetReturnConveyorContinuous();
}

void hmiSetReturnConveyorSpeedHz(float hz) {
  machineSetReturnConveyorSpeedHz(hz);
}

float hmiGetReturnConveyorSpeedHz() {
  return machineGetReturnConveyorSpeedHz();
}

// --------------------------------------------------
// légtechnika kézi parancsok
// --------------------------------------------------

void hmiSetFan1ManualRun(bool v) {
  g_hmi_fan1_manual_run = v;
}

bool hmiGetFan1ManualRun() {
  return g_hmi_fan1_manual_run;
}

void hmiSetFan2ManualRun(bool v) {
  g_hmi_fan2_manual_run = v;
}

bool hmiGetFan2ManualRun() {
  return g_hmi_fan2_manual_run;
}

void hmiSetFan1ManualFreqHz(float v) {
  g_hmi_fan1_manual_freq_hz = v;
}

float hmiGetFan1ManualFreqHz() {
  return g_hmi_fan1_manual_freq_hz;
}

void hmiSetFan2ManualFreqHz(float v) {
  g_hmi_fan2_manual_freq_hz = v;
}

float hmiGetFan2ManualFreqHz() {
  return g_hmi_fan2_manual_freq_hz;
}

void hmiSetVentManualMode(bool v) {
  g_hmi_vent_manual_mode = v;
}

bool hmiGetVentManualMode() {
  return g_hmi_vent_manual_mode;
}

void hmiSetFan1DamperSetPct(float v) {
  g_hmi_fan1_damper_set_pct = clampPct(v);
}

float hmiGetFan1DamperSetPct() {
  return g_hmi_fan1_damper_set_pct;
}

// --------------------------------------------------
// HMI státusz
// --------------------------------------------------

void hmiUpdateStatus() {
  g_hmi_status.op_mode = machineGetOpMode();
  g_hmi_status.state = machineGetState();
  g_hmi_status.fault = machineGetFault();

  g_hmi_status.fault_active = machineIsFaultLatched();
  g_hmi_status.reset_request = machineIsResetRequestActive();

  g_hmi_status.handle_mode = machineGetHandleMode();
  g_hmi_status.service_enable = machineGetServiceEnable();

  g_hmi_status.wash_stop_request = machineIsWashStopRequested();

  g_hmi_status.return_running = machineIsReturnConveyorRunning();
  g_hmi_status.return_force_empty = machineGetReturnForceEmpty();
  g_hmi_status.return_continuous = machineGetReturnConveyorContinuous();

  g_hmi_status.vent_manual_mode = hmiGetVentManualMode();

  // RETURN
  g_hmi_status.return_freq_cmd_hz = machineGetReturnConveyorSpeedHz();
  g_hmi_status.return_freq_act_hz = machineGetReturnActualHz();

  // FAN1 / FAN2 accepted érték
  if (g_hmi_status.vent_manual_mode) {
    g_hmi_status.fan1_freq_cmd_hz = clampHmiFanHz(hmiGetFan1ManualFreqHz());
    g_hmi_status.fan2_freq_cmd_hz = clampHmiFanHz(hmiGetFan2ManualFreqHz());
  } else {
    g_hmi_status.fan1_freq_cmd_hz = 0.0f;
    g_hmi_status.fan2_freq_cmd_hz = 0.0f;
  }

  // aktuális frekvencia
  g_hmi_status.fan1_freq_act_hz = vfdGetActualHz(VfdId::FAN1);
  g_hmi_status.fan2_freq_act_hz = vfdGetActualHz(VfdId::FAN2);

  // futási visszajelzés
  g_hmi_status.fan1_running = machineIsFan1Running();
  g_hmi_status.fan2_running = vfdGetState(VfdId::FAN2).running;

  // FAN1 damper
  g_hmi_status.fan1_damper_open_cmd = machineIsFan1DamperOpenCmd();
  g_hmi_status.fan1_damper_open_fb = machineIsFan1DamperOpenFb();
  g_hmi_status.fan1_run_permitted = machineIsFan1RunPermitted();
  g_hmi_status.fan1_damper_cmd_pct = machineGetFan1DamperCmdPct();
  g_hmi_status.fan1_damper_fb_pct = machineGetFan1DamperFbPct();

  g_hmi_status.push_active = machineIsPushActive();
}

const HMIStatus& hmiGetStatus() {
  return g_hmi_status;
}