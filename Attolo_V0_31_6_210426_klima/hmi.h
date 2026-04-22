#pragma once
#include "types.h"

struct HMIStatus {
  // üzemmód / fő állapot
  OpMode op_mode = OpMode::AUTO;
  State  state   = State::IDLE;
  Fault  fault   = Fault::NONE;

  // általános állapotok
  bool fault_active  = false;
  bool reset_request = false;

  // engedélyek
  bool handle_mode    = false;
  bool service_enable = false;

  // mosó szalag
  bool wash_stop_request = false;

  // return conveyor
  bool  return_running      = false;
  bool  return_force_empty  = false;
  bool  return_continuous   = false;
  float return_freq_cmd_hz  = 0.0f;
  float return_freq_act_hz  = 0.0f;

  // ventilátorok
  float fan1_freq_cmd_hz = 0.0f;
  float fan1_freq_act_hz = 0.0f;
  float fan2_freq_cmd_hz = 0.0f;
  float fan2_freq_act_hz = 0.0f;

  // Fan1 damper
  bool  fan1_running           = false;
  bool  fan2_running           = false;   // ÚJ
  bool  fan1_damper_open_cmd   = false;
  bool  fan1_damper_open_fb    = false;
  bool  fan1_run_permitted     = false;
  float fan1_damper_cmd_pct    = 0.0f;
  float fan1_damper_fb_pct     = 0.0f;

  // légtechnika
  bool vent_manual_mode = false;

  // áttoló
  bool push_active = false;

};

void hmiSetupDefaults();

// általános HMI parancsok
void hmiSetHandleMode(bool v);
void hmiSetServiceEnable(bool v);
void hmiPulseReset();

// return conveyor parancsok
void hmiSetReturnForceEmpty(bool v);
bool hmiGetReturnForceEmpty();

void hmiSetReturnConveyorContinuous(bool v);
bool hmiGetReturnConveyorContinuous();

void hmiSetReturnConveyorSpeedHz(float hz);
float hmiGetReturnConveyorSpeedHz();

// légtechnika
void hmiSetVentManualMode(bool v);
bool hmiGetVentManualMode();

void hmiSetFan1ManualRun(bool v);
bool hmiGetFan1ManualRun();

void hmiSetFan2ManualRun(bool v);
bool hmiGetFan2ManualRun();

void hmiSetFan1ManualFreqHz(float v);
float hmiGetFan1ManualFreqHz();

void hmiSetFan2ManualFreqHz(float v);
float hmiGetFan2ManualFreqHz();

// ÚJ: kézi damper setpoint 0..100 %
void hmiSetFan1DamperSetPct(float v);
float hmiGetFan1DamperSetPct();

// HMI státusz
const HMIStatus& hmiGetStatus();
void hmiUpdateStatus();