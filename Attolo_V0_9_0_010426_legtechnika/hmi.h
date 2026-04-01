#pragma once
#include "types.h"

struct HMIStatus {
  // üzemmód / fő állapot
  OpMode op_mode = OpMode::AUTO;
  State state = State::IDLE;
  Fault fault = Fault::NONE;

  // általános állapotok
  bool fault_active = false;
  bool reset_request = false;

  // engedélyek
  bool handle_mode = false;
  bool service_enable = false;

  // mosó szalag
  bool wash_stop_request = false;

  // return conveyor
  bool return_running = false;
  bool return_force_empty = false;
  bool return_continuous = false;
  float return_freq_cmd_hz = 0.0f;
  float return_freq_act_hz = 0.0f;

  // Fan1 damper
  bool fan1_running = false;
  bool fan1_damper_open_cmd = false;
  bool fan1_damper_open_fb = false;
  bool fan1_run_permitted = false;
  float fan1_damper_cmd_pct = 0.0f;
  float fan1_damper_fb_pct = 0.0f;

  // légtechnika
  bool vent_auto_enable;

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

//  légtechnika
void hmiSetVentAutoEnable(bool v);
bool hmiGetVentAutoEnable();

// HMI státusz
const HMIStatus& hmiGetStatus();
void hmiUpdateStatus();