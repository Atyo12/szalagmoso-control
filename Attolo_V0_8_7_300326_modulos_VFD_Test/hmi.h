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

// HMI státusz
const HMIStatus& hmiGetStatus();
void hmiUpdateStatus();