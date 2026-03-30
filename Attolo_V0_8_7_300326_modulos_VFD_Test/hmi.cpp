#include "hmi.h"
#include "machine.h"

static HMIStatus g_hmi_status;

void hmiSetupDefaults()
{
  g_hmi_status = HMIStatus{};
}

// általános HMI parancsok
void hmiSetHandleMode(bool v)
{
  machineSetHmiHandleMode(v);
}

void hmiSetServiceEnable(bool v)
{
  machineSetHmiServiceEnable(v);
}

void hmiPulseReset()
{
  machinePulseHmiReset();
}

// return conveyor parancsok
void hmiSetReturnForceEmpty(bool v)
{
  machineSetReturnForceEmpty(v);
}

bool hmiGetReturnForceEmpty()
{
  return machineGetReturnForceEmpty();
}

void hmiSetReturnConveyorContinuous(bool v)
{
  machineSetReturnConveyorContinuous(v);
}

bool hmiGetReturnConveyorContinuous()
{
  return machineGetReturnConveyorContinuous();
}

void hmiSetReturnConveyorSpeedHz(float hz)
{
  machineSetReturnConveyorSpeedHz(hz);
}

float hmiGetReturnConveyorSpeedHz()
{
  return machineGetReturnConveyorSpeedHz();
}

// HMI státusz
void hmiUpdateStatus()
{
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
  g_hmi_status.return_freq_cmd_hz = machineGetReturnConveyorSpeedHz();
  g_hmi_status.return_freq_act_hz = machineGetReturnActualHz();
  g_hmi_status.push_active = machineIsPushActive();
}

const HMIStatus& hmiGetStatus()
{
  return g_hmi_status;
}