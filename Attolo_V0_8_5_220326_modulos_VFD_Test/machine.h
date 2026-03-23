#pragma once
#include "types.h"

void machineSetupDefaults();
void machineLoop();

// HMI parancsok beállítása kívülről
void machineSetHmiHandleMode(bool v);
void machineSetHmiServiceEnable(bool v);
void machinePulseHmiReset();

// Return conveyor HMI parancsok
void machineSetReturnForceEmpty(bool v);
bool machineGetReturnForceEmpty();

void machineSetReturnConveyorSpeedHz(float hz);
float machineGetReturnConveyorSpeedHz();

void machineSetReturnConveyorContinuous(bool v);
bool machineGetReturnConveyorContinuous();

// HMI státuszhoz szükséges publikus getterek
OpMode machineGetOpMode();
State machineGetState();
Fault machineGetFault();

bool machineIsFaultLatched();
bool machineIsResetRequestActive();

bool machineGetHandleMode();
bool machineGetServiceEnable();

bool machineIsPushActive();
bool machineIsReturnConveyorRunning();
bool machineIsWashStopRequested();

float machineGetReturnActualHz();