#pragma once

void machineSetupDefaults();
void machineLoop();

// HMI parancsok beállítása kívülről (pl. SIM-ből)
void machineSetHmiHandleMode(bool v);
void machineSetHmiServiceEnable(bool v);
void machinePulseHmiReset();

// Return conveyor beállított frekvenciája Hz-ben
// Később ezt a HMI fogja írni.
void machineSetReturnConveyorSpeedHz(float hz);
float machineGetReturnConveyorSpeedHz();

// Return conveyor üzemmód
void machineSetReturnConveyorContinuous(bool v);
bool machineGetReturnConveyorContinuous();