#include <Arduino.h>
#include "types.h"
#include "io.h"
#include "sim.h"
#include "machine.h"
#include "vfd.h"
#include "cli.h"

void setup() {
  Serial.begin(115200);
  delay(200);

#if SIM_MODE
  simSetup();
#endif

  machineSetupDefaults();
  vfdSetup();

#if CLI_MODE
  cliSetup();
#endif
}

void loop() {
#if SIM_MODE
  simLoop();
#endif

#if CLI_MODE
  cliLoop();
#endif

  vfdLoop();
  machineLoop();

  delay(10);
}