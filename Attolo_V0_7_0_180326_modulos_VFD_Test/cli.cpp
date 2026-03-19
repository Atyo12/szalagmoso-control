#include <Arduino.h>
#include "io.h"
#include "cli.h"
#include "vfd.h"

#if CLI_MODE

void cliSetup()
{
  Serial.println("CLI ready.");
  Serial.println("Commands:");
  Serial.println("  start");
  Serial.println("  stop");
  Serial.println("  freq=XX");
  Serial.println("  status");
}

void cliLoop()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "start") {
    vfdRequestRun(VfdId::RETURN, true);
    Serial.println("CMD: START");
  }
  else if (cmd == "stop") {
    vfdRequestRun(VfdId::RETURN, false);
    Serial.println("CMD: STOP");
  }
  else if (cmd.startsWith("freq=")) {
    float f = cmd.substring(5).toFloat();
    vfdRequestFreqHz(VfdId::RETURN, f);

    Serial.print("CMD: FREQ=");
    Serial.println(f);
  }
  else if (cmd == "status") {
    vfdPrintAll();
  }
  else {
    Serial.println("Commands: start, stop, freq=XX, status");
  }
}

#endif