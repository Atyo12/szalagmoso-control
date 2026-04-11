
#include <Arduino.h>
#include "modbus_tcp.h"
#include "types.h"
#include "io.h"
#include "sim.h"
#include "machine.h"
#include "hmi.h"
#include "vfd.h"
#include "trh.h"
#include "cli.h"

void setup() {
  Serial.begin(115200);
  delay(200);

#if SIM_MODE
  simSetup();
#endif

  machineSetupDefaults();
  hmiSetupDefaults();
  vfdSetup();
  trhSetup();
  modbusTcpSetup();

#if CLI_MODE
  cliSetup();
#endif


pinMode(CONTROLLINO_DO0, OUTPUT);
pinMode(CONTROLLINO_DO1, OUTPUT);
pinMode(CONTROLLINO_DO2, OUTPUT);
pinMode(CONTROLLINO_DO3, OUTPUT);
pinMode(CONTROLLINO_DO4, OUTPUT);
pinMode(CONTROLLINO_DO5, OUTPUT);
pinMode(CONTROLLINO_DO6, OUTPUT);
pinMode(CONTROLLINO_DO7, OUTPUT);
pinMode(CONTROLLINO_R0, OUTPUT);
pinMode(CONTROLLINO_R3, OUTPUT);


// induláskor minden lekapcsolva
digitalWrite(CONTROLLINO_DO0, LOW);
digitalWrite(CONTROLLINO_DO1, LOW);
digitalWrite(CONTROLLINO_DO2, LOW);
digitalWrite(CONTROLLINO_DO3, LOW);
digitalWrite(CONTROLLINO_DO4, LOW);
digitalWrite(CONTROLLINO_DO5, LOW);
digitalWrite(CONTROLLINO_DO6, LOW);
digitalWrite(CONTROLLINO_DO7, LOW);

} 

void loop() {
#if SIM_MODE
  simLoop();
#endif

#if CLI_MODE
  cliLoop();
#endif
  modbusTcpLoop();
  vfdLoop();
  machineLoop();
  hmiUpdateStatus();
   static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    vfdPrintAll();
  }
  
  delay(2);
}