#include "io.h"
#include "cli.h"
#include "vfd.h"
#include "hmi.h"
#include "machine.h"

#if CLI_MODE

static bool parseTwoFloats(const String& cmd, float& a, float& b)
{
  int p1 = cmd.indexOf(' ');
  if (p1 < 0) return false;

  while (p1 < (int)cmd.length() && cmd.charAt(p1) == ' ') p1++;
  if (p1 >= (int)cmd.length()) return false;

  int p2 = cmd.indexOf(' ', p1);
  if (p2 < 0) return false;

  while (p2 < (int)cmd.length() && cmd.charAt(p2) == ' ') p2++;
  if (p2 >= (int)cmd.length()) return false;

  String s1 = cmd.substring(p1, cmd.indexOf(' ', p1));
  String s2 = cmd.substring(p2);

  s1.trim();
  s2.trim();

  a = s1.toFloat();
  b = s2.toFloat();
  return true;
}

void cliSetup()
{
  Serial.println("CLI ready.");
  Serial.println("Commands:");
  Serial.println(" start");
  Serial.println(" stop");
  Serial.println(" freq=XX");
  Serial.println(" status");
  Serial.println(" clt on");
  Serial.println(" clt off");
  Serial.println(" clt status");
  Serial.println(" t1 <tempC> <rh>");
  Serial.println(" t2 <tempC> <rh>");
  Serial.println(" t3 <tempC> <rh>");
  Serial.println(" t1valid <0|1>");
  Serial.println(" t2valid <0|1>");
  Serial.println(" t3valid <0|1>");
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
  else if (cmd == "clt on") {
    machineSetClimateTestOverride(true);
    Serial.println("CLIMATE TEST OVERRIDE = ON");
    machinePrintClimateTestState();
  }
  else if (cmd == "clt off") {
    machineSetClimateTestOverride(false);
    Serial.println("CLIMATE TEST OVERRIDE = OFF");
    machinePrintClimateTestState();
  }
  else if (cmd == "clt status") {
    machinePrintClimateTestState();
  }
  else if (cmd.startsWith("t1 ")) {
    float t = 0.0f, rh = 0.0f;
    if (parseTwoFloats(cmd, t, rh)) {
      machineSetClimateTestTrh1(t, rh, true);
      Serial.println("TRH1 TEST UPDATED");
      machinePrintClimateTestState();
    } else {
      Serial.println("Usage: t1 <tempC> <rh>");
    }
  }
  else if (cmd.startsWith("t2 ")) {
    float t = 0.0f, rh = 0.0f;
    if (parseTwoFloats(cmd, t, rh)) {
      machineSetClimateTestTrh2(t, rh, true);
      Serial.println("TRH2 TEST UPDATED");
      machinePrintClimateTestState();
    } else {
      Serial.println("Usage: t2 <tempC> <rh>");
    }
  }
  else if (cmd.startsWith("t3 ")) {
    float t = 0.0f, rh = 0.0f;
    if (parseTwoFloats(cmd, t, rh)) {
      machineSetClimateTestTrh3(t, rh, true);
      Serial.println("TRH3 TEST UPDATED");
      machinePrintClimateTestState();
    } else {
      Serial.println("Usage: t3 <tempC> <rh>");
    }
  }
  else if (cmd.startsWith("t1valid ")) {
    int v = cmd.substring(8).toInt();
    machineSetClimateTestTrhValid(1, v != 0);
    machinePrintClimateTestState();
  }
  else if (cmd.startsWith("t2valid ")) {
    int v = cmd.substring(8).toInt();
    machineSetClimateTestTrhValid(2, v != 0);
    machinePrintClimateTestState();
  }
  else if (cmd.startsWith("t3valid ")) {
    int v = cmd.substring(8).toInt();
    machineSetClimateTestTrhValid(3, v != 0);
    machinePrintClimateTestState();
  }
  else {
    Serial.println("Commands: start, stop, freq=XX, status, clt on/off/status, t1/t2/t3, t1valid/t2valid/t3valid");
  }
}

#endif