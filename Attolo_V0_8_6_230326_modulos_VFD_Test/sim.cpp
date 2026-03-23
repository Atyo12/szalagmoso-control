#include <Arduino.h>
#include "io.h"
#include "sim.h"
#include "machine.h"
#include "hmi.h"

// ------------------------------------------------------------
// SIM tömbök (io.h extern deklaráció)
// ------------------------------------------------------------
bool g_sim_di[32] = {0};
bool g_sim_do[32] = {0};

// ------------------------------------------------------------
// SIM MacroRunner
// ------------------------------------------------------------
enum class SimMacro {
  NONE,
  CYCLE_OK,
  FAIL_NO_EXT,
  FAIL_TAIL,
  FAIL_HOME
};

static SimMacro g_macro = SimMacro::NONE;
static uint32_t g_macro_t0 = 0;

static void macroStop();

static void macroStart(SimMacro m) {
  // biztos alaphelyzet
  g_sim_di[PIN_DI_WASH_IN_OPTO]   = 0;
  g_sim_di[PIN_DI_WASH_OUT_OPTO]  = 0;
  g_sim_di[PIN_DI_TABLE_SW]       = 0;
  g_sim_di[PIN_DI_PUSH_HOME]      = 1;
  g_sim_di[PIN_DI_PUSH_EXT]       = 0;
  g_sim_di[PIN_DI_RET_START_OCC]  = 0;
  g_sim_di[PIN_DI_RET_END_OCC]    = 0;
  g_sim_di[PIN_DI_RET_TAIL_OCC]   = 0;
  g_sim_di[PIN_DI_RESET]          = 0;
  g_sim_di[PIN_DI_JOG_DIR_FWD]    = 0;
  g_sim_di[PIN_DI_JOG_DIR_REV]    = 0;
  g_sim_di[PIN_DI_JOG_BTN]        = 0;

  hmiSetHandleMode(false);
  hmiSetServiceEnable(false);
  hmiPulseReset();

  g_macro = m;
  g_macro_t0 = millis();

  Serial.println("MACRO START (clean baseline + resetpulse)");
}

static void macroRun() {
  if (g_macro == SimMacro::NONE) return;

  uint32_t t = millis() - g_macro_t0;

  switch (g_macro) {
    case SimMacro::CYCLE_OK:
      if (t > 3000  && t < 3200)  g_sim_di[PIN_DI_TABLE_SW]      = 1; // raklap az asztalon
      if (t > 5000  && t < 5200)  g_sim_di[PIN_DI_PUSH_HOME]     = 0; // kar elindul
      if (t > 7000  && t < 7200)  g_sim_di[PIN_DI_TABLE_SW]      = 0; // asztal kiürül
      if (t > 9000  && t < 9200)  g_sim_di[PIN_DI_RET_START_OCC] = 1; // start zóna foglalt
      if (t > 11000 && t < 11200) g_sim_di[PIN_DI_PUSH_EXT]      = 1; // EXT megvan
      if (t > 13000 && t < 13200) g_sim_di[PIN_DI_RET_TAIL_OCC]  = 1; // veszélyzóna foglalt
      if (t > 17000 && t < 17200) g_sim_di[PIN_DI_RET_TAIL_OCC]  = 0; // veszélyzóna felszabadul
      if (t > 19000 && t < 19200) g_sim_di[PIN_DI_PUSH_EXT]      = 0; // EXT elenged
      if (t > 21000 && t < 21200) g_sim_di[PIN_DI_RET_START_OCC] = 0; // start zóna kiürül
      if (t > 23000 && t < 23200) g_sim_di[PIN_DI_PUSH_HOME]     = 1; // kar hazaér
      if (t > 25000) macroStop();
      break;

    case SimMacro::FAIL_NO_EXT:
      if (t > 3000 && t < 3200) g_sim_di[PIN_DI_TABLE_SW]      = 1;
      if (t > 5000 && t < 5200) g_sim_di[PIN_DI_PUSH_HOME]     = 0;
      if (t > 7000 && t < 7200) g_sim_di[PIN_DI_TABLE_SW]      = 0;
      if (t > 9000 && t < 9200) g_sim_di[PIN_DI_RET_START_OCC] = 1;
      // szándékosan nincs EXT
      if (t > 18000) macroStop();
      break;

    case SimMacro::FAIL_TAIL:
      if (t > 3000 && t < 3200) g_sim_di[PIN_DI_TABLE_SW]      = 1;
      if (t > 5000 && t < 5200) g_sim_di[PIN_DI_PUSH_HOME]     = 0;
      if (t > 7000 && t < 7200) g_sim_di[PIN_DI_TABLE_SW]      = 0;
      if (t > 9000 && t < 9200) g_sim_di[PIN_DI_RET_START_OCC] = 1;
      if (t > 11000 && t < 11200) g_sim_di[PIN_DI_PUSH_EXT]    = 1;
      if (t > 13000 && t < 13200) g_sim_di[PIN_DI_RET_TAIL_OCC]= 1;
      // tail nem ürül ki
      if (t > 22000) macroStop();
      break;

    case SimMacro::FAIL_HOME:
      if (t > 3000 && t < 3200) g_sim_di[PIN_DI_TABLE_SW]      = 1;
      if (t > 5000 && t < 5200) g_sim_di[PIN_DI_PUSH_HOME]     = 0;
      if (t > 7000 && t < 7200) g_sim_di[PIN_DI_TABLE_SW]      = 0;
      if (t > 9000 && t < 9200) g_sim_di[PIN_DI_RET_START_OCC] = 1;
      if (t > 11000 && t < 11200) g_sim_di[PIN_DI_PUSH_EXT]    = 1;
      if (t > 13000 && t < 13200) g_sim_di[PIN_DI_RET_TAIL_OCC]= 1;
      if (t > 17000 && t < 17200) g_sim_di[PIN_DI_RET_TAIL_OCC]= 0;
      if (t > 19000 && t < 19200) g_sim_di[PIN_DI_PUSH_EXT]    = 0;
      if (t > 21000 && t < 21200) g_sim_di[PIN_DI_RET_START_OCC] = 0;
      // szándékosan nincs HOME visszaadás
      if (t > 30000) macroStop();
      break;

    default:
      break;
  }
}

static void macroStop() {
  g_macro = SimMacro::NONE;
  Serial.println("MACRO STOP");
}

// ------------------------------------------------------------
// belső segéd
// ------------------------------------------------------------
static void simSetBool(int pin, int v) {
  if (pin < 0 || pin >= 32) return;
  g_sim_di[pin] = (v != 0);
}

// ------------------------------------------------------------
// HELP
// ------------------------------------------------------------
void simHelp() {
  Serial.println("SIM commands (0/1):");
  Serial.println(" home= ext= table= retstart= retend= rettail=");
  Serial.println(" reset= key= jog=");
  Serial.println(" handle= service=");
  Serial.println(" resetpulse");
  Serial.println(" show");
  Serial.println(" cycle_ok");
  Serial.println(" fail_no_ext");
  Serial.println(" fail_tail");
  Serial.println(" fail_home");
  Serial.println(" macro_stop");
}

// ------------------------------------------------------------
// SHOW
// ------------------------------------------------------------
void simShow() {
  Serial.print("DI: home=");     Serial.print(g_sim_di[PIN_DI_PUSH_HOME]);
  Serial.print(" ext=");         Serial.print(g_sim_di[PIN_DI_PUSH_EXT]);
  Serial.print(" table=");       Serial.print(g_sim_di[PIN_DI_TABLE_SW]);
  Serial.print(" rs=");          Serial.print(g_sim_di[PIN_DI_RET_START_OCC]);
  Serial.print(" re=");          Serial.print(g_sim_di[PIN_DI_RET_END_OCC]);
  Serial.print(" tailOcc=");     Serial.print(g_sim_di[PIN_DI_RET_TAIL_OCC]);
  Serial.print(" reset=");       Serial.print(g_sim_di[PIN_DI_RESET]);
  Serial.print(" jog_fwd="); Serial.print(g_sim_di[PIN_DI_JOG_DIR_FWD]);
  Serial.print(" jog_rev="); Serial.print(g_sim_di[PIN_DI_JOG_DIR_REV]);
  Serial.print(" jog_btn="); Serial.print(g_sim_di[PIN_DI_JOG_BTN]);
  Serial.println();
}

// ------------------------------------------------------------
// SERIAL PARSER
// ------------------------------------------------------------
static void simSerialParse() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  s.toLowerCase();

  if (s == "help")      { simHelp(); return; }
  if (s == "show")      { simShow(); return; }
  if (s == "forcehome") {
    g_sim_di[PIN_DI_PUSH_HOME] = true;
    Serial.print("FORCE: home=");
    Serial.println(g_sim_di[PIN_DI_PUSH_HOME]);
    return;
  }

  if (s == "resetpulse") { hmiPulseReset(); return; }

  if (s == "cycle_ok")     { macroStart(SimMacro::CYCLE_OK); return; }
  if (s == "fail_no_ext")  { macroStart(SimMacro::FAIL_NO_EXT); return; }
  if (s == "fail_tail")    { macroStart(SimMacro::FAIL_TAIL); return; }
  if (s == "fail_home")    { macroStart(SimMacro::FAIL_HOME); return; }
  if (s == "macro_stop")   { macroStop(); return; }

  auto parseKV = [&](const char* key, int pin) -> bool {
    int klen = strlen(key);
    if (s.startsWith(key)) {
      int v = s.substring(klen).toInt();
      simSetBool(pin, v);
      Serial.print("OK: ");
      Serial.print(key);
      Serial.print(v);
      Serial.print(" -> pin ");
      Serial.print(pin);
      Serial.print(" = ");
      Serial.println(g_sim_di[pin]);
      return true;
    }
    return false;
  };

  if (parseKV("home=",     PIN_DI_PUSH_HOME))     return;
  if (parseKV("ext=",      PIN_DI_PUSH_EXT))      return;
  if (parseKV("table=",    PIN_DI_TABLE_SW))      return;
  if (parseKV("retstart=", PIN_DI_RET_START_OCC)) return;
  if (parseKV("retend=",   PIN_DI_RET_END_OCC))   return;
  if (parseKV("rettail=",  PIN_DI_RET_TAIL_OCC))  return;
  if (parseKV("reset=",    PIN_DI_RESET))         return;
  if (parseKV("jog_fwd=", PIN_DI_JOG_DIR_FWD))    return;
  if (parseKV("jog_rev=", PIN_DI_JOG_DIR_REV))    return;
  if (parseKV("jog_btn=", PIN_DI_JOG_BTN))        return;

  if (s.startsWith("handle=")) {
    bool v = s.substring(7).toInt();
    hmiSetHandleMode(v);
    return;
  }

  if (s.startsWith("service=")) {
    bool v = s.substring(8).toInt();
    hmiSetServiceEnable(v);
    return;
  }

  Serial.println("Unknown cmd. Type 'help'.");
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void simSetup() {
  Serial.println("SIM_MODE ready. Type 'help'.");

  // induló állapot
  simSetBool(PIN_DI_PUSH_HOME, 1);
  simSetBool(PIN_DI_PUSH_EXT, 0);
  simSetBool(PIN_DI_RET_TAIL_OCC, 0);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void simLoop() {
  simSerialParse();
  macroRun();
}