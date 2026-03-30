#include <Arduino.h>
#include "types.h"
#include "io.h"
#include "machine.h"
#include "sim.h"
#include "vfd.h"

/* ============================================================
   1) Debounce + élérzékelés
   ============================================================ */
struct DebouncedInput {
  bool raw = false;
  bool stable = false;
  bool last_stable = false;

  bool last_raw = false;
  uint32_t last_raw_change_ms = 0;

  void update(bool new_raw, uint32_t now, uint16_t debounce_ms) {
    raw = new_raw;

    if (raw != last_raw) {
      last_raw = raw;
      last_raw_change_ms = now;
    }

    if ((uint32_t)(now - last_raw_change_ms) >= debounce_ms) {
      if (stable != raw) {
        last_stable = stable;
        stable = raw;
      } else {
        last_stable = stable;
      }
    } else {
      last_stable = stable;
    }
  }

  bool rise() const { return (!last_stable && stable); }
  bool fall() const { return (last_stable && !stable); }
};

/* ============================================================
   2) Belső HMI parancsok
   ============================================================ */
static bool hmi_handle_mode    = false;
static bool hmi_service_enable = false;
static bool hmi_reset_cmd      = false;
static bool hmi_ret_force_empty = false;

/* ============================================================
   3) Globális állapotok
   ============================================================ */
static uint32_t g_now = 0;

static State g_state = State::IDLE;
static Fault g_fault = Fault::NONE;
static bool  g_fault_latched = false;

// Buzzer limit
static const uint32_t T_BUZZER_MAX_MS = 30000;

static bool g_startup_lock = true;   // power-on után mindig 1

// Diagnosztika: first/last fault
static Fault    g_fault_first = Fault::NONE;
static Fault    g_fault_last  = Fault::NONE;
static uint32_t g_fault_first_ms = 0;
static uint32_t g_fault_last_ms  = 0;

// HMI epoch – itt 0
static uint32_t g_now_epoch = 0;
static uint32_t g_last_reset_epoch = 0;

// Számlálók
static uint32_t g_in_count = 0;
static uint32_t g_out_count = 0;
static uint32_t g_ret_in_count = 0;
static uint32_t g_ret_out_count = 0;

static inline int32_t wash_load() { return (int32_t)g_in_count - (int32_t)g_out_count; }
static inline int32_t ret_load()  { return (int32_t)g_ret_in_count - (int32_t)g_ret_out_count; }

// Fizikai bemenetek (szűrt)
static DebouncedInput di_in_opto;
static DebouncedInput di_out_opto;
static DebouncedInput di_table_sw;
static DebouncedInput di_push_home;
static DebouncedInput di_push_ext;
static DebouncedInput di_ret_start;
static DebouncedInput di_ret_end;
static DebouncedInput di_ret_tail_occ;   // 1=foglalt
static DebouncedInput di_reset_phys;
static DebouncedInput di_jog_dir_fwd;
static DebouncedInput di_jog_dir_rev;
static DebouncedInput di_jog_btn;

// OUT opto időmérések
static Timer t_out_block;
static Timer t_out_free;
static uint32_t last_t_block_ms = 0;
static uint32_t last_t_free_ms  = 0;

// Danger ablak
static Timer t_danger;

// Áttoló felügyelet
static Timer t_push_ext;
static Timer t_push_cycle;
static bool push_active = false;
static bool push_left_home = false;
static bool push_ext_seen = false;

// PUSH fázisok (OUT -> WAIT_TAIL_CLEAR -> RETURN)
enum class PushPhase { IDLE, OUT, WAIT_TAIL_CLEAR, RETURN };
static PushPhase g_push_phase = PushPhase::IDLE;
static Timer t_tail_clear_wait;

// TABLE_SW latch
static bool table_latched = false;
static Timer t_table_wait;

// Visszahordó felügyelet
static Timer t_ret_start_clear;
static Timer t_ret_end_soft;
static Timer t_ret_end_hard;
static Timer t_ret_ghost;

// Kimeneti parancsok
static bool cmd_stop_req = false;
static bool cmd_ret_conv = false;
static bool cmd_push_motor = false;
static bool cmd_buzzer = false;
static bool cmd_push_rev = false;

// Return conveyor beállított frekvencia (később HMI írja majd)
static float g_ret_conv_speed_hz = 20.0f;

// Return conveyor üzemmód:
// false = léptetett (jelenlegi logika)
// true  = folyamatos
static bool hmi_ret_conv_continuous = false;

/* ============================================================
   4) Módok
   ============================================================ */
static OpMode g_mode = OpMode::AUTO;

/* ============================================================
   5) Controlled return-to-home (AUTO fault alatt)
   ============================================================ */
static bool  g_fault_hold_until_home = false;
static Timer t_fault_home_recover;

/* ============================================================
   6) Segédek
   ============================================================ */
static bool estopOk() {
  // nálatok E-stop tápelvágás, SIM-ben is mindig OK
  return true;
}

// ret_conveyor leállításának sgédfüggvénye, ha nem veszik le a raklapot időben
static bool returnEndSoftStopActive() {
  return di_ret_end.stable &&
         t_ret_end_soft.running &&
         t_ret_end_soft.expired(g_now);
}

static void updateHmiCommands() {
  // SIM-ben: most üres
}

static void resetReturnState() {
  g_ret_in_count  = 0;
  g_ret_out_count = 0;

  t_ret_start_clear.stop();
  t_ret_end_soft.stop();
  t_ret_end_hard.stop();
  t_ret_ghost.stop();
}

static bool s_prev_hmi_ret_force_empty = false;

 
 
static void handleReturnForceEmptyEdge() {
  bool falling = (s_prev_hmi_ret_force_empty && !hmi_ret_force_empty);

  if (falling) {
    resetReturnState();
  }

  s_prev_hmi_ret_force_empty = hmi_ret_force_empty;
}

static bool resetRequestedHmiRise() {
  if (hmi_reset_cmd) {
    hmi_reset_cmd = false;   // egy ciklusos impulzus
    return true;
  }
  return false;
}

static bool resetRequested() {
  return resetRequestedHmiRise() || di_reset_phys.rise();
}

static bool resetAllowed() {
  if (!estopOk()) return false;
  if (!di_push_home.stable) return false;
  if (di_push_home.stable && di_push_ext.stable) return false;
  return true;
}

static OpMode computeMode() {
  bool jog_dir_valid = (di_jog_dir_fwd.stable ^ di_jog_dir_rev.stable);

  if (hmi_handle_mode &&
      hmi_service_enable &&
      jog_dir_valid) {
    return OpMode::LOCAL_JOG;
  }

  if (hmi_handle_mode) return OpMode::HANDLE;
  return OpMode::AUTO;
}

/* ============================================================
   7) Hibakezelés
   ============================================================ */
static bool isHardStopFault(Fault f) {
  switch (f) {
    case Fault::E900_ESTOP:
    case Fault::E303_PUSH_SENSOR_CONFLICT:
    case Fault::E301_PUSH_NO_EXT:
    case Fault::E302_PUSH_HOME_TIMEOUT:
    case Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT:
      return true;
    default:
      return false;
  }
}

static void setFault(Fault f) {
  g_fault_last = f;
  g_fault_last_ms = g_now;

  if (g_fault_latched) return;

  g_fault = f;
  g_fault_latched = true;
  g_state = State::ERROR;

  g_fault_first = f;
  g_fault_first_ms = g_now;

  // Alap biztonsági reakció faultnál
  cmd_stop_req = true;
  cmd_ret_conv = false;
  cmd_buzzer   = true;

  // Hard stop faultnál az áttoló motor azonnal álljon le
  if (isHardStopFault(f)) {
    cmd_push_motor = false;
    g_fault_hold_until_home = false;
    t_fault_home_recover.stop();
    return;
  }

  // Csak nem-hard-stop faultnál engedjük a controlled returnt
  if (g_mode == OpMode::AUTO &&
      push_active &&
      !di_push_home.stable) {
    g_fault_hold_until_home = true;
    t_fault_home_recover.start(g_now, T_CYCLE_MAX_MS);
  } else {
    cmd_push_motor = false;
  }
}

/* ============================================================
   8) Bemenetek frissítése
   ============================================================ */
static void updateInputs() {
  di_in_opto.update(safeRead(PIN_DI_WASH_IN_OPTO), g_now, DEBOUNCE_MS);
  di_out_opto.update(safeRead(PIN_DI_WASH_OUT_OPTO), g_now, DEBOUNCE_MS);
  di_table_sw.update(safeRead(PIN_DI_TABLE_SW), g_now, DEBOUNCE_MS);
  di_push_home.update(safeRead(PIN_DI_PUSH_HOME), g_now, DEBOUNCE_MS);
  di_push_ext.update(safeRead(PIN_DI_PUSH_EXT), g_now, DEBOUNCE_MS);
  di_ret_start.update(safeRead(PIN_DI_RET_START_OCC), g_now, DEBOUNCE_MS);
  di_ret_end.update(safeRead(PIN_DI_RET_END_OCC), g_now, DEBOUNCE_MS);
  di_ret_tail_occ.update(safeRead(PIN_DI_RET_TAIL_OCC), g_now, DEBOUNCE_MS);

  di_reset_phys.update(safeRead(PIN_DI_RESET), g_now, DEBOUNCE_MS);
  di_jog_dir_fwd.update(safeRead(PIN_DI_JOG_DIR_FWD), g_now, DEBOUNCE_MS);
  di_jog_dir_rev.update(safeRead(PIN_DI_JOG_DIR_REV), g_now, DEBOUNCE_MS);
  di_jog_btn.update(safeRead(PIN_DI_JOG_BTN), g_now, DEBOUNCE_MS);
}

/* ============================================================
   9) Számlálók
   ============================================================ */
static void updateCounters() {
  if (di_in_opto.rise())  g_in_count++;
  if (di_out_opto.rise()) g_out_count++;

  if (di_ret_start.rise()) g_ret_in_count++;
  if (di_ret_end.rise())   g_ret_out_count++;
}

static void superviseReturnGhostLoad() {

  bool no_physical_pallet =
      !di_ret_start.stable &&
      !di_ret_end.stable;

  if (ret_load() > 0 && no_physical_pallet) {

    if (!t_ret_ghost.running)
      t_ret_ghost.start(g_now, 30000);   // 30 s

    if (t_ret_ghost.expired(g_now)) {
      resetReturnState();
    }

  } else {
    t_ret_ghost.stop();
  }
}

/* ============================================================
   10) OUT opto felügyelet
   ============================================================ */
static void superviseOutOpto() {
  if (di_out_opto.rise()) {
    if (t_out_free.running) {
      last_t_free_ms = t_out_free.elapsed(g_now);
      t_out_free.stop();
    }

    t_out_block.start(g_now, T_BLOCK_MAX_MS);

    if (wash_load() >= 2) {
      if (last_t_free_ms > 0 && last_t_free_ms < T_FREE_MIN_MS) {
        setFault(Fault::E201_SPACING_COLLAPSE);
      }
    }
  }

  if (di_out_opto.fall()) {
    if (t_out_block.running) {
      last_t_block_ms = t_out_block.elapsed(g_now);
      t_out_block.stop();
    }

    if (last_t_block_ms > T_BLOCK_MAX_MS) {
      setFault(Fault::E202_OUT_OPTO_STUCK_OR_DIRTY);
    }

    t_out_free.start(g_now, 0xFFFFFFFF);
  }

  if (di_out_opto.stable && t_out_block.running && t_out_block.expired(g_now)) {
    setFault(Fault::E202_OUT_OPTO_STUCK_OR_DIRTY);
  }
}

/* ============================================================
   11) Danger ablak
   ============================================================ */
static void superviseDangerWindow() {
  if (di_out_opto.rise()) {
    uint32_t stop_limit = (T_DANGER_TIME_MS > T_DANGER_MARGIN_MS)
                        ? (T_DANGER_TIME_MS - T_DANGER_MARGIN_MS)
                        : T_DANGER_TIME_MS;
    t_danger.start(g_now, stop_limit);
  }

  if (t_danger.running && t_danger.expired(g_now)) {
    if (!di_push_home.stable) {
      setFault(Fault::E501_DANGER_HOME_LATE);
    }
    t_danger.stop();
  }
}

/* ============================================================
   12) Push ciklus
   ============================================================ */
static void startPushCycle() {
  push_active = true;
  push_left_home = false;
  push_ext_seen = false;

  g_push_phase = PushPhase::OUT;

  cmd_push_motor = true;

  t_push_ext.start(g_now, T_EXT_MAX_MS);
  t_push_cycle.start(g_now, T_CYCLE_MAX_MS);

  t_tail_clear_wait.stop();
}

static void stopPushCycle() {
  cmd_push_motor = false;
  cmd_push_rev   = false;

  push_active = false;
  push_left_home = false;
  push_ext_seen = false;

  g_push_phase = PushPhase::IDLE;

  t_push_ext.stop();
  t_push_cycle.stop();
  t_tail_clear_wait.stop();
}

static void supervisePushCycle() {
  if (!push_active) return;

  if (di_push_home.fall()) {
  push_left_home = true;

  if (!hmi_ret_conv_continuous) {
    cmd_ret_conv = false;
  }
}

  // OUT fázis
  if (g_push_phase == PushPhase::OUT) {
    if (di_push_ext.rise()) {
  push_ext_seen = true;

  if (!hmi_ret_conv_continuous) {
    cmd_ret_conv = true;
  }

  cmd_push_motor = false;
  g_push_phase = PushPhase::WAIT_TAIL_CLEAR;
  t_tail_clear_wait.start(g_now, T_TAIL_CLEAR_MAX_MS);
}

    if (!push_ext_seen && t_push_ext.running && t_push_ext.expired(g_now)) {
  cmd_push_motor = false;
  setFault(Fault::E301_PUSH_NO_EXT);
  return;
}
  }

  // WAIT_TAIL_CLEAR fázis
  if (g_push_phase == PushPhase::WAIT_TAIL_CLEAR) {
    // 1=foglalt -> clear, ha 0
    bool tail_clear = !di_ret_tail_occ.stable;

    if (t_tail_clear_wait.running && t_tail_clear_wait.expired(g_now)) {
  cmd_push_motor = false;
  setFault(Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT);
  return;
}

    if (tail_clear) {
      t_tail_clear_wait.stop();
      cmd_push_motor = true;
      g_push_phase = PushPhase::RETURN;
    } else {
      cmd_push_motor = false;
    }
  }

  // teljes ciklus timeout
  if (t_push_cycle.running && t_push_cycle.expired(g_now)) {
  cmd_push_motor = false;
  setFault(Fault::E302_PUSH_HOME_TIMEOUT);
  return;
}

  // Ciklus vége
  if (push_left_home && di_push_home.rise()) {
    stopPushCycle();
    table_latched = false;
  }
}

/* ============================================================
   13) Return conveyor felügyelet
   ============================================================ */
static void superviseReturnConveyorFaults() {

  if (di_ret_end.stable) {

    if (!t_ret_end_soft.running) t_ret_end_soft.start(g_now, T_RET_END_SOFT_MS);
    if (!t_ret_end_hard.running) t_ret_end_hard.start(g_now, T_RET_END_HARD_MS);

    if (t_ret_end_hard.expired(g_now)) {
      cmd_stop_req = true;
      setFault(Fault::E403_RETURN_END_BLOCKED_HARD);
    }

  } else {
    t_ret_end_soft.stop();
    t_ret_end_hard.stop();
  }

  bool should_run = cmd_ret_conv && !g_fault_latched && estopOk();

  if (should_run && di_ret_start.stable) {

    if (!t_ret_start_clear.running) t_ret_start_clear.start(g_now, T_RET_START_CLEAR_MAX_MS);

    if (t_ret_start_clear.expired(g_now)) {
      setFault(Fault::E401_RETURN_START_STUCK);
    }

  } else {
    t_ret_start_clear.stop();
  }
}

/* ============================================================
   14) Reset
   ============================================================ */
static void doReset() {
  g_fault = Fault::NONE;
  g_fault_latched = false;
  g_state = State::IDLE;

  g_fault_first = Fault::NONE;
  g_fault_last  = Fault::NONE;
  g_fault_first_ms = 0;
  g_fault_last_ms  = 0;

  g_fault_hold_until_home = false;
  t_fault_home_recover.stop();

  stopPushCycle();

  table_latched = false;
  push_active = false;
  push_left_home = false;
  push_ext_seen = false;

  t_out_block.stop();
  t_out_free.stop();
  t_danger.stop();
  t_push_ext.stop();
  t_push_cycle.stop();
  t_tail_clear_wait.stop();
  t_table_wait.stop();
  t_ret_start_clear.stop();
  t_ret_end_soft.stop();
  t_ret_end_hard.stop();

  cmd_stop_req = false;
  cmd_ret_conv = false;
  cmd_push_motor = false;
  cmd_buzzer = false;
  cmd_push_rev = false;
}

/* ============================================================
   15) AUTO state machine
   ============================================================ */
static void runStateMachineAuto() {
  if (!estopOk()) {
    setFault(Fault::E900_ESTOP);
  }

  if (resetRequested()) {
    if (resetAllowed()) {
      g_last_reset_epoch = g_now_epoch;
      doReset();
      return;
    }
  }

  if (g_fault_latched) {
    cmd_stop_req = true;
    cmd_buzzer = true;

    if (g_fault_hold_until_home && estopOk()) {
      if (t_fault_home_recover.running && t_fault_home_recover.expired(g_now)) {
        cmd_push_motor = false;
        g_fault_hold_until_home = false;
      } else {
        cmd_push_motor = true;
      }
    } else {
      cmd_push_motor = false;
    }

    cmd_ret_conv = false;
    g_state = State::ERROR;
    return;
  }

  cmd_stop_req = false;
  cmd_buzzer = false;

  if (di_table_sw.rise()) {
    table_latched = true;
  }

  static uint32_t table_absent_since = 0;
  if (!push_active) {
    if (!di_table_sw.stable) {
      if (table_absent_since == 0) table_absent_since = g_now;
      if ((uint32_t)(g_now - table_absent_since) >= 2000) {
        table_latched = false;
      }
    } else {
      table_absent_since = 0;
    }
  } else {
    table_absent_since = 0;
  }

  bool table_present = di_table_sw.stable || table_latched;

  if (!push_active) {

  bool return_blocked_at_end = returnEndSoftStopActive();

  bool normal_need_return =
      (ret_load() > 0) ||
      di_ret_start.stable ||
      di_ret_end.stable;

  bool force_empty_need_return = hmi_ret_force_empty;

  bool need_return =
      !return_blocked_at_end &&
      (normal_need_return || force_empty_need_return);

  cmd_ret_conv = need_return;
}

  bool cannot_push_now = table_present && (di_ret_start.stable || !di_push_home.stable);

  if (cannot_push_now) {
    cmd_stop_req = true;

    if (!t_table_wait.running)
      t_table_wait.start(g_now, T_TABLE_WAIT_MAX_MS);

    if (t_table_wait.expired(g_now)) {
      setFault(Fault::E502_TABLE_WAIT_TIMEOUT);
    }
  } else {
    t_table_wait.stop();
  }

  switch (g_state) {
    case State::IDLE:
      if (table_present && di_push_home.stable) {
        g_state = State::WAIT_RETURN_FREE;
      }
      break;

    case State::WAIT_RETURN_FREE:
      if (table_present && di_push_home.stable && !di_ret_start.stable) {
        startPushCycle();
        g_state = State::PUSH_ACTIVE;
      }
      break;

    case State::PUSH_ACTIVE:
      if (!push_active) {
        g_state = State::IDLE;
      }
      break;

    default:
      g_state = State::IDLE;
      break;
  }
}

/* ============================================================
   16) Mód overlay + AUTO futtatás
   ============================================================ */
static void runModeOverlayOrAuto() {
  g_mode = computeMode();

  // Startup lock
  if (g_startup_lock) {
    if (resetRequested() && resetAllowed()) {
      g_last_reset_epoch = g_now_epoch;
      g_startup_lock = false;
      doReset();
      return;
    }

    if (g_mode == OpMode::LOCAL_JOG) {
      cmd_stop_req = true;
      cmd_ret_conv = false;
      cmd_buzzer   = false;

      stopPushCycle();
      push_active = false;
      g_state = State::IDLE;

      bool jog_enable = hmi_service_enable;
      bool jog_btn    = di_jog_btn.stable;
      bool jog_fwd    = di_jog_dir_fwd.stable;
      bool jog_rev    = di_jog_dir_rev.stable;
      bool valid_dir  = (jog_fwd ^ jog_rev);

      cmd_push_motor = false;
      cmd_push_rev   = false;

      if (estopOk() && jog_enable && jog_btn && valid_dir) {
        if (jog_fwd && !di_push_ext.stable) {
          cmd_push_motor = true;   // üzemi irány
        }
        if (jog_rev && !di_push_home.stable) {
          cmd_push_rev = true;     // szerviz visszairány
        }
      }

      if (di_push_home.stable && di_push_ext.stable) {
        cmd_push_motor = false;
        cmd_push_rev   = false;
      }

      return;
    }

    cmd_stop_req   = true;
    cmd_ret_conv   = false;
    cmd_push_motor = false;
    cmd_push_rev   = false;
    cmd_buzzer     = false;

    stopPushCycle();
    g_state = State::IDLE;
    return;
  }

  if (resetRequested()) {
    if (resetAllowed()) {
      doReset();
      return;
    }
  }

  if (!estopOk()) {
    setFault(Fault::E900_ESTOP);
  }

  if (g_mode == OpMode::LOCAL_JOG) {
    cmd_stop_req = true;
    cmd_ret_conv = false;
    cmd_buzzer   = false;

    stopPushCycle();
    push_active = false;
    g_state = State::IDLE;

    bool jog_enable = hmi_service_enable;
    bool jog_btn    = di_jog_btn.stable;
    bool jog_fwd    = di_jog_dir_fwd.stable;
    bool jog_rev    = di_jog_dir_rev.stable;
    bool valid_dir  = (jog_fwd ^ jog_rev);

    cmd_push_motor = false;
    cmd_push_rev   = false;

    if (estopOk() && jog_enable && jog_btn && valid_dir) {
      if (jog_fwd && !di_push_ext.stable) {
        cmd_push_motor = true;   // üzemi irány
      }
      if (jog_rev && !di_push_home.stable) {
        cmd_push_rev = true;     // szerviz visszairány
      }
    }

    if (di_push_home.stable && di_push_ext.stable) {
      cmd_push_motor = false;
      cmd_push_rev   = false;
    }

    return;
  }

  if (g_mode == OpMode::HANDLE) {
    cmd_stop_req   = true;
    cmd_ret_conv   = false;
    cmd_push_motor = false;
    cmd_push_rev   = false;
    cmd_buzzer     = false;

    stopPushCycle();
    push_active = false;
    g_state = State::IDLE;
    return;
  }

  // AUTO
  if (di_push_home.stable && di_push_ext.stable) {
    setFault(Fault::E303_PUSH_SENSOR_CONFLICT);
  }

  superviseOutOpto();
  superviseDangerWindow();
  supervisePushCycle();
  runStateMachineAuto();
  superviseReturnConveyorFaults();

  if (g_fault_latched && g_fault_hold_until_home && di_push_home.stable) {
    cmd_push_motor = false;
    cmd_push_rev   = false;
    g_fault_hold_until_home = false;
    t_fault_home_recover.stop();
  }
}

/* ============================================================
   17) Kimenetek + buzzer limit + SIM státusz print
   ============================================================ */
static void applyOutputs() {
  bool stop_req = cmd_stop_req || g_fault_latched || !estopOk();
  safeWrite(PIN_DO_STOP_REQ, stop_req);

#if CLI_VFD_DIRECT_TEST
  // CLI közvetlen VFD teszt alatt a machine.cpp nem szól bele a Return VFD-be
  safeWrite(PIN_DO_RET_CONV, false);
#else
  // Return conveyor VFD vezérlés
  bool ret_run = false;
bool ret_soft_stop = returnEndSoftStopActive();

if (g_startup_lock) {
  ret_run = false;
} else if (hmi_ret_conv_continuous) {
  ret_run = !ret_soft_stop && !g_fault_latched && estopOk();
} else {
  ret_run = cmd_ret_conv && !ret_soft_stop && !g_fault_latched && estopOk();
}

  // frekvencia kérés külön paraméterből
  vfdRequestFreqHz(VfdId::RETURN, g_ret_conv_speed_hz);
  vfdRequestRun(VfdId::RETURN, ret_run);

  // opcionális debug jel
  safeWrite(PIN_DO_RET_CONV, ret_run);
#endif

  bool push_fwd = cmd_push_motor && !cmd_push_rev && !g_fault_latched && estopOk();
  bool push_rev = cmd_push_rev && !cmd_push_motor && !g_fault_latched && estopOk();

  safeWrite(PIN_DO_PUSH_MOTOR, push_fwd);
  safeWrite(PIN_DO_PUSH_REV,   push_rev);

  static bool buzzer_last_req = false;
  static uint32_t buzzer_start_ms = 0;

  bool buzzer_req = cmd_buzzer || (g_fault_latched && (g_mode == OpMode::AUTO));

  if (buzzer_req && !buzzer_last_req) {
    buzzer_start_ms = g_now;
  }
  if (!buzzer_req) {
    buzzer_start_ms = 0;
  }

  bool buzzer_allowed = false;
  if (buzzer_req) {
    if (buzzer_start_ms == 0) buzzer_start_ms = g_now;
    buzzer_allowed = (uint32_t)(g_now - buzzer_start_ms) < T_BUZZER_MAX_MS;
  }

  buzzer_last_req = buzzer_req;
  safeWrite(PIN_DO_BUZZER, buzzer_allowed);

  #if DEBUG_VFD_MACHINE
// ------------------------------------------------------------
// Debug print
// ------------------------------------------------------------
static uint32_t lastPrint = 0;
if ((uint32_t)(g_now - lastPrint) >= 2000) {
  lastPrint = g_now;

  VfdState retSt = vfdGetState(VfdId::RETURN);
  bool retRunReq = ret_run;

  Serial.print("ST="); Serial.print((uint16_t)g_state);
  Serial.print(" MD="); Serial.print((uint16_t)g_mode);
  Serial.print(" FLT="); Serial.print((uint16_t)g_fault);
  Serial.print(" L="); Serial.print(g_fault_latched);

  Serial.print(" CONT=");
Serial.print(hmi_ret_conv_continuous);

Serial.print(" CMD_RET=");
Serial.print(cmd_ret_conv);

Serial.print(" FLT_L=");
Serial.print(g_fault_latched);

Serial.print(" RET_RUN=");
Serial.print(ret_run);

Serial.print(" RET_HZ=");
Serial.print(g_ret_conv_speed_hz);

Serial.print(" VFD_RUN_FB=");
Serial.print(retSt.running);

Serial.print(" VFD_FAULT=");
Serial.print(retSt.fault_code);

#if SIM_MODE
  Serial.print(" | DO stop="); Serial.print(g_sim_do[PIN_DO_STOP_REQ]);
  Serial.print(" ret="); Serial.print(g_sim_do[PIN_DO_RET_CONV]);
  Serial.print(" push="); Serial.print(g_sim_do[PIN_DO_PUSH_MOTOR]);
  Serial.print(" buz="); Serial.print(g_sim_do[PIN_DO_BUZZER]);
#endif

  Serial.print(" | retCmd="); Serial.print(cmd_ret_conv);
  Serial.print(" retRunReq="); Serial.print(retRunReq);
  Serial.print(" retHzSet="); Serial.print(g_ret_conv_speed_hz);

  Serial.print(" | retComm="); Serial.print(retSt.comm_ok);
  Serial.print(" retRun="); Serial.print(retSt.running);
  Serial.print(" retActHz="); Serial.print(retSt.actual_hz);
  Serial.print(" retFault="); Serial.print(retSt.fault_code);

  Serial.println();
}
#endif
}

/* ============================================================
   18) Publikus API
   ============================================================ */
void machineSetHmiHandleMode(bool v)
{
  hmi_handle_mode = v;
}

void machineSetHmiServiceEnable(bool v)
{
  hmi_service_enable = v;
}

void machinePulseHmiReset()
{
  hmi_reset_cmd = true;
}

void machineSetReturnForceEmpty(bool v)
{
  hmi_ret_force_empty = v;
}

bool machineGetReturnForceEmpty()
{
  return hmi_ret_force_empty;
}

void machineSetReturnConveyorSpeedHz(float hz)
{
  if (hz < 0.0f) hz = 0.0f;
  g_ret_conv_speed_hz = hz;
}

float machineGetReturnConveyorSpeedHz()
{
  return g_ret_conv_speed_hz;
}

void machineSetReturnConveyorContinuous(bool v)
{
  hmi_ret_conv_continuous = v;
}

bool machineGetReturnConveyorContinuous()
{
  return hmi_ret_conv_continuous;
}

OpMode machineGetOpMode()
{
  return g_mode;
}

State machineGetState()
{
  return g_state;
}

Fault machineGetFault()
{
  return g_fault;
}

bool machineIsFaultLatched()
{
  return g_fault_latched;
}

bool machineIsResetRequestActive()
{
  return hmi_reset_cmd;
}

bool machineGetHandleMode()
{
  return hmi_handle_mode;
}

bool machineGetServiceEnable()
{
  return hmi_service_enable;
}

bool machineIsPushActive()
{
  return push_active;
}

bool machineIsReturnConveyorRunning()
{
#if USE_VFD_RETURN
  VfdStatus retSt = vfdGetStatus(VfdId::RETURN);
  return retSt.running;
#else
  bool ret_soft_stop = returnEndSoftStopActive();

  if (g_startup_lock) return false;

  if (hmi_ret_conv_continuous) {
    return !ret_soft_stop && !g_fault_latched && estopOk();
  }

  return cmd_ret_conv && !ret_soft_stop && !g_fault_latched && estopOk();
#endif
}

bool machineIsWashStopRequested()
{
  return cmd_stop_req;
}

float machineGetReturnActualHz()
{
#if USE_VFD_RETURN
  VfdStatus retSt = vfdGetStatus(VfdId::RETURN);
  return retSt.actual_hz;
#else
  return 0.0f;
#endif
}

void machineSetupDefaults()
{
  g_state = State::IDLE;
  g_fault = Fault::NONE;
  g_fault_latched = false;

  g_startup_lock = true;

  g_fault_first = Fault::NONE;
  g_fault_last = Fault::NONE;
  g_fault_first_ms = 0;
  g_fault_last_ms = 0;

  g_now_epoch = 0;
  g_last_reset_epoch = 0;

  g_in_count = 0;
  g_out_count = 0;
  g_ret_in_count = 0;
  g_ret_out_count = 0;

  push_active = false;
  push_left_home = false;
  push_ext_seen = false;
  table_latched = false;

  g_mode = OpMode::AUTO;
  g_push_phase = PushPhase::IDLE;

  g_fault_hold_until_home = false;

  cmd_stop_req = false;
  cmd_ret_conv = false;
  cmd_push_motor = false;
  cmd_buzzer = false;

  g_ret_conv_speed_hz = 20.0f;
  hmi_ret_conv_continuous = false;

  hmi_handle_mode = false;
  hmi_service_enable = false;
  hmi_reset_cmd = false;
  hmi_ret_force_empty = false;

  s_prev_hmi_ret_force_empty = false;

  t_out_block.stop();
  t_out_free.stop();
  t_danger.stop();
  t_push_ext.stop();
  t_push_cycle.stop();
  t_tail_clear_wait.stop();
  t_table_wait.stop();
  t_ret_start_clear.stop();
  t_ret_end_soft.stop();
  t_ret_end_hard.stop();
  t_ret_ghost.stop();
  t_fault_home_recover.stop();
  
}

void machineLoop()
{
  g_now = millis();

  updateInputs();
  updateHmiCommands();
  handleReturnForceEmptyEdge();

  
  //   Ideiglenes kiíratások tesztelés céljából   //

  static uint32_t lastPrint = 0;

if (millis() - lastPrint > 2000) {
  lastPrint = millis();

  Serial.print("HOME=");
  Serial.print(di_push_home.stable);

  Serial.print(" RESET=");
  Serial.print(di_reset_phys.stable);

  Serial.print(" RESET_RISE=");
  Serial.print(di_reset_phys.rise());

  Serial.print(" startup_lock=");
  Serial.print(g_startup_lock);

  Serial.print(" resetReq=");
  Serial.print(resetRequested());

  Serial.print(" resetAllowed=");
  Serial.println(resetAllowed());
}

updateCounters();
superviseReturnGhostLoad();

runModeOverlayOrAuto();

applyOutputs();
}