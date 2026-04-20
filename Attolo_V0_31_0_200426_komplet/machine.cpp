#include <Arduino.h>
#include <EEPROM.h>
#include "types.h"
#include "io.h"
#include "machine.h"
#include "sim.h"
#include "vfd.h"
#include "trh.h"
#include "hmi.h"

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
static bool hmi_handle_mode     = false;
static bool hmi_service_enable  = false;
static bool hmi_reset_cmd       = false;
static bool hmi_ret_force_empty = false;

/* ============================================================
   3) Globális állapotok
   ============================================================ */
static uint32_t g_now = 0;

static State g_state = State::IDLE;
static Fault g_fault = Fault::NONE;
static bool  g_fault_latched = false;

// ------------------------------------------------------------
// Fan1 damper állapot
// ------------------------------------------------------------
static float g_fan1_damper_cmd_pct = 0.0f;   // 0..100
static float g_fan1_damper_fb_pct  = 0.0f;   // 0..100
static bool  g_fan1_run_permitted  = false;

// nyit/zár felügyelet
static Timer t_fan1_damper_open_timeout;
static Timer t_fan1_damper_close_timeout;

// ez az "üzemi igény" a Fan1-re
static bool g_fan1_should_run = false;

static float g_fan1_command_hz = 0.0f;
static float g_fan2_command_hz = 0.0f;

// ------------------------------------------------------------
// AUTO klíma teszt override
// ------------------------------------------------------------
static bool  g_clima_test_override = false;

static bool  g_test_trh1_valid = true;
static bool  g_test_trh2_valid = true;
static bool  g_test_trh3_valid = true;

static float g_test_trh1_tempC = 25.0f;
static float g_test_trh1_rh    = 50.0f;

static float g_test_trh2_tempC = 25.0f;
static float g_test_trh2_rh    = 50.0f;

static float g_test_trh3_tempC = 25.0f;
static float g_test_trh3_rh    = 50.0f;

static uint32_t g_reset_lamp_blink_ms = 0;
static bool g_reset_lamp_blink_state = false;

static uint32_t clampU32(uint32_t v, uint32_t vmin, uint32_t vmax)
{
  if (v < vmin) return vmin;
  if (v > vmax) return vmax;
  return v;
}

static float clampFloat(float v, float vmin, float vmax)
{
  if (v < vmin) return vmin;
  if (v > vmax) return vmax;
  return v;
}

static float lerpClamped(float x, float x0, float x1, float y0, float y1)
{
  if (x <= x0) return y0;
  if (x >= x1) return y1;

  const float t = (x - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

static float humidityDemandPct(float rh)
{
  // RH <= 55 -> 0%
  // RH = 65  -> 40%
  // RH = 75  -> 75%
  // RH >= 85 -> 100%

  if (rh <= 55.0f) return 0.0f;
  if (rh <= 65.0f) return lerpClamped(rh, 55.0f, 65.0f, 0.0f, 40.0f);
  if (rh <= 75.0f) return lerpClamped(rh, 65.0f, 75.0f, 40.0f, 75.0f);
  if (rh <= 85.0f) return lerpClamped(rh, 75.0f, 85.0f, 75.0f, 100.0f);
  return 100.0f;
}

static float temperatureDemandPct(float tc)
{
  // T <= 28 -> 0%
  // T = 32  -> 35%
  // T = 36  -> 70%
  // T >= 40 -> 100%

  if (tc <= 28.0f) return 0.0f;
  if (tc <= 32.0f) return lerpClamped(tc, 28.0f, 32.0f, 0.0f, 35.0f);
  if (tc <= 36.0f) return lerpClamped(tc, 32.0f, 36.0f, 35.0f, 70.0f);
  if (tc <= 40.0f) return lerpClamped(tc, 36.0f, 40.0f, 70.0f, 100.0f);
  return 100.0f;
}

struct ClimateInput {
  bool  trh1_ok;
  bool  trh2_ok;
  bool  trh3_ok;

  float trh1_tempC;
  float trh1_rh;

  float trh2_tempC;
  float trh2_rh;

  float trh3_tempC;
  float trh3_rh;
};

static ClimateInput getClimateInput()
{
  ClimateInput in{};

  if (g_clima_test_override) {
    in.trh1_ok = g_test_trh1_valid;
    in.trh2_ok = g_test_trh2_valid;
    in.trh3_ok = g_test_trh3_valid;

    in.trh1_tempC = g_test_trh1_tempC;
    in.trh1_rh    = g_test_trh1_rh;

    in.trh2_tempC = g_test_trh2_tempC;
    in.trh2_rh    = g_test_trh2_rh;

    in.trh3_tempC = g_test_trh3_tempC;
    in.trh3_rh    = g_test_trh3_rh;

    return in;
  }

  const MachineParams& p = machineGetParams();

  const TrhState& trh1 = trhGet(TrhId::TRH1);
  const TrhState& trh2 = trhGet(TrhId::TRH2);
  const TrhState& trh3 = trhGet(TrhId::TRH3);

  in.trh1_ok = trh1.valid && trhIsFresh(TrhId::TRH1, p.trh_fresh_timeout_ms);
  in.trh2_ok = trh2.valid && trhIsFresh(TrhId::TRH2, p.trh_fresh_timeout_ms);
  in.trh3_ok = trh3.valid && trhIsFresh(TrhId::TRH3, p.trh_fresh_timeout_ms);

  in.trh1_tempC = trh1.tempC;
  in.trh1_rh    = trh1.humRH;

  in.trh2_tempC = trh2.tempC;
  in.trh2_rh    = trh2.humRH;

  in.trh3_tempC = trh3.tempC;
  in.trh3_rh    = trh3.humRH;

  return in;
}

static bool  g_climate_auto_run = false;
static Timer t_climate_min_run;

static Timer t_climate_enable_off_delay;
static Timer t_fan1_damper_hold;

static bool g_fan1_keep_open = false;

// fix idők – később HMI-re is kitehető, de most egyszerűen maradjon itt
static constexpr uint32_t CLIMATE_ENABLE_OFF_DELAY_MS = 120000UL; // 120 s
static constexpr uint32_t FAN1_DAMPER_HOLD_MS         = 120000UL;  // 90 s

static bool climateEnableEffective();

static void applyAutoClimate()
{
  const MachineParams& p = machineGetParams();
  ClimateInput in = getClimateInput();

  const bool climate_enable_ok = climateEnableEffective();

  float hum_demand  = 0.0f;
  float temp_demand = 0.0f;

  // Javasolt: hő TRH1-ről
  if (in.trh2_ok) hum_demand  = humidityDemandPct(in.trh2_rh);
  if (in.trh1_ok) temp_demand = temperatureDemandPct(in.trh1_tempC);

  float main_demand = (hum_demand > temp_demand) ? hum_demand : temp_demand;

float climate_on_demand_pct  = p.climate_on_demand_pct_x10 / 10.0f;
float climate_off_demand_pct = p.climate_off_demand_pct_x10 / 10.0f;
uint32_t climate_min_run_ms  = (uint32_t)p.climate_min_run_sec * 1000UL;

float fan2_auto_min_hz = p.fan2_auto_min_hz_x10 / 10.0f;
float fan2_auto_max_hz = p.fan2_auto_max_hz_x10 / 10.0f;
float fan1_auto_min_hz = p.fan1_auto_min_hz_x10 / 10.0f;

float hot_offset_temp_c = p.hot_offset_temp_c_x10 / 10.0f;
float hot_force_temp_c  = p.hot_force_temp_c_x10 / 10.0f;
float hot_force_fan2_min_hz = p.hot_force_fan2_min_hz_x10 / 10.0f;
float hot_force_fan1_min_hz = p.hot_force_fan1_min_hz_x10 / 10.0f;

  // 1) Hiszterézis: run állapot eldöntése
  if (!g_climate_auto_run) {
  if (main_demand >= climate_on_demand_pct) {
    g_climate_auto_run = true;
    t_climate_min_run.start(g_now, climate_min_run_ms);
  }
} else {
  if (t_climate_min_run.expired(g_now)) {
    if (main_demand <= climate_off_demand_pct) {
      g_climate_auto_run = false;
      t_climate_min_run.stop();
    }
  }
}

  float fan2_hz = 0.0f;
  float fan1_hz = 0.0f;

  // FAN2 = master elszívás
  if (!g_climate_auto_run) {
    fan2_hz = 0.0f;
  } else {
    fan2_hz = fan2_auto_min_hz +
          (main_demand / 100.0f) * (fan2_auto_max_hz - fan2_auto_min_hz);
  }

  // Enyhe alulnyomás
  float offset_hz = (float)p.negative_pressure_offset_x10 / 10.0f;
  if (in.trh1_ok && in.trh1_tempC > hot_offset_temp_c) {
  if (offset_hz < 4.0f) offset_hz = 4.0f;
}

  // FAN1 = követő befúvás
  if (fan2_hz <= 0.1f) {
    fan1_hz = 0.0f;
  } else {
    fan1_hz = fan2_hz - offset_hz;
    if (fan1_hz < fan1_auto_min_hz) fan1_hz = fan1_auto_min_hz;
  }

  // Kényszerített hővédelem
  if (in.trh1_ok && in.trh1_tempC >= hot_force_temp_c) {
  if (fan2_hz < hot_force_fan2_min_hz) fan2_hz = hot_force_fan2_min_hz;
  if (fan1_hz < hot_force_fan1_min_hz) fan1_hz = hot_force_fan1_min_hz;
  g_climate_auto_run = true;
}

  // Fallback, ha mindkét fő szenzor kiesik
  if ((!in.trh2_ok) && (!in.trh1_ok)) {
    g_climate_auto_run = true;
    if (!t_climate_min_run.running) {
      t_climate_min_run.start(g_now, climate_min_run_ms);
    }
fan2_hz = fan2_auto_min_hz;
fan1_hz = fan1_auto_min_hz;
  }

  if (!g_climate_auto_run) {
  t_climate_min_run.stop();
}

fan2_hz = clampFloat(fan2_hz, 0.0f, 80.0f);
fan1_hz = clampFloat(fan1_hz, 0.0f, 80.0f);

// külső enable gate + utófutás
if (!climate_enable_ok) {
  g_climate_auto_run = false;
  t_climate_min_run.stop();

  fan2_hz = 0.0f;
  fan1_hz = 0.0f;
}

// damper hold
if (fan1_hz > 0.1f) {
  t_fan1_damper_hold.start(g_now, FAN1_DAMPER_HOLD_MS);
  g_fan1_keep_open = true;
} else {
  if (t_fan1_damper_hold.running && !t_fan1_damper_hold.expired(g_now)) {
    g_fan1_keep_open = true;
  } else {
    t_fan1_damper_hold.stop();
    g_fan1_keep_open = false;
  }
}

g_fan1_should_run = (fan1_hz > 0.1f);

g_fan1_command_hz = fan1_hz;
g_fan2_command_hz = fan2_hz;

vfdRequestFreqHz(VfdId::FAN2, fan2_hz);
vfdRequestFreqHz(VfdId::FAN1, fan1_hz);

vfdRequestRun(VfdId::FAN2, fan2_hz > 0.1f);
vfdRequestRun(VfdId::FAN1, (fan1_hz > 0.1f) && g_fan1_run_permitted);
}

static MachineParams machineParamsDefaults()
{
  MachineParams p{};

  p.debounce_ms = 50;

  p.free_min_ms = 5000;
  p.block_max_ms = 6000;

#if SIM_MODE
  p.push_ext_max_ms = 15000;
  p.push_cycle_max_ms = 35000;
  p.tail_clear_max_ms = 10000;
#else
  p.push_ext_max_ms = 9000;
  p.push_cycle_max_ms = 22000;
  p.tail_clear_max_ms = 5000;
#endif

  p.ret_start_clear_max_ms = 10000;
  p.ret_end_soft_ms = 5000;
  p.ret_end_hard_ms = 25000;

  p.danger_time_ms = 4500;
  p.danger_margin_ms = 500;

  p.table_wait_max_ms = 30000;
  p.buzzer_max_ms = 30000;

  p.ret_ghost_timeout_ms = 120000;
  p.fan1_damper_move_timeout_ms = 20000;
  p.trh_fresh_timeout_ms = 10000;
  p.negative_pressure_offset_x10 = 20;   // 2.0 Hz
  p.semafor_pulses = 5;

  p.climate_on_demand_pct_x10 = 80;
  p.climate_off_demand_pct_x10 = 30;
  p.climate_min_run_sec = 60;

  p.fan2_auto_min_hz_x10 = 190;
  p.fan2_auto_max_hz_x10 = 450;
  p.fan1_auto_min_hz_x10 = 100;

  p.hot_offset_temp_c_x10 = 300;
  p.hot_force_temp_c_x10 = 420;
  p.hot_force_fan2_min_hz_x10 = 400;
  p.hot_force_fan1_min_hz_x10 = 360;

  return p;
}

static void clampMachineParams(MachineParams& p)
{
  p.debounce_ms = (uint16_t)clampU32(p.debounce_ms, 20, 500);

  p.free_min_ms = clampU32(p.free_min_ms, 100, 5000);
  p.block_max_ms = clampU32(p.block_max_ms, 500, 90000);

  p.push_ext_max_ms = clampU32(p.push_ext_max_ms, 1000, 30000);
  p.push_cycle_max_ms = clampU32(p.push_cycle_max_ms, 2000, 60000);
  p.tail_clear_max_ms = clampU32(p.tail_clear_max_ms, 500, 20000);

  p.ret_start_clear_max_ms = clampU32(p.ret_start_clear_max_ms, 500, 30000);
  p.ret_end_soft_ms = clampU32(p.ret_end_soft_ms, 500, 30000);
  p.ret_end_hard_ms = clampU32(p.ret_end_hard_ms, 1000, 60000);

  p.danger_time_ms = clampU32(p.danger_time_ms, 500, 15000);
  p.danger_margin_ms = clampU32(p.danger_margin_ms, 0, 5000);

  p.table_wait_max_ms = clampU32(p.table_wait_max_ms, 1000, 60000);
  p.buzzer_max_ms = clampU32(p.buzzer_max_ms, 1000, 60000);

  p.ret_ghost_timeout_ms = clampU32(p.ret_ghost_timeout_ms, 10000, 600000);
  p.fan1_damper_move_timeout_ms = clampU32(p.fan1_damper_move_timeout_ms, 5000, 300000);
  p.trh_fresh_timeout_ms = clampU32(p.trh_fresh_timeout_ms, 2000, 60000);
  p.negative_pressure_offset_x10 =
      (int16_t)clampU32((uint32_t)p.negative_pressure_offset_x10, 0, 100);

  if (p.semafor_pulses < 1) {
  p.semafor_pulses = 10;
}

// ------------------------------------------------------------
// OUT opto hézag minimum idő védelem
// ------------------------------------------------------------
if (p.free_min_ms == 0) {
  p.free_min_ms = 5000UL;
}

  if (p.push_cycle_max_ms < p.push_ext_max_ms + 1000) {
    p.push_cycle_max_ms = p.push_ext_max_ms + 1000;
  }

  if (p.ret_end_hard_ms < p.ret_end_soft_ms + 1000) {
    p.ret_end_hard_ms = p.ret_end_soft_ms + 1000;
  }

    if (p.semafor_pulses < 1)    p.semafor_pulses = 1;
  if (p.semafor_pulses > 1000) p.semafor_pulses = 1000;

    p.climate_on_demand_pct_x10  = (uint16_t)clampU32(p.climate_on_demand_pct_x10, 0, 1000);
  p.climate_off_demand_pct_x10 = (uint16_t)clampU32(p.climate_off_demand_pct_x10, 0, 1000);
  p.climate_min_run_sec        = (uint16_t)clampU32(p.climate_min_run_sec, 0, 3600);

p.fan2_auto_min_hz_x10 = (uint16_t)clampU32(p.fan2_auto_min_hz_x10, 0, 800);
p.fan2_auto_max_hz_x10 = (uint16_t)clampU32(p.fan2_auto_max_hz_x10, 0, 800);
p.fan1_auto_min_hz_x10 = (uint16_t)clampU32(p.fan1_auto_min_hz_x10, 0, 800);
p.hot_offset_temp_c_x10      = (uint16_t)clampU32(p.hot_offset_temp_c_x10, 0, 800);
p.hot_force_temp_c_x10       = (uint16_t)clampU32(p.hot_force_temp_c_x10, 0, 800);
p.hot_force_fan2_min_hz_x10 = (uint16_t)clampU32(p.hot_force_fan2_min_hz_x10, 0, 800);
p.hot_force_fan1_min_hz_x10 = (uint16_t)clampU32(p.hot_force_fan1_min_hz_x10, 0, 800);

  if (p.climate_off_demand_pct_x10 > p.climate_on_demand_pct_x10)
    p.climate_off_demand_pct_x10 = p.climate_on_demand_pct_x10;

  if (p.fan2_auto_max_hz_x10 < p.fan2_auto_min_hz_x10)
    p.fan2_auto_max_hz_x10 = p.fan2_auto_min_hz_x10;

  if (p.hot_force_temp_c_x10 < p.hot_offset_temp_c_x10)
    p.hot_force_temp_c_x10 = p.hot_offset_temp_c_x10;
}

static MachineParams g_params = machineParamsDefaults();

static constexpr uint32_t MACHINE_EEPROM_MAGIC   = 0x534D5031UL; // "SMP1"
static constexpr uint16_t MACHINE_EEPROM_VERSION = 1;
static constexpr int MACHINE_EEPROM_ADDR         = 0;

struct MachineParamsEepromImage {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  MachineParams params;
};

bool machineLoadParamsFromEeprom()
{
  MachineParamsEepromImage img{};
  EEPROM.get(MACHINE_EEPROM_ADDR, img);

  if (img.magic != MACHINE_EEPROM_MAGIC) {
    return false;
  }

  if (img.version != MACHINE_EEPROM_VERSION) {
    return false;
  }

  g_params = img.params;
  clampMachineParams(g_params);
  return true;
}

void machineSaveParamsToEeprom()
{
  MachineParamsEepromImage img{};
  img.magic = MACHINE_EEPROM_MAGIC;
  img.version = MACHINE_EEPROM_VERSION;
  img.reserved = 0;
  img.params = g_params;

  clampMachineParams(img.params);
  EEPROM.put(MACHINE_EEPROM_ADDR, img);
}

static bool g_startup_lock = true;   // power-on után mindig 1
static bool g_ret_feed_pending = false;
static bool g_ret_start_seen_in_cycle = false;
static bool g_e305_recover_active = false;

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
int32_t machineGetReturnLoad() { return ret_load(); }

// ------------------------------------------------------------
// Szemafor logika
// ------------------------------------------------------------
static bool     g_semafor_green = false;
static bool     g_semafor_yellow = false;

static bool     g_semafor_wait_encoder = false;

static uint32_t g_encoder_count = 0;
static uint32_t g_encoder_target = 0;

// élérzékeléshez
static bool g_last_wash_in_opto = false;
static bool g_last_encoder = false;

// induláskori inicializálás + encoder szűrés
static bool     g_semafor_initialized = false;
static uint32_t g_last_encoder_edge_ms = 0;


// Fizikai bemenetek (szűrt)
static DebouncedInput di_in_opto;
static DebouncedInput di_out_opto;
static DebouncedInput di_table_sw;
static DebouncedInput di_push_home;
static DebouncedInput di_push_ext;
static DebouncedInput di_ret_start;
static DebouncedInput di_ret_end;
static DebouncedInput di_ret_tail_occ; // 1=foglalt
static DebouncedInput di_reset_main;
static DebouncedInput di_climate_enable;
static DebouncedInput di_jog_dir_fwd;
static DebouncedInput di_jog_dir_rev;
static DebouncedInput di_jog_btn;

// OUT opto időmérések
static Timer t_out_block;
static Timer t_out_free;
static uint32_t last_t_block_ms = 0;
static uint32_t last_t_free_ms  = 0;

// Danger ablak / HOME-várás az OUT optó új szerepéhez
static Timer t_danger;

// Wash stop retesz: ha OUT optó takart és nincs HOME,
// a mosó csak HOME után indulhat újra
static bool  g_wash_hold_until_home = false;

// Áttoló felügyelet
static Timer t_push_ext;
static Timer t_push_cycle;
static Timer t_ext_tail_delay;   // EXT utáni átbukási holtidő
static bool push_active = false;
static bool push_left_home = false;
static bool push_ext_seen = false;

// static constexpr uint32_t EXT_TAIL_DELAY_MS = 1200;

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

// ------------------------------------------------------------
// Fan1 damper analóg segédek
// ------------------------------------------------------------
static constexpr float FAN1_DAMPER_OPEN_PCT  = 80.0f;
static constexpr float FAN1_DAMPER_CLOSE_PCT = 20.0f;

// ---- feedback kalibráció ----
// ZÁRT állásnál mért nyers AI12 érték
static constexpr float FAN1_AI_RAW_CLOSED = 96.0f;

// NYITOTT állásnál mért nyers AI12 érték
static constexpr float FAN1_AI_RAW_OPEN   = 930.0f;

// AO1 kimenet nálad ténylegesen 0..1000 raw tartományban használható
static constexpr float FAN1_AO_RAW_MAX    = 255.0f;

static inline bool fan1DamperOpenCmd()
{
  return g_fan1_damper_cmd_pct >= FAN1_DAMPER_OPEN_PCT;
}

static inline bool fan1DamperOpenFb()
{
  return g_fan1_damper_fb_pct >= FAN1_DAMPER_OPEN_PCT;
}

static inline bool fan1DamperClosedFb()
{
  return g_fan1_damper_fb_pct <= FAN1_DAMPER_CLOSE_PCT;
}

static float readFan1DamperFeedbackVolt()
{
#if SIM_MODE
  return (g_fan1_damper_fb_pct * 10.0f) / 100.0f;
#else
  long sum = 0;
  const int N = 20;

  for (int i = 0; i < N; i++) {
    sum += analogRead(PIN_AI_FAN1_DAMPER_FB);
    delay(2);
  }

  float rawAvg = sum / (float)N;
  float fbVolt = rawAvg * 10.0f / 1023.0f;
  return fbVolt;
#endif
}

static float readFan1DamperFeedbackPct()
{
#if SIM_MODE
  return g_fan1_damper_cmd_pct;
#else
  float fbVolt = readFan1DamperFeedbackVolt();
  float pct = (fbVolt / 10.0f) * 100.0f;
  return clampFloat(pct, 0.0f, 100.0f);
#endif
}

static void writeFan1DamperCommandPct(float pct)
{
  pct = clampFloat(pct, 0.0f, 100.0f);
  g_fan1_damper_cmd_pct = pct;

#if !SIM_MODE
  int raw = (int)((pct / 100.0f) * FAN1_AO_RAW_MAX + 0.5f);
  if (raw < 0) raw = 0;
  if (raw > (int)FAN1_AO_RAW_MAX) raw = (int)FAN1_AO_RAW_MAX;
  analogWrite(PIN_AO_FAN1_DAMPER_CMD, raw);
#endif
}

static void writeFan1DamperCommandRaw(float rawCmd)
{
  rawCmd = clampFloat(rawCmd, 0.0f, FAN1_AO_RAW_MAX);

#if !SIM_MODE
  analogWrite(PIN_AO_FAN1_DAMPER_CMD, (int)(rawCmd + 0.5f));
#endif

  g_fan1_damper_cmd_pct = (rawCmd * 100.0f) / FAN1_AO_RAW_MAX;
}

// ret_conveyor leállításának segédfüggvénye, ha nem veszik le a raklapot időben
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
  g_ret_feed_pending = false;

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
  bool reset_main = di_reset_main.rise();

  bool jog_dir_valid = (di_jog_dir_fwd.stable ^ di_jog_dir_rev.stable);
  bool local_jog_active = (hmi_handle_mode &&
                           hmi_service_enable &&
                           jog_dir_valid);

  bool jog_as_reset = (!local_jog_active) && di_jog_btn.rise();

  return resetRequestedHmiRise() || reset_main || jog_as_reset;
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

  // E305 és E501 ugyanabba a "megáll, majd reset után HOME" ágba menjen
  if (f == Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT ||
      f == Fault::E501_DANGER_HOME_LATE) {
    cmd_push_motor = false;
    cmd_push_rev   = false;
    g_fault_hold_until_home = false;
    t_fault_home_recover.stop();
    return;
  }

  // Hard stop faultnál az áttoló motor azonnal álljon le
  if (isHardStopFault(f)) {
    cmd_push_motor = false;
    cmd_push_rev   = false;
    g_fault_hold_until_home = false;
    t_fault_home_recover.stop();
    return;
  }

  // Egyéb nem-hard-stop faultoknál maradhat a régi controlled return logika
  if (g_mode == OpMode::AUTO &&
      push_active &&
      !di_push_home.stable) {
    g_fault_hold_until_home = true;
    t_fault_home_recover.start(g_now, g_params.push_cycle_max_ms);
  } else {
    cmd_push_motor = false;
    cmd_push_rev   = false;
  }
}

/* ============================================================
   8) Bemenetek frissítése
   ============================================================ */
static void updateInputs() {
  di_in_opto.update(safeRead(PIN_DI_WASH_IN_OPTO), g_now, g_params.debounce_ms);
  di_out_opto.update(safeRead(PIN_DI_WASH_OUT_OPTO), g_now, g_params.debounce_ms);
  di_table_sw.update(safeRead(PIN_DI_TABLE_SW), g_now, g_params.debounce_ms);
  di_push_home.update(safeRead(PIN_DI_PUSH_HOME), g_now, g_params.debounce_ms);
  di_push_ext.update(safeRead(PIN_DI_PUSH_EXT), g_now, g_params.debounce_ms);
  di_ret_start.update(safeRead(PIN_DI_RET_START_OCC), g_now, g_params.debounce_ms);
  di_ret_end.update(safeRead(PIN_DI_RET_END_OCC), g_now, g_params.debounce_ms);
  di_ret_tail_occ.update(safeRead(PIN_DI_RET_TAIL_OCC), g_now, g_params.debounce_ms);

  di_reset_main.update(safeRead(PIN_DI_RESET_MAIN), g_now, g_params.debounce_ms);

  di_climate_enable.update(safeRead(PIN_DI_CLIMATE_ENABLE), g_now, g_params.debounce_ms);

  di_jog_dir_fwd.update(safeRead(PIN_DI_JOG_DIR_FWD), g_now, g_params.debounce_ms);
  di_jog_dir_rev.update(safeRead(PIN_DI_JOG_DIR_REV), g_now, g_params.debounce_ms);
  di_jog_btn.update(safeRead(PIN_DI_JOG_BTN), g_now, g_params.debounce_ms);
}

static bool climateEnableEffective()
{
  if (di_climate_enable.stable) {
    t_climate_enable_off_delay.stop();
    return true;
  }

  if (di_climate_enable.fall()) {
    t_climate_enable_off_delay.start(g_now, CLIMATE_ENABLE_OFF_DELAY_MS);
  }

  if (t_climate_enable_off_delay.running) {
    if (!t_climate_enable_off_delay.expired(g_now)) {
      return true;
    }
    t_climate_enable_off_delay.stop();
  }

  return false;
}

/* ============================================================
   9) Számlálók
   ============================================================ */
static void updateCounters() {
  if (di_in_opto.rise()) g_in_count++;

  // Az OUT optó új szerepben már nem kimenő raklap-számláló.
  // if (di_out_opto.rise()) g_out_count++;

  // Return belépés elfogadása:
  // ha EXT után várunk raklapot, akkor egy ret_start él VAGY már aktív stabil jel is elfogadható.
  if (g_ret_feed_pending && (di_ret_start.rise() || di_ret_start.stable)) {
  g_ret_in_count++;
  g_ret_feed_pending = false;
  g_ret_start_seen_in_cycle = true;

  // Valódi áttolásnál azonnal induljon a return
  // continuous módban is
  cmd_ret_conv = true;
}

  if (di_ret_end.rise()) {
    g_ret_out_count++;
  }
}

static void superviseReturnGhostLoad() {
  bool no_physical_pallet =
      !di_ret_start.stable &&
      !di_ret_end.stable;

  // Ha a számláló szerint még lenne raklap, de fizikailag már
  // nincs sem start, sem end jel, akkor ez ghost load helyzet.
  if (ret_load() > 0 && no_physical_pallet) {
    if (!t_ret_ghost.running) {
      t_ret_ghost.start(g_now, g_params.ret_ghost_timeout_ms);
    }

    if (t_ret_ghost.expired(g_now)) {
      resetReturnState();
      cmd_ret_conv = false;
      t_ret_ghost.stop();
    }
  } else {
    t_ret_ghost.stop();
  }
}

/* ============================================================
   10) OUT opto felügyelet
   ============================================================ */
static void superviseOutOpto()
{
  // ÚJ szerep:
  // ha az OUT optó takart és a kar még nincs HOME-ban,
  // a mosót azonnal leállítjuk, és csak HOME oldhatja fel.

  if (di_out_opto.stable && !di_push_home.stable) {
    if (!g_wash_hold_until_home) {
      g_wash_hold_until_home = true;
      t_danger.start(g_now, g_params.danger_time_ms);   // E501 timeout
    }
  }

  // Feloldás kizárólag HOME-ra
  if (di_push_home.stable) {
    g_wash_hold_until_home = false;
    t_danger.stop();
  }

  // Ha túl sokáig nincs HOME -> E501
  if (g_wash_hold_until_home &&
      !di_push_home.stable &&
      t_danger.running &&
      t_danger.expired(g_now)) {
    setFault(Fault::E501_DANGER_HOME_LATE);
  }
}

/* ============================================================
   11) Danger ablak
   ============================================================ */
static void superviseDangerWindow() {
  // Az OUT optó új szerepe miatt a régi danger-window logika már nem használatos.
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

  t_push_ext.start(g_now, g_params.push_ext_max_ms);
  t_push_cycle.start(g_now, g_params.push_cycle_max_ms);

  t_tail_clear_wait.stop();
  t_ext_tail_delay.stop();
  g_ret_start_seen_in_cycle = false;
}

static void stopPushCycle() {
  cmd_push_motor = false;
  cmd_push_rev   = false;

  push_active = false;
  push_left_home = false;
  push_ext_seen = false;
  g_ret_start_seen_in_cycle = false;

  g_push_phase = PushPhase::IDLE;

  t_push_ext.stop();
  t_push_cycle.stop();
  t_tail_clear_wait.stop();
  t_ext_tail_delay.stop();

  // Ha a ciklus végéig semmilyen return oldali bizonyíték nem jött,
  // ne maradjon bent hamis feed pending.
  if (ret_load() <= 0 &&
      !di_ret_start.stable &&
      !di_ret_end.stable) {
    g_ret_feed_pending = false;
    
  }
}

static void supervisePushCycle() {
  if (!push_active) return;

  if (di_push_home.fall()) {
    push_left_home = true;

    if (!hmi_ret_conv_continuous) {
      cmd_ret_conv = false;
    }
  }

  // ------------------------------------------------------------
  // OUT fázis
  // ------------------------------------------------------------
   if (g_push_phase == PushPhase::OUT) {
  if (di_push_ext.rise()) {
    push_ext_seen = true;
    g_ret_feed_pending = true;

    // EXT-re nem állunk meg azonnal
    t_ext_tail_delay.start(g_now, g_params.danger_margin_ms);
  }

  if (!push_ext_seen && t_push_ext.running && t_push_ext.expired(g_now)) {
    cmd_push_motor = false;
    setFault(Fault::E301_PUSH_NO_EXT);
    return;
  }

  // amíg nincs EXT, kifelé megy
  if (!push_ext_seen) {
    cmd_push_motor = true;
    return;
  }

  // EXT megvan -> a push motor alapból még megy a holtidő végéig
  if (t_ext_tail_delay.running && !t_ext_tail_delay.expired(g_now)) {
    cmd_push_motor = true;
    return;
  }

  // holtidő lejárt -> itt döntünk
  if (di_ret_tail_occ.stable) {
    cmd_push_motor = false;

    if (g_push_phase != PushPhase::WAIT_TAIL_CLEAR) {
      g_push_phase = PushPhase::WAIT_TAIL_CLEAR;
      if (!t_tail_clear_wait.running) {
        t_tail_clear_wait.start(g_now, g_params.tail_clear_max_ms);
      }
    }
    return;
  }

  // nincs tail foglaltság -> indulhat a visszaút
  g_push_phase = PushPhase::RETURN;
  t_tail_clear_wait.stop();
  cmd_push_motor = true;
  return;
}

  // ------------------------------------------------------------
  // WAIT_TAIL_CLEAR fázis
  // ------------------------------------------------------------
  if (g_push_phase == PushPhase::WAIT_TAIL_CLEAR) {

    cmd_push_motor = false;
  cmd_push_rev   = false;

  if (!t_tail_clear_wait.running) {
    t_tail_clear_wait.start(g_now, g_params.tail_clear_max_ms);
  }

  bool tail_clear = !di_ret_tail_occ.stable;

  if (tail_clear) {
    t_tail_clear_wait.stop();
    g_push_phase = PushPhase::RETURN;
    cmd_push_motor = true;
    return;
  }

  if (t_tail_clear_wait.expired(g_now)) {
    cmd_push_motor = false;
    cmd_push_rev   = false;
    setFault(Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT);
    return;
  }

  return;
}

  // ------------------------------------------------------------
  // RETURN fázis
  // ------------------------------------------------------------
  if (g_push_phase == PushPhase::RETURN) {
    cmd_push_motor = true;
  }

  // teljes ciklus timeout
  if (t_push_cycle.running && t_push_cycle.expired(g_now)) {
    cmd_push_motor = false;
    setFault(Fault::E302_PUSH_HOME_TIMEOUT);
    return;
  }

  // ciklus vége HOME-ra
if (push_left_home && di_push_home.rise()) {
  // Ha a teljes push ciklus alatt nem jött ret_start él,
  // akkor nem került raklap a returnre.
  if (!g_ret_start_seen_in_cycle) {
    g_ret_feed_pending = false;
    cmd_ret_conv = false;
  }

  stopPushCycle();
  table_latched = false;
}

}

/* ============================================================
   13) Return conveyor felügyelet
   ============================================================ */
static void superviseReturnConveyorFaults() {
  if (di_ret_end.stable) {
    if (!t_ret_end_soft.running) t_ret_end_soft.start(g_now, g_params.ret_end_soft_ms);
    if (!t_ret_end_hard.running) t_ret_end_hard.start(g_now, g_params.ret_end_hard_ms);

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
    if (!t_ret_start_clear.running) t_ret_start_clear.start(g_now, g_params.ret_start_clear_max_ms);

    if (t_ret_start_clear.expired(g_now)) {
      setFault(Fault::E401_RETURN_START_STUCK);
    }
  } else {
    t_ret_start_clear.stop();
  }
}

/* ============================================================
   13/b) Fan1 damper felügyelet
   ============================================================ */
static void superviseFan1Damper()
{
  g_fan1_damper_fb_pct = readFan1DamperFeedbackPct();

  // ============================================================
  // KÉZI KLÍMA MÓD
  // ============================================================
  if (hmiGetVentManualMode()) {
    const float cmdPct = clampFloat(hmiGetFan1DamperSetPct(), 0.0f, 100.0f);

    writeFan1DamperCommandPct(cmdPct);
    g_fan1_damper_cmd_pct = cmdPct;

    // Nyitott állapot felügyelet
    if (cmdPct > 1.0f) {
      if (!fan1DamperOpenFb()) {
        if (!t_fan1_damper_open_timeout.running) {
          t_fan1_damper_open_timeout.start(g_now, g_params.fan1_damper_move_timeout_ms);
        }

        if (t_fan1_damper_open_timeout.expired(g_now)) {
          setFault(Fault::E601_FAN1_DAMPER_NOT_OPEN);
          return;
        }
      } else {
        t_fan1_damper_open_timeout.stop();
      }

      g_fan1_run_permitted = fan1DamperOpenFb();
      t_fan1_damper_close_timeout.stop();
    }
    // Zárt állapot felügyelet
    else {
      g_fan1_run_permitted = false;

      if (!fan1DamperClosedFb()) {
        if (!t_fan1_damper_close_timeout.running) {
          t_fan1_damper_close_timeout.start(g_now, g_params.fan1_damper_move_timeout_ms);
        }

        if (t_fan1_damper_close_timeout.expired(g_now)) {
          setFault(Fault::E602_FAN1_DAMPER_NOT_CLOSED);
          return;
        }
      } else {
        t_fan1_damper_close_timeout.stop();
      }

      t_fan1_damper_open_timeout.stop();
    }

    return;
  }

  // ============================================================
// AUTO KLÍMA MÓD
// g_fan1_should_run értékét az applyAutoClimate() állítja
// ============================================================
const bool damper_should_be_open = g_fan1_should_run || g_fan1_keep_open;

if (damper_should_be_open) {
  writeFan1DamperCommandPct(100.0f);
  g_fan1_damper_cmd_pct = 100.0f;

  if (!fan1DamperOpenFb()) {
    if (!t_fan1_damper_open_timeout.running) {
      t_fan1_damper_open_timeout.start(g_now, g_params.fan1_damper_move_timeout_ms);
    }

    if (t_fan1_damper_open_timeout.expired(g_now)) {
      setFault(Fault::E601_FAN1_DAMPER_NOT_OPEN);
      return;
    }
  } else {
    t_fan1_damper_open_timeout.stop();
  }

  g_fan1_run_permitted = fan1DamperOpenFb();
  t_fan1_damper_close_timeout.stop();
}
else {
  g_fan1_run_permitted = false;

  writeFan1DamperCommandPct(0.0f);
  g_fan1_damper_cmd_pct = 0.0f;

  if (!fan1DamperClosedFb()) {
    if (!t_fan1_damper_close_timeout.running) {
      t_fan1_damper_close_timeout.start(g_now, g_params.fan1_damper_move_timeout_ms);
    }

    if (t_fan1_damper_close_timeout.expired(g_now)) {
      setFault(Fault::E602_FAN1_DAMPER_NOT_CLOSED);
      return;
    }
  } else {
    t_fan1_damper_close_timeout.stop();
  }

  t_fan1_damper_open_timeout.stop();
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
  g_e305_recover_active = false;
  t_fault_home_recover.stop();

  stopPushCycle();

  table_latched = false;
  push_active = false;
  push_left_home = false;
  push_ext_seen = false;
  g_ret_feed_pending = false;
  g_ret_start_seen_in_cycle = false;

  t_out_block.stop();
  t_out_free.stop();
  t_danger.stop();
  t_push_ext.stop();
  t_push_cycle.stop();
  t_tail_clear_wait.stop();
  t_ext_tail_delay.stop();
  t_table_wait.stop();

  g_wash_hold_until_home = false;

  t_ret_start_clear.stop();
  t_ret_end_soft.stop();
  t_ret_end_hard.stop();
  
  cmd_stop_req = false;
  cmd_ret_conv = false;
  cmd_push_motor = false;
  cmd_buzzer = false;
  cmd_push_rev = false;

  g_fan1_damper_cmd_pct = 0.0f;
g_fan1_damper_fb_pct  = 0.0f;
g_fan1_run_permitted  = false;
g_fan1_should_run     = false;
g_fan1_keep_open      = false;
g_fan1_command_hz     = 0.0f;
g_fan2_command_hz     = 0.0f;

t_fan1_damper_open_timeout.stop();
t_fan1_damper_close_timeout.stop();
t_fan1_damper_hold.stop();
t_climate_enable_off_delay.stop();

#if !SIM_MODE
  analogWrite(PIN_AO_FAN1_DAMPER_CMD, 0);
#endif
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

    // E305 + E501 közös reset utáni recovery
    const bool can_recover_home =
        g_fault_latched &&
        !di_push_home.stable &&
        (
          (g_fault == Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT && !di_ret_tail_occ.stable) ||
          (g_fault == Fault::E501_DANGER_HOME_LATE)
        );

    if (can_recover_home) {
      g_e305_recover_active = true;
      g_fault_hold_until_home = false;
      cmd_push_motor = false;
      cmd_push_rev   = false;
      t_fault_home_recover.start(g_now, g_params.push_cycle_max_ms);
      return;
    }
  }

  if (g_fault_latched) {
    cmd_stop_req = true;
    cmd_buzzer = true;
    cmd_ret_conv = false;
    g_state = State::ERROR;

    // Fault alatti LOCAL_JOG recovery
    if (g_mode == OpMode::LOCAL_JOG && estopOk()) {
      bool jog_enable = hmi_service_enable;
      bool jog_btn    = di_jog_btn.stable;
      bool jog_fwd    = di_jog_dir_fwd.stable;
      bool jog_rev    = di_jog_dir_rev.stable;
      bool valid_dir  = (jog_fwd ^ jog_rev);

      cmd_push_motor = false;
      cmd_push_rev   = false;

      if (jog_enable && jog_btn && valid_dir) {
        if (jog_fwd) cmd_push_motor = true;
        if (jog_rev) cmd_push_rev   = true;
      }

      return;
    }

    // E305 + E501 közös HOME recovery
    if (g_e305_recover_active &&
        (g_fault == Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT ||
         g_fault == Fault::E501_DANGER_HOME_LATE) &&
        estopOk()) {

      if (g_fault == Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT && di_ret_tail_occ.stable) {
        cmd_push_motor = false;
        cmd_push_rev   = false;
        return;
      }

      if (t_fault_home_recover.running && t_fault_home_recover.expired(g_now)) {
        cmd_push_motor = false;
        cmd_push_rev   = false;
        g_e305_recover_active = false;
        return;
      }

      if (di_push_home.stable) {
        cmd_push_motor = false;
        cmd_push_rev   = false;
        g_e305_recover_active = false;
        doReset();
        g_ret_feed_pending = true;
        return;
      }

      cmd_push_motor = true;
      cmd_push_rev   = false;
      return;
    }

    // régi controlled return logika más, nem-hard-stop faultokra
    if (g_fault_hold_until_home && estopOk()) {
      if (t_fault_home_recover.running && t_fault_home_recover.expired(g_now)) {
        cmd_push_motor = false;
        g_fault_hold_until_home = false;
      } else {
        cmd_push_motor = true;
      }
    } else {
      cmd_push_motor = false;
      cmd_push_rev   = false;
    }

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
        di_ret_end.stable;

    bool force_empty_need_return = hmi_ret_force_empty;

    bool need_return =
        !return_blocked_at_end &&
        (normal_need_return || force_empty_need_return);

    cmd_ret_conv = need_return;
  }

  bool cannot_push_now = table_present && !push_active &&
                         (di_ret_start.stable || !di_push_home.stable);

  if (cannot_push_now) {
    cmd_stop_req = true;

    if (!t_table_wait.running)
      t_table_wait.start(g_now, g_params.table_wait_max_ms);

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
        if (jog_fwd) cmd_push_motor = true;
        if (jog_rev) cmd_push_rev   = true;
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

  // Normál reset (csak HOME-ban engedett)
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
      if (jog_fwd) cmd_push_motor = true;
      if (jog_rev) cmd_push_rev   = true;
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

  // AUTO / KLÍMA
  if (di_push_home.stable && di_push_ext.stable) {
    setFault(Fault::E303_PUSH_SENSOR_CONFLICT);
  }

  superviseOutOpto();
  superviseDangerWindow();
  supervisePushCycle();
  runStateMachineAuto();
Serial.print("[AFTER AUTO] cmd_push_motor=");
Serial.print(cmd_push_motor);
Serial.print(" cmd_push_rev=");
Serial.println(cmd_push_rev);

  superviseReturnConveyorFaults();
  applyAutoClimate();
  superviseFan1Damper();
}

/* ============================================================
   17) Kimenetek + buzzer limit + SIM státusz print
   ============================================================ */
static void applyOutputs() {
  bool stop_req = cmd_stop_req || g_fault_latched || g_wash_hold_until_home || !estopOk();
  safeWrite(PIN_STOP_REQ, stop_req);

  bool reset_required = g_startup_lock || g_fault_latched;

  if ((uint32_t)(g_now - g_reset_lamp_blink_ms) >= 500) {
    g_reset_lamp_blink_ms = g_now;
    g_reset_lamp_blink_state = !g_reset_lamp_blink_state;
  }

  bool reset_light_cmd = reset_required ? g_reset_lamp_blink_state : true;

  safeWrite(PIN_DO_GREEN_LIGHT, reset_light_cmd);
  safeWrite(PIN_DO_JOG_LIGHT,   reset_light_cmd);

#if CLI_VFD_DIRECT_TEST
  // CLI közvetlen VFD teszt alatt a machine.cpp nem szól bele a Return VFD-be
  safeWrite(PIN_DO_RET_CONV, false);
  bool ret_run = false;
#else
  // Return conveyor VFD vezérlés
  bool ret_run = false;
  bool ret_soft_stop = returnEndSoftStopActive();

  if (g_startup_lock) {
  ret_run = false;
} else {
  ret_run = cmd_ret_conv && !ret_soft_stop && !g_fault_latched && estopOk();
}

  // frekvencia kérés külön paraméterből
  vfdRequestFreqHz(VfdId::RETURN, g_ret_conv_speed_hz);
  vfdRequestRun(VfdId::RETURN, ret_run);

  // ------------------------------------------------------------
  // Klíma kézi mód: FAN1 / FAN2 közvetlen VFD vezérlés HMI-ről
  // OFF = AUTO, ON = MANUAL
  // ------------------------------------------------------------
  if (hmiGetVentManualMode()) {
    bool fan1Run = hmiGetFan1ManualRun();
    bool fan2Run = hmiGetFan2ManualRun();

    float fan1Hz = hmiGetFan1ManualFreqHz();
    float fan2Hz = hmiGetFan2ManualFreqHz();

    if (fan1Hz < 0.0f) fan1Hz = 0.0f;
    if (fan2Hz < 0.0f) fan2Hz = 0.0f;

    if (fan1Hz > 50.0f) fan1Hz = 50.0f;
    if (fan2Hz > 50.0f) fan2Hz = 50.0f;

    bool fan1RunFinal = fan1Run && g_fan1_run_permitted;

    vfdRequestFreqHz(VfdId::FAN1, fan1Hz);
    vfdRequestRun(VfdId::FAN1, fan1RunFinal);

    vfdRequestFreqHz(VfdId::FAN2, fan2Hz);
    vfdRequestRun(VfdId::FAN2, fan2Run);
  } 

  // opcionális debug jel
  //safeWrite(PIN_DO_RET_CONV, ret_run);
#endif

  // Fan1 damper AO kimenet frissítése minden ciklusban
  writeFan1DamperCommandPct(g_fan1_damper_cmd_pct);

  bool allow_e305_forward_recovery =
  g_e305_recover_active &&
  g_fault_latched &&
  g_fault == Fault::E305_PUSH_TAIL_CLEAR_TIMEOUT;

bool allow_fault_local_jog =
  g_fault_latched &&
  g_mode == OpMode::LOCAL_JOG &&
  estopOk();

// 305 alatt NEM engedjük a normál előremenetet.
// Előremenet csak akkor mehet fault alatt, ha kifejezetten
// az E305 recovery aktív.
bool push_fwd =
  !cmd_push_rev &&
  estopOk() &&
  (
    (cmd_push_motor && !g_fault_latched) ||
    allow_e305_forward_recovery ||
    allow_fault_local_jog
  );

bool push_rev =
  cmd_push_rev &&
  !cmd_push_motor &&
  estopOk() &&
  (
    !g_fault_latched ||
    allow_fault_local_jog
  );

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
    buzzer_allowed = (uint32_t)(g_now - buzzer_start_ms) < g_params.buzzer_max_ms;
  }

  buzzer_last_req = buzzer_req;
  safeWrite(PIN_DO_BUZZER, buzzer_allowed);

  // ------------------------------------------------------------
  // Szemafor kimenetek
  // ------------------------------------------------------------
  safeWrite(PIN_DO_SEMAFOR_GREEN,  g_semafor_green);
  safeWrite(PIN_DO_SEMAFOR_YELLOW, g_semafor_yellow);

#if DEBUG_VFD_MACHINE
  // ------------------------------------------------------------
  // Debug print
  // ------------------------------------------------------------
  static uint32_t lastPrint = 0;
  if ((uint32_t)(g_now - lastPrint) >= 2000) {
    lastPrint = g_now;

    VfdState retSt = vfdGetState(VfdId::RETURN);
    VfdState fan1St = vfdGetState(VfdId::FAN1);
    bool retRunReq = ret_run;

    Serial.print("ST="); Serial.print((uint16_t)g_state);
    Serial.print(" MD="); Serial.print((uint16_t)g_mode);
    Serial.print(" FLT="); Serial.print((uint16_t)g_fault);
    Serial.print(" L="); Serial.print(g_fault_latched);

    Serial.print(" CONT=");
    Serial.print(hmi_ret_conv_continuous);

    Serial.print(" CMD_RET=");
    Serial.print(cmd_ret_conv);
    Serial.print(" ret_load=");
    Serial.print(ret_load());

    Serial.print(" RET_START=");
    Serial.print(di_ret_start.stable);

    Serial.print(" RET_END=");
    Serial.print(di_ret_end.stable);

    Serial.print(" FLT_L=");
    Serial.print(g_fault_latched);

    Serial.print(" E305_REC=");
    Serial.print(g_e305_recover_active);

    Serial.print(" RET_RUN=");
    Serial.print(ret_run);

    Serial.print(" RET_HZ=");
    Serial.print(g_ret_conv_speed_hz);

    Serial.print(" VFD_RUN_FB=");
    Serial.print(retSt.running);

    Serial.print(" VFD_FAULT=");
    Serial.print(retSt.fault_code);

    Serial.print(" FAN1_RUN=");
    Serial.print(fan1St.running);

    Serial.print(" FAN1_DMP_CMD=");
    Serial.print(g_fan1_damper_cmd_pct);

    Serial.print(" FAN1_DMP_FB=");
    Serial.print(g_fan1_damper_fb_pct);

    Serial.print(" FAN1_CMD_V=");
    Serial.print((g_fan1_damper_cmd_pct * 10.0f) / 100.0f, 2);

    Serial.print(" FAN1_FB_V=");
    Serial.print(readFan1DamperFeedbackVolt(), 2);

#if SIM_MODE
    Serial.print(" | STOP_REQ="); Serial.print(g_sim_do[PIN_STOP_REQ]);
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

const MachineParams& machineGetParams()
{
  return g_params;
}

void machineSetParams(const MachineParams& p)
{
  g_params = p;
  clampMachineParams(g_params);
}

void machineResetParamsToDefaults()
{
  g_params = machineParamsDefaults();
  clampMachineParams(g_params);
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

bool ventilationAutoAllowed() {
  return !hmiGetVentManualMode();
}

float machineGetReturnActualHz()
{
  return vfdGetActualHz(VfdId::RETURN);
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

void machineSetClimateTestOverride(bool en)
{
  g_clima_test_override = en;
}

bool machineGetClimateTestOverride()
{
  return g_clima_test_override;
}

void machineSetClimateTestTrh1(float tempC, float rh, bool valid)
{
  g_test_trh1_tempC = tempC;
  g_test_trh1_rh    = rh;
  g_test_trh1_valid = valid;
}

void machineSetClimateTestTrh2(float tempC, float rh, bool valid)
{
  g_test_trh2_tempC = tempC;
  g_test_trh2_rh    = rh;
  g_test_trh2_valid = valid;
}

void machineSetClimateTestTrh3(float tempC, float rh, bool valid)
{
  g_test_trh3_tempC = tempC;
  g_test_trh3_rh    = rh;
  g_test_trh3_valid = valid;
}

void machineSetClimateTestTrhValid(uint8_t idx, bool valid)
{
  if (idx == 1) g_test_trh1_valid = valid;
  if (idx == 2) g_test_trh2_valid = valid;
  if (idx == 3) g_test_trh3_valid = valid;
}

void machinePrintClimateTestState()
{
  Serial.print("CLIMATE TEST OVERRIDE: ");
  Serial.println(g_clima_test_override ? "ON" : "OFF");

  Serial.print("TRH1: valid=");
  Serial.print(g_test_trh1_valid ? 1 : 0);
  Serial.print(" T=");
  Serial.print(g_test_trh1_tempC, 1);
  Serial.print(" RH=");
  Serial.println(g_test_trh1_rh, 1);

  Serial.print("TRH2: valid=");
  Serial.print(g_test_trh2_valid ? 1 : 0);
  Serial.print(" T=");
  Serial.print(g_test_trh2_tempC, 1);
  Serial.print(" RH=");
  Serial.println(g_test_trh2_rh, 1);

  Serial.print("TRH3: valid=");
  Serial.print(g_test_trh3_valid ? 1 : 0);
  Serial.print(" T=");
  Serial.print(g_test_trh3_tempC, 1);
  Serial.print(" RH=");
  Serial.println(g_test_trh3_rh, 1);
}

bool machineIsReturnConveyorRunning()
{
#if USE_VFD_RETURN
  VfdState retSt = vfdGetState(VfdId::RETURN);
  return retSt.running;
#else
  bool ret_soft_stop = returnEndSoftStopActive();

  if (g_startup_lock) return false;

return cmd_ret_conv && !ret_soft_stop && !g_fault_latched && estopOk();
#endif
}

bool machineIsWashStopRequested()
{
  return cmd_stop_req;
}

float machineGetFan1DamperCmdPct()
{
  return g_fan1_damper_cmd_pct;
}

float machineGetFan1DamperFbPct()
{
  return g_fan1_damper_fb_pct;
}

bool machineIsFan1DamperOpenCmd()
{
  return fan1DamperOpenCmd();
}

bool machineIsFan1DamperOpenFb()
{
  return fan1DamperOpenFb();
}

bool machineIsFan1RunPermitted()
{
  return g_fan1_run_permitted;
}

bool machineIsFan1Running()
{
  VfdState fan1St = vfdGetState(VfdId::FAN1);
  return fan1St.running;
}

float machineGetFan1CommandHz()
{
  return g_fan1_command_hz;
}

float machineGetFan2CommandHz()
{
  return g_fan2_command_hz;
}

void machineSetupDefaults()
{
  g_params = machineParamsDefaults();
  clampMachineParams(g_params);

// ------------------------------------------------------------
// EEPROM betöltés (ha van érvényes mentés)
// ------------------------------------------------------------
if (machineLoadParamsFromEeprom()) {
  clampMachineParams(g_params);
}

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
  g_ret_feed_pending = false;
  g_ret_start_seen_in_cycle = false;

  g_mode = OpMode::AUTO;
  g_push_phase = PushPhase::IDLE;

  g_fault_hold_until_home = false;
  g_e305_recover_active = false;
  g_wash_hold_until_home = false;

  cmd_stop_req = false;
  cmd_ret_conv = false;
  cmd_push_motor = false;
  cmd_buzzer = false;
  cmd_push_rev = false;

  g_ret_conv_speed_hz = 20.0f;
  hmi_ret_conv_continuous = false;

  hmi_handle_mode = false;
  hmi_service_enable = false;
  hmi_reset_cmd = false;
  hmi_ret_force_empty = false;

g_fan1_damper_cmd_pct = 0.0f;
g_fan1_damper_fb_pct  = 0.0f;
g_fan1_run_permitted  = false;
g_fan1_should_run     = false;
g_fan1_keep_open      = false;
g_fan1_command_hz     = 0.0f;
g_fan2_command_hz     = 0.0f;

  s_prev_hmi_ret_force_empty = false;

  t_out_block.stop();
  t_out_free.stop();
  t_danger.stop();
  t_push_ext.stop();
  t_push_cycle.stop();
  t_tail_clear_wait.stop();
  t_ext_tail_delay.stop();
  t_table_wait.stop();
  t_ret_start_clear.stop();
  t_ret_end_soft.stop();
  t_ret_end_hard.stop();
  t_ret_ghost.stop();
  t_fault_home_recover.stop();
  t_fan1_damper_open_timeout.stop();
  t_fan1_damper_close_timeout.stop();
  
  // OUT optó új szerepben már nem spacing / dirty felügyeletet végez
  last_t_free_ms = 0;
  last_t_block_ms = 0;

#if !SIM_MODE
  analogWrite(PIN_AO_FAN1_DAMPER_CMD, 0);
#endif

    // ------------------------------------------------------------
  // Szemafor indulási állapot
  // Inverz OPTO:
  //   true  = szabad
  //   false = raklap az optó előtt
  // ------------------------------------------------------------
  bool opto = safeRead(PIN_DI_WASH_IN_OPTO);

  if (opto) {
    // szabad -> zöld
    g_semafor_green = true;
    g_semafor_yellow = false;
  } else {
    // raklap ott van -> sárga
    g_semafor_green = false;
    g_semafor_yellow = true;
  }

  g_semafor_wait_encoder = false;
  g_encoder_count = 0;
  g_encoder_target = 0;

  g_last_wash_in_opto = opto;
  g_last_encoder = safeRead(PIN_AI_ENCODER);
}

void machineLoop()
{
  g_now = millis();

  updateInputs();
  updateHmiCommands();
  handleReturnForceEmptyEdge();

  /*/ Ideiglenes kiíratások tesztelés céljából
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint > 2000) {
    lastPrint = millis();

    Serial.print("HOME=");
    Serial.print(di_push_home.stable);

    Serial.print(" RESET=");
    Serial.print(di_reset_main.stable);

    Serial.print(" RESET_RISE=");
    Serial.print(di_reset_main.rise());

    Serial.print(" startup_lock=");
    Serial.print(g_startup_lock);

    Serial.print(" resetAllowed=");
    Serial.println(resetAllowed());
  }*/

  updateCounters();
  superviseReturnGhostLoad();

  runModeOverlayOrAuto();

    // ------------------------------------------------------------
  // Szemafor logika
  // Inverz OPTO:
  //   true  = szabad
  //   false = raklap az optó előtt
  // ------------------------------------------------------------
  bool opto    = safeRead(PIN_DI_WASH_IN_OPTO);
  bool encoder = safeRead(PIN_AI_ENCODER);

  bool opto_rise = (!g_last_wash_in_opto && opto);   // felszabadult az optó
  bool opto_fall = (g_last_wash_in_opto && !opto);   // raklap belépett az optó elé
  bool enc_rise  = (!g_last_encoder && encoder);

  // ------------------------------------------------------------
  // Első induláskori inicializálás
  // ------------------------------------------------------------
  if (!g_semafor_initialized) {
    g_semafor_initialized = true;
    g_semafor_wait_encoder = false;

    if (!opto) {
      // Raklap van az optó előtt -> sárga
      g_semafor_green  = false;
      g_semafor_yellow = true;
    } else {
      // Szabad az optó -> zöld
      g_semafor_green  = true;
      g_semafor_yellow = false;
    }
  }

  // ------------------------------------------------------------
  // Encoder impulzus számlálás szűréssel
  // Szenzoros encoderhez indulásnak 5 ms
  // ------------------------------------------------------------
  if (enc_rise) {
    uint32_t now = millis();

    if ((uint32_t)(now - g_last_encoder_edge_ms) >= 5) {
      g_last_encoder_edge_ms = now;
      g_encoder_count++;
    }
  }

  // ------------------------------------------------------------
  // Raklap belépett az optó elé -> azonnal sárga
  // Inverz szenzornál ez FALL él
  // ------------------------------------------------------------
  if (opto_fall) {
    g_semafor_wait_encoder = false;
    g_encoder_target = g_encoder_count;
    g_semafor_green  = false;
    g_semafor_yellow = true;
  }

  // ------------------------------------------------------------
  // Raklap elment az optó elől -> indul az encoder számlálás
  // Inverz szenzornál ez RISE él
  // ------------------------------------------------------------
  if (opto_rise) {
    g_semafor_wait_encoder = true;
    g_encoder_target = g_encoder_count + g_params.semafor_pulses;
  }

  // ------------------------------------------------------------
  // Megvan az előírt impulzusszám -> vissza zöldre
  // ------------------------------------------------------------
  if (g_semafor_wait_encoder && (g_encoder_count >= g_encoder_target)) {
    g_semafor_wait_encoder = false;
    g_semafor_green  = true;
    g_semafor_yellow = false;
  }

  g_last_wash_in_opto = opto;
  g_last_encoder      = encoder;

  applyOutputs();
}