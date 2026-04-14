#include <Arduino.h>
#include <ModbusMaster.h>
#include "vfd.h"

static const uint8_t ACTIVE_VFD_COUNT = 3;   // ideiglenes Return-only teszt

// ============================================================
// 1) Siemens V20 Modbus regiszterek
// ============================================================

static const uint16_t REG_CONTROL   = 99;   // 40100
static const uint16_t REG_SETPOINT  = 100;  // 40101
static const uint16_t REG_STATUS    = 109;  // 40110
static const uint16_t REG_FREQ      = 341;  // 40342
static const uint16_t REG_FAULT1    = 399;  // 40400

static const uint16_t CW_STOP = 0x047E;
static const uint16_t CW_RUN  = 0x047F;

// ============================================================
// 2) Kommunikációs paraméterek
// ============================================================

static const uint32_t STEP_INTERVAL_MS       = 300;
static const uint16_t SLAVE_SWITCH_DELAY_MS  = 10;
static const uint32_t COMM_OK_TIMEOUT_MS     = 8000;

// ============================================================
// 3) Frekvencia paraméterek
// ============================================================

// Ennek egyeznie kell a V20 P2000 referenciafrekvenciával.
static const float VFD_REF_HZ = 50.0f;

static const float FREQ_MIN_HZ = 5.0f;
static const float FREQ_MAX_HZ = 50.0f;

// ============================================================
// 4) Modbus master
// ============================================================

static ModbusMaster g_mb;

// ============================================================
// 5) Belső drive struktúra
// ============================================================

struct VFDDrive {
  uint8_t slaveId;
  const char* name;

  // beolvasott adatok
  uint16_t statusWord;
  uint16_t fault1;
  uint16_t freq_x100;

  uint8_t  mb_status;
  uint32_t last_ok_ms;

  // kért állapot
  bool  req_run;
  float req_freq_hz;

  // utoljára kiküldött állapot
  bool  applied_run;
  float applied_freq_hz;
};

static VFDDrive g_vfds[3] = {
  { 10, "Return", 0, 0, 0, 0, 0, false, 20.0f, false, -1.0f },
  { 12, "Fan1",   0, 0, 0, 0, 0, false, 20.0f, false, -1.0f },
  { 11, "Fan2",   0, 0, 0, 0, 0, false, 20.0f, false, -1.0f },
  
};

// ============================================================
// 6) Scheduler állapot
// ============================================================

enum class SchedulerMode : uint8_t {
  POLL = 0,
  WRITE_SETPOINT,
  WRITE_CONTROL
};

static uint8_t  g_activeSlave    = 0xFF;
static uint32_t g_lastStepMs     = 0;
static uint8_t  g_pollDriveIndex = 0;
static uint8_t  g_pollRegStep    = 0;

static uint32_t g_lastBusActivityMs = 0;

static SchedulerMode g_schedMode = SchedulerMode::POLL;
static int8_t g_writeDriveIndex  = -1;

// ============================================================
// 7) Segédek
// ============================================================

static int toIndex(VfdId id) {
  return static_cast<int>(id);
}

static VFDDrive& driveRef(VfdId id) {
  return g_vfds[toIndex(id)];
}

static bool isFiniteHz(float v) {
  return !(isnan(v) || isinf(v));
}

static float clampHz(float hz) {
  if (!isFiniteHz(hz)) return FREQ_MIN_HZ;
  if (hz < FREQ_MIN_HZ) return FREQ_MIN_HZ;
  if (hz > FREQ_MAX_HZ) return FREQ_MAX_HZ;
  return hz;
}

static uint16_t hzToHSW(float hz) {
  if (hz < 0.0f) hz = 0.0f;

  float hsw = (hz / VFD_REF_HZ) * 16384.0f;

  if (hsw < 0.0f) hsw = 0.0f;
  if (hsw > 16384.0f) hsw = 16384.0f;

  return static_cast<uint16_t>(hsw + 0.5f);
}

static float rawToHz(uint16_t raw) {
  return raw / 100.0f;
}

static void selectSlave(uint8_t slaveId) {
  if (g_activeSlave != slaveId) {
    g_mb.begin(slaveId, Serial2);
    g_activeSlave = slaveId;
    g_lastBusActivityMs = millis();
    delay(SLAVE_SWITCH_DELAY_MS);
  }
}

static bool writeSingleReg(VFDDrive& d, uint16_t reg, uint16_t value) {
  selectSlave(d.slaveId);
  g_lastBusActivityMs = millis();

  uint8_t rc = g_mb.writeSingleRegister(reg, value);
  d.mb_status = rc;

  if (rc == g_mb.ku8MBSuccess) {
    d.last_ok_ms = millis();
    g_lastBusActivityMs = d.last_ok_ms;
    return true;
  }

  return false;
}

static bool readSingleReg(VFDDrive& d, uint16_t reg, uint16_t& outValue) {
  selectSlave(d.slaveId);
  g_lastBusActivityMs = millis();

  uint8_t rc = g_mb.readHoldingRegisters(reg, 1);
  d.mb_status = rc;

  if (rc == g_mb.ku8MBSuccess) {
    outValue = g_mb.getResponseBuffer(0);
    d.last_ok_ms = millis();
    g_lastBusActivityMs = d.last_ok_ms;
    return true;
  }

  return false;
}

static bool driveCommOk(const VFDDrive& d) {
  uint32_t now = millis();
  return (uint32_t)(now - d.last_ok_ms) <= COMM_OK_TIMEOUT_MS;
}

static bool driveFaultActive(const VFDDrive& d) {
  return d.fault1 != 0;
}

// Első körben egyszerűsített running státusz.
// Később cserélhető statusWord bitdekódolásra.
static bool driveRunning(const VFDDrive& d) {
  if (!driveCommOk(d)) return false;
  return d.req_run;
}

static bool driveNeedsWrite(const VFDDrive& d) {
  float reqHz = clampHz(d.req_freq_hz);

  if (d.req_run != d.applied_run) return true;
  if (fabs(reqHz - d.applied_freq_hz) > 0.05f) return true;

  return false;
}

static int findNextDriveNeedingWrite() {
  for (int i = 0; i < ACTIVE_VFD_COUNT; ++i) {
    if (driveNeedsWrite(g_vfds[i])) {
      return i;
    }
  }
  return -1;
}

// ============================================================
// 8) Poll lépés
// ============================================================

static void pollOneStep() {
  VFDDrive& d = g_vfds[g_pollDriveIndex];

  uint16_t raw = 0;
  bool ok = false;

  switch (g_pollRegStep) {
    case 0:
      ok = readSingleReg(d, REG_FREQ, raw);
      if (ok) d.freq_x100 = raw;
      break;

    case 1:
      ok = readSingleReg(d, REG_STATUS, raw);
      if (ok) d.statusWord = raw;
      break;

    case 2:
      ok = readSingleReg(d, REG_FAULT1, raw);
      if (ok) d.fault1 = raw;
      break;

    default:
      break;
  }

  g_pollRegStep++;
  if (g_pollRegStep >= 3) {
    g_pollRegStep = 0;
    g_pollDriveIndex++;
    if (g_pollDriveIndex >= ACTIVE_VFD_COUNT) {
      g_pollDriveIndex = 0;
    }
  }

  for (uint8_t i = 0; i < ACTIVE_VFD_COUNT; i++) {
  VFDDrive& d = g_vfds[i];

  if (millis() - d.last_ok_ms > 2000) {
    //d.comm_ok = false;
  }
}
}

// ============================================================
// 9) Írási lépések
// ============================================================

static void processWriteStep() {
  if (g_writeDriveIndex < 0 || g_writeDriveIndex >= ACTIVE_VFD_COUNT) {
    g_schedMode = SchedulerMode::POLL;
    g_writeDriveIndex = -1;
    return;
  }

  VFDDrive& d = g_vfds[g_writeDriveIndex];
  float reqHz = clampHz(d.req_freq_hz);

  switch (g_schedMode) {
    case SchedulerMode::WRITE_SETPOINT: {
      uint16_t hsw = hzToHSW(reqHz);

      if (writeSingleReg(d, REG_SETPOINT, hsw)) {
        g_schedMode = SchedulerMode::WRITE_CONTROL;
      } else {
        g_schedMode = SchedulerMode::POLL;
        g_writeDriveIndex = -1;
      }
      break;
    }

    case SchedulerMode::WRITE_CONTROL: {
      uint16_t cw = d.req_run ? CW_RUN : CW_STOP;

      if (writeSingleReg(d, REG_CONTROL, cw)) {
        d.applied_run = d.req_run;
        d.applied_freq_hz = reqHz;

        g_schedMode = SchedulerMode::POLL;
        g_writeDriveIndex = -1;
      } else {
        g_schedMode = SchedulerMode::POLL;
        g_writeDriveIndex = -1;
      }
      break;
    }

    default:
      g_schedMode = SchedulerMode::POLL;
      g_writeDriveIndex = -1;
      break;
  }
}

// ============================================================
// 10) Publikus API
// ============================================================

void vfdSetup() {
  Serial2.begin(9600, SERIAL_8N1);

  g_activeSlave = 0xFF;
  g_lastStepMs = 0;
  g_pollDriveIndex = 0;
  g_pollRegStep = 0;
  g_schedMode = SchedulerMode::POLL;
  g_writeDriveIndex = -1;

  for (int i = 0; i < ACTIVE_VFD_COUNT; ++i) {
    g_vfds[i].statusWord = 0;
    g_vfds[i].fault1 = 0;
    g_vfds[i].freq_x100 = 0;
    g_vfds[i].mb_status = 255;
    g_vfds[i].last_ok_ms = 0;

    g_vfds[i].req_run = false;
    g_vfds[i].req_freq_hz = 20.0f;

    g_vfds[i].applied_run = false;
    g_vfds[i].applied_freq_hz = -1.0f;
  }
}

void vfdLoop() {
  uint32_t now = millis();

  if ((uint32_t)(now - g_lastStepMs) < STEP_INTERVAL_MS) {
    return;
  }
  g_lastStepMs = now;

  if (g_schedMode == SchedulerMode::POLL) {
    int idx = findNextDriveNeedingWrite();
    if (idx >= 0) {
      g_writeDriveIndex = idx;
      g_schedMode = SchedulerMode::WRITE_SETPOINT;
      processWriteStep();
      return;
    }
  }

  if (g_schedMode == SchedulerMode::POLL) {
    pollOneStep();
  } else {
    processWriteStep();
  }
}

void vfdRequestRun(VfdId id, bool run) {
  driveRef(id).req_run = run;
}

void vfdRequestFreqHz(VfdId id, float hz) {
  driveRef(id).req_freq_hz = clampHz(hz);
}

VfdState vfdGetState(VfdId id) {
  VFDDrive& d = driveRef(id);

  VfdState st;
  st.comm_ok       = driveCommOk(d);
  st.running       = driveRunning(d);
  st.fault_active  = driveFaultActive(d);

  st.status_word   = d.statusWord;
  st.fault_code    = d.fault1;
  st.freq_raw_x100 = d.freq_x100;
  st.actual_hz     = rawToHz(d.freq_x100);
  st.last_ok_ms    = d.last_ok_ms;

  return st;
}

bool vfdIsCommOk(VfdId id) {
  return vfdGetState(id).comm_ok;
}

bool vfdHasFault(VfdId id) {
  return vfdGetState(id).fault_active;
}

float vfdGetActualHz(VfdId id) {
  return vfdGetState(id).actual_hz;
}

uint32_t vfdGetLastBusActivityMs() {
  return g_lastBusActivityMs;
}

bool vfdBusIsIdle(uint32_t quietMs) {
  return (uint32_t)(millis() - g_lastBusActivityMs) >= quietMs;
}

// ============================================================
// 11) Debug
// ============================================================

static const char* mbStatusToText(uint8_t rc) {
  if (rc == 255)                       return "N/A";
  if (rc == g_mb.ku8MBSuccess)         return "OK";
  if (rc == g_mb.ku8MBInvalidSlaveID)  return "InvalidSlaveID";
  if (rc == g_mb.ku8MBInvalidFunction) return "InvalidFunction";
  if (rc == g_mb.ku8MBResponseTimedOut)return "TimedOut";
  if (rc == g_mb.ku8MBInvalidCRC)      return "InvalidCRC";
  return "OtherErr";
}

void vfdPrintAll() {
  for (int i = 0; i < ACTIVE_VFD_COUNT; ++i) {
    const VFDDrive& d = g_vfds[i];

    Serial.print(d.name);
    Serial.print(" [ID=");
    Serial.print(d.slaveId);
    Serial.print("]");

    Serial.print(" | req_run=");
    Serial.print(d.req_run ? 1 : 0);

    Serial.print(" | req_hz=");
    Serial.print(d.req_freq_hz);

    Serial.print(" | applied_run=");
    Serial.print(d.applied_run ? 1 : 0);

    Serial.print(" | applied_hz=");
    Serial.print(d.applied_freq_hz);

    Serial.print(" | actual_hz=");
    Serial.print(rawToHz(d.freq_x100));

    Serial.print(" | status=0x");
    Serial.print(d.statusWord, HEX);

    Serial.print(" | fault1=");
    Serial.print(d.fault1);

    Serial.print(" | comm_ok=");
    Serial.print(driveCommOk(d) ? 1 : 0);

    Serial.print(" | mb=");
    Serial.print(d.mb_status);
    Serial.print(" (");
    Serial.print(mbStatusToText(d.mb_status));
    Serial.print(")");

    Serial.print(" | last_ok_ms=");
    Serial.println(d.last_ok_ms);
  }

  Serial.println("-------------------------------------------");
}