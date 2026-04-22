#include <Arduino.h>
#include <ModbusMaster.h>

#include "trh.h"
#include "vfd.h"

// A korábbi működő donor kód alapján
static constexpr float TEMP_SCALE = 28.1f;

// lassú, stabil polling ugyanazon az RS485 buszon
static constexpr uint32_t TRH_POLL_INTERVAL_MS = 2000;
static constexpr uint32_t TRH_REQUIRED_BUS_IDLE_MS = 120;

static ModbusMaster g_trhMb;
static TrhState g_trh[3];
static uint8_t g_nextIdx = 0;
static uint32_t g_lastStepMs = 0;

static int toIndex(TrhId id) {
  return static_cast<int>(id);
}

const TrhState& trhGet(TrhId id) {
  return g_trh[toIndex(id)];
}

bool trhIsFresh(TrhId id, uint32_t maxAgeMs) {
  const TrhState& s = trhGet(id);
  if (!s.enabled || !s.valid) return false;
  return (millis() - s.lastOkMs) <= maxAgeMs;
}

void trhSetSlaveId(TrhId id, uint8_t slaveId) {
  g_trh[toIndex(id)].slaveId = slaveId;
}

void trhSetEnabled(TrhId id, bool enabled) {
  g_trh[toIndex(id)].enabled = enabled;
}

void trhSetup() {
  g_nextIdx = 0;
  g_lastStepMs = 0;

  // TRH1 = slave 2  -> kompresszor fölött / befúvó, legmelegebb
  g_trh[0].slaveId = 2;
  g_trh[0].enabled = true;
  g_trh[0].valid = false;
  g_trh[0].lastMbError = 0xFF;
  g_trh[0].rawTemp = 0;
  g_trh[0].rawHum = 0;
  g_trh[0].tempC = 0.0f;
  g_trh[0].humRH = 0.0f;
  g_trh[0].lastPollMs = 0;
  g_trh[0].lastOkMs = 0;

  // TRH2 = slave 1  -> kifúvó, legpárásabb pont
  g_trh[1].slaveId = 1;
  g_trh[1].enabled = true;
  g_trh[1].valid = false;
  g_trh[1].lastMbError = 0xFF;
  g_trh[1].rawTemp = 0;
  g_trh[1].rawHum = 0;
  g_trh[1].tempC = 0.0f;
  g_trh[1].humRH = 0.0f;
  g_trh[1].lastPollMs = 0;
  g_trh[1].lastOkMs = 0;

  // TRH3 = slave 3  -> monoblokk közelében / külsőhöz hasonló referencia
  g_trh[2].slaveId = 3;
  g_trh[2].enabled = true;
  g_trh[2].valid = false;
  g_trh[2].lastMbError = 0xFF;
  g_trh[2].rawTemp = 0;
  g_trh[2].rawHum = 0;
  g_trh[2].tempC = 0.0f;
  g_trh[2].humRH = 0.0f;
  g_trh[2].lastPollMs = 0;
  g_trh[2].lastOkMs = 0;
}

void trhLoop() {
  uint32_t now = millis();

  if ((uint32_t)(now - g_lastStepMs) < TRH_POLL_INTERVAL_MS) {
    return;
  }

  // Csak akkor mehet TRH olvasás, ha a VFD már rövid ideje
  // nem használta a közös Serial2 buszt.
  if (!vfdBusIsIdle(TRH_REQUIRED_BUS_IDLE_MS)) {
    return;
  }

  // csak akkor léptetjük az időt, ha tényleg próbálunk olvasni
  g_lastStepMs = now;

  // egyszerre csak egy szenzort kérdezünk
  for (int n = 0; n < 3; ++n) {
    uint8_t idx = g_nextIdx % 3;
    g_nextIdx++;

    TrhState& s = g_trh[idx];
    if (!s.enabled) continue;

    s.lastPollMs = now;

    g_trhMb.begin(s.slaveId, Serial2);
    uint8_t result = g_trhMb.readHoldingRegisters(0x0000, 2);
    s.lastMbError = result;

    if (result == g_trhMb.ku8MBSuccess) {
      s.rawTemp = g_trhMb.getResponseBuffer(0);
      s.rawHum  = g_trhMb.getResponseBuffer(1);
      s.tempC   = s.rawTemp / TEMP_SCALE;
      s.humRH   = s.rawHum / 10.0f;
      s.valid   = true;
      s.lastOkMs = now;
    }

    break;
  }
}