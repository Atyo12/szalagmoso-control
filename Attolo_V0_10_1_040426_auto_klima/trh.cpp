#include <Arduino.h>
#include <ModbusMaster.h>
#include "trh.h"

// A te tesztkódodból:
static constexpr float TEMP_SCALE = 28.1f;

// Lassú, stabil polling ugyanazon az RS485 buszon
static constexpr uint32_t TRH_POLL_INTERVAL_MS = 1200;
static constexpr uint32_t TRH_FRESH_MS         = 10000;

static ModbusMaster g_trhMb;
static uint8_t g_nextIdx = 0;
static uint32_t g_lastStepMs = 0;

// ------------------------------------------------------------
// FIGYELEM:
// Ezeket a slave ID-ket állítsd be a saját kiosztásod szerint.
// Most úgy hagytam, hogy a 2 meglévő otthoni szenzorral gyorsan induljon.
// ------------------------------------------------------------
static TrhState g_trh[3];

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

  // TRH1
  g_trh[0].slaveId = 1;
  g_trh[0].enabled = true;
  g_trh[0].valid = false;
  g_trh[0].lastMbError = 0xFF;
  g_trh[0].rawTemp = 0;
  g_trh[0].rawHum = 0;
  g_trh[0].tempC = 0.0f;
  g_trh[0].humRH = 0.0f;
  g_trh[0].lastPollMs = 0;
  g_trh[0].lastOkMs = 0;

  // TRH2
  g_trh[1].slaveId = 3;
  g_trh[1].enabled = true;
  g_trh[1].valid = false;
  g_trh[1].lastMbError = 0xFF;
  g_trh[1].rawTemp = 0;
  g_trh[1].rawHum = 0;
  g_trh[1].tempC = 0.0f;
  g_trh[1].humRH = 0.0f;
  g_trh[1].lastPollMs = 0;
  g_trh[1].lastOkMs = 0;

  // TRH3
  g_trh[2].slaveId = 2;
  g_trh[2].enabled = false;   // most még nincs fent
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
  g_lastStepMs = now;

  // Következő engedélyezett szenzor keresése
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

      s.tempC = s.rawTemp / TEMP_SCALE;
      s.humRH = s.rawHum / 10.0f;

      s.valid = true;
      s.lastOkMs = now;
    } else {
      // Régi jó értéket megtartjuk, csak a fresh státusz fog lejárni
    }

    break; // egyszerre csak egy szenzor
  }
}