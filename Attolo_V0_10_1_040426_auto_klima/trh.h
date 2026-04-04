#pragma once
#include <Arduino.h>

enum class TrhId : uint8_t {
  TRH1 = 0,
  TRH2 = 1,
  TRH3 = 2
};

struct TrhState {
  uint8_t  slaveId = 1;
  bool     enabled = false;

  bool     valid = false;
  uint8_t  lastMbError = 0xFF;

  uint16_t rawTemp = 0;
  uint16_t rawHum  = 0;

  float    tempC = 0.0f;
  float    humRH = 0.0f;

  uint32_t lastPollMs = 0;
  uint32_t lastOkMs   = 0;
};

void trhSetup();
void trhLoop();

const TrhState& trhGet(TrhId id);
bool trhIsFresh(TrhId id, uint32_t maxAgeMs = 10000);

void trhSetSlaveId(TrhId id, uint8_t slaveId);
void trhSetEnabled(TrhId id, bool enabled);