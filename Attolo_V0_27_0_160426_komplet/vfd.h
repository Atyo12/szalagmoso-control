#pragma once
#include <Arduino.h>

enum class VfdId : uint8_t {
  RETURN = 0,
  FAN1   = 1,
  FAN2   = 2
};

struct VfdState {
  bool     comm_ok;
  bool     running;
  bool     fault_active;

  uint16_t status_word;
  uint16_t fault_code;

  uint16_t freq_raw_x100;
  float    actual_hz;

  uint32_t last_ok_ms;
};

void vfdSetup();
void vfdLoop();

void vfdRequestRun(VfdId id, bool run);
void vfdRequestFreqHz(VfdId id, float hz);

VfdState vfdGetState(VfdId id);
bool vfdIsCommOk(VfdId id);
bool vfdHasFault(VfdId id);
float vfdGetActualHz(VfdId id);

void vfdPrintAll();

uint32_t vfdGetLastBusActivityMs();
bool vfdBusIsIdle(uint32_t quietMs = 120);