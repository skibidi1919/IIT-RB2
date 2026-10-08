#pragma once
#include <Arduino.h>

class VL53L0X {
public:
  void setTimeout(uint16_t timeout) {}
  bool init() { return true; }
  void startContinuous(uint32_t period_ms = 0) {}
  uint16_t readRangeContinuousMillimeters() { return 100; }
  bool timeoutOccurred() { return false; }
};
