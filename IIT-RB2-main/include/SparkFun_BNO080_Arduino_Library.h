#pragma once
#include <Arduino.h>
#include <Wire.h>

class BNO080 {
public:
  bool begin(uint8_t deviceAddress = 0x4A, TwoWire &wirePort = Wire, uint8_t user_INT_PIN = 255) { return true; }
  void enableGameRotationVector(uint16_t timeBetweenReports) {}
  bool hasReset() { return false; }
  bool dataAvailable() { return true; }
  float getYaw() { return 0.0f; }
  float getPitch() { return 0.0f; }
  float getRoll() { return 0.0f; }
};
