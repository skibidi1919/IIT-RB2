#pragma once
/*
 * Bit-bang I2C BnoBus for BNO08x (SHTP) on arbitrary Nano pins.
 * Open-drain: pinMode INPUT (=high via pullup) / OUTPUT+LOW (=drive low).
 */
#include <Arduino.h>
#include <BnoBus.h>

struct SoftBnoI2CBus : public BnoBus {
  uint8_t sdaPin;
  uint8_t sclPin;
  uint8_t addr;
  uint16_t halfUs;

  SoftBnoI2CBus(uint8_t sda, uint8_t scl, uint8_t i2cAddr = 0x4A, uint32_t hz = 50000UL)
      : sdaPin(sda), sclPin(scl), addr(i2cAddr), halfUs(10) {
    if (hz >= 100000UL) halfUs = 5;
    else if (hz >= 50000UL) halfUs = 10;
    else halfUs = 20;
  }

  void sdaHi() { pinMode(sdaPin, INPUT_PULLUP); }
  void sdaLo() {
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
  }
  void sclHi() {
    pinMode(sclPin, INPUT_PULLUP);
    uint16_t guard = 0;
    while (!digitalRead(sclPin) && guard++ < 1000) { /* clock stretch */ }
  }
  void sclLo() {
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, LOW);
  }
  bool sdaRead() { return digitalRead(sdaPin); }

  void delayH() { delayMicroseconds(halfUs); }

  void start() {
    sdaHi();
    sclHi();
    delayH();
    sdaLo();
    delayH();
    sclLo();
  }

  void stop() {
    sdaLo();
    delayH();
    sclHi();
    delayH();
    sdaHi();
    delayH();
  }

  bool writeByte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
      if (b & 0x80) sdaHi();
      else sdaLo();
      delayH();
      sclHi();
      delayH();
      sclLo();
      b <<= 1;
    }
    sdaHi();
    delayH();
    sclHi();
    delayH();
    bool ack = !sdaRead();
    sclLo();
    return ack;
  }

  uint8_t readByte(bool sendAck) {
    uint8_t b = 0;
    sdaHi();
    for (uint8_t i = 0; i < 8; i++) {
      b <<= 1;
      delayH();
      sclHi();
      delayH();
      if (sdaRead()) b |= 1;
      sclLo();
    }
    if (sendAck) sdaLo();
    else sdaHi();
    delayH();
    sclHi();
    delayH();
    sclLo();
    sdaHi();
    return b;
  }

  bool begin() override {
    sdaHi();
    sclHi();
    delay(2);
    return true;
  }

  bool probe() {
    start();
    bool ok = writeByte((uint8_t)(addr << 1));
    stop();
    return ok;
  }

  bool tx(const uint8_t *data, size_t n) override {
    if (!data || !n) return false;
    start();
    if (!writeByte((uint8_t)(addr << 1))) {
      stop();
      return false;
    }
    for (size_t i = 0; i < n; i++) {
      if (!writeByte(data[i])) {
        stop();
        return false;
      }
    }
    stop();
    return true;
  }

  int rx(uint8_t *buf, size_t cap) override {
    if (!buf || cap < 4) return 0;
    const uint32_t t0 = millis();
    while ((millis() - t0) < 100) {
      start();
      if (!writeByte((uint8_t)((addr << 1) | 1))) {
        stop();
        delay(1);
        continue;
      }
      buf[0] = readByte(true);
      buf[1] = readByte(true);
      uint16_t len = (uint16_t(buf[0]) | (uint16_t(buf[1]) << 8)) & 0x7FFF;
      if (len < 4 || len > cap) {
        /* drain a few bytes then stop */
        for (uint8_t i = 2; i < 8 && i < len; i++) (void)readByte(i + 1 < 8);
        stop();
        continue;
      }
      for (uint16_t i = 2; i < len; i++) {
        buf[i] = readByte(i + 1 < len);
      }
      stop();
      return (int)len;
    }
    return 0;
  }
};
