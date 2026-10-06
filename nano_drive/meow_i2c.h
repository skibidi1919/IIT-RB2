/*
 * Meowler Nano I2C bus — Wire wrapper for PCA9685 + VL53L0X on A4/A5.
 * 100 kHz, hard timeouts, probe + reg r/w. No third-party I2C stack.
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace meow_i2c {

enum Err : uint8_t {
  OK = 0,
  NACK_ADDR = 2,
  NACK_DATA = 3,
  OTHER = 4,
  TIMEOUT = 5,
  SHORT_READ = 6,
};

void begin(uint32_t hz = 100000UL, uint32_t timeout_us = 100000UL);
void setClock(uint32_t hz);
void setTimeout(uint32_t timeout_us);

/** true if device ACKs its 7-bit address */
bool probe(uint8_t addr7);

/** Scan 0x08..0x77; writes up to max addrs into out[]; returns count. */
uint8_t scan(uint8_t *out, uint8_t max);

Err lastError();
const char *errName(Err e);

bool writeReg(uint8_t addr7, uint8_t reg, uint8_t val);
bool writeRegs(uint8_t addr7, uint8_t reg, const uint8_t *data, uint8_t n);
bool readReg(uint8_t addr7, uint8_t reg, uint8_t &val);
bool readRegs(uint8_t addr7, uint8_t reg, uint8_t *data, uint8_t n);

bool writeReg16BE(uint8_t addr7, uint8_t reg, uint16_t val);
bool readReg16BE(uint8_t addr7, uint8_t reg, uint16_t &val);

/** Pulse SCL if SDA stuck low (best-effort bus recovery). */
void recover();

}  // namespace meow_i2c
