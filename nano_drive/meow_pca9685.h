/*
 * Meowler PCA9685 — arm servos only (CH0 base, CH1 height, CH2 grip).
 * Default PWM 100 Hz; angles → pulse width in microseconds → 12-bit ticks.
 */
#pragma once

#include <Arduino.h>

namespace meow_pca {

static const uint8_t ADDR = 0x40;
static const uint8_t CH_BASE = 0;
static const uint8_t CH_HEIGHT = 1;
static const uint8_t CH_GRIP = 2;
static const uint16_t FREQ_HZ = 100;
static const uint16_t US_MIN = 500;
static const uint16_t US_MAX = 2500;

/** Probe + reset + set FREQ_HZ. Returns false if chip missing/timeout. */
bool begin(uint16_t freq_hz = FREQ_HZ);

bool ok();
uint16_t frequencyHz();

/** Set channel pulse in microseconds (clamped). */
bool setPulseUs(uint8_t ch, uint16_t us);

/** Map 0..180° → US_MIN..US_MAX and write. */
bool setAngle(uint8_t ch, int angle_deg);

/** Write base/height/grip angles (any order; missing = leave via caller). */
bool setArm(int base_deg, int height_deg, int grip_deg);

}  // namespace meow_pca
