#include "meow_pca9685.h"
#include "meow_i2c.h"

namespace meow_pca {

static bool g_ok = false;
static uint16_t g_hz = FREQ_HZ;

static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_LED0 = 0x06;
static const uint8_t REG_PRESCALE = 0xFE;

/*
 * Proven Adafruit bench mapping was ticks 150..600 at 50 Hz (~733..2930 µs).
 * At 100 Hz the period is half, so double the ticks for the same pulse width.
 */
static const uint16_t TICK_MIN_50 = 150;
static const uint16_t TICK_MAX_50 = 600;

bool ok() { return g_ok; }
uint16_t frequencyHz() { return g_hz; }

/** Adafruit_PWMServoDriver::setPWMFreq — exact sequence. */
static bool setPWMFreq(uint16_t hz) {
  if (hz < 24) hz = 24;
  if (hz > 1526) hz = 1526;

  float prescaleval = 25000000.0f;
  prescaleval /= 4096.0f;
  prescaleval /= (float)hz;
  prescaleval -= 1.0f;
  uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

  uint8_t oldmode = 0;
  if (!meow_i2c::readReg(ADDR, REG_MODE1, oldmode)) return false;

  uint8_t sleepmode = (uint8_t)((oldmode & 0x7F) | 0x10);
  if (!meow_i2c::writeReg(ADDR, REG_MODE1, sleepmode)) return false;
  if (!meow_i2c::writeReg(ADDR, REG_PRESCALE, prescale)) return false;
  if (!meow_i2c::writeReg(ADDR, REG_MODE1, oldmode)) return false;
  delay(5);
  // auto-increment + restart (0xA0) ; keep ALLCALL if it was on
  if (!meow_i2c::writeReg(ADDR, REG_MODE1, (uint8_t)(oldmode | 0xA1))) return false;

  meow_i2c::writeReg(ADDR, REG_MODE2, 0x04);  // OUTDRV totem-pole
  g_hz = hz;
  return true;
}

/** Adafruit setPWM — ON/OFF times, 4-byte write with AI. */
static bool setPWM(uint8_t ch, uint16_t on, uint16_t off) {
  if (ch > 15) return false;
  uint8_t reg = (uint8_t)(REG_LED0 + 4 * ch);
  uint8_t buf[4] = {
      (uint8_t)on,
      (uint8_t)(on >> 8),
      (uint8_t)off,
      (uint8_t)(off >> 8),
  };
  if (meow_i2c::writeRegs(ADDR, reg, buf, 4)) return true;

  // Fallback: byte-by-byte if AI/multiwrite failed
  for (uint8_t i = 0; i < 4; i++) {
    if (!meow_i2c::writeReg(ADDR, (uint8_t)(reg + i), buf[i])) return false;
  }
  return true;
}

bool begin(uint16_t freq_hz) {
  g_ok = false;
  meow_i2c::setTimeout(100000UL);
  if (!meow_i2c::probe(ADDR)) return false;

  // MODE1 reset like Adafruit reset()
  if (!meow_i2c::writeReg(ADDR, REG_MODE1, 0x00)) return false;
  delay(10);
  if (!setPWMFreq(freq_hz)) return false;

  g_ok = true;
  return true;
}

bool setPulseUs(uint8_t ch, uint16_t us) {
  if (!g_ok) return false;
  // Convert µs → ticks at current period
  uint32_t period_us = 1000000UL / (uint32_t)g_hz;
  uint32_t ticks = ((uint32_t)us * 4096UL + period_us / 2) / period_us;
  if (ticks < 1) ticks = 1;
  if (ticks > 4095) ticks = 4095;
  return setPWM(ch, 0, (uint16_t)ticks);
}

static int clamp180(int d) { return d < 0 ? 0 : (d > 180 ? 180 : d); }

bool setAngle(uint8_t ch, int angle_deg) {
  if (!g_ok) return false;
  angle_deg = clamp180(angle_deg);
  // Scale Adafruit 50 Hz ticks to current frequency
  uint16_t tmin = (uint16_t)((uint32_t)TICK_MIN_50 * g_hz / 50U);
  uint16_t tmax = (uint16_t)((uint32_t)TICK_MAX_50 * g_hz / 50U);
  uint16_t ticks = (uint16_t)map(angle_deg, 0, 180, tmin, tmax);
  return setPWM(ch, 0, ticks);
}

bool setArm(int base_deg, int height_deg, int grip_deg) {
  bool a = setAngle(CH_BASE, base_deg);
  bool b = setAngle(CH_HEIGHT, height_deg);
  bool c = setAngle(CH_GRIP, grip_deg);
  return a && b && c;
}

}  // namespace meow_pca
