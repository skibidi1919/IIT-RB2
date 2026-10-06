/*
 * TEMP — ESP32-S3 PCA9685 probe
 * Confirmed wiring: SDA=21 SCL=47 @ 0x40
 * Serial 115200. USB flash, watch MODE1 + CH0–2 wiggle.
 */

#include <Arduino.h>
#include <Wire.h>
#include "boot_ota0.h"

/* Prefer confirmed PCA bus first */
static const int kPairs[][2] = {{21, 47}, {38, 39}, {41, 42}};

static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_LED0 = 0x06;
static const uint8_t REG_PRESCALE = 0xFE;
static const uint16_t SERVO_US_MIN = 500;
static const uint16_t SERVO_US_MAX = 2500;
static const uint16_t SERVO_PERIOD_US = 6667;

static TwoWire *bus = &Wire;
static uint8_t pcaAddr = 0;
static int pinSda = 21, pinScl = 47;

static void busRecover(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
  }
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  delayMicroseconds(5);
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
}

static void dumpIdleLevels(const char *tag, int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(2);
  Serial.printf("%s idle SDA=%d SCL=%d (want 1/1)\n", tag, digitalRead(sda), digitalRead(scl));
}

/* Returns device count. Ghost bus = every addr ACKs (~112). */
static int i2cScan(TwoWire &w, const char *tag) {
  Serial.printf("--- I2C scan %s ---\n", tag);
  int n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    w.beginTransmission(a);
    if (w.endTransmission() != 0) continue;
    Serial.printf("  FOUND 0x%02X", a);
    if (a >= 0x40 && a <= 0x47) Serial.print(" PCA?");
    if (a == 0x29) Serial.print(" VL53");
    if (a == 0x4A || a == 0x4B) Serial.print(" BNO?");
    if (a == 0x70) Serial.print(" ALLCALL");
    Serial.println();
    n++;
  }
  Serial.printf("devices=%d%s\n", n, n > 16 ? " (GHOST — SDA stuck ACK)" : "");
  return n;
}

static bool write8(uint8_t addr, uint8_t reg, uint8_t val) {
  bus->beginTransmission(addr);
  bus->write(reg);
  bus->write(val);
  return bus->endTransmission() == 0;
}

static bool read8(uint8_t addr, uint8_t reg, uint8_t &val) {
  bus->beginTransmission(addr);
  bus->write(reg);
  if (bus->endTransmission(false) != 0) return false;
  if (bus->requestFrom((int)addr, 1) != 1) return false;
  val = bus->read();
  return true;
}

/* Real PCA: MODE1 read returns a stable non-0xFF value. Ghost scan alone is ignored. */
static uint8_t findRealPca(TwoWire &w, int foundCount) {
  bus = &w;
  for (uint8_t a = 0x40; a <= 0x47; a++) {
    uint8_t m1a = 0xFF, m1b = 0xFE;
    if (!read8(a, REG_MODE1, m1a)) continue;
    delay(2);
    if (!read8(a, REG_MODE1, m1b)) continue;
    if (m1a != m1b) continue;
    if (m1a == 0xFF) continue;  // open bus / no ACK data
    if (foundCount > 16)
      Serial.printf("WARN scan ghost (%d) but MODE1 ok @0x%02X=0x%02X\n", foundCount, a, m1a);
    else
      Serial.printf("PCA @0x%02X MODE1=0x%02X (real)\n", a, m1a);
    return a;
  }
  return 0;
}

static bool setPwm(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t reg = REG_LED0 + 4 * ch;
  return write8(pcaAddr, reg + 0, on & 0xFF) && write8(pcaAddr, reg + 1, on >> 8) &&
         write8(pcaAddr, reg + 2, off & 0xFF) && write8(pcaAddr, reg + 3, off >> 8);
}

static uint16_t usToTicks(uint32_t us) {
  uint32_t ticks = (us * 4096UL) / SERVO_PERIOD_US;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

static bool setAngle(uint8_t ch, int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  uint32_t us = (uint32_t)map(deg, 0, 180, (long)SERVO_US_MIN, (long)SERVO_US_MAX);
  bool ok = setPwm(ch, 0, usToTicks(us));
  Serial.printf("  CH%u -> %d (%luus) %s\n", ch, deg, (unsigned long)us, ok ? "ok" : "FAIL");
  return ok;
}

static bool initPca(uint8_t addr) {
  if (!write8(addr, REG_MODE1, 0x10)) return false;
  delay(2);
  if (!write8(addr, REG_PRESCALE, 40)) return false;
  if (!write8(addr, REG_MODE1, 0x00)) return false;
  delay(5);
  if (!write8(addr, REG_MODE1, 0xA1)) return false;
  write8(addr, REG_MODE2, 0x04);
  uint8_t m1 = 0xFF;
  if (!read8(addr, REG_MODE1, m1)) {
    Serial.println("MODE1 read FAIL");
    return false;
  }
  Serial.printf("MODE1=0x%02X after init\n", m1);
  return true;
}

static void wiggle(uint8_t ch) {
  Serial.printf(">>> wiggle CH%u\n", ch);
  setAngle(ch, 90);
  delay(400);
  setAngle(ch, 45);
  delay(500);
  setAngle(ch, 135);
  delay(500);
  setAngle(ch, 90);
  delay(400);
}

static bool openPair(int sda, int scl) {
  busRecover(sda, scl);
  dumpIdleLevels("idle", sda, scl);
  Wire.end();
  Wire1.end();
  delay(5);
  /* 21/47 on Wire; other pairs on Wire1 */
  if (sda == 21 && scl == 47) {
    bus = &Wire;
    Wire.begin(sda, scl, 100000);
  } else {
    bus = &Wire1;
    Wire1.begin(sda, scl, 100000);
  }
  delay(20);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(400);
  bootOtaPinNextResetTo0();
  Serial.println();
  Serial.println("=== PCA talk SDA=21 SCL=47 preferred ===");

  for (unsigned i = 0; i < sizeof(kPairs) / sizeof(kPairs[0]); i++) {
    int sda = kPairs[i][0], scl = kPairs[i][1];
    Serial.printf("\n==== pair SDA=%d SCL=%d ====\n", sda, scl);
    openPair(sda, scl);
    char tag[28];
    snprintf(tag, sizeof(tag), "SDA=%d SCL=%d", sda, scl);
    int n = i2cScan(*bus, tag);
    uint8_t a = findRealPca(*bus, n);
    if (!a) continue;
    pcaAddr = a;
    pinSda = sda;
    pinScl = scl;
    Serial.printf("USING PCA @0x%02X on SDA=%d SCL=%d\n", pcaAddr, pinSda, pinScl);
    break;
  }

  if (!pcaAddr) {
    Serial.println("FAIL: no real PCA (need MODE1 read). Wire SDA=21 SCL=47 VCC=3.3V GND OE=GND");
    return;
  }

  if (!initPca(pcaAddr)) {
    Serial.println("FAIL: PCA init");
    return;
  }
  Serial.printf("PCA OK — wiggling CH0-2 on SDA=%d SCL=%d\n", pinSda, pinScl);
  for (uint8_t ch = 0; ch < 3; ch++) wiggle(ch);
  for (uint8_t ch = 0; ch < 3; ch++) setAngle(ch, 90);
  delay(300);
  for (uint8_t ch = 0; ch < 3; ch++) setPwm(ch, 0, 0x1000);
  Serial.println("=== DONE — servos should have moved ===");
}

void loop() {
  bootOta0PollButton();
  static uint32_t last;
  if (millis() - last < 3000) return;
  last = millis();
  if (!pcaAddr) {
    openPair(21, 47);
    int n = i2cScan(Wire, "retry 21/47");
    pcaAddr = findRealPca(Wire, n);
    if (pcaAddr) {
      pinSda = 21;
      pinScl = 47;
      Serial.printf("PCA appeared @0x%02X SDA=21 SCL=47\n", pcaAddr);
      initPca(pcaAddr);
    }
    return;
  }
  uint8_t m1 = 0;
  if (read8(pcaAddr, REG_MODE1, m1))
    Serial.printf("alive MODE1=0x%02X SDA=%d SCL=%d\n", m1, pinSda, pinScl);
  else
    Serial.println("PCA read lost");
}
