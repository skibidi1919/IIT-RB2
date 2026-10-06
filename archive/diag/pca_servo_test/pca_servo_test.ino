/*
 * Meowler — scan I2C then PCA9685 servo test @ 100 Hz
 * FQBN: arduino:avr:nano:cpu=atmega328old
 * Serial: 115200
 * Close Serial Monitor before upload.
 */

#include <Wire.h>

static const uint8_t PCA = 0x40;
static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_LED0 = 0x06;
static const uint8_t REG_PRESCALE = 0xFE;
static const uint16_t TICK_MIN = 300;
static const uint16_t TICK_MAX = 1200;

static bool g_ok = false;

static void busRecover() {
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(A5, HIGH);
    delayMicroseconds(5);
    digitalWrite(A5, LOW);
    delayMicroseconds(5);
  }
  pinMode(A4, OUTPUT);
  digitalWrite(A4, LOW);
  delayMicroseconds(5);
  digitalWrite(A5, HIGH);
  delayMicroseconds(5);
  digitalWrite(A4, HIGH);
  delayMicroseconds(5);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
}

static bool write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool read8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(PCA);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)PCA, 1) != 1) return false;
  val = Wire.read();
  return true;
}

static bool setPWM(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t reg = REG_LED0 + 4 * ch;
  return write8(reg + 0, on & 0xFF) && write8(reg + 1, on >> 8) &&
         write8(reg + 2, off & 0xFF) && write8(reg + 3, off >> 8);
}

static bool setFreq(uint16_t hz) {
  // Write-only sequence (avoid repeated-start reads — flaky on this bus)
  uint8_t p = (uint8_t)(25000000.0f / (4096.0f * (float)hz) - 1.0f + 0.5f);
  if (!write8(REG_MODE1, 0x10)) return false;  // sleep
  delay(2);
  if (!write8(REG_PRESCALE, p)) return false;
  if (!write8(REG_MODE1, 0x00)) return false;  // wake
  delay(5);
  if (!write8(REG_MODE1, 0xA1)) return false;  // AI + restart + allcall
  write8(REG_MODE2, 0x04);
  return true;
}

static void setAngle(uint8_t ch, int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  uint16_t ticks = (uint16_t)map(deg, 0, 180, TICK_MIN, TICK_MAX);
  setPWM(ch, 0, ticks);
  Serial.print(F("CH"));
  Serial.print(ch);
  Serial.print('=');
  Serial.println(deg);
}

/* returns bit0=saw PCA 0x40 */
static uint8_t scan() {
  uint8_t n = 0;
  uint8_t flags = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  FOUND 0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      if (a == 0x40) {
        Serial.print(F(" PCA"));
        flags |= 1;
      }
      if (a == 0x29) Serial.print(F(" TOF"));
      if (a == 0x70) Serial.print(F(" ALLCALL"));
      Serial.println();
      n++;
    }
  }
  Serial.print(F("devices="));
  Serial.println(n);
  return flags;
}

static void testChannel(uint8_t ch) {
  Serial.println();
  Serial.print(F(">>> wiggle CH"));
  Serial.println(ch);
  setAngle(0, 90);
  setAngle(1, 90);
  setAngle(2, 90);
  delay(300);
  for (uint8_t i = 0; i < 3; i++) {
    setAngle(ch, 50);
    delay(550);
    setAngle(ch, 130);
    delay(550);
  }
  setAngle(ch, 90);
  delay(300);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("======== SCAN + SERVO ========"));
  busRecover();
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(30000, true);  // don't hang forever on stuck I2C

  // Keep trying — battery / PCA VCC can come up late
  for (uint8_t round = 0; round < 40; round++) {
    Serial.print(F("scan #"));
    Serial.println(round);
    uint8_t flags = scan();
    // Trust scan hit — a second probe often false-NACKs on this bus
    if (flags & 1) {
      g_ok = true;
      Serial.println(F("PCA seen in scan"));
      break;
    }
    busRecover();
    Wire.begin();
    Wire.setClock(100000);
    delay(250);
  }

  if (!g_ok) {
    Serial.println(F("GIVE UP — PCA not on bus"));
    Serial.println(F("Need: PCA VCC=Nano 5V, V+=battery, GND common"));
    return;
  }

  Serial.println(F("PCA seen — set 100Hz"));
  bool freq_ok = false;
  for (uint8_t t = 0; t < 8; t++) {
    Wire.clearWireTimeoutFlag();
    busRecover();
    Wire.begin();
    Wire.setClock(100000);
    Wire.setWireTimeout(30000, true);
    delay(20);
    Serial.print(F("freq try "));
    Serial.println(t);
    if (setFreq(100)) {
      freq_ok = true;
      Serial.println(F("freq OK"));
      break;
    }
    delay(50);
  }
  if (!freq_ok) {
    Serial.println(F("setFreq FAIL — still trying servo writes"));
  }

  // Drive channels even if freq verify failed (defaults ~200Hz still move)
  testChannel(0);
  testChannel(1);
  testChannel(2);
  Serial.println(F("DONE isolation — looping"));
}

void loop() {
  if (!g_ok) {
    delay(500);
    busRecover();
    Wire.begin();
    Wire.setClock(100000);
    Wire.beginTransmission(PCA);
    if (Wire.endTransmission() == 0) {
      write8(REG_MODE1, 0x00);
      delay(5);
      if (setFreq(100)) {
        g_ok = true;
        Serial.println(F("PCA recovered"));
      }
    }
    return;
  }

  static uint8_t ch = 0;
  static uint8_t phase = 0;
  static uint32_t t0 = 0;
  if (millis() - t0 < 700) return;
  t0 = millis();

  setAngle(0, 90);
  setAngle(1, 90);
  setAngle(2, 90);
  int deg = (phase == 0) ? 50 : (phase == 1) ? 130 : 90;
  setAngle(ch, deg);
  phase++;
  if (phase > 2) {
    phase = 0;
    ch = (uint8_t)((ch + 1) % 3);
  }
}
