/*
 * Meowler control panel firmware
 * FQBN: arduino:avr:nano:cpu=atmega328old
 * USB 115200 text protocol
 *
 * PCA9685 @0x40 100Hz (Wire A4/A5)
 *   CH0 base  CH1 height  CH2 grip  (CH4-9 held off)
 * Drive both wheels from Nano GPIO (PCA motor channels were flaky):
 *   M1 = L298N OUT3/OUT4 : ENB=D10 IN3=A2 IN4=A3  (pull ENB jumper)
 *   M2 = L298N OUT1/OUT2 : ENA=D9  IN1=A0 IN2=A1  (pull ENA jumper)
 * VL53L0X @0x29 on Wire
 * BNO08x soft-I2C D7=SDA D8=SCL @0x4A/0x4B
 * Encoders: M1 D3/D4  M2 D5/D6
 * Wheel diameter 43 mm
 *
 * Cmds: A/B/H/G/C  D,l,r  S  M  Z  ?  T telem
 */

#include <Wire.h>
#include <math.h>
#include <VL53L0X.h>
#define BNO_USE_I2C
#include <7Semi_BNO08x.h>
#include "soft_bno_i2c.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const uint8_t PCA = 0x40;
static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_LED0 = 0x06;
static const uint8_t REG_PRESCALE = 0xFE;
/* 100 Hz frame = 10000 µs. Hobby servos: ~500 µs = 0°, ~2500 µs = 180°. */
static const uint16_t SERVO_US_MIN = 500;
static const uint16_t SERVO_US_MAX = 2500;
static const uint16_t SERVO_PERIOD_US = 10000;

/* M1 = OUT3/4  (cmdL) */
static const uint8_t PIN_ENB = 10, PIN_IN3 = A2, PIN_IN4 = A3;
/* M2 = OUT1/2  (cmdR) */
static const uint8_t PIN_ENA = 9, PIN_IN1 = A0, PIN_IN2 = A1;

static const uint8_t PIN_M1_C1 = 3, PIN_M1_C2 = 4;
static const uint8_t PIN_M2_C1 = 5, PIN_M2_C2 = 6;
static const uint8_t PIN_BNO_SDA = 7, PIN_BNO_SCL = 8;

static const float WHEEL_DIAM_MM = 43.0f;
/* N20 hall encoder ~12 CPR motor × gear; tune if distance looks off */
static const float STEPS_PER_REV = 600.0f;
static const float MM_PER_STEP = (WHEEL_DIAM_MM * (float)M_PI) / STEPS_PER_REV;

static const uint8_t IMU_A = 0x4A, IMU_B = 0x4B;

static bool pcaOk = false, tofOk = false, imuOk = false;
static int curB = 90, curH = 90, curG = 90;
static int16_t cmdL = 0, cmdR = 0;
static uint16_t distMm = 0;
static int16_t tofDispMm = 0;
static uint16_t tofOriginMm = 0;
static bool tofOriginSet = false;
static int32_t encL = 0, encR = 0;
static uint8_t prevM1 = 0, prevM2 = 0;
static int32_t yawCdeg = 0, pitchCdeg = 0, rollCdeg = 0;
static uint32_t lastTofMs = 0, lastTelemMs = 0, lastImuMs = 0;

static VL53L0X tof;
static SoftBnoI2CBus softBus(PIN_BNO_SDA, PIN_BNO_SCL, IMU_A, 50000UL);
static BNO08x_7Semi bno(softBus);

static char lineBuf[48];
static uint8_t lineLen = 0;

static bool write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static void setPwmRaw(uint8_t ch, uint16_t on, uint16_t off) {
  if (!pcaOk || ch > 15) return;
  uint8_t reg = REG_LED0 + 4 * ch;
  Wire.beginTransmission(PCA);
  Wire.write(reg);
  Wire.write((uint8_t)(on & 0xFF));
  Wire.write((uint8_t)(on >> 8));
  Wire.write((uint8_t)(off & 0xFF));
  Wire.write((uint8_t)(off >> 8));
  Wire.endTransmission();
}

static void pcaFullOff(uint8_t ch) { setPwmRaw(ch, 0, 0x1000); }

static uint16_t degToTicks(int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  uint32_t us = (uint32_t)map(deg, 0, 180, (long)SERVO_US_MIN, (long)SERVO_US_MAX);
  uint16_t ticks = (uint16_t)((us * 4096UL) / SERVO_PERIOD_US);
  if (ticks > 4095) ticks = 4095;
  return ticks;
}

static void setAngle(uint8_t ch, int deg) {
  if (!pcaOk) return;
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  uint16_t off = degToTicks(deg);
  setPwmRaw(ch, 0, off);
  /* second write — some PCA boards drop the first extreme pulse */
  delayMicroseconds(300);
  setPwmRaw(ch, 0, off);
}

static void applyArm(int b, int h, int g) {
  if (b < 0) b = 0;
  if (b > 180) b = 180;
  if (h < 0) h = 0;
  if (h > 180) h = 180;
  if (g < 0) g = 0;
  if (g > 180) g = 180;
  curB = b;
  curH = h;
  curG = g;
  setAngle(0, b);
  setAngle(1, h);
  setAngle(2, g);
  Serial.print(F("OK "));
  Serial.print(curB);
  Serial.print(',');
  Serial.print(curH);
  Serial.print(',');
  Serial.println(curG);
}

static void clampMag(int16_t *spd, uint8_t *mag) {
  if (*spd > 255) *spd = 255;
  if (*spd < -255) *spd = -255;
  *mag = (uint8_t)(*spd < 0 ? -*spd : *spd);
  if (*mag > 0 && *mag < 80) *mag = 80;  // N20 kick
}

static void applySideGpio(int16_t spd, uint8_t inA, uint8_t inB, uint8_t en) {
  uint8_t mag = 0;
  clampMag(&spd, &mag);
  if (spd > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(en, mag ? mag : 255);
  } else if (spd < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(en, mag ? mag : 255);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(en, 0);
  }
}

static void holdPcaMotorsOff() {
  if (!pcaOk) return;
  for (uint8_t ch = 4; ch <= 9; ch++) pcaFullOff(ch);
}

static void applyDrive() {
  applySideGpio(cmdL, PIN_IN3, PIN_IN4, PIN_ENB);  // M1 OUT3/4
  applySideGpio(cmdR, PIN_IN1, PIN_IN2, PIN_ENA);  // M2 OUT1/2
}

static void setDrive(int16_t l, int16_t r) {
  cmdL = l;
  cmdR = r;
  applyDrive();
  Serial.print(F("OK D"));
  Serial.print(cmdL);
  Serial.print(',');
  Serial.println(cmdR);
}

static void pollEncoders() {
  uint8_t m1 = ((digitalRead(PIN_M1_C1) ? 1 : 0) << 1) | (digitalRead(PIN_M1_C2) ? 1 : 0);
  uint8_t m2 = ((digitalRead(PIN_M2_C1) ? 1 : 0) << 1) | (digitalRead(PIN_M2_C2) ? 1 : 0);
  static const int8_t lut[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  encL += lut[(prevM1 << 2) | m1];
  encR += lut[(prevM2 << 2) | m2];
  prevM1 = m1;
  prevM2 = m2;
}

static int32_t wheelMm(int32_t steps) {
  return (int32_t)lroundf((float)steps * MM_PER_STEP);
}

static void zeroOdom() {
  encL = 0;
  encR = 0;
  tofOriginMm = distMm;
  tofOriginSet = tofOk && distMm > 0;
  tofDispMm = 0;
  Serial.print(F("OK Z enc=0 tof0="));
  Serial.println(tofOriginMm);
}

static bool initPcaOnce() {
  Wire.beginTransmission(PCA);
  if (Wire.endTransmission() != 0) return false;
  write8(REG_MODE1, 0x10);
  delay(2);
  write8(REG_PRESCALE, 0x3C);
  write8(REG_MODE1, 0x00);
  delay(5);
  write8(REG_MODE1, 0xA1);
  write8(REG_MODE2, 0x04);
  return true;
}

static bool initTofOnce() {
  Wire.beginTransmission(0x29);
  if (Wire.endTransmission() != 0) return false;
  tof.setTimeout(100);
  if (!tof.init()) return false;
  tof.setMeasurementTimingBudget(33000);
  tof.startContinuous(50);
  return true;
}

static void quatYPR(float qi, float qj, float qk, float qr, float *y, float *p, float *r) {
  float sqi = qi * qi, sqj = qj * qj, sqk = qk * qk, sqr = qr * qr;
  *y = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  float sinp = -2.0f * (qi * qk - qj * qr);
  if (sinp > 1) sinp = 1;
  if (sinp < -1) sinp = -1;
  *p = asinf(sinp);
  *r = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

static bool initImuAt(uint8_t a) {
  softBus.addr = a;
  softBus.begin();
  if (!softBus.probe()) return false;
  if (!bno.begin()) return false;
  delay(20);
  if (!bno.enableGameRotationVector(50)) return false;
  imuOk = true;
  return true;
}

static bool initImu() {
  imuOk = false;
  if (initImuAt(IMU_A) || initImuAt(IMU_B)) return true;
  return false;
}

static void updateImu() {
  if (!imuOk || millis() - lastImuMs < 20) return;
  lastImuMs = millis();
  bno.processData();
  float qi, qj, qk, qr;
  if (!bno.getGameRotationVector(qi, qj, qk, qr)) return;
  float y, p, r;
  quatYPR(qi, qj, qk, qr, &y, &p, &r);
  const float k = 18000.0f / (float)M_PI;
  yawCdeg = (int32_t)lroundf(y * k);
  pitchCdeg = (int32_t)lroundf(p * k);
  rollCdeg = (int32_t)lroundf(r * k);
}

static void updateTof() {
  if (!tofOk || millis() - lastTofMs < 50) return;
  lastTofMs = millis();
  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred() || mm > 8000) distMm = 0;
  else distMm = mm;
  if (tofOriginSet && distMm > 0)
    tofDispMm = (int16_t)((int32_t)tofOriginMm - (int32_t)distMm);
  else
    tofDispMm = 0;
}

static void sendTelem() {
  /* T,tof,cmdL,cmdR,b,h,g,pca,tofOk,encL,encR,wLmm,wRmm,disp,imu,yaw,pitch,roll */
  Serial.print(F("T,"));
  Serial.print(distMm);
  Serial.print(',');
  Serial.print(cmdL);
  Serial.print(',');
  Serial.print(cmdR);
  Serial.print(',');
  Serial.print(curB);
  Serial.print(',');
  Serial.print(curH);
  Serial.print(',');
  Serial.print(curG);
  Serial.print(',');
  Serial.print(pcaOk ? 1 : 0);
  Serial.print(',');
  Serial.print(tofOk ? 1 : 0);
  Serial.print(',');
  Serial.print(encL);
  Serial.print(',');
  Serial.print(encR);
  Serial.print(',');
  Serial.print(wheelMm(encL));
  Serial.print(',');
  Serial.print(wheelMm(encR));
  Serial.print(',');
  Serial.print(tofDispMm);
  Serial.print(',');
  Serial.print(imuOk ? 1 : 0);
  Serial.print(',');
  Serial.print(yawCdeg);
  Serial.print(',');
  Serial.print(pitchCdeg);
  Serial.print(',');
  Serial.println(rollCdeg);
}

static int parseInt(const char **pp, int *out) {
  const char *s = *pp;
  while (*s == ' ' || *s == ',') s++;
  bool neg = false;
  if (*s == '-') {
    neg = true;
    s++;
  }
  if (*s < '0' || *s > '9') return 0;
  int v = 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (*s - '0');
    s++;
  }
  *out = neg ? -v : v;
  *pp = s;
  return 1;
}

static void ensurePca() {
  if (pcaOk) return;
  pcaOk = initPcaOnce();
  if (pcaOk) Serial.println(F("PCA OK"));
}

static void handleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return;
  char cmd = s[0];
  if (cmd >= 'a' && cmd <= 'z') cmd = (char)(cmd - 32);

  if (cmd == 'S') {
    setDrive(0, 0);
    return;
  }
  if (cmd == 'Z') {
    zeroOdom();
    return;
  }
  if (cmd == 'M') {
    holdPcaMotorsOff();
    Serial.println(F("MOTOR TEST M1 OUT3/4 (D10/A2/A3)"));
    setDrive(255, 0);
    delay(900);
    Serial.println(F("MOTOR TEST M2 OUT1/2 (D9/A0/A1)"));
    setDrive(0, 255);
    delay(900);
    Serial.println(F("MOTOR TEST BOTH"));
    setDrive(255, 255);
    delay(900);
    setDrive(0, 0);
    Serial.println(F("MOTOR TEST DONE"));
    Serial.println(F("WIRE M1 OUT3/4: ENB<-D10 IN3<-A2 IN4<-A3 (pull ENB jumper)"));
    Serial.println(F("WIRE M2 OUT1/2: ENA<-D9  IN1<-A0 IN2<-A1 (pull ENA jumper)"));
    return;
  }
  if (cmd == 'D') {
    int l = 0, r = 0;
    const char *p = s + 1;
    if (!parseInt(&p, &l) || !parseInt(&p, &r)) {
      Serial.println(F("ERR D"));
      return;
    }
    setDrive((int16_t)l, (int16_t)r);
    return;
  }

  ensurePca();

  if (cmd == 'C') {
    applyArm(90, 90, 90);
    return;
  }
  if (cmd == '?') {
    sendTelem();
    return;
  }
  if (cmd == 'A') {
    int b = curB, h = curH, g = curG;
    const char *p = s + 1;
    if (!parseInt(&p, &b) || !parseInt(&p, &h) || !parseInt(&p, &g)) {
      Serial.println(F("ERR"));
      return;
    }
    applyArm(b, h, g);
    return;
  }
  if (cmd == 'B' || cmd == 'H' || cmd == 'G') {
    int v = 90;
    const char *p = s + 1;
    if (!parseInt(&p, &v)) {
      Serial.println(F("ERR"));
      return;
    }
    if (v < 0) v = 0;
    if (v > 180) v = 180;
    if (cmd == 'B') {
      curB = v;
      setAngle(0, curB);
      Serial.print(F("OK B"));
      Serial.println(curB);
    } else if (cmd == 'H') {
      curH = v;
      setAngle(1, curH);
      Serial.print(F("OK H"));
      Serial.println(curH);
    } else {
      curG = v;
      setAngle(2, curG);
      Serial.print(F("OK G"));
      Serial.println(curG);
    }
    return;
  }
  Serial.println(F("ERR"));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);

  pinMode(PIN_M1_C1, INPUT_PULLUP);
  pinMode(PIN_M1_C2, INPUT_PULLUP);
  pinMode(PIN_M2_C1, INPUT_PULLUP);
  pinMode(PIN_M2_C2, INPUT_PULLUP);
  prevM1 = ((digitalRead(PIN_M1_C1) ? 1 : 0) << 1) | (digitalRead(PIN_M1_C2) ? 1 : 0);
  prevM2 = ((digitalRead(PIN_M2_C1) ? 1 : 0) << 1) | (digitalRead(PIN_M2_C2) ? 1 : 0);

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

  Wire.begin();
  Wire.setClock(100000);

  pcaOk = initPcaOnce();
  if (!pcaOk) {
    delay(300);
    pcaOk = initPcaOnce();
  }
  tofOk = initTofOnce();
  initImu();

  if (pcaOk) {
    applyArm(90, 90, 90);
    holdPcaMotorsOff();
  }
  setDrive(0, 0);
  if (tofOk && distMm == 0) {
    delay(60);
    updateTof();
  }
  zeroOdom();

  Serial.print(F("READY pca="));
  Serial.print(pcaOk ? 1 : 0);
  Serial.print(F(" tof="));
  Serial.print(tofOk ? 1 : 0);
  Serial.print(F(" imu="));
  Serial.println(imuOk ? 1 : 0);
  Serial.println(F("DRIVE M1=OUT3/4 D10/A2/A3  M2=OUT1/2 D9/A0/A1"));
  Serial.println(F("ENC=D3-6  BNO soft-I2C D7/D8"));
  Serial.print(F("WHEEL_D_MM=43 steps/rev="));
  Serial.println((int)STEPS_PER_REV);

  Serial.println(F("MOTOR BLIP"));
  setDrive(255, 255);
  delay(400);
  setDrive(0, 0);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen) handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }

  pollEncoders();
  applyDrive();
  if (pcaOk) holdPcaMotorsOff();
  updateTof();
  updateImu();

  if (millis() - lastTelemMs >= 200) {
    lastTelemMs = millis();
    sendTelem();
  }
}
