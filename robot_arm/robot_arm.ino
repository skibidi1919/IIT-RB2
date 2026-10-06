/*
 * Meowler robot arm — Nano + PCA9685 + VL53L0X + BNO085 + nanopb
 *
 * I2C (shared bus A4/A5 @ 100 kHz):
 *   PCA9685 @ 0x40
 *   VL53L0X @ 0x29 (XSHUT must be HIGH)
 *   BNO085/BNO080 @ 0x4A (or 0x4B if ADR/DI high)
 *     VIN->5V (Adafruit regulator) or 3V3, GND, SDA, SCL
 *     RST/INT unused (-1); do not wire RST to D8 (OE)
 *
 * User move @ 500 deg/s linear lerp + torque settle cycle.
 * Frame: 0xA5 | len | protobuf | xor(len+payload)
 */

#include <Wire.h>
#include <math.h>
#include <VL53L0X.h>
#define BNO_USE_I2C
#include <7Semi_BNO08x.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "meowler.pb.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const uint8_t PCA_ADDR = 0x40;
static const uint8_t TOF_ADDR_DEFAULT = 0x29;
static const uint8_t TOF_ADDR_ALT = 0x70;  // try when 0x29 missing (user GY-530 / bus map)
static const uint8_t IMU_ADDR_A = 0x4A;   // Adafruit default
static const uint8_t IMU_ADDR_B = 0x4B;   // ADR/DI pulled high
static const uint8_t PIN_OE = 8;
static const uint8_t FRAME_MAGIC = 0xA5;

static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_LED0_ON_L = 0x06;
static const uint8_t REG_PRESCALE = 0xFE;
static const uint8_t MODE1_ALLCALL = 0x01;
static const uint8_t MODE1_SLEEP = 0x10;
static const uint8_t MODE1_AI = 0x20;
static const uint8_t MODE1_RESTART = 0x80;
static const uint8_t MODE2_OUTDRV = 0x04;

// Pulse width in microseconds → PCA tick count at 50 Hz (20 ms / 4096).
// 500–2500 µs is the usual 0–180° range for hobby servos.
static const uint16_t SERVO_US_MIN = 500;
static const uint16_t SERVO_US_MAX = 2500;
static const uint32_t OSC_HZ = 25000000UL;
// Hobby servos expect ~50 Hz (20 ms frame). NOT 50 kHz — that breaks pulse timing.
static const float SERVO_PWM_HZ = 50.0f;
static const uint16_t SERVO_PERIOD_US = 20000;

static const uint8_t CH_BASE = 0;
static const uint8_t CH_HEIGHT = 1;
static const uint8_t CH_GRIP = 2;

static const uint16_t SPEED_DPS = 500;  // deg/s
static const uint16_t MIN_MOVE_MS = 250;  // never cut torque before servo can arrive
static const uint16_t HOLD_MS = 300;      // settle at target before idle
// Keep torque on after user moves so physical angle matches command.
static const bool USER_LIMP_AFTER_MOVE = false;
static const uint16_t RELAX_MS = 400;     // only used if limp enabled

enum : uint8_t {
  ACT_NONE = 0,
  ACT_STATUS = 1,
  ACT_CENTER = 2,
  ACT_DEMO = 3,
};

enum Phase : uint8_t {
  PHASE_IDLE = 0,
  PHASE_DRIVE = 1,    // torque on, lerp to target
  PHASE_HOLD = 2,     // torque on, parked at target
  PHASE_RELAX = 3,    // torque off
  PHASE_RESEAT = 4,   // torque on, lerp to same target again
};

float curB = 90, curH = 90, curG = 90;
float startB = 90, startH = 90, startG = 90;
float tgtB = 90, tgtH = 90, tgtG = 90;
float holdB = 90, holdH = 90, holdG = 90;
uint32_t moveStartMs = 0;
uint32_t moveDurMs = 0;
uint32_t phaseStartMs = 0;
uint32_t lastStatusMs = 0;
Phase phase = PHASE_IDLE;
bool settleAfterMove = true;
bool pcaOk = false;
bool torqueOn = true;
bool tofOk = false;
uint8_t tofAddr = 0;
uint16_t distanceMm = 0;
uint32_t lastTofMs = 0;
uint32_t lastIdleStatusMs = 0;

bool imuOk = false;
uint8_t imuAddr = 0;
int32_t yawCdeg = 0;
int32_t pitchCdeg = 0;
int32_t rollCdeg = 0;

VL53L0X tof;
static BnoI2CBus bnoBus(Wire, -1, -1, IMU_ADDR_A, 100000UL, -1, -1);
static BNO08x_7Semi bno(bnoBus);

static const int8_t DEMO_PTS[][3] = {
    {45, 90, 90}, {135, 90, 90}, {90, 90, 90},
    {90, 60, 90}, {90, 120, 90}, {90, 90, 90},
    {90, 90, 40}, {90, 90, 140}, {90, 90, 90},
};
uint8_t demoIdx = 0;
bool demoActive = false;

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int clampAngle(int deg) {
  if (deg < 0) return 0;
  if (deg > 180) return 180;
  return deg;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

uint8_t pcaRead8(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(PCA_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool pcaWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool pcaWriteLED(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t reg = REG_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}

void pcaSoftReset() {
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(10);
}

bool pcaSetPWMFreq(float freq) {
  float prescaleval = (OSC_HZ / (freq * 4096.0f)) + 0.5f - 1.0f;
  if (prescaleval < 3) prescaleval = 3;
  if (prescaleval > 255) prescaleval = 255;
  uint8_t prescale = (uint8_t)prescaleval;
  uint8_t oldmode = pcaRead8(REG_MODE1);
  if (!pcaWrite8(REG_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP)) return false;
  if (!pcaWrite8(REG_PRESCALE, prescale)) return false;
  if (!pcaWrite8(REG_MODE1, oldmode)) return false;
  delay(5);
  // Keep ALLCALL enabled for servo board defaults; ToF probe may clear it later.
  return pcaWrite8(REG_MODE1, (oldmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI | MODE1_ALLCALL);
}

void pcaDisableAllCall() {
  uint8_t m = pcaRead8(REG_MODE1);
  if (m == 0xFF) return;
  pcaWrite8(REG_MODE1, (m & ~MODE1_ALLCALL) | MODE1_AI);
}

uint8_t i2cReadReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool looksLikeVl53(uint8_t addr) {
  // IDENTIFICATION_MODEL_ID @ 0xC0 == 0xEE on VL53L0X
  return i2cReadReg(addr, 0xC0) == 0xEE;
}

bool startTofAt(uint8_t addr) {
  if (!i2cPresent(addr)) return false;
  tof.setAddressForce(addr);
  tof.setTimeout(120);
  if (!tof.init()) return false;
  tof.setSignalRateLimit(0.1);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  tof.setMeasurementTimingBudget(33000);
  tof.startContinuous(50);
  tofAddr = addr;
  tofOk = true;
  return true;
}

void writeServoDeg(uint8_t ch, float deg) {
  if (!pcaOk) return;
  int d = clampAngle((int)lroundf(deg));
  uint32_t us = (uint32_t)map(d, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
  uint16_t pulselen = (uint16_t)((us * 4096UL) / SERVO_PERIOD_US);
  if (pulselen > 4095) pulselen = 4095;
  pcaWriteLED(ch, 0, pulselen);
}

void applyCurrent() {
  if (!torqueOn || !pcaOk) return;
  writeServoDeg(CH_BASE, curB);
  writeServoDeg(CH_HEIGHT, curH);
  writeServoDeg(CH_GRIP, curG);
}

void setTorqueOff() {
  if (!pcaOk) return;
  pcaWriteLED(CH_BASE, 0, 4096);
  pcaWriteLED(CH_HEIGHT, 0, 4096);
  pcaWriteLED(CH_GRIP, 0, 4096);
  torqueOn = false;
}

void setTorqueOn() {
  torqueOn = true;
  // Rewrite pulses after full-off (0,4096) so outputs restart cleanly
  applyCurrent();
  delayMicroseconds(200);
  applyCurrent();
}

bool initPca() {
  pcaOk = false;
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, LOW);
  if (!i2cPresent(PCA_ADDR)) return false;
  pcaSoftReset();
  delay(5);
  if (!pcaWrite8(REG_MODE2, MODE2_OUTDRV)) return false;
  if (!pcaWrite8(REG_MODE1, MODE1_AI | MODE1_ALLCALL)) return false;
  if (!pcaSetPWMFreq(SERVO_PWM_HZ)) return false;
  if (!pcaWriteLED(0, 0, 300)) return false;
  delay(2);
  uint16_t off = pcaRead8(0x08) | ((uint16_t)pcaRead8(0x09) << 8);
  if (off != 300) return false;
  pcaOk = true;
  return true;
}

bool initTof() {
  tofOk = false;
  tofAddr = 0;
  distanceMm = 0;

  // Prefer real VL53L0X default address.
  if (looksLikeVl53(TOF_ADDR_DEFAULT) && startTofAt(TOF_ADDR_DEFAULT)) return true;
  if (i2cPresent(TOF_ADDR_DEFAULT) && startTofAt(TOF_ADDR_DEFAULT)) return true;

  // 0x70 is usually PCA9685 ALLCALL. Drop ALLCALL, then probe 0x70 as VL53.
  if (pcaOk) pcaDisableAllCall();
  delay(2);
  if (looksLikeVl53(TOF_ADDR_ALT) && startTofAt(TOF_ADDR_ALT)) return true;
  if (i2cPresent(TOF_ADDR_ALT) && startTofAt(TOF_ADDR_ALT)) return true;

  return false;
}

void quatToYawPitchRoll(float qi, float qj, float qk, float qr,
                        float *yaw, float *pitch, float *roll) {
  // Same convention as Adafruit quaternion_yaw_pitch_roll example
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;
  float sqr = qr * qr;
  *yaw = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  float sinp = -2.0f * (qi * qk - qj * qr);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  *pitch = asinf(sinp);
  *roll = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

bool initImuAt(uint8_t addr) {
  if (!i2cPresent(addr)) return false;
  bnoBus.addr = addr;
  if (!bno.begin()) return false;
  Wire.setClock(100000);
  delay(30);
  if (!bno.enableGameRotationVector(50)) return false;
  imuAddr = addr;
  imuOk = true;
  return true;
}

bool initImu() {
  imuOk = false;
  imuAddr = 0;
  yawCdeg = pitchCdeg = rollCdeg = 0;
  // Adafruit STEMMA default is 0x4A; some modules use 0x4B
  if (initImuAt(IMU_ADDR_A)) return true;
  if (initImuAt(IMU_ADDR_B)) return true;
  return false;
}

void updateImu() {
  if (!imuOk) return;
  bno.processData();
  float qi, qj, qk, qr;
  if (!bno.getGameRotationVector(qi, qj, qk, qr)) return;
  float yaw, pitch, roll;
  quatToYawPitchRoll(qi, qj, qk, qr, &yaw, &pitch, &roll);
  const float rad2cdeg = (18000.0f / (float)M_PI);
  yawCdeg = (int32_t)lroundf(yaw * rad2cdeg);
  pitchCdeg = (int32_t)lroundf(pitch * rad2cdeg);
  rollCdeg = (int32_t)lroundf(roll * rad2cdeg);
}

void updateTof() {
  if (!tofOk) return;
  if (millis() - lastTofMs < 50) return;
  lastTofMs = millis();
  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred() || mm > 8000) {
    // keep last good reading but mark soft fail in status via high sentinel
    distanceMm = 0;
  } else {
    distanceMm = mm;
  }
}

uint8_t xorChecksum(uint8_t len, const uint8_t *payload) {
  uint8_t x = len;
  for (uint8_t i = 0; i < len; i++) x ^= payload[i];
  return x;
}

static float absf(float v) { return v < 0 ? -v : v; }

void sendStatus() {
  meowler_ArmStatus st = meowler_ArmStatus_init_zero;
  if (phase == PHASE_DRIVE || phase == PHASE_RESEAT) {
    st.base = (uint16_t)clampAngle((int)lroundf(curB));
    st.height = (uint16_t)clampAngle((int)lroundf(curH));
    st.grip = (uint16_t)clampAngle((int)lroundf(curG));
  } else {
    st.base = (uint16_t)clampAngle((int)lroundf(holdB));
    st.height = (uint16_t)clampAngle((int)lroundf(holdH));
    st.grip = (uint16_t)clampAngle((int)lroundf(holdG));
  }
  st.pca_ok = pcaOk;
  st.moving = (phase != PHASE_IDLE) || demoActive;
  st.distance_mm = distanceMm;
  st.tof_ok = tofOk && (distanceMm > 0);
  st.imu_ok = imuOk;
  st.yaw_cdeg = yawCdeg;
  st.pitch_cdeg = pitchCdeg;
  st.roll_cdeg = rollCdeg;

  uint8_t buf[meowler_ArmStatus_size];
  pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&stream, meowler_ArmStatus_fields, &st)) return;
  uint8_t len = (uint8_t)stream.bytes_written;
  Serial.write(FRAME_MAGIC);
  Serial.write(len);
  Serial.write(buf, len);
  Serial.write(xorChecksum(len, buf));
}

void startLerpTo(float b, float h, float g, Phase nextPhase) {
  setTorqueOn();
  startB = curB;
  startH = curH;
  startG = curG;
  tgtB = clampf(b, 0, 180);
  tgtH = clampf(h, 0, 180);
  tgtG = clampf(g, 0, 180);

  float maxDelta = absf(tgtB - startB);
  float dh = absf(tgtH - startH);
  float dg = absf(tgtG - startG);
  if (dh > maxDelta) maxDelta = dh;
  if (dg > maxDelta) maxDelta = dg;

  uint32_t dur = (uint32_t)lroundf((maxDelta * 1000.0f) / (float)SPEED_DPS);
  if (dur < MIN_MOVE_MS) dur = MIN_MOVE_MS;
  moveDurMs = dur;
  moveStartMs = millis();
  phase = nextPhase;
}

void startUserMove(float b, float h, float g) {
  holdB = clampf(b, 0, 180);
  holdH = clampf(h, 0, 180);
  holdG = clampf(g, 0, 180);
  settleAfterMove = USER_LIMP_AFTER_MOVE;
  startLerpTo(holdB, holdH, holdG, PHASE_DRIVE);
}

void startDemoMove(float b, float h, float g) {
  holdB = clampf(b, 0, 180);
  holdH = clampf(h, 0, 180);
  holdG = clampf(g, 0, 180);
  settleAfterMove = false;
  startLerpTo(holdB, holdH, holdG, PHASE_DRIVE);
}

void updateMotion() {
  uint32_t now = millis();

  if (phase == PHASE_HOLD) {
    applyCurrent();  // keep commanding target while holding
    if (now - phaseStartMs < HOLD_MS) return;
    if (settleAfterMove) {
      setTorqueOff();
      phase = PHASE_RELAX;
      phaseStartMs = now;
    } else {
      phase = PHASE_IDLE;
    }
    sendStatus();
    lastStatusMs = now;
    return;
  }

  if (phase == PHASE_RELAX) {
    if (now - phaseStartMs < RELAX_MS) return;
    // Re-drive from last commanded software pose; do not teleport cur*
    // (teleport made reseat a no-op when the arm had sagged).
    startLerpTo(holdB, holdH, holdG, PHASE_RESEAT);
    sendStatus();
    lastStatusMs = now;
    return;
  }

  if (phase == PHASE_IDLE) {
    // Hold PWM so angle stays at the last command
    if (torqueOn) applyCurrent();
    return;
  }

  if (phase != PHASE_DRIVE && phase != PHASE_RESEAT) return;

  float t = (moveDurMs == 0) ? 1.0f : (float)(now - moveStartMs) / (float)moveDurMs;
  if (t >= 1.0f) {
    curB = tgtB;
    curH = tgtH;
    curG = tgtG;
    applyCurrent();
    if (phase == PHASE_DRIVE) {
      // Always hold powered at target before optional limp
      phase = PHASE_HOLD;
      phaseStartMs = now;
    } else {
      // End of reseat — stay powered
      phase = PHASE_IDLE;
      setTorqueOn();
    }
    sendStatus();
    lastStatusMs = now;
    return;
  }

  curB = startB + (tgtB - startB) * t;
  curH = startH + (tgtH - startH) * t;
  curG = startG + (tgtG - startG) * t;
  applyCurrent();
  if (now - lastStatusMs >= 50) {
    sendStatus();
    lastStatusMs = now;
  }
}

void handleCommand(const meowler_ArmCommand &cmd) {
  uint8_t action = cmd.has_action ? cmd.action : ACT_NONE;

  if (action == ACT_STATUS) {
    sendStatus();
    return;
  }
  if (action == ACT_CENTER) {
    demoActive = false;
    startUserMove(90, 90, 90);
    sendStatus();
    return;
  }
  if (action == ACT_DEMO) {
    demoActive = true;
    demoIdx = 0;
    startDemoMove(DEMO_PTS[0][0], DEMO_PTS[0][1], DEMO_PTS[0][2]);
    sendStatus();
    return;
  }

  float b = cmd.has_base ? (float)cmd.base : holdB;
  float h = cmd.has_height ? (float)cmd.height : holdH;
  float g = cmd.has_grip ? (float)cmd.grip : holdG;
  if (cmd.has_base || cmd.has_height || cmd.has_grip) {
    demoActive = false;
    startUserMove(b, h, g);
  }
  sendStatus();
}

void pollSerial() {
  static uint8_t state = 0;
  static uint8_t len = 0;
  static uint8_t got = 0;
  static uint8_t payload[meowler_ArmCommand_size];

  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (state == 0) {
      if (b == FRAME_MAGIC) state = 1;
    } else if (state == 1) {
      len = b;
      if (len > meowler_ArmCommand_size) state = 0;
      else if (len == 0) state = 3;
      else {
        got = 0;
        state = 2;
      }
    } else if (state == 2) {
      payload[got++] = b;
      if (got >= len) state = 3;
    } else {
      state = 0;
      if (b != xorChecksum(len, payload)) continue;
      meowler_ArmCommand cmd = meowler_ArmCommand_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(payload, len);
      if (!pb_decode(&stream, meowler_ArmCommand_fields, &cmd)) continue;
      handleCommand(cmd);
    }
  }
}

void updateDemo() {
  if (!demoActive || phase != PHASE_IDLE) return;
  demoIdx++;
  if (demoIdx >= (uint8_t)(sizeof(DEMO_PTS) / sizeof(DEMO_PTS[0]))) {
    demoActive = false;
    sendStatus();
    return;
  }
  startDemoMove(DEMO_PTS[demoIdx][0], DEMO_PTS[demoIdx][1], DEMO_PTS[demoIdx][2]);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, LOW);
  Wire.begin();
  Wire.setClock(100000);
  initPca();
  initTof();
  initImu();
  setTorqueOn();
  delay(50);
  updateTof();
  updateImu();
  sendStatus();
}

void loop() {
  pollSerial();
  updateMotion();
  updateDemo();
  updateTof();
  updateImu();
  // Stream sensors even when idle so GUI stays live
  if (phase == PHASE_IDLE && (millis() - lastIdleStatusMs) >= 200) {
    lastIdleStatusMs = millis();
    sendStatus();
  }
}
