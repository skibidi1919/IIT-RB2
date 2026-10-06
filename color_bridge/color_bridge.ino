/*
 * Meowler ROBOT ESP32 (NodeMCU-32S)
 *   TCS3200 color + BNO085 + ESP-NOW + UART→Nano
 *   Nano owns: L298N wheels + PCA9685 arm + VL53L0X ToF (SoftSerial nanopb)
 *
 * Laptop --USB--> hub_bridge --ESP-NOW--> THIS BOARD --UART--> nano_drive
 *
 * TCS3200: S0=4 S1=2 S2=18 S3=19 OUT=5 LED=13
 * I2C: SDA=21 SCL=22  BNO@0x4A/0x4B
 * UART Nano: TX2=17->D11(RX)  RX2=16<-D12(TX via divider)
 *
 * FQBN: esp32:esp32:nodemcu-32s:UploadSpeed=115200
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define BNO_USE_I2C
#include <7Semi_BNO08x.h>

#include "meowler.pb.h"
#include "meow_frame.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- pins ----
static const uint8_t PIN_S0 = 4, PIN_S1 = 2, PIN_S2 = 18, PIN_S3 = 19, PIN_OUT = 5, PIN_LED = 13;
static const uint8_t PIN_NANO_RX = 16, PIN_NANO_TX = 17;
static const uint8_t PIN_SDA = 21, PIN_SCL = 22;

static const uint8_t IMU_A = 0x4A, IMU_B = 0x4B;

static const uint8_t FLAG_NANO = 1, FLAG_PCA = 2, FLAG_TOF = 4, FLAG_IMU = 8;
static const uint8_t FLAG_MOVING = 16, FLAG_CAL = 32;

// ---- state ----
HardwareSerial &Nano = Serial2;
static uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool espnowOk = false;
static uint16_t telemSeq = 0;

static MeowFrameParser nanoParser, espParser;
static bool nanoOk = false;
static uint32_t lastNanoMs = 0;
static int16_t cmdL = 0, cmdR = 0;
static int32_t encL = 0, encR = 0;

// mirror from NanoStatus (arm + ToF)
static bool pcaOk = false;
static bool tofOk = false;
static bool armMoving = false;
static uint16_t armBase = 90, armHeight = 90, armGrip = 90;
static uint16_t distanceMm = 0;

// color
static float gainR = 1.08f, gainG = 1.35f, gainB = 0.88f;
static bool colorCal = false;
static float emaR = 0, emaG = 0, emaB = 0, emaC = 0;
static bool emaInit = false;
static const char *stickyName = nullptr;
static meowler_ColorLabel colorLabel = meowler_ColorLabel_COLOR_NONE;
static uint8_t colorConf = 0;
static uint16_t rUs = 0, gUs = 0, bUs = 0, cUs = 0;

// sensors (IMU stays on robot ESP)
static BnoI2CBus bnoBus(Wire, -1, -1, IMU_A, 100000UL, -1, -1);
static BNO08x_7Semi bno(bnoBus);
static bool imuOk = false;
static int32_t yawCdeg = 0, pitchCdeg = 0, rollCdeg = 0;

static bool i2cPresent(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

static void sendEspFrame(const uint8_t *frame, size_t n) {
  if (espnowOk && n && n <= 250) esp_now_send(broadcastAddr, frame, n);
}

static void emitLog(meowler_LogLevel level, const char *text) {
  meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
  msg.which_payload = meowler_LinkMessage_log_tag;
  msg.payload.log.t_ms = millis();
  msg.payload.log.level = level;
  size_t n = strnlen(text, 39);
  memcpy(msg.payload.log.text.bytes, text, n);
  msg.payload.log.text.size = n;
  uint8_t frame[64];
  size_t fl = meow_frame_encode(meowler_LinkMessage_fields, &msg, frame, sizeof(frame));
  if (fl) sendEspFrame(frame, fl);
  Serial.printf("[%u] %s\n", (unsigned)level, text);
}

// ---- IMU ----
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
  if (!i2cPresent(a)) return false;
  bnoBus.addr = a;
  if (!bno.begin()) return false;
  Wire.setClock(100000);
  delay(20);
  if (!bno.enableGameRotationVector(50)) return false;
  imuOk = true;
  return true;
}
static bool initImu() {
  if (initImuAt(IMU_A)) return true;
  if (initImuAt(IMU_B)) return true;
  return false;
}
static void updateImu() {
  if (!imuOk) return;
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

// ---- TCS3200 (compact) ----
static const uint8_t SAMPLES = 5;
static const uint32_t PULSE_TO = 25000UL;
static int cmpU32(const void *a, const void *b) {
  uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
  return (x > y) - (x < y);
}
static uint32_t medianPulse() {
  uint32_t buf[SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    uint32_t t = pulseIn(PIN_OUT, LOW, PULSE_TO);
    if (!t) t = pulseIn(PIN_OUT, HIGH, PULSE_TO);
    if (t) buf[n++] = t;
    yield();
  }
  if (!n) return 0;
  qsort(buf, n, sizeof(uint32_t), cmpU32);
  return buf[n / 2];
}
static uint32_t readCh(bool s2, bool s3) {
  digitalWrite(PIN_S2, s2);
  digitalWrite(PIN_S3, s3);
  delay(2);
  pulseIn(PIN_OUT, LOW, 5000UL);
  return medianPulse();
}
static float invP(uint32_t p) { return p ? 1000.0f / (float)p : 0; }
static void rgbHSV(float R, float G, float B, float *h, float *s, float *v) {
  float mx = fmaxf(R, fmaxf(G, B)), mn = fminf(R, fminf(G, B));
  *v = mx;
  float d = mx - mn;
  *s = mx < 1e-6f ? 0 : d / mx;
  if (*s < 1e-4f) {
    *h = 0;
    return;
  }
  if (mx == R) *h = 60 * fmodf(((G - B) / d) + 6, 6);
  else if (mx == G) *h = 60 * (((B - R) / d) + 2);
  else *h = 60 * (((R - G) / d) + 4);
}
static const char *classify(float R, float G, float B, float h, float s, float v, float ci, float avg) {
  static uint8_t weak = 0;
  if (ci < 0.01f || avg > 160 || v < 0.1f || s < 0.08f) {
    if (++weak >= 8) {
      stickyName = nullptr;
      return "NONE";
    }
    return stickyName ? stickyName : "NONE";
  }
  weak = 0;
  float rDom = R - fmaxf(G, B), gDom = G - fmaxf(R, B), yPair = fminf(R, G) - B;
  float yBal = 1.0f - fminf(1.0f, fabsf(R - G) / 0.40f);
  float sR = fmaxf(0.0f, rDom) * 2.4f + fmaxf(0.0f, R - G) * 0.7f;
  if (h < 22 || h >= 340) sR += 0.4f * s;
  float sG = fmaxf(0.0f, gDom) * 3.0f + fmaxf(0.0f, G - R) * 1.1f;
  if (h >= 60 && h <= 180) sG += 0.55f * s;
  float sY = fmaxf(0.0f, yPair) * 2.0f * fmaxf(0.35f, yBal);
  if (h >= 32 && h <= 78) sY += 0.35f * s;
  if (rDom > 0.1f) sY *= 0.3f;
  if (gDom > 0.08f) sY *= 0.15f;
  float k = 0.35f + 0.65f * fminf(1.0f, fmaxf(0.0f, (s - 0.06f) / 0.45f));
  sR *= k;
  sY *= k;
  sG *= k;
  const char *best = "RED";
  float bs = sR;
  if (sY > bs) {
    best = "YELLOW";
    bs = sY;
  }
  if (sG > bs) {
    best = "GREEN";
    bs = sG;
  }
  if (bs < 0.18f) return stickyName ? stickyName : "NONE";
  stickyName = best;
  return best;
}
static meowler_ColorLabel labelOf(const char *n) {
  if (n && n[0] == 'R') return meowler_ColorLabel_COLOR_RED;
  if (n && n[0] == 'Y') return meowler_ColorLabel_COLOR_YELLOW;
  if (n && n[0] == 'G') return meowler_ColorLabel_COLOR_GREEN;
  return meowler_ColorLabel_COLOR_NONE;
}

// ---- Nano UART (drive + arm) ----
static void nanoSend(const meowler_NanoCommand &cmd) {
  uint8_t frame[48];
  size_t n = meow_frame_encode(meowler_NanoCommand_fields, &cmd, frame, sizeof(frame));
  if (n) Nano.write(frame, n);
}

static void nanoDrive(int16_t l, int16_t r) {
  cmdL = l;
  cmdR = r;
  meowler_NanoCommand cmd = meowler_NanoCommand_init_zero;
  cmd.has_drive = true;
  cmd.drive.left = l;
  cmd.drive.right = r;
  nanoSend(cmd);
}

static void nanoArm(const meowler_ArmCommand &arm) {
  meowler_NanoCommand cmd = meowler_NanoCommand_init_zero;
  cmd.has_arm = true;
  cmd.arm = arm;
  // SoftSerial is half-duplex — retry so we land in a Nano RX gap
  for (uint8_t i = 0; i < 3; i++) {
    nanoSend(cmd);
    delay(30);
    pollNano();
  }
  emitLog(meowler_LogLevel_LOG_INFO, "arm -> nano");
}

static void nanoAction(uint32_t action) {
  meowler_NanoCommand cmd = meowler_NanoCommand_init_zero;
  cmd.has_action = true;
  cmd.action = action;
  for (uint8_t i = 0; i < 3; i++) {
    nanoSend(cmd);
    delay(30);
    pollNano();
  }
}

static void pollNano() {
  while (Nano.available() > 0) {
    uint8_t plen = 0;
    if (meow_parser_feed(&nanoParser, (uint8_t)Nano.read(), &plen)) {
      meowler_NanoStatus st = meowler_NanoStatus_init_zero;
      if (meow_frame_decode(meowler_NanoStatus_fields, &st, nanoParser.buf, plen)) {
        if (st.has_drive) {
          encL = st.drive.enc_left;
          encR = st.drive.enc_right;
          cmdL = st.drive.left;
          cmdR = st.drive.right;
        }
        armBase = (uint16_t)st.base;
        armHeight = (uint16_t)st.height;
        armGrip = (uint16_t)st.grip;
        pcaOk = st.pca_ok;
        armMoving = st.moving;
        distanceMm = (uint16_t)st.distance_mm;
        tofOk = st.tof_ok;
        nanoOk = true;
        lastNanoMs = millis();
      }
    }
  }
}

// ---- telemetry / commands ----
static void sendTelemetry() {
  meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
  msg.which_payload = meowler_LinkMessage_telem_tag;
  meowler_RobotTelemetry &t = msg.payload.telem;
  t.seq = ++telemSeq;
  t.t_ms = millis();
  t.color = colorLabel;
  t.conf = colorConf;
  t.r_us = rUs;
  t.g_us = gUs;
  t.b_us = bUs;
  t.c_us = cUs;
  t.left = cmdL;
  t.right = cmdR;
  t.enc_l = encL;
  t.enc_r = encR;
  t.base = armBase;
  t.height = armHeight;
  t.grip = armGrip;
  t.distance_mm = distanceMm;
  t.yaw_cdeg = yawCdeg;
  t.pitch_cdeg = pitchCdeg;
  t.roll_cdeg = rollCdeg;
  uint16_t f = 0;
  if (nanoOk && millis() - lastNanoMs < 800) f |= FLAG_NANO;
  if (pcaOk) f |= FLAG_PCA;
  if (tofOk) f |= FLAG_TOF;
  if (imuOk) f |= FLAG_IMU;
  if (armMoving) f |= FLAG_MOVING;
  if (colorCal) f |= FLAG_CAL;
  t.flags = f;

  uint8_t frame[112];
  size_t n = meow_frame_encode(meowler_LinkMessage_fields, &msg, frame, sizeof(frame));
  if (n) sendEspFrame(frame, n);
}

static void applyRobotCmd(const meowler_RobotCommand &cmd) {
  if (cmd.has_action) {
    switch (cmd.action) {
      case meowler_RobotAction_ROBOT_ACT_STOP:
        nanoDrive(0, 0);
        emitLog(meowler_LogLevel_LOG_INFO, "STOP");
        break;
      case meowler_RobotAction_ROBOT_ACT_STATUS:
      case meowler_RobotAction_ROBOT_ACT_PING:
        sendTelemetry();
        emitLog(meowler_LogLevel_LOG_DEBUG, "ping/status");
        break;
      case meowler_RobotAction_ROBOT_ACT_CAL_COLOR: {
        uint32_t r = readCh(false, false), b = readCh(false, true), g = readCh(true, true);
        float ri = invP(r), gi = invP(g), bi = invP(b);
        float peak = fmaxf(ri, fmaxf(gi, bi));
        if (peak > 1e-6f) {
          gainR = peak / fmaxf(ri, 1e-6f);
          gainG = peak / fmaxf(gi, 1e-6f);
          gainB = peak / fmaxf(bi, 1e-6f);
          colorCal = true;
          emaInit = false;
          stickyName = nullptr;
          emitLog(meowler_LogLevel_LOG_INFO, "color CAL OK");
        } else {
          emitLog(meowler_LogLevel_LOG_ERROR, "color CAL FAIL");
        }
        break;
      }
      case meowler_RobotAction_ROBOT_ACT_ARM_CENTER:
        nanoAction(5);
        emitLog(meowler_LogLevel_LOG_INFO, "arm center -> nano");
        break;
      case meowler_RobotAction_ROBOT_ACT_ARM_DEMO:
        nanoAction(6);
        emitLog(meowler_LogLevel_LOG_INFO, "arm demo -> nano");
        break;
      default:
        break;
    }
  }
  if (cmd.has_drive) {
    int16_t l = cmd.drive.left, r = cmd.drive.right;
    if (l > 255) l = 255;
    if (l < -255) l = -255;
    if (r > 255) r = 255;
    if (r < -255) r = -255;
    nanoDrive(l, r);
  }
  if (cmd.has_arm) nanoArm(cmd.arm);
}

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  for (int i = 0; i < len; i++) {
    uint8_t plen = 0;
    if (meow_parser_feed(&espParser, data[i], &plen)) {
      meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
      if (meow_frame_decode(meowler_LinkMessage_fields, &msg, espParser.buf, plen) &&
          msg.which_payload == meowler_LinkMessage_cmd_tag) {
        applyRobotCmd(msg.payload.cmd);
      }
    }
  }
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onEspNowRecvNew(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  onEspNowRecv(info ? info->src_addr : nullptr, data, len);
}
#endif

static bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(40);
  if (esp_now_init() != ESP_OK) return false;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_now_register_recv_cb(onEspNowRecvNew);
#else
  esp_now_register_recv_cb(onEspNowRecv);
#endif
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_S0, HIGH);
  digitalWrite(PIN_S1, LOW);
  digitalWrite(PIN_LED, HIGH);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  Nano.begin(57600, SERIAL_8N1, PIN_NANO_RX, PIN_NANO_TX);  // match Nano SoftSerial
  meow_parser_init(&nanoParser);
  meow_parser_init(&espParser);
  espnowOk = initEspNow();

  char boot[40];
  snprintf(boot, sizeof(boot), "boot espnow=%d mac=", espnowOk ? 1 : 0);
  emitLog(meowler_LogLevel_LOG_INFO, boot);
  Serial.println(WiFi.macAddress());

  emitLog(meowler_LogLevel_LOG_INFO, "PCA+VL53 on Nano (UART)");

  if (initImu()) emitLog(meowler_LogLevel_LOG_INFO, "BNO08x OK");
  else emitLog(meowler_LogLevel_LOG_WARN, "BNO08x missing");

  emitLog(meowler_LogLevel_LOG_INFO, "ROBOT READY");
}

void loop() {
  pollNano();
  updateImu();

  uint32_t rp = readCh(false, false);
  uint32_t bp = readCh(false, true);
  uint32_t cp = readCh(true, false);
  uint32_t gp = readCh(true, true);
  rUs = (uint16_t)rp;
  gUs = (uint16_t)gp;
  bUs = (uint16_t)bp;
  cUs = (uint16_t)cp;

  if (rp || gp || bp) {
    float ri = invP(rp) * gainR, gi = invP(gp) * gainG, bi = invP(bp) * gainB, ci = invP(cp);
    float peak = fmaxf(ri, fmaxf(gi, bi));
    if (peak < 1e-9f) peak = 1;
    float R = ri / peak, G = gi / peak, B = bi / peak;
    if (!emaInit) {
      emaR = R;
      emaG = G;
      emaB = B;
      emaC = ci;
      emaInit = true;
    } else {
      emaR = 0.4f * R + 0.6f * emaR;
      emaG = 0.4f * G + 0.6f * emaG;
      emaB = 0.4f * B + 0.6f * emaB;
      emaC = 0.4f * ci + 0.6f * emaC;
    }
    float h, s, v;
    rgbHSV(emaR, emaG, emaB, &h, &s, &v);
    const char *name = classify(emaR, emaG, emaB, h, s, v, emaC, (rp + gp + bp) / 3.0f);
    colorLabel = labelOf(name);
    colorConf = (uint8_t)constrain((int)lroundf(s * (colorCal ? 100 : 85)), 0, 99);
    Serial.printf("R=%u G=%u B=%u C=%u => %s conf=%u d=%u yaw=%ld pca=%d\n", (unsigned)rp,
                  (unsigned)gp, (unsigned)bp, (unsigned)cp, name, (unsigned)colorConf,
                  (unsigned)distanceMm, (long)yawCdeg, pcaOk ? 1 : 0);
  }

  static uint32_t lastTelem = 0;
  if (millis() - lastTelem >= 50) {
    lastTelem = millis();
    sendTelemetry();
  }
}
