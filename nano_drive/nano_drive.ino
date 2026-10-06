/*
 * Meowler Nano — USB LinkMessage + meow_i2c / meow_pca @ 100 Hz
 *
 * FQBN: arduino:avr:nano:cpu=atmega328old
 * USB: 500000 8N1
 *
 * I2C A4/A5: PCA9685@0x40 (meow_pca) + VL53L0X@0x29  OE→GND
 * CH0=base  CH1=height  CH2=grip
 *
 * L298N: ENA=D9 ENB=D10  IN1..IN4=A0..A3
 * Encoders: M1 D3/D4  M2 D5/D6
 * TCS3200: S0=D7 S1=D8 S2=D11 S3=D12 OUT=D2 LED=D13
 */

#include <math.h>
#include <string.h>

#include "meow_i2c.h"
#include "meow_pca9685.h"
#include "meowler.pb.h"
#include "meow_frame.h"

static const uint32_t USB_BAUD = 500000UL;
static const uint16_t TELEM_PERIOD_MS = 50;
static const uint16_t COLOR_PERIOD_MS = 80;
static const uint32_t PULSE_TO_US = 8000UL;
static const uint8_t TOF_ADDR = 0x29;

static const uint8_t PIN_ENA = 9, PIN_ENB = 10;
static const uint8_t PIN_IN1 = A0, PIN_IN2 = A1, PIN_IN3 = A2, PIN_IN4 = A3;
static const uint8_t PIN_M1_C1 = 3, PIN_M1_C2 = 4, PIN_M2_C1 = 5, PIN_M2_C2 = 6;
static const uint8_t PIN_S0 = 7, PIN_S1 = 8, PIN_S2 = 11, PIN_S3 = 12;
static const uint8_t PIN_OUT = 2, PIN_LED = 13;

static const uint8_t FLAG_NANO = 1, FLAG_PCA = 2, FLAG_TOF = 4;
static const uint8_t FLAG_MOVING = 16, FLAG_CAL = 32;

static int32_t encL = 0, encR = 0;
static uint8_t prevM1 = 0, prevM2 = 0;
static int16_t cmdL = 0, cmdR = 0;
static MeowFrameParser usbParser;
static uint32_t lastTelemMs = 0, lastColorMs = 0;
static uint16_t telemSeq = 0;
static uint8_t txFrame[112];

static bool pcaOk = false;
static bool moving = false;
static int curB = 90, curH = 90, curG = 90;

static bool tofOk = false;
static uint16_t distanceMm = 0;

static const int8_t DEMO_PTS[][3] PROGMEM = {
    {45, 90, 90}, {135, 90, 90}, {90, 90, 90}, {90, 60, 90},
    {90, 120, 90}, {90, 90, 90}, {90, 90, 40}, {90, 90, 140}, {90, 90, 90},
};

static void pollUsb();

static float gainR = 1.08f, gainG = 1.35f, gainB = 0.88f;
static bool colorCal = false;
static float emaR = 0, emaG = 0, emaB = 0, emaC = 0;
static bool emaInit = false;
static uint8_t sticky = 0;
static meowler_ColorLabel colorLabel = meowler_ColorLabel_COLOR_NONE;
static uint8_t colorConf = 0;
static uint16_t rUs = 0, gUs = 0, bUs = 0, cUs = 0;

static int clampAngle(int d) { return d < 0 ? 0 : (d > 180 ? 180 : d); }

// ---- Drive (ENA/ENB = PWM D9/D10) ----
static void applySide(int16_t spd, uint8_t inA, uint8_t inB, uint8_t en) {
  if (spd > 255) spd = 255;
  if (spd < -255) spd = -255;
  uint8_t mag = (uint8_t)(spd < 0 ? -spd : spd);
  if (spd > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(en, mag);
  } else if (spd < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(en, mag);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(en, 0);
  }
}
static void applyDrive() {
  applySide(cmdL, PIN_IN1, PIN_IN2, PIN_ENA);
  applySide(cmdR, PIN_IN3, PIN_IN4, PIN_ENB);
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

// ---- Arm via meow_pca @ 100 Hz (Adafruit-equivalent ticks) ----
static void moveServo(uint8_t ch, int angle) {
  if (!pcaOk) return;
  angle = clampAngle(angle);
  meow_i2c::setTimeout(50000UL);
  meow_pca::setAngle(ch, angle);
  delay(30);
}

static void moveArmTo(int b, int h, int g) {
  moving = true;
  b = clampAngle(b);
  h = clampAngle(h);
  g = clampAngle(g);
  curB = b;
  curH = h;
  curG = g;
  moveServo(meow_pca::CH_BASE, b);
  pollUsb();
  moveServo(meow_pca::CH_HEIGHT, h);
  pollUsb();
  moveServo(meow_pca::CH_GRIP, g);
  moving = false;
}

static bool initPca() {
  meow_i2c::setTimeout(100000UL);
  if (!meow_pca::begin(100)) {
    pcaOk = false;
    return false;
  }
  pcaOk = true;
  moveArmTo(90, 90, 90);
  return true;
}

/* VL53 init is heavy + wedges Wire on this Nano build — defer (probe-only). */
static bool initTof() {
  tofOk = false;
  distanceMm = 0;
  if (!meow_i2c::probe(TOF_ADDR)) return false;
  // Keep bus clean for PCA; enable ranging later once RAM/init is solid.
  return false;
}

static void runDemo() {
  if (!pcaOk) return;
  uint8_t n = sizeof(DEMO_PTS) / sizeof(DEMO_PTS[0]);
  for (uint8_t i = 0; i < n; i++) {
    int b = (int)pgm_read_byte(&DEMO_PTS[i][0]);
    int h = (int)pgm_read_byte(&DEMO_PTS[i][1]);
    int g = (int)pgm_read_byte(&DEMO_PTS[i][2]);
    moveArmTo(b, h, g);
    delay(200);
    pollUsb();
  }
}

// ---- TCS3200 (compact) ----
static uint32_t readPulse() {
  uint32_t t = pulseIn(PIN_OUT, LOW, PULSE_TO_US);
  if (!t) t = pulseIn(PIN_OUT, HIGH, PULSE_TO_US);
  return t;
}
static uint32_t readCh(bool s2, bool s3) {
  digitalWrite(PIN_S2, s2);
  digitalWrite(PIN_S3, s3);
  delayMicroseconds(400);
  return readPulse();
}
static float invP(uint32_t p) { return p ? 1000.0f / (float)p : 0; }
static void updateColor() {
  if (millis() - lastColorMs < COLOR_PERIOD_MS) return;
  lastColorMs = millis();
  uint32_t rp = readCh(false, false);
  pollUsb();
  uint32_t bp = readCh(false, true);
  pollUsb();
  uint32_t cp = readCh(true, false);
  pollUsb();
  uint32_t gp = readCh(true, true);
  rUs = (uint16_t)rp;
  gUs = (uint16_t)gp;
  bUs = (uint16_t)bp;
  cUs = (uint16_t)cp;
  if (!(rp || gp || bp)) return;
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
    emaR = 0.45f * R + 0.55f * emaR;
    emaG = 0.45f * G + 0.55f * emaG;
    emaB = 0.45f * B + 0.55f * emaB;
    emaC = 0.45f * ci + 0.55f * emaC;
  }
  float mx = fmaxf(emaR, fmaxf(emaG, emaB)), mn = fminf(emaR, fminf(emaG, emaB));
  float v = mx, d = mx - mn, s = mx < 1e-6f ? 0 : d / mx;
  static uint8_t weak = 0;
  if (ci < 0.01f || v < 0.1f || s < 0.08f) {
    if (++weak >= 8) {
      sticky = 0;
      colorLabel = meowler_ColorLabel_COLOR_NONE;
      colorConf = 0;
    }
    return;
  }
  weak = 0;
  float rDom = emaR - fmaxf(emaG, emaB), gDom = emaG - fmaxf(emaR, emaB);
  float yPair = fminf(emaR, emaG) - emaB;
  float sR = fmaxf(0.0f, rDom) * 2.4f;
  float sG = fmaxf(0.0f, gDom) * 3.0f;
  float sY = fmaxf(0.0f, yPair) * 2.0f;
  uint8_t best = 1;
  float bs = sR;
  if (sY > bs) {
    best = 2;
    bs = sY;
  }
  if (sG > bs) {
    best = 3;
    bs = sG;
  }
  if (bs < 0.18f) return;
  sticky = best;
  colorLabel = (meowler_ColorLabel)best;
  colorConf = (uint8_t)constrain((int)lround(s * (colorCal ? 100 : 85)), 0, 99);
}
static void calColor() {
  uint32_t r = readCh(false, false), b = readCh(false, true), g = readCh(true, true);
  float ri = invP(r), gi = invP(g), bi = invP(b);
  float peak = fmaxf(ri, fmaxf(gi, bi));
  if (peak > 1e-6f) {
    gainR = peak / fmaxf(ri, 1e-6f);
    gainG = peak / fmaxf(gi, 1e-6f);
    gainB = peak / fmaxf(bi, 1e-6f);
    colorCal = true;
    emaInit = false;
    sticky = 0;
  }
}

// ---- USB nanopb ----
static void sendFrameMsg(const meowler_LinkMessage &msg) {
  // AVR TX buffer is 64 → availableForWrite() max is 63; never require 64.
  if (Serial.availableForWrite() < 16) return;
  size_t n = meow_frame_encode(meowler_LinkMessage_fields, &msg, txFrame, sizeof(txFrame));
  if (n) Serial.write(txFrame, n);
}
static void emitLog(const char *text) {
  meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
  msg.which_payload = meowler_LinkMessage_log_tag;
  msg.payload.log.t_ms = millis();
  msg.payload.log.level = meowler_LogLevel_LOG_INFO;
  size_t n = strnlen(text, 39);
  memcpy(msg.payload.log.text.bytes, text, n);
  msg.payload.log.text.size = n;
  sendFrameMsg(msg);
}
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
  t.base = (uint16_t)curB;
  t.height = (uint16_t)curH;
  t.grip = (uint16_t)curG;
  t.distance_mm = distanceMm;
  uint16_t f = FLAG_NANO;
  if (pcaOk) f |= FLAG_PCA;
  if (tofOk && distanceMm) f |= FLAG_TOF;
  if (moving) f |= FLAG_MOVING;
  if (colorCal) f |= FLAG_CAL;
  t.flags = f;
  sendFrameMsg(msg);
}

static void applyRobotCmd(const meowler_RobotCommand &cmd) {
  if (cmd.has_action) {
    switch (cmd.action) {
      case meowler_RobotAction_ROBOT_ACT_STOP:
        cmdL = 0;
        cmdR = 0;
        applyDrive();
        break;
      case meowler_RobotAction_ROBOT_ACT_STATUS:
      case meowler_RobotAction_ROBOT_ACT_PING:
        break;
      case meowler_RobotAction_ROBOT_ACT_CAL_COLOR:
        calColor();
        emitLog(colorCal ? "CAL OK" : "CAL FAIL");
        break;
      case meowler_RobotAction_ROBOT_ACT_ARM_CENTER:
        moveArmTo(90, 90, 90);
        break;
      case meowler_RobotAction_ROBOT_ACT_ARM_DEMO:
        runDemo();
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
    cmdL = l;
    cmdR = r;
    applyDrive();
  }
  if (cmd.has_arm) {
    if (cmd.arm.has_action) {
      if (cmd.arm.action == 2) moveArmTo(90, 90, 90);
      else if (cmd.arm.action == 3) runDemo();
    }
    if (cmd.arm.has_base || cmd.arm.has_height || cmd.arm.has_grip) {
      int b = cmd.arm.has_base ? (int)cmd.arm.base : curB;
      int h = cmd.arm.has_height ? (int)cmd.arm.height : curH;
      int g = cmd.arm.has_grip ? (int)cmd.arm.grip : curG;
      moveArmTo(b, h, g);
    }
  }
  sendTelemetry();
}

static void pollUsb() {
  while (Serial.available() > 0) {
    uint8_t plen = 0;
    if (meow_parser_feed(&usbParser, (uint8_t)Serial.read(), &plen)) {
      meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
      if (meow_frame_decode(meowler_LinkMessage_fields, &msg, usbParser.buf, plen) &&
          msg.which_payload == meowler_LinkMessage_cmd_tag) {
        applyRobotCmd(msg.payload.cmd);
      }
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(80);
  meow_parser_init(&usbParser);
  emitLog("BOOT");
  meow_i2c::begin(100000UL, 100000UL);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_M1_C1, INPUT_PULLUP);
  pinMode(PIN_M1_C2, INPUT_PULLUP);
  pinMode(PIN_M2_C1, INPUT_PULLUP);
  pinMode(PIN_M2_C2, INPUT_PULLUP);
  prevM1 = ((digitalRead(PIN_M1_C1) ? 1 : 0) << 1) | (digitalRead(PIN_M1_C2) ? 1 : 0);
  prevM2 = ((digitalRead(PIN_M2_C1) ? 1 : 0) << 1) | (digitalRead(PIN_M2_C2) ? 1 : 0);

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_S0, HIGH);
  digitalWrite(PIN_S1, LOW);
  digitalWrite(PIN_LED, HIGH);

  applyDrive();

  emitLog("I2C start");
  {
    uint8_t found[6];
    uint8_t n = meow_i2c::scan(found, 6);
    char msg[28];
    msg[0] = 'I';
    msg[1] = '2';
    msg[2] = 'C';
    msg[3] = ':';
    uint8_t o = 4;
    if (n == 0) {
      emitLog("I2C:none");
    } else {
      static const char kHex[] = "0123456789ABCDEF";
      for (uint8_t i = 0; i < n && o + 3 < sizeof(msg); i++) {
        if (i) msg[o++] = ',';
        msg[o++] = kHex[found[i] >> 4];
        msg[o++] = kHex[found[i] & 0x0F];
      }
      msg[o] = 0;
      emitLog(msg);
    }
  }
  bool pca = initPca();
  emitLog(pca ? "PCA 100Hz OK" : "PCA FAIL");
  initTof();  // probe only — do not run Pololu init (wedges bus)
  emitLog("ready");
}

void loop() {
  pollUsb();
  pollEncoders();
  applyDrive();
  updateColor();
  pollUsb();

  uint32_t now = millis();
  if (now - lastTelemMs >= TELEM_PERIOD_MS) {
    lastTelemMs = now;
    sendTelemetry();
  }
}
