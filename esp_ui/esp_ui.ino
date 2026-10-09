/*
 * Meowler robot brain — Freenove ESP32-S3-WROOM N16R8 (camera REMOVED)
 * FQBN tip:
 *   esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB
 *   plus --build-property build.partitions=partitions (sketch partitions.csv: 8MB ota_0 + 8MB ota_1)
 * USB: CH343 UART port @ 115200 (same text protocol as arm_ui)
 *
 * Camera OFF + SD unused → those GPIOs free for robot I/O.
 * Avoid only: 35–37 (OPI PSRAM). Careful: 0/3/45/46 (strapping).
 *
 * I2C: PCA @0x40 + VL53 @0x29 + BNO08x @0x4A/0x4B on Wire SDA=21 SCL=47.
 * BNO VIN: GPIO38 HIGH ≈3.3 V rail (board 5 V is unsafe for 3V3-only IMU).
 * GPIO14 = FLASH LED (not I2C). Do not use 38/39 as I2C (38 = IMU power).
 * Drive (3.3V → L298N; pull ENA/ENB jumpers):
 *   M1 OUT3/4: ENB=10 IN3=13 IN4=8
 *   M2 OUT1/2: ENA=9  IN1=11 IN2=12
 * Enc: M1 4/5  M2 6/7
 * TCS3200: S0=5V S1=GND  S2=15 S3=16 OUT=17 LED=18
 * PCA: CH0/1/2 = MG90 (move→relax→reassert) · CH4 SG90-360 conveyor
 *
 * Network: WiFi STA + TCP :3333 — length-prefixed protobuf (proto/meowler.proto)
 *   frame = uint32le len | RobotToClient / ClientToRobot
 * Serial: same framed RobotToClient (Log / optional mirror); text cmds still accepted
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <stdarg.h>
#include <math.h>
#include <VL53L0X.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "meow_frame.h"
#include "color_ryg.h"
#include "ota_pb_update.h"
#include "boot_ota0.h"
#include <stdlib.h>
#include <string.h>

enum : uint32_t { LOG_DBG = 0, LOG_INFO = 1, LOG_WARN = 2, LOG_ERR = 3 };

#ifndef WIFI_SSID
#define WIFI_SSID "DarshIshaan"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "Darsh@3001"
#endif
/* SoftAP fallback when PC hotspot / STA is unreachable (PC joins this SSID) */
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "Meowler"
#endif
#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS "Darsh@3001"
#endif
/* Static STA IP always ends in .222 on whatever subnet DHCP/gateway gives */
#ifndef WIFI_HOST_OCTET
#define WIFI_HOST_OCTET 222
#endif
#ifndef WIFI_IP
#define WIFI_IP 192, 168, 137, WIFI_HOST_OCTET /* bootstrap only; overridden after DHCP */
#endif
#ifndef WIFI_GW
#define WIFI_GW 192, 168, 137, 1
#endif
#ifndef WIFI_SN
#define WIFI_SN 255, 255, 255, 0
#endif
#ifndef WIFI_DNS
#define WIFI_DNS 192, 168, 137, 1
#endif
static const uint16_t NET_PORT = 3333;
static const uint8_t MAX_CLIENTS = 3;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* PCA + VL53 + BNO on Wire 21/47. GPIO14 = FLASH LED. No OTA chainload. */
static const int PIN_I2C0_SDA = 21, PIN_I2C0_SCL = 47;  // Wire = PCA + VL53 + BNO
/* GPIO HIGH ≈ 3.3 V — powers 3V3-only BNO08x (do NOT feed board 5 V into VIN). */
static const uint8_t PIN_IMU_3V3 = 38;
static int i2cSda = PIN_I2C0_SDA, i2cScl = PIN_I2C0_SCL;
static bool i2cLocked = false;

static uint8_t pcaAddr = 0x40;
static TwoWire *pcaBus = &Wire;
static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_LED0 = 0x06;
static const uint8_t REG_PRESCALE = 0xFE;
/*
 * PCA @ 150 Hz. MG90 travel = pulse WIDTH (µs), not Hz.
 * 500–2500 µs ≈ full 180° on MG90; 400 Hz would shrink usable pulse.
 */
static const uint16_t SERVO_US_MIN = 500;   // MG90 0°
static const uint16_t SERVO_US_MAX = 2500;  // MG90 180°
/* Analog MG90s want ~50 Hz. 150 Hz made height (gravity load) sluggish/weak. */
static const uint16_t SERVO_PERIOD_US = 20000;  // 1e6/50
static const uint8_t PCA_PRESCALE = 121;        // ≈50 Hz (25MHz/(4096*50)-1)
static const uint8_t CH_BASE = 0, CH_HEIGHT = 1, CH_GRIP = 2;
static const uint8_t CH_CONVEYOR = 4;  // SG90-360 continuous (UM belt)
static const uint16_t CONV_US_STOP = 1500;
static const uint16_t CONV_US_MIN = 1000;
static const uint16_t CONV_US_MAX = 2000;

/* MG90: snap to X then HOLD with continuous PWM (stall torque).
 * PWM-off only on explicit relax (E-stop / axesRelaxAll) — never auto-droop. */
enum : uint8_t { AX_MOVE = 0, AX_HOLD, AX_IDLE };
static const uint32_t AX_PWM_REFRESH_MS = 80;       // base/grip hold refresh
static const uint32_t AX_HEIGHT_REFRESH_MS = 35;    // height fights gravity — reassert often
static const uint32_t AX_HOLD_MS = AX_PWM_REFRESH_MS;  /* legacy name */
/* Unused when soft=false (snap). Kept for any future soft axis. */
static const float AXIS_VMAX = 720.0f;
static const float AXIS_ACCEL = 2400.0f;
static const float AXIS_SMOOTH = 1.0f;
static const uint16_t AXIS_MIN_TICK_DELTA = 1;

struct Mg90Axis {
  uint8_t ch;
  bool soft;  // true = ramp; false = snap PWM (base/height/grip)
  int tgt;
  int cur;
  float live;
  float smooth;
  float vel;
  uint16_t lastTicks;
  uint8_t phase;
  uint32_t phaseMs;
};

/* soft=false → immediate PCA pulse (no software ramp lag) */
static Mg90Axis axB = {CH_BASE, false, 90, 90, 90.0f, 90.0f, 0.0f, 0xFFFF, AX_IDLE, 0};
static Mg90Axis axH = {CH_HEIGHT, false, 90, 90, 90.0f, 90.0f, 0.0f, 0xFFFF, AX_IDLE, 0};
static Mg90Axis axG = {CH_GRIP, false, 90, 90, 90.0f, 90.0f, 0.0f, 0xFFFF, AX_IDLE, 0};
static uint32_t lastAxisUs = 0;
static uint32_t lastPcaHoldMs = 0;

/* M1 = OUT3/4 */
static const uint8_t PIN_ENB = 10, PIN_IN3 = 13, PIN_IN4 = 8;
/* M2 = OUT1/2 */
static const uint8_t PIN_ENA = 9, PIN_IN1 = 11, PIN_IN2 = 12;

static const uint8_t PIN_M1_C1 = 4, PIN_M1_C2 = 5;
static const uint8_t PIN_M2_C1 = 6, PIN_M2_C2 = 7;

static const uint8_t PIN_S2 = 15, PIN_S3 = 16, PIN_OUT = 17, PIN_LED = 18;

static const float WHEEL_DIAM_MM = 43.0f;
static const float STEPS_PER_REV = 600.0f;
static const float MM_PER_STEP = (WHEEL_DIAM_MM * (float)M_PI) / STEPS_PER_REV;

static bool pcaOk = false, tofOk = false, imuOk = false;
/* telem mirrors — updated from Mg90Axis */
static int curB = 90, curH = 90, curG = 90;
static int16_t curConv = 0;  // -255..255 conveyor speed
static int16_t cmdL = 0, cmdR = 0;
static uint32_t lastDriveMs = 0;
static bool recOn = false;
static uint32_t recT0 = 0;
static uint32_t recCount = 0;
static char recName[32] = "move";
static int16_t recLastL = 0x7FFF, recLastR = 0x7FFF;
static uint16_t distMm = 0;
static int16_t tofDispMm = 0;
static uint16_t tofOriginMm = 0;
static bool tofOriginSet = false;
/* Interrupt-driven quadrature — loop polling misses edges under WiFi/I2C load */
static volatile int32_t encL = 0, encR = 0;
static volatile uint8_t prevM1 = 0, prevM2 = 0;
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static const int8_t ENC_LUT[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static int32_t yawCdeg = 0, pitchCdeg = 0, rollCdeg = 0;
static uint32_t lastTofMs = 0, lastTelemMs = 0, lastColorMs = 0;

static uint8_t colorLabel = 0;  // 0 unknown 1 R 2 Y 3 G
static uint8_t colorConf = 0;
static uint8_t colorR100 = 0, colorG100 = 0, colorB100 = 0;
static uint16_t lastRp = 0, lastGp = 0, lastBp = 0, lastCp = 0;
static ColorRyg colorRyg;
static bool wifiOk = false;
static bool wifiApOn = false;
static bool chainloadOta1 = false;
static uint32_t chainloadAtMs = 0;
static bool chainloadTimerOn = false;
static bool otaRebootPending = false;
static uint32_t otaRebootAtMs = 0;
static IPAddress wifiIp, wifiGw, wifiSn, wifiDns;
static bool wifiHaveNet = false;  /* learned gw/mask for static .222 */
static uint32_t wifiLastAttemptMs = 0;
static uint32_t wifiLastStatusMs = 0;
static uint8_t wifiFailStreak = 0;
static char serLine[80];
static uint8_t serLen = 0;
static uint8_t mtestPhase = 0;  /* 0 idle · 1 L · 2 R · 3 both */
static uint32_t mtestMs = 0;

static VL53L0X tof;
static TwoWire *tofBus = &Wire;

static WiFiServer netServer(NET_PORT);
static WiFiClient netClients[MAX_CLIENTS];
static MeowRx netRx[MAX_CLIENTS];

static void handleClientMsg(const meowler_ClientToRobot &msg);
static void broadcastTelem();
static int32_t wheelMm(int32_t steps);
static uint8_t findPcaAddr(TwoWire &bus);
static bool initPcaOn(TwoWire &bus, uint8_t addr);

static void broadcastRobot(const meowler_RobotToClient &msg) {
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (netClients[i] && netClients[i].connected()) {
      meowSendRobot(netClients[i], msg);
    }
  }
}

/* Framed protobuf log → Serial (+ TCP clients). Do not mix raw Serial.print with this. */
static void meowLog(uint32_t level, const char *text) {
  meowler_RobotToClient out = meowler_RobotToClient_init_zero;
  out.which_msg = meowler_RobotToClient_log_tag;
  out.msg.log.level = level;
  strncpy(out.msg.log.text, text ? text : "", sizeof(out.msg.log.text) - 1);
  out.msg.log.text[sizeof(out.msg.log.text) - 1] = '\0';
  meowSendRobotSerial(out);
  broadcastRobot(out);
}

static void meowLogf(uint32_t level, const char *fmt, ...) {
  char buf[sizeof(((meowler_Log *)0)->text)];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  meowLog(level, buf);
}

static void sendAck(uint32_t code) {
  meowler_RobotToClient out = meowler_RobotToClient_init_zero;
  out.which_msg = meowler_RobotToClient_ack_tag;
  out.msg.ack = code;
  broadcastRobot(out);
}

static void recEmit(uint32_t op, int32_t a, int32_t b, int32_t c, uint32_t mask) {
  meowler_RobotToClient out = meowler_RobotToClient_init_zero;
  out.which_msg = meowler_RobotToClient_rec_tag;
  meowler_RecEvent *e = &out.msg.rec;
  *e = meowler_RecEvent_init_zero;
  e->t_ms = recOn ? (uint32_t)(millis() - recT0) : 0;
  e->op = op;
  e->a = a;
  e->b = b;
  e->c = c;
  e->mask = mask;
  e->count = recCount;
  strncpy(e->name, recName, sizeof(e->name) - 1);
  e->name[sizeof(e->name) - 1] = '\0';
  broadcastRobot(out);
}

static void recStart(const char *name) {
  recOn = true;
  recT0 = millis();
  recCount = 0;
  recLastL = 0x7FFF;
  recLastR = 0x7FFF;
  memset(recName, 0, sizeof(recName));
  if (name && name[0])
    strncpy(recName, name, sizeof(recName) - 1);
  else
    strncpy(recName, "move", sizeof(recName) - 1);
  recEmit(0, 0, 0, 0, 0);
  meowLogf(LOG_INFO, "REC stream start %s", recName);
}

static void recStop() {
  if (!recOn) return;
  recEmit(255, 0, 0, 0, 0);
  recOn = false;
  meowLogf(LOG_INFO, "REC stream end %s n=%u", recName, (unsigned)recCount);
}

static void recNote(uint32_t op, int32_t a, int32_t b, int32_t c, uint32_t mask) {
  if (!recOn) return;
  recCount++;
  recEmit(op, a, b, c, mask);
}

static void recNoteDrive(int16_t l, int16_t r) {
  if (!recOn) return;
  if (l == recLastL && r == recLastR) return;
  recLastL = l;
  recLastR = r;
  recNote(1, (int32_t)l, (int32_t)r, 0, 0);
}

static void sendHelloTo(WiFiClient &c) {
  meowler_RobotToClient out = meowler_RobotToClient_init_zero;
  out.which_msg = meowler_RobotToClient_hello_tag;
  IPAddress ip = WiFi.localIP();
  out.msg.hello.ip = ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) |
                     ((uint32_t)ip[2] << 8) | (uint32_t)ip[3];
  out.msg.hello.port = NET_PORT;
  out.msg.hello.pca_ok = pcaOk;
  out.msg.hello.tof_ok = tofOk;
  out.msg.hello.imu_ok = imuOk;
  meowSendRobot(c, out);
}

static void pollNet() {
  if (!wifiOk) return;
  WiFiClient incoming = netServer.available();
  if (incoming) {
    incoming.setNoDelay(true);
    bool slotted = false;
    for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
      if (!netClients[i] || !netClients[i].connected()) {
        netClients[i].stop();
        netClients[i] = incoming;
        meowRxReset(&netRx[i]);
        slotted = true;
        sendHelloTo(netClients[i]);
        meowLog(LOG_INFO, "TCP client connected");
        break;
      }
    }
    if (!slotted) incoming.stop();
  }
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (!netClients[i]) continue;
    if (!netClients[i].connected()) {
      netClients[i].stop();
      meowRxReset(&netRx[i]);
      continue;
    }
  static uint8_t tmp[1024];
  static meowler_ClientToRobot msg;
    while (netClients[i].available() > 0) {
      int n = netClients[i].read(tmp, sizeof(tmp));
      if (n <= 0) break;
      size_t off = 0;
      while (off < (size_t)n) {
        size_t used = 0;
        msg = meowler_ClientToRobot_init_zero;
        bool got = meowRxPump(&netRx[i], tmp + off, (size_t)n - off, &used, &msg);
        if (used == 0) break;
        off += used;
        if (got) handleClientMsg(msg);
      }
    }
  }
}

static bool wifiWaitConnected_(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(200);
    yield();
  }
  return WiFi.status() == WL_CONNECTED;
}

static void wifiStopClients() {
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (netClients[i]) netClients[i].stop();
    meowRxReset(&netRx[i]);
  }
}

static void wifiBindServer() {
  netServer.begin();
  netServer.setNoDelay(true);
  if (MDNS.begin("meowler")) {
    MDNS.addService("meowler", "tcp", NET_PORT);
  }
}

static void wifiStopSoftAp_() {
  if (!wifiApOn) return;
  WiFi.softAPdisconnect(true);
  wifiApOn = false;
  meowLog(LOG_INFO, "WIFI SoftAP off");
}

/* Reachable when STA cannot join PC hotspot — connect phone/PC to Meowler → 192.168.4.1:3333 */
static bool wifiStartSoftAp_() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname("meowler");
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS)) {
    meowLog(LOG_ERR, "WIFI SoftAP FAIL");
    wifiApOn = false;
    return false;
  }
  delay(100);
  wifiApOn = true;
  wifiBindServer();
  meowLogf(LOG_INFO, "WIFI SoftAP '%s' tcp://%s:%u (STA still retrying %s)",
           WIFI_AP_SSID, WiFi.softAPIP().toString().c_str(), (unsigned)NET_PORT, WIFI_SSID);
  return true;
}

static void wifiLearnFromDhcp_() {
  IPAddress gw = WiFi.gatewayIP();
  IPAddress sn = WiFi.subnetMask();
  IPAddress dhcpIp = WiFi.localIP();
  IPAddress dns = WiFi.dnsIP();
  if (dns == IPAddress(0, 0, 0, 0)) dns = gw;
  if (sn == IPAddress(0, 0, 0, 0)) sn = IPAddress(255, 255, 255, 0);
  if (gw == IPAddress(0, 0, 0, 0))
    gw = IPAddress(dhcpIp[0], dhcpIp[1], dhcpIp[2], 1);

  IPAddress ip(gw[0], gw[1], gw[2], (uint8_t)WIFI_HOST_OCTET);
  if (sn != IPAddress(255, 255, 255, 0)) {
    uint32_t net = ((uint32_t)dhcpIp[0] << 24) | ((uint32_t)dhcpIp[1] << 16) |
                   ((uint32_t)dhcpIp[2] << 8) | (uint32_t)dhcpIp[3];
    uint32_t mask = ((uint32_t)sn[0] << 24) | ((uint32_t)sn[1] << 16) |
                    ((uint32_t)sn[2] << 8) | (uint32_t)sn[3];
    uint32_t host = (net & mask) | (uint32_t)WIFI_HOST_OCTET;
    ip = IPAddress((uint8_t)(host >> 24), (uint8_t)(host >> 16), (uint8_t)(host >> 8),
                   (uint8_t)host);
  }
  wifiIp = ip;
  wifiGw = gw;
  wifiSn = sn;
  wifiDns = dns;
  wifiHaveNet = true;
  meowLogf(LOG_INFO, "WIFI learned gw=%s mask=%s -> %s",
           gw.toString().c_str(), sn.toString().c_str(), ip.toString().c_str());
}

static bool wifiConnectStatic_() {
  if (!wifiHaveNet) return false;
  WiFi.disconnect(false, false);
  delay(150);
  if (!WiFi.config(wifiIp, wifiGw, wifiSn, wifiDns)) {
    meowLog(LOG_ERR, "WIFI static config() FAIL");
    return false;
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  meowLogf(LOG_INFO, "WIFI static %s", wifiIp.toString().c_str());
  return wifiWaitConnected_(20000UL);
}

static bool wifiConnectDhcp_() {
  WiFi.disconnect(false, false);
  delay(150);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  meowLogf(LOG_INFO, "WIFI DHCP %s", WIFI_SSID);
  return wifiWaitConnected_(25000UL);
}

/* DHCP → learn → static *.222; on failure SoftAP so TCP still works */
static void startWifi() {
  WiFi.mode(wifiApOn ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname("meowler");
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  wifiStopClients();
  wifiOk = false;

  /* Fast path: reuse learned static after a drop */
  if (wifiHaveNet) {
    meowLog(LOG_INFO, "WIFI reconnect (cached static)");
    if (wifiConnectStatic_()) {
      wifiStopSoftAp_();
      WiFi.mode(WIFI_STA);
      wifiOk = true;
      wifiBindServer();
      meowLogf(LOG_INFO, "WIFI ok tcp://%s:%u rssi=%d",
               WiFi.localIP().toString().c_str(), (unsigned)NET_PORT, WiFi.RSSI());
      return;
    }
    meowLog(LOG_WARN, "WIFI cached static FAIL — full DHCP");
  }

  if (!wifiConnectDhcp_()) {
    meowLog(LOG_ERR, "WIFI FAIL (DHCP) — SoftAP fallback");
    if (wifiStartSoftAp_()) wifiOk = true;
    return;
  }
  wifiLearnFromDhcp_();

  if (!wifiConnectStatic_()) {
    meowLog(LOG_WARN, "WIFI static FAIL — DHCP fallback");
    if (!wifiConnectDhcp_()) {
      meowLog(LOG_ERR, "WIFI FAIL — SoftAP fallback");
      if (wifiStartSoftAp_()) wifiOk = true;
      return;
    }
  }

  wifiStopSoftAp_();
  WiFi.mode(WIFI_STA);
  wifiOk = true;
  wifiBindServer();
  meowLogf(LOG_INFO, "WIFI ok tcp://%s:%u rssi=%d",
           WiFi.localIP().toString().c_str(), (unsigned)NET_PORT, WiFi.RSSI());
}

/* Keep STA + TCP server alive across hotspot blips / USB resets */
static void serviceWifi() {
  uint32_t now = millis();
  if (now - wifiLastStatusMs < 400) return;
  wifiLastStatusMs = now;

  if (WiFi.status() == WL_CONNECTED) {
    if (wifiApOn) wifiStopSoftAp_();
    if (!wifiOk) {
      wifiOk = true;
      wifiBindServer();
      meowLogf(LOG_INFO, "WIFI up %s rssi=%d", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
    wifiFailStreak = 0;
    return;
  }

  /* SoftAP alone is a valid link — do not tear it down every few seconds */
  if (wifiApOn) {
    wifiOk = true;
    uint32_t gap = (WiFi.softAPgetStationNum() > 0) ? 20000UL : 10000UL;
    if (now - wifiLastAttemptMs < gap) return;
    wifiLastAttemptMs = now;
    wifiFailStreak++;
    meowLogf(LOG_INFO, "WIFI STA retry #%u (SoftAP up clients=%u)",
             (unsigned)wifiFailStreak, (unsigned)WiFi.softAPgetStationNum());
    /* Try STA without dropping SoftAP */
    WiFi.mode(WIFI_AP_STA);
    if (wifiHaveNet && wifiConnectStatic_()) {
      wifiStopSoftAp_();
      WiFi.mode(WIFI_STA);
      wifiOk = true;
      wifiBindServer();
      wifiFailStreak = 0;
      meowLogf(LOG_INFO, "WIFI ok tcp://%s:%u rssi=%d",
               WiFi.localIP().toString().c_str(), (unsigned)NET_PORT, WiFi.RSSI());
    } else if (wifiConnectDhcp_()) {
      wifiLearnFromDhcp_();
      if (wifiConnectStatic_() || WiFi.status() == WL_CONNECTED) {
        wifiStopSoftAp_();
        WiFi.mode(WIFI_STA);
        wifiOk = true;
        wifiBindServer();
        wifiFailStreak = 0;
        meowLogf(LOG_INFO, "WIFI ok tcp://%s:%u rssi=%d",
                 WiFi.localIP().toString().c_str(), (unsigned)NET_PORT, WiFi.RSSI());
      }
    }
    return;
  }

  if (wifiOk) {
    wifiOk = false;
    wifiStopClients();
    meowLog(LOG_WARN, "WIFI down");
  }

  uint32_t gap = (wifiFailStreak >= 4) ? 12000UL : 3500UL;
  if (now - wifiLastAttemptMs < gap) return;
  wifiLastAttemptMs = now;
  wifiFailStreak++;
  meowLogf(LOG_INFO, "WIFI retry #%u", (unsigned)wifiFailStreak);
  startWifi();
  if (wifiOk) wifiFailStreak = 0;
}

/* L298N EN: PWM on GPIO9/10 (not cam FPC). Speed slider actually throttles. */
#ifndef DRIVE_USE_PWM
#define DRIVE_USE_PWM 1
#endif

static void setupDrivePins() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
#if DRIVE_USE_PWM
  ledcDetach(PIN_ENA);
  ledcDetach(PIN_ENB);
  ledcAttach(PIN_ENA, 1000, 8);
  ledcAttach(PIN_ENB, 1000, 8);
  ledcWrite(PIN_ENA, 0);
  ledcWrite(PIN_ENB, 0);
#else
  ledcDetach(PIN_ENA);
  ledcDetach(PIN_ENB);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);
#endif
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

static void serviceMotorTest() {
  if (mtestPhase == 0) return;
  uint32_t now = millis();
  if (now - mtestMs < 900) return;
  mtestMs = now;
  mtestPhase++;
  if (mtestPhase == 2) {
    setDrive(0, 255);
    meowLog(LOG_INFO, "MTEST R");
  } else if (mtestPhase == 3) {
    setDrive(255, 255);
    meowLog(LOG_INFO, "MTEST BOTH");
  } else {
    setDrive(0, 0);
    mtestPhase = 0;
    meowLog(LOG_INFO, "MTEST DONE");
  }
}

static void serialPrintHelp() {
  meowLog(LOG_INFO, "CMD: s|w|d L R|x|m|v 0|1|OTA0|OTA1|?");
}

static void serialPrintStatus() {
  int32_t el = 0, er = 0;
  encSnapshot_(&el, &er);
  IPAddress ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : (wifiApOn ? WiFi.softAPIP() : WiFi.localIP());
  meowLogf(LOG_INFO, "STAT wifi=%d ap=%d ip=%s rssi=%d pca=%d tof=%d imu=%d cmd=%d/%d enc=%ld/%ld",
           (WiFi.status() == WL_CONNECTED) ? 1 : 0, wifiApOn ? 1 : 0, ip.toString().c_str(), WiFi.RSSI(),
           pcaOk ? 1 : 0, tofOk ? 1 : 0, imuOk ? 1 : 0, (int)cmdL, (int)cmdR, (long)el, (long)er);
}

static void handleSerialLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;
  if (!strcmp(line, "OTA0") || !strcmp(line, "ota0")) {
    meowLog(LOG_INFO, "serial: boot OTA0 updater");
    bootOta0Now();
    return;
  }
  if (!strcmp(line, "OTA1") || !strcmp(line, "ota1")) {
    meowLog(LOG_INFO, "serial: boot OTA1 app");
    bootOta1Now();
    return;
  }
  if (!strcmp(line, "?") || !strcmp(line, "help")) {
    serialPrintHelp();
    return;
  }
  if (!strcmp(line, "a") || !strcmp(line, "auto")) {
    meowLog(LOG_INFO, "serial: run auto sequence (no gyro)");
    runAutoSequence();
    return;
  }
  if (!strcmp(line, "s") || !strcmp(line, "status")) {
    serialPrintStatus();
    return;
  }
  if (!strcmp(line, "w") || !strcmp(line, "wifi")) {
    meowLog(LOG_INFO, "WIFI manual reconnect");
    wifiLastAttemptMs = 0;
    wifiFailStreak = 0;
    startWifi();
    serialPrintStatus();
    return;
  }
  if (!strcmp(line, "x") || !strcmp(line, "stop")) {
    mtestPhase = 0;
    setDrive(0, 0);
    meowLog(LOG_INFO, "DRIVE 0 0");
    return;
  }
  if (!strcmp(line, "m") || !strcmp(line, "motor")) {
    setupDrivePins();
    mtestPhase = 1;
    mtestMs = millis();
    setDrive(255, 0);
    meowLog(LOG_INFO, "MTEST L");
    return;
  }
  if (line[0] == 'v' && (line[1] == ' ' || line[1] == '\t' || line[1] == 0)) {
    int on = 1;
    if (line[1]) on = atoi(line + 1);
    colorRyg.setLog(on != 0);
    meowLogf(LOG_INFO, "COLOR_LOG %d", colorRyg.logEnabled() ? 1 : 0);
    return;
  }
  if (line[0] == 'd' && (line[1] == ' ' || line[1] == '\t')) {
    int l = 0, r = 0;
    if (sscanf(line + 1, "%d %d", &l, &r) >= 1) {
      if (l > 255) l = 255;
      if (l < -255) l = -255;
      if (r > 255) r = 255;
      if (r < -255) r = -255;
      setupDrivePins();
      setDrive((int16_t)l, (int16_t)r);
      meowLogf(LOG_INFO, "DRIVE %d %d", l, r);
    }
    return;
  }
  meowLogf(LOG_WARN, "UNK CMD '%s'", line);
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serLine[serLen] = 0;
      handleSerialLine(serLine);
      serLen = 0;
      continue;
    }
    if (serLen + 1 < sizeof(serLine)) serLine[serLen++] = c;
  }
}

static bool write8(uint8_t reg, uint8_t val) {
  pcaBus->beginTransmission(pcaAddr);
  pcaBus->write(reg);
  pcaBus->write(val);
  return pcaBus->endTransmission() == 0;
}

static void setPwmRaw(uint8_t ch, uint16_t on, uint16_t off) {
  if (!pcaOk || ch > 15) return;
  uint8_t reg = REG_LED0 + 4 * ch;
  pcaBus->beginTransmission(pcaAddr);
  pcaBus->write(reg);
  pcaBus->write((uint8_t)(on & 0xFF));
  pcaBus->write((uint8_t)(on >> 8));
  pcaBus->write((uint8_t)(off & 0xFF));
  pcaBus->write((uint8_t)(off >> 8));
  pcaBus->endTransmission();
  /* Height channel: second write — I2C glitches were dropping CH1 under load */
  if (ch == CH_HEIGHT) {
    pcaBus->beginTransmission(pcaAddr);
    pcaBus->write(reg);
    pcaBus->write((uint8_t)(on & 0xFF));
    pcaBus->write((uint8_t)(on >> 8));
    pcaBus->write((uint8_t)(off & 0xFF));
    pcaBus->write((uint8_t)(off >> 8));
    pcaBus->endTransmission();
  }
}

static void pcaFullOff(uint8_t ch) { setPwmRaw(ch, 0, 0x1000); }

static uint16_t usToTicks(uint32_t us) {
  if (us > SERVO_PERIOD_US - 50) us = SERVO_PERIOD_US - 50;
  uint16_t ticks = (uint16_t)((us * 4096UL) / SERVO_PERIOD_US);
  if (ticks > 4095) ticks = 4095;
  return ticks;
}

/* Degrees → pulse µs → PCA ticks (keeps full 500–2500 µs at any Hz) */
static uint16_t degToTicks(int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  uint32_t us = (uint32_t)map(deg, 0, 180, (long)SERVO_US_MIN, (long)SERVO_US_MAX);
  return usToTicks(us);
}

static uint16_t degFToTicks(float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  float us = (float)SERVO_US_MIN + (deg / 180.0f) * (float)(SERVO_US_MAX - SERVO_US_MIN);
  return usToTicks((uint32_t)lroundf(us));
}

static void writeAxisPwm(Mg90Axis &a, float deg) {
  if (!pcaOk) return;
  uint16_t off = degFToTicks(deg);
  if (a.lastTicks != 0xFFFF) {
    int16_t d = (int16_t)off - (int16_t)a.lastTicks;
    if (d < 0) d = (int16_t)-d;
    if (d < (int16_t)AXIS_MIN_TICK_DELTA && off != degToTicks(a.tgt)) return;
  }
  a.lastTicks = off;
  setPwmRaw(a.ch, 0, off);
}

static void axisRelax(Mg90Axis &a) {
  if (!pcaOk) return;
  pcaFullOff(a.ch);
  a.lastTicks = 0xFFFF;  // force next write
}

static void axisEnter(Mg90Axis &a, uint8_t phase) {
  a.phase = phase;
  a.phaseMs = millis();
}

/* New target → immediate PWM (snap) then HOLD with continuous stall torque. */
static void axisRequest(Mg90Axis &a, int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  a.tgt = deg;
  if (!a.soft) {
    a.live = a.smooth = (float)deg;
    a.vel = 0.0f;
    a.cur = deg;
    a.lastTicks = 0xFFFF;  /* always push PCA — no coalesce lag on retarget */
    writeAxisPwm(a, (float)deg);
    axisEnter(a, AX_HOLD);
    /* Height: force immediate re-refresh next stepMg90 (gravity load) */
    if (a.ch == CH_HEIGHT) a.phaseMs = 0;
    return;
  }
  axisEnter(a, AX_MOVE);
}

static void axisIntegrateMove(Mg90Axis &a, float dt) {
  if (!a.soft) {
    a.live = a.smooth = (float)a.tgt;
    a.vel = 0.0f;
    a.cur = a.tgt;
    writeAxisPwm(a, a.smooth);
    return;
  }

  float err = (float)a.tgt - a.live;
  float absErr = fabsf(err);
  if (absErr < 0.35f && fabsf(a.vel) < 1.5f) {
    a.live = a.smooth = (float)a.tgt;
    a.vel = 0.0f;
    a.cur = a.tgt;
    writeAxisPwm(a, a.smooth);
    return;
  }

  float dir = (err >= 0.0f) ? 1.0f : -1.0f;
  float vAbs = fabsf(a.vel);
  float stopDist = (vAbs * vAbs) / (2.0f * AXIS_ACCEL) + 1.0f;
  float aCmd;
  if (a.vel * err < 0.0f) aCmd = -copysignf(AXIS_ACCEL, a.vel);
  else if (absErr <= stopDist) aCmd = -dir * AXIS_ACCEL;
  else if (vAbs < AXIS_VMAX) aCmd = dir * AXIS_ACCEL;
  else {
    aCmd = 0.0f;
    a.vel = dir * AXIS_VMAX;
  }

  a.vel += aCmd * dt;
  if (a.vel > AXIS_VMAX) a.vel = AXIS_VMAX;
  if (a.vel < -AXIS_VMAX) a.vel = -AXIS_VMAX;
  if (absErr > 0.8f && fabsf(a.vel) < 4.0f && (a.vel * err >= 0.0f || fabsf(a.vel) < 0.5f))
    a.vel = dir * fmaxf(fabsf(a.vel), 4.0f);

  float step = a.vel * dt;
  if ((err > 0.0f && step > err) || (err < 0.0f && step < err)) {
    a.live = (float)a.tgt;
    a.vel = 0.0f;
  } else {
    a.live += step;
  }
  if (a.live < 0.0f) {
    a.live = 0.0f;
    a.vel = 0.0f;
  }
  if (a.live > 180.0f) {
    a.live = 180.0f;
    a.vel = 0.0f;
  }
  a.smooth += AXIS_SMOOTH * (a.live - a.smooth);
  a.cur = (int)lroundf(a.smooth);
  writeAxisPwm(a, a.smooth);
}

static bool axisAtTarget(const Mg90Axis &a) {
  return fabsf((float)a.tgt - a.live) < 0.4f && fabsf(a.vel) < 1.5f;
}

static void stepMg90(Mg90Axis &a, float dt) {
  uint32_t now = millis();
  switch (a.phase) {
    case AX_MOVE:
      axisIntegrateMove(a, dt);
      if (axisAtTarget(a)) {
        a.live = a.smooth = (float)a.tgt;
        a.vel = 0.0f;
        a.cur = a.tgt;
        writeAxisPwm(a, (float)a.tgt);
        axisEnter(a, AX_HOLD);
      }
      break;
    case AX_HOLD:
      /* Continuous pulse = stall torque. Height refreshes faster (gravity). */
      a.cur = a.tgt;
      {
        const uint32_t refresh =
            (a.ch == CH_HEIGHT) ? AX_HEIGHT_REFRESH_MS : AX_PWM_REFRESH_MS;
        if (a.phaseMs == 0 || (now - a.phaseMs) >= refresh) {
          a.phaseMs = now;
          a.lastTicks = 0xFFFF;
          writeAxisPwm(a, (float)a.tgt);
        }
      }
      break;
    case AX_IDLE:
    default:
      /* Torque off — only after explicit relax */
      a.cur = a.tgt;
      break;
  }
}

static void updateMg90Axes() {
  if (!pcaOk) return;
  uint32_t now = micros();
  if (lastAxisUs == 0) lastAxisUs = now;
  uint32_t dtu = now - lastAxisUs;
  /* ~500 Hz axis loop — snappy command→PWM */
  if (dtu < 2000UL) return;
  lastAxisUs = now;
  float dt = (float)dtu * 1e-6f;
  if (dt > 0.06f) dt = 0.06f;

  stepMg90(axB, dt);
  stepMg90(axH, dt);
  stepMg90(axG, dt);
  curB = axB.cur;
  curH = axH.cur;
  curG = axG.cur;
}

static bool armBusy() {
  return axB.phase == AX_MOVE || axH.phase == AX_MOVE || axG.phase == AX_MOVE;
}

static void requestBase(int b) { axisRequest(axB, b); }
static void requestHeight(int h) { axisRequest(axH, h); }
static void requestGrip(int g) { axisRequest(axG, g); }

static void applyArm(int b, int h, int g) {
  requestBase(b);
  requestHeight(h);
  requestGrip(g);
}

static void axesRelaxAll() {
  axisRelax(axB);
  axisRelax(axH);
  axisRelax(axG);
  axisEnter(axB, AX_IDLE);
  axisEnter(axH, AX_IDLE);
  axisEnter(axG, AX_IDLE);
}

static void axesResetPose(int deg) {
  axB.tgt = axH.tgt = axG.tgt = deg;
  axB.cur = axH.cur = axG.cur = deg;
  axB.live = axH.live = axG.live = (float)deg;
  axB.smooth = axH.smooth = axG.smooth = (float)deg;
  axB.vel = axH.vel = axG.vel = 0.0f;
  axB.lastTicks = axH.lastTicks = axG.lastTicks = 0xFFFF;
  if (pcaOk) {
    writeAxisPwm(axB, (float)deg);
    writeAxisPwm(axH, (float)deg);
    writeAxisPwm(axG, (float)deg);
    axisEnter(axB, AX_HOLD);
    axisEnter(axH, AX_HOLD);
    axisEnter(axG, AX_HOLD);
  } else {
    axB.phase = axH.phase = axG.phase = AX_IDLE;
  }
  curB = curH = curG = deg;
}

/* Continuous SG90-360: speed -255..255 → pulse.
 * Stop = FULL OFF on CH4 (1500µs still creeps on most 360° units). */
static void setConveyor(int16_t spd) {
  if (spd > 255) spd = 255;
  if (spd < -255) spd = -255;
  if (spd > -20 && spd < 20) spd = 0;  // deadband
  curConv = spd;
  if (!pcaOk) return;
  if (spd == 0) {
    pcaFullOff(CH_CONVEYOR);
    delayMicroseconds(300);
    pcaFullOff(CH_CONVEYOR);
    return;
  }
  uint32_t us = (uint32_t)map((long)spd, -255L, 255L, (long)CONV_US_MIN, (long)CONV_US_MAX);
  uint16_t ticks = usToTicks(us);
  setPwmRaw(CH_CONVEYOR, 0, ticks);
  delayMicroseconds(300);
  setPwmRaw(CH_CONVEYOR, 0, ticks);
}

static void clampMag(int16_t *spd, uint8_t *mag) {
  if (*spd > 255) *spd = 255;
  if (*spd < -255) *spd = -255;
  *mag = (uint8_t)(*spd < 0 ? -*spd : *spd);
  if (*mag > 0 && *mag < 80) *mag = 80;
}

static void writeEn_(uint8_t en, uint8_t mag) {
#if DRIVE_USE_PWM
  ledcWrite(en, mag);
#else
  digitalWrite(en, mag ? HIGH : LOW);
#endif
}

static void applySideGpio(int16_t spd, uint8_t inA, uint8_t inB, uint8_t en) {
  uint8_t mag = 0;
  clampMag(&spd, &mag);
  if (spd > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    writeEn_(en, mag ? mag : 255);
  } else if (spd < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    writeEn_(en, mag ? mag : 255);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    writeEn_(en, 0);
  }
}

static void holdPcaMotorsOff() {
  if (!pcaOk) return;
  /* CH0–2 = arm (axis FSM). CH4 = conveyor — never kill while running. */
  for (uint8_t ch = 5; ch <= 14; ch++) pcaFullOff(ch);
  if (curConv == 0) pcaFullOff(CH_CONVEYOR);
  else setConveyor(curConv);  /* refresh CH4 after bus glitches */
}

static void applyDrive() {
  applySideGpio(cmdL, PIN_IN3, PIN_IN4, PIN_ENB);
  applySideGpio(cmdR, PIN_IN1, PIN_IN2, PIN_ENA);
}

static void serviceDriveHold() {
  if (mtestPhase != 0) return;
  if (cmdL == 0 && cmdR == 0) return;
  if (millis() - lastDriveMs > 280) setDrive(0, 0);
}

static void setDrive(int16_t l, int16_t r) {
  if (l > 255) l = 255;
  if (l < -255) l = -255;
  if (r > 255) r = 255;
  if (r < -255) r = -255;
  cmdL = l;
  cmdR = r;
  lastDriveMs = millis();
  applyDrive();
  recNoteDrive(l, r);
}

static void broadcastOtaStatus() {
  meowler_RobotToClient out = meowler_RobotToClient_init_zero;
  out.which_msg = meowler_RobotToClient_ota_tag;
  otaFillStatus(&out.msg.ota);
  broadcastRobot(out);
}

static void handleOtaCmd(const meowler_OtaCmd &cmd) {
  switch (cmd.action) {
    case 1: {  // begin
      chainloadOta1 = false;
      setDrive(0, 0);
      if (pcaOk) setConveyor(0);
      bool ok = cmd.has_begin && otaBegin(cmd.begin);
      meowLogf(ok ? LOG_INFO : LOG_ERR, "OTA begin raw=%u -> inactive slot",
               (unsigned)(cmd.has_begin ? cmd.begin.raw_size : 0));
      broadcastOtaStatus();
      sendAck(ok ? 0 : 1);
      break;
    }
    case 2:  // chunk
      if (!cmd.has_chunk || !otaChunk(cmd.chunk)) {
        broadcastOtaStatus();
        sendAck(1);
      } else {
        if ((gOta.received & 0xFFFF) == 0 || gOta.received == gOta.packedSize)
          broadcastOtaStatus();
      }
      break;
    case 3: {  // finish → inflate + write inactive slot
      meowLog(LOG_INFO, "OTA finish — verify CRC, close slot");
      bool ok = otaFinish();
      meowLogf(ok ? LOG_INFO : LOG_ERR, "OTA finish %s err=%u %s", ok ? "READY" : "FAIL",
               (unsigned)gOta.error, gOta.detail);
      broadcastOtaStatus();
      sendAck(ok ? 0 : 1);
      break;
    }
    case 4:  // abort
      otaReset();
      otaSetDetail("aborted");
      broadcastOtaStatus();
      sendAck(0);
      meowLog(LOG_INFO, "OTA abort");
      break;
    case 5:  // apply → reboot into new partition
      if (!otaApply()) {
        broadcastOtaStatus();
        sendAck(1);
        break;
      }
      broadcastOtaStatus();
      sendAck(0);
      meowLog(LOG_INFO, "OTA apply — reboot new slot in 400ms");
      otaRebootPending = true;
      otaRebootAtMs = millis() + 400;
      break;
    case 7:  // boot OTA0 updater
      bootOta0Now();
      break;
    case 8:  // boot last OTA1 app
      bootOta1Now();
      break;
    case 6:  // query
      broadcastOtaStatus();
      sendAck(0);
      break;
    default:
      sendAck(1);
      break;
  }
}

static inline uint8_t encSample_(uint8_t c1, uint8_t c2) {
  return (uint8_t)(((gpio_get_level((gpio_num_t)c1) ? 1 : 0) << 1) |
                   (gpio_get_level((gpio_num_t)c2) ? 1 : 0));
}

void IRAM_ATTR encIsrM1_() {
  portENTER_CRITICAL_ISR(&encMux);
  uint8_t m = encSample_(PIN_M1_C1, PIN_M1_C2);
  int8_t d = ENC_LUT[(prevM1 << 2) | m];
  if (d) encL += d;
  prevM1 = m;
  portEXIT_CRITICAL_ISR(&encMux);
}

void IRAM_ATTR encIsrM2_() {
  portENTER_CRITICAL_ISR(&encMux);
  uint8_t m = encSample_(PIN_M2_C1, PIN_M2_C2);
  int8_t d = ENC_LUT[(prevM2 << 2) | m];
  if (d) encR += d;
  prevM2 = m;
  portEXIT_CRITICAL_ISR(&encMux);
}

static void setupEncoders() {
  pinMode(PIN_M1_C1, INPUT_PULLUP);
  pinMode(PIN_M1_C2, INPUT_PULLUP);
  pinMode(PIN_M2_C1, INPUT_PULLUP);
  pinMode(PIN_M2_C2, INPUT_PULLUP);
  prevM1 = encSample_(PIN_M1_C1, PIN_M1_C2);
  prevM2 = encSample_(PIN_M2_C1, PIN_M2_C2);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_C1), encIsrM1_, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_C2), encIsrM1_, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_C1), encIsrM2_, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_C2), encIsrM2_, CHANGE);
  meowLog(LOG_INFO, "ENC IRQ M1=4/5 M2=6/7 CHANGE");
}

static void pollEncoders() {
  /* Catch-up sample under mux (same decoder as ISR) */
  portENTER_CRITICAL(&encMux);
  uint8_t m1 = encSample_(PIN_M1_C1, PIN_M1_C2);
  int8_t d1 = ENC_LUT[(prevM1 << 2) | m1];
  if (d1) encL += d1;
  prevM1 = m1;
  uint8_t m2 = encSample_(PIN_M2_C1, PIN_M2_C2);
  int8_t d2 = ENC_LUT[(prevM2 << 2) | m2];
  if (d2) encR += d2;
  prevM2 = m2;
  portEXIT_CRITICAL(&encMux);
}

static int32_t wheelMm(int32_t steps) {
  return (int32_t)lroundf((float)steps * MM_PER_STEP);
}

static void encSnapshot_(int32_t *l, int32_t *r) {
  portENTER_CRITICAL(&encMux);
  *l = encL;
  *r = encR;
  portEXIT_CRITICAL(&encMux);
}

static void zeroOdom() {
  portENTER_CRITICAL(&encMux);
  encL = 0;
  encR = 0;
  portEXIT_CRITICAL(&encMux);
  tofOriginMm = distMm;
  tofOriginSet = tofOk && distMm > 0;
  tofDispMm = 0;
}

static bool i2cProbe(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  uint8_t err = bus.endTransmission();
  return err == 0;
}

static void i2cScan(TwoWire &bus, const char *tag) {
  char line[96];
  size_t n = 0;
  n += (size_t)snprintf(line + n, sizeof(line) - n, "I2C %s:", tag);
  bool any = false;
  uint8_t nack = 0, other = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    bus.beginTransmission(a);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      any = true;
      n += (size_t)snprintf(line + n, sizeof(line) - n, " 0x%02X", a);
      if (n + 8 >= sizeof(line)) break;
    } else if (err == 2) {
      nack++;
    } else {
      other++;
    }
  }
  if (!any) snprintf(line + n, sizeof(line) - n, " (none nack=%u oth=%u)", nack, other);
  meowLog(LOG_INFO, line);
}

/* Idle check MUST run before Wire.begin. pinMode after begin detaches I2C → all NACKs. */
static void logI2cIdle(const char *tag, int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(1);
  meowLogf(LOG_INFO, "%s SDA=%d SCL=%d idle %d/%d (want 1/1)", tag, sda, scl,
           (int)digitalRead(sda), (int)digitalRead(scl));
}

static void i2cBusRecover(int sda, int scl) {
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

static void i2cOpen(TwoWire &bus, int sda, int scl, uint32_t hz) {
  i2cBusRecover(sda, scl);
  logI2cIdle("idle", sda, scl);
  bus.end();
  delay(5);
  bus.setBufferSize(256);  /* BNO08x SHTP frames */
  bus.begin(sda, scl, hz);
  delay(20);
}

/* Freenove ESP32-S3-WROOM: GPIO14 = camera FLASH LED (not I2C).
   Using it as Wire SCL left the pin open-drain + any LED/FET pulled it LOW. */
static void fixGpio14FlashLed_() {
  rtc_gpio_hold_dis(GPIO_NUM_14);
  gpio_hold_dis(GPIO_NUM_14);
  gpio_reset_pin(GPIO_NUM_14);
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  delay(2);
  pinMode(14, INPUT_PULLUP);
  gpio_pulldown_dis(GPIO_NUM_14);
  gpio_pullup_en(GPIO_NUM_14);
  delay(2);
  meowLogf(LOG_INFO, "GPIO14 FLASH-LED released idle=%d (want 1)", (int)digitalRead(14));
}

static uint8_t findPcaAddr(TwoWire &bus) {
  /* Ghost buses ACK every address — require MODE1 readback. */
  uint8_t hits = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (i2cProbe(bus, a)) hits++;
    if (hits > 16) return 0;
  }
  for (uint8_t a = 0x40; a <= 0x47; a++) {
    if (!i2cProbe(bus, a)) continue;
    bus.beginTransmission(a);
    bus.write(REG_MODE1);
    if (bus.endTransmission(false) != 0) continue;
    if (bus.requestFrom((int)a, 1) != 1) continue;
    (void)bus.read();
    return a;
  }
  return 0;
}

static void beginI2C() {
  fixGpio14FlashLed_();
  Wire.end();
  Wire1.end();
  delay(10);
  meowLog(LOG_INFO, "I2C Wire SDA=21 SCL=47 PCA+VL53");
  i2cOpen(Wire, PIN_I2C0_SDA, PIN_I2C0_SCL, 100000);
  i2cScan(Wire, "21/47");
  if (i2cProbe(Wire, 0x29) || i2cProbe(Wire, 0x40)) i2cLocked = true;
}

static bool initPcaOn(TwoWire &bus, uint8_t addr) {
  pcaBus = &bus;
  pcaAddr = addr;
  bus.setClock(100000);
  delay(2);
  if (!i2cProbe(bus, addr)) {
    meowLogf(LOG_WARN, "PCA probe fail @0x%02X", addr);
    return false;
  }
  if (!write8(REG_MODE1, 0x10)) {
    meowLog(LOG_WARN, "PCA MODE1 sleep fail");
    return false;
  }
  delay(5);
  if (!write8(REG_PRESCALE, PCA_PRESCALE)) {
    meowLog(LOG_WARN, "PCA PRESCALE fail");
    return false;
  }
  if (!write8(REG_MODE1, 0x00)) {
    meowLog(LOG_WARN, "PCA MODE1 wake fail");
    return false;
  }
  delay(5);
  if (!write8(REG_MODE1, 0xA1)) {
    meowLog(LOG_WARN, "PCA MODE1 autoinc fail");
    return false;
  }
  if (!write8(REG_MODE2, 0x04)) {
    meowLog(LOG_WARN, "PCA MODE2 fail");
    return false;
  }
  delay(2);
  if (!i2cProbe(bus, addr)) {
    meowLog(LOG_WARN, "PCA gone after init");
    return false;
  }
  return true;
}

static bool tryPcaBus(TwoWire &bus, int sda, int scl, bool reopen) {
  if (reopen) {
    i2cOpen(bus, sda, scl, 100000);
    i2cScan(bus, "try");
  }
  if (i2cProbe(bus, 0x29) || i2cProbe(bus, 0x40) || i2cProbe(bus, 0x70)) {
    i2cSda = sda;
    i2cScl = scl;
    i2cLocked = true;
  }
  uint8_t a = findPcaAddr(bus);
  if (!a) return false;
  if (!initPcaOn(bus, a)) return false;
  i2cSda = sda;
  i2cScl = scl;
  i2cLocked = true;
  meowLogf(LOG_INFO, "PCA OK SDA=%d SCL=%d @0x%02X", sda, scl, a);
  return true;
}

static bool initPcaOnce() {
  if (i2cLocked) {
    return tryPcaBus(Wire, i2cSda, i2cScl, false);
  }
  /* Only 21/47 — GPIO38 is IMU 3V3 power; never open I2C there. */
  return tryPcaBus(Wire, PIN_I2C0_SDA, PIN_I2C0_SCL, true);
}

static bool initTofOn(TwoWire &bus) {
  if (!i2cProbe(bus, 0x29)) return false;
  tof.setBus(&bus);
  tofBus = &bus;
  tof.setTimeout(200);
  delay(10);
  if (!tof.init()) {
    delay(50);
    if (!tof.init()) return false;
  }
  tof.setMeasurementTimingBudget(33000);
  tof.startContinuous(50);
  delay(40);
  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (!tof.timeoutOccurred() && mm > 0 && mm < 8000) distMm = mm;
  return true;
}

static bool initTofOnce() {
  if (initTofOn(Wire)) {
    meowLog(LOG_INFO, "TOF OK @0x29 on current Wire");
    return true;
  }
  return false;
}

// Open-loop Motor Control Functions (no gyro required)
static void stopMotors() {
  setDrive(0, 0);
}

static void moveForward(int speed, int durationMillis) {
  setupDrivePins();
  setDrive((int16_t)speed, (int16_t)speed);
  delay(durationMillis);
  stopMotors();
}

static void moveBackward(int speed, int durationMillis) {
  setupDrivePins();
  setDrive((int16_t)(-speed), (int16_t)(-speed));
  delay(durationMillis);
  stopMotors();
}

static void turnRight(int speed) {
  setupDrivePins();
  setDrive((int16_t)speed, (int16_t)(-speed));
}

static void turnRight(int speed, int durationMillis) {
  setupDrivePins();
  setDrive((int16_t)speed, (int16_t)(-speed));
  delay(durationMillis);
  stopMotors();
}

static void turnLeft(int speed) {
  setupDrivePins();
  setDrive((int16_t)(-speed), (int16_t)speed);
}

static void turnLeft(int speed, int durationMillis) {
  setupDrivePins();
  setDrive((int16_t)(-speed), (int16_t)speed);
  delay(durationMillis);
  stopMotors();
}

static void updateTof() {
  static uint8_t tofFailStreak = 0;
  if (millis() - lastTofMs < 20) return;
  lastTofMs = millis();
  if (!tofOk) {
    static uint32_t lastTry;
    if (millis() - lastTry > 1500) {
      lastTry = millis();
      tofOk = initTofOnce();
      if (tofOk) tofFailStreak = 0;
    }
    return;
  }
  uint16_t mm = tof.readRangeContinuousMillimeters();
  /* Keep last good range — zeroing on every glitch made the UI look dead. */
  if (!tof.timeoutOccurred() && mm > 0 && mm < 8000) {
    distMm = mm;
    tofOk = true;
    tofFailStreak = 0;
  } else if (++tofFailStreak >= 10) {
    tofOk = false;
    tofFailStreak = 0;
    meowLog(LOG_WARN, "TOF lost — will retry");
  }
  if (tofOriginSet && distMm > 0)
    tofDispMm = (int16_t)((int32_t)tofOriginMm - (int32_t)distMm);
  else
    tofDispMm = 0;
}

static void updateColor() {
  bool moving = (cmdL != 0 || cmdR != 0);
  ColorRygOut o = colorRyg.update(moving, distMm, tofOk);
  /* Public label: 0 UNKNOWN · 1 RED · 2 YELLOW · 3 GREEN */
  colorLabel = o.label;
  colorConf = o.conf;
  /* Keep raw/chroma in telem for diagnostics (panel still shows label only) */
  colorR100 = o.r100;
  colorG100 = o.g100;
  colorB100 = o.y100; /* telem color_b = yellow Δ% (no blue channel) */
  lastRp = o.rp;
  lastGp = o.gp;
  lastBp = 0;
  lastCp = o.cp;
  lastColorMs = millis();
}

static void broadcastTelem() {
  meowler_RobotToClient out = meowler_RobotToClient_init_zero;
  out.which_msg = meowler_RobotToClient_telem_tag;
  meowler_Telemetry *t = &out.msg.telem;
  *t = meowler_Telemetry_init_zero;
  t->distance_mm = distMm;
  t->cmd_l = cmdL;
  t->cmd_r = cmdR;
  t->base = curB;
  t->height = curH;
  t->grip = curG;
  t->pca_ok = pcaOk;
  t->tof_ok = tofOk;
  t->imu_ok = imuOk;
  t->wifi_ok = wifiOk;
  int32_t el = 0, er = 0;
  encSnapshot_(&el, &er);
  t->enc_l = el;
  t->enc_r = er;
  t->wheel_l_mm = wheelMm(el);
  t->wheel_r_mm = wheelMm(er);
  t->tof_disp_mm = tofDispMm;
  t->yaw_cdeg = yawCdeg;
  t->pitch_cdeg = pitchCdeg;
  t->roll_cdeg = rollCdeg;
  t->color = colorLabel; /* 0 UNKNOWN 1 RED 2 YELLOW 3 GREEN */
  t->color_conf = colorConf;
  t->color_r = colorR100;
  t->color_g = colorG100;
  t->color_b = colorB100;
  t->color_rp = lastRp;
  t->color_gp = lastGp;
  t->color_bp = lastBp;
  t->color_cp = lastCp;
  t->conveyor = curConv;
  /* TCP clients get telem; skip USB encode when linked (cuts latency). */
  bool tcp = false;
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (netClients[i] && netClients[i].connected()) {
      tcp = true;
      break;
    }
  }
  if (tcp)
    broadcastRobot(out);
  else
    meowSendRobotSerial(out);
}

static void ensurePca() {
  if (pcaOk) return;
  pcaOk = initPcaOnce();
  if (pcaOk) meowLog(LOG_INFO, "PCA OK");
}

static void handleClientMsg(const meowler_ClientToRobot &msg) {
  switch (msg.which_op) {
    case meowler_ClientToRobot_stop_tag:
      setDrive(0, 0);
      ensurePca();
      setConveyor(0);
      recNote(3, 0, 0, 0, 0);
      sendAck(0);
      break;
    case meowler_ClientToRobot_zero_tag:
      zeroOdom();
      recNote(5, 0, 0, 0, 0);
      sendAck(0);
      break;
    case meowler_ClientToRobot_drive_tag:
      setDrive((int16_t)msg.op.drive.left, (int16_t)msg.op.drive.right);
      sendAck(0);
      break;
    case meowler_ClientToRobot_center_tag:
      ensurePca();
      applyArm(90, 90, 90);
      recNote(4, 90, 90, 90, 7);
      sendAck(0);
      break;
    case meowler_ClientToRobot_arm_tag: {
      ensurePca();
      int b = axB.tgt, h = axH.tgt, g = axG.tgt;
      uint32_t mask = 0;
      if (msg.op.arm.set_base) { b = msg.op.arm.base; mask |= 1; }
      if (msg.op.arm.set_height) { h = msg.op.arm.height; mask |= 2; }
      if (msg.op.arm.set_grip) { g = msg.op.arm.grip; mask |= 4; }
      if (msg.op.arm.set_base && msg.op.arm.set_height && msg.op.arm.set_grip) {
        applyArm(b, h, g);
      } else {
        if (msg.op.arm.set_base) requestBase(b);
        if (msg.op.arm.set_height) requestHeight(h);
        if (msg.op.arm.set_grip) requestGrip(g);
      }
      recNote(2, b, h, g, mask);
      sendAck(0);
      break;
    }
    case meowler_ClientToRobot_motor_test_tag:
      holdPcaMotorsOff();
      setupDrivePins();
      mtestPhase = 1;
      mtestMs = millis();
      recNote(7, 0, 0, 0, 0);
      setDrive(255, 0);
      meowLog(LOG_INFO, "MTEST L (tcp)");
      sendAck(0);
      break;
    case meowler_ClientToRobot_get_telem_tag:
      broadcastTelem();
      break;
    case meowler_ClientToRobot_conveyor_tag:
      ensurePca();
      setConveyor((int16_t)msg.op.conveyor.speed);
      recNote(6, (int32_t)msg.op.conveyor.speed, 0, 0, 0);
      sendAck(0);
      break;
    case meowler_ClientToRobot_color_cal_tag:
      colorRyg.calibrate(msg.op.color_cal.mode);
      sendAck(0);
      break;
    case meowler_ClientToRobot_rec_tag:
      if (msg.op.rec.action == 1) recStart(msg.op.rec.name);
      else recStop();
      sendAck(0);
      break;
    case meowler_ClientToRobot_ota_tag:
      handleOtaCmd(msg.op.ota);
      break;
    default:
      sendAck(1);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  meowLog(LOG_INFO, "MEOWLER boot (no gyro)");
  serialPrintHelp();

  setupDrivePins();
  setupEncoders();

  ColorRygPins cpins = {PIN_S2, PIN_S3, PIN_OUT, PIN_LED};
  colorRyg.begin(cpins);

  beginI2C();

  pcaOk = initPcaOnce();
  if (!pcaOk) {
    delay(400);
    pcaOk = initPcaOnce();
  }
  tofOk = initTofOnce();
  if (!tofOk) {
    delay(300);
    tofOk = initTofOnce();
  }

  if (pcaOk) {
    axesResetPose(90);  /* HOLD with continuous PWM = stall torque at 90° */
    setConveyor(0);
    holdPcaMotorsOff();  /* unused CH only — arm CH0–2 stay powered */
    lastPcaHoldMs = millis();
    lastAxisUs = micros();
    meowLog(LOG_INFO, "PCA 50Hz MG90 snap+HOLD (height fast-refresh)");
  }
  setDrive(0, 0);
  if (tofOk) {
    delay(60);
    updateTof();
  }
  zeroOdom();
  colorRyg.setYield([]() { pollNet(); });
  updateColor();
  startWifi();

  /* Stay on whatever slot we booted. OTA writes inactive slot; apply reboots into it. */
  chainloadOta1 = false;
  {
    const esp_partition_t *run = esp_ota_get_running_partition();
    meowLogf(LOG_INFO, "boot slot=%s — WiFi OTA to inactive (no chainload)",
             run && run->label ? run->label : "?");
  }

  meowLogf(LOG_INFO, "READY pca=%d tof=%d imu=%d wifi=%d tcp=:%u",
           pcaOk ? 1 : 0, tofOk ? 1 : 0, imuOk ? 1 : 0, wifiOk ? 1 : 0, (unsigned)NET_PORT);
  if (pcaOk)
    meowLogf(LOG_INFO, "PCA OK bus=Wire-%d/%d addr=0x%02X", i2cSda, i2cScl, pcaAddr);
  else
    meowLog(LOG_WARN, "PCA MISSING - SDA=21 SCL=47");
  if (!tofOk) meowLog(LOG_WARN, "TOF MISSING - VL53 @0x29 on 21/47");
  meowLogf(LOG_INFO, "COLOR RAW R/G/C=%u/%u/%u", lastRp, lastGp, lastCp);
  serialPrintStatus();
  colorRyg.setLog(false);  /* raw DEC= would break protobuf framing — keep off */
  meowLog(LOG_INFO, "Serial protobuf logs ready");
  meowLog(LOG_INFO, "TCP OTA ready — begin/chunk/finish/apply (inactive slot)");
}

void loop() {
  pollSerial();
  bootOta0PollButton();
  serviceWifi();
  if (otaRebootPending && (int32_t)(millis() - otaRebootAtMs) >= 0) {
    otaRebootPending = false;
    ESP.restart();
  }
  pollNet();
  serviceMotorTest();

  pollEncoders();
  applyDrive();
  serviceDriveHold();
  if (pcaOk) {
    /* Don't spam I2C every loop — that jittered the base servo */
    if (millis() - lastPcaHoldMs >= 1000) {
      lastPcaHoldMs = millis();
      holdPcaMotorsOff();
    }
    updateMg90Axes();
  } else {
    static uint32_t lastPcaTry = 0;
    if (millis() - lastPcaTry > 2000) {
      lastPcaTry = millis();
      if (initPcaOnce()) {
        pcaOk = true;
        axesResetPose(90);
        applyArm(axB.tgt, axH.tgt, axG.tgt);
        holdPcaMotorsOff();
        lastPcaHoldMs = millis();
        lastAxisUs = micros();
        meowLog(LOG_INFO, "PCA recovered");
      }
    }
  }
  pollNet();
  updateTof();
  pollNet();
  /* Color pulseIn blocks — throttle hard so drive/TOF stay snappy */
  static uint32_t lastColorPollMs = 0;
  if (!armBusy() && (millis() - lastColorPollMs >= 120)) {
    lastColorPollMs = millis();
    updateColor();
  }
  pollNet();

  if (millis() - lastTelemMs >= 40) {
    lastTelemMs = millis();
    broadcastTelem();
  }
}
