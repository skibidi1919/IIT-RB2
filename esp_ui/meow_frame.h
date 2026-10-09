#pragma once
#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <lwip/sockets.h>
#endif
#include "pb_encode.h"
#include "pb_decode.h"
#include "meowler.pb.h"

/* RX holds 4096B OTA chunks. TX (logs/status) stays small so loop stack survives. */
static const size_t MEOW_MAX_FRAME = 4224;
/* Telemetry worst-case ~271B; leave headroom for Log/OtaStatus */
static const size_t MEOW_TX_FRAME = 512;

/* Non-blocking TCP TX — NetworkClient::write can sit in select() up to ~10s when the
 * peer window is full, which freezes pollNet/arm and makes the host report timeouts. */
inline bool meowWriteFrame(WiFiClient &c, const uint8_t *data, size_t n) {
  if (!c || !c.connected() || n > MEOW_MAX_FRAME) return false;
  uint8_t hdr[4] = {
      (uint8_t)(n & 0xFF), (uint8_t)((n >> 8) & 0xFF),
      (uint8_t)((n >> 16) & 0xFF), (uint8_t)((n >> 24) & 0xFF)};
#if defined(ARDUINO_ARCH_ESP32)
  int sock = c.fd();
  if (sock >= 0) {
    ssize_t w1 = ::send(sock, hdr, 4, MSG_DONTWAIT);
    if (w1 != 4) return false;
    ssize_t w2 = ::send(sock, data, n, MSG_DONTWAIT);
    return w2 == (ssize_t)n;
  }
#endif
  if (c.write(hdr, 4) != 4) return false;
  return c.write(data, n) == n;
}

inline bool meowWriteFrameSerial(const uint8_t *data, size_t n) {
  if (!data || n > MEOW_MAX_FRAME) return false;
  uint8_t hdr[4] = {
      (uint8_t)(n & 0xFF), (uint8_t)((n >> 8) & 0xFF),
      (uint8_t)((n >> 16) & 0xFF), (uint8_t)((n >> 24) & 0xFF)};
  if (Serial.write(hdr, 4) != 4) return false;
  return Serial.write(data, n) == n;
}

inline bool meowEncodeRobot(const meowler_RobotToClient &msg, uint8_t *buf, size_t cap, size_t *outLen) {
  pb_ostream_t stream = pb_ostream_from_buffer(buf, cap);
  if (!pb_encode(&stream, meowler_RobotToClient_fields, &msg)) return false;
  *outLen = stream.bytes_written;
  return true;
}

inline bool meowSendRobot(WiFiClient &c, const meowler_RobotToClient &msg) {
  static uint8_t buf[MEOW_TX_FRAME];
  size_t n = 0;
  if (!meowEncodeRobot(msg, buf, sizeof(buf), &n)) return false;
  return meowWriteFrame(c, buf, n);
}

inline bool meowSendRobotSerial(const meowler_RobotToClient &msg) {
  static uint8_t buf[MEOW_TX_FRAME];
  size_t n = 0;
  if (!meowEncodeRobot(msg, buf, sizeof(buf), &n)) return false;
  return meowWriteFrameSerial(buf, n);
}

struct MeowRx {
  uint8_t hdr[4];
  uint8_t hdrN;
  uint8_t body[MEOW_MAX_FRAME];
  uint32_t need;
  uint32_t got;
  bool inBody;
};

inline void meowRxReset(MeowRx *rx) {
  rx->hdrN = 0;
  rx->need = 0;
  rx->got = 0;
  rx->inBody = false;
}

/* Feed one byte. Returns true when a full ClientToRobot message is decoded into out. */
inline bool meowRxFeed(MeowRx *rx, uint8_t b, meowler_ClientToRobot *out) {
  if (!rx->inBody) {
    rx->hdr[rx->hdrN++] = b;
    if (rx->hdrN < 4) return false;
    rx->need = (uint32_t)rx->hdr[0] | ((uint32_t)rx->hdr[1] << 8) |
               ((uint32_t)rx->hdr[2] << 16) | ((uint32_t)rx->hdr[3] << 24);
    rx->hdrN = 0;
    rx->got = 0;
    if (rx->need == 0 || rx->need > MEOW_MAX_FRAME) {
      meowRxReset(rx);
      return false;
    }
    rx->inBody = true;
    return false;
  }
  rx->body[rx->got++] = b;
  if (rx->got < rx->need) return false;
  pb_istream_t stream = pb_istream_from_buffer(rx->body, rx->need);
  *out = meowler_ClientToRobot_init_zero;
  bool ok = pb_decode(&stream, meowler_ClientToRobot_fields, out);
  meowRxReset(rx);
  return ok;
}

/* Copy as many bytes as possible. *used is always set. True = one message decoded. */
inline bool meowRxPump(MeowRx *rx, const uint8_t *p, size_t n, size_t *used,
                       meowler_ClientToRobot *out) {
  *used = 0;
  if (!p || n == 0) return false;
  if (!rx->inBody) {
    while (*used < n && rx->hdrN < 4) {
      rx->hdr[rx->hdrN++] = p[(*used)++];
    }
    if (rx->hdrN < 4) return false;
    rx->need = (uint32_t)rx->hdr[0] | ((uint32_t)rx->hdr[1] << 8) |
               ((uint32_t)rx->hdr[2] << 16) | ((uint32_t)rx->hdr[3] << 24);
    rx->hdrN = 0;
    rx->got = 0;
    if (rx->need == 0 || rx->need > MEOW_MAX_FRAME) {
      meowRxReset(rx);
      return false;
    }
    rx->inBody = true;
    if (*used >= n) return false;
  }
  size_t want = rx->need - rx->got;
  size_t take = n - *used;
  if (take > want) take = want;
  memcpy(rx->body + rx->got, p + *used, take);
  rx->got += (uint32_t)take;
  *used += take;
  if (rx->got < rx->need) return false;
  pb_istream_t stream = pb_istream_from_buffer(rx->body, rx->need);
  *out = meowler_ClientToRobot_init_zero;
  bool ok = pb_decode(&stream, meowler_ClientToRobot_fields, out);
  meowRxReset(rx);
  return ok;
}
