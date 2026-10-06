#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "meowler.pb.h"

static const size_t MEOW_MAX_FRAME = 4224;

inline bool meowWriteFrame(WiFiClient &c, const uint8_t *data, size_t n) {
  if (!c || !c.connected() || n > MEOW_MAX_FRAME) return false;
  uint8_t hdr[4] = {
      (uint8_t)(n & 0xFF), (uint8_t)((n >> 8) & 0xFF),
      (uint8_t)((n >> 16) & 0xFF), (uint8_t)((n >> 24) & 0xFF)};
  if (c.write(hdr, 4) != 4) return false;
  return c.write(data, n) == n;
}

inline bool meowSendRobot(WiFiClient &c, const meowler_RobotToClient &msg) {
  uint8_t buf[MEOW_MAX_FRAME];
  pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&stream, meowler_RobotToClient_fields, &msg)) return false;
  return meowWriteFrame(c, buf, stream.bytes_written);
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
