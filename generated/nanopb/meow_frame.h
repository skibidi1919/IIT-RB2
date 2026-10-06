#pragma once
/*
 * Shared 0xA5 framed nanopb helper (USB / SoftSerial / ESP-NOW payload).
 * Frame: magic | len | protobuf | xor(len ^ payload...)
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

static const uint8_t MEOW_FRAME_MAGIC = 0xA5;
static const size_t MEOW_FRAME_MAX_PAYLOAD = 120;

static inline uint8_t meow_xor_checksum(uint8_t length, const uint8_t *payload) {
  uint8_t x = length;
  for (uint8_t i = 0; i < length; i++) x ^= payload[i];
  return x;
}

static inline size_t meow_frame_encode(const pb_msgdesc_t *fields, const void *msg,
                                       uint8_t *out, size_t out_cap) {
  if (out_cap < 3) return 0;
  uint8_t payload[MEOW_FRAME_MAX_PAYLOAD];
  pb_ostream_t stream = pb_ostream_from_buffer(payload, sizeof(payload));
  if (!pb_encode(&stream, fields, msg)) return 0;
  size_t len = stream.bytes_written;
  if (len > 255 || (3 + len) > out_cap) return 0;
  out[0] = MEOW_FRAME_MAGIC;
  out[1] = (uint8_t)len;
  memcpy(out + 2, payload, len);
  out[2 + len] = meow_xor_checksum((uint8_t)len, payload);
  return 3 + len;
}

typedef struct {
  uint8_t buf[MEOW_FRAME_MAX_PAYLOAD + 4];
  uint8_t len;
  uint8_t state;  // 0=magic 1=len 2=payload 3=chk
  uint8_t need;
} MeowFrameParser;

static inline void meow_parser_init(MeowFrameParser *p) {
  p->len = 0;
  p->state = 0;
  p->need = 0;
}

/* Feed one byte. Returns true when a full checksum-valid frame is in p->buf[2..].
 * Payload length is p->need; payload pointer is p->buf. */
static inline bool meow_parser_feed(MeowFrameParser *p, uint8_t b, uint8_t *payload_len) {
  if (p->state == 0) {
    if (b == MEOW_FRAME_MAGIC) {
      p->state = 1;
      p->len = 0;
    }
    return false;
  }
  if (p->state == 1) {
    p->need = b;
    p->len = 0;
    p->state = (b == 0) ? 3 : 2;
    return false;
  }
  if (p->state == 2) {
    if (p->len < MEOW_FRAME_MAX_PAYLOAD) p->buf[p->len++] = b;
    if (p->len >= p->need) p->state = 3;
    return false;
  }
  // checksum
  p->state = 0;
  if (b != meow_xor_checksum(p->need, p->buf)) return false;
  *payload_len = p->need;
  return true;
}

static inline bool meow_frame_decode(const pb_msgdesc_t *fields, void *msg,
                                     const uint8_t *payload, uint8_t len) {
  pb_istream_t stream = pb_istream_from_buffer(payload, len);
  return pb_decode(&stream, fields, msg);
}


