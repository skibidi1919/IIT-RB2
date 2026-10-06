#pragma once
/*
 * Protobuf WiFi OTA — A/B slots, no chainload.
 * Running app accepts begin/chunk/finish/apply over TCP :3333.
 * Writes always go to the inactive slot (esp_ota_get_next_update_partition).
 * apply() sets boot partition to that slot and reboots.
 * Bad image / crash → ESP-IDF rolls back to previous slot.
 */
#include <Arduino.h>
#include <string.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "meowler.pb.h"

enum : uint32_t {
  OTA_ST_IDLE = 0,
  OTA_ST_RECV = 1,
  OTA_ST_READY = 2,
  OTA_ST_APPLY = 3,
  OTA_ST_ERR = 4,
  OTA_ST_REBOOT = 5,
};

enum : uint32_t {
  OTA_E_OK = 0,
  OTA_E_ARGS = 1,
  OTA_E_NOMEM = 2,
  OTA_E_RANGE = 3,
  OTA_E_INCOMPLETE = 4,
  OTA_E_INFLATE = 5,
  OTA_E_CRC = 6,
  OTA_E_BEGIN = 7,
  OTA_E_WRITE = 8,
  OTA_E_END = 9,
  OTA_E_STATE = 10,
};

#ifndef OTA_MAX_RAW
#define OTA_MAX_RAW (8U * 1024U * 1024U)
#endif

struct OtaPbState {
  uint32_t state;
  uint32_t error;
  char detail[48];
  uint32_t rawSize;
  uint32_t rawCrc;
  uint32_t packedSize;
  uint32_t received;
  uint32_t written;
  uint32_t crcRun;
  bool handleOpen;
  esp_ota_handle_t handle;
  const esp_partition_t *part;
  char version[24];
};

static OtaPbState gOta = {};

static uint32_t otaCrc32Add(uint32_t c, const uint8_t *data, size_t n) {
  for (size_t i = 0; i < n; i++) {
    c ^= data[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return c;
}

static void otaSetDetail(const char *s) {
  memset(gOta.detail, 0, sizeof(gOta.detail));
  if (s) strncpy(gOta.detail, s, sizeof(gOta.detail) - 1);
}

static void otaCloseHandle_() {
  if (gOta.handleOpen) {
    esp_ota_abort(gOta.handle);
    gOta.handleOpen = false;
    gOta.handle = 0;
  }
}

static void otaFail(uint32_t err, const char *detail) {
  gOta.state = OTA_ST_ERR;
  gOta.error = err;
  otaSetDetail(detail);
  otaCloseHandle_();
}

static void otaReset() {
  otaCloseHandle_();
  memset(&gOta, 0, sizeof(gOta));
  gOta.state = OTA_ST_IDLE;
  gOta.crcRun = 0xFFFFFFFFu;
}

static void otaFillStatus(meowler_OtaStatus *st) {
  *st = meowler_OtaStatus_init_zero;
  st->state = gOta.state;
  st->received = gOta.received;
  st->packed_size = gOta.packedSize;
  st->written = gOta.written;
  st->error = gOta.error;
  strncpy(st->detail, gOta.detail, sizeof(st->detail) - 1);
}

static bool otaBegin(const meowler_OtaBegin &b) {
  otaReset();
  uint32_t sz = b.raw_size ? b.raw_size : b.packed_size;
  if (sz == 0 || sz > OTA_MAX_RAW) {
    otaFail(OTA_E_ARGS, "bad size");
    return false;
  }
  if (b.codec != 0) {
    otaFail(OTA_E_ARGS, "raw only (no compress)");
    return false;
  }
  if (b.packed_size && b.packed_size != sz) {
    otaFail(OTA_E_ARGS, "packed must equal raw");
    return false;
  }

  gOta.part = esp_ota_get_next_update_partition(NULL);
  if (!gOta.part) {
    otaFail(OTA_E_BEGIN, "no inactive OTA slot");
    return false;
  }
  if (sz > gOta.part->size) {
    otaFail(OTA_E_ARGS, "bigger than OTA slot");
    return false;
  }

  esp_err_t e = esp_ota_begin(gOta.part, OTA_WITH_SEQUENTIAL_WRITES, &gOta.handle);
  if (e != ESP_OK) {
    otaFail(OTA_E_BEGIN, "esp_ota_begin fail");
    return false;
  }
  gOta.handleOpen = true;
  gOta.rawSize = sz;
  gOta.rawCrc = b.raw_crc32;
  gOta.packedSize = sz;
  gOta.received = 0;
  gOta.written = 0;
  gOta.crcRun = 0xFFFFFFFFu;
  gOta.state = OTA_ST_RECV;
  gOta.error = OTA_E_OK;
  strncpy(gOta.version, b.version, sizeof(gOta.version) - 1);
  char d[48];
  snprintf(d, sizeof(d), "recv %s", gOta.part->label ? gOta.part->label : "?");
  otaSetDetail(gOta.version[0] ? gOta.version : d);
  return true;
}

static bool otaChunk(const meowler_OtaChunk &ch) {
  if (gOta.state != OTA_ST_RECV || !gOta.handleOpen) {
    otaFail(OTA_E_STATE, "chunk not receiving");
    return false;
  }
  uint32_t n = ch.data.size;
  if (n == 0 || n > 4096) {
    otaFail(OTA_E_RANGE, "bad chunk len");
    return false;
  }
  if (ch.offset != gOta.written) {
    otaFail(OTA_E_RANGE, "chunks must be sequential");
    return false;
  }
  if ((uint64_t)ch.offset + n > gOta.rawSize) {
    otaFail(OTA_E_RANGE, "chunk overrun");
    return false;
  }
  esp_err_t e = esp_ota_write(gOta.handle, ch.data.bytes, n);
  if (e != ESP_OK) {
    otaFail(OTA_E_WRITE, "esp_ota_write fail");
    return false;
  }
  gOta.crcRun = otaCrc32Add(gOta.crcRun, ch.data.bytes, n);
  gOta.written += n;
  gOta.received = gOta.written;
  return true;
}

static bool otaFinish() {
  if (gOta.state != OTA_ST_RECV || !gOta.handleOpen) {
    otaFail(OTA_E_STATE, "finish not receiving");
    return false;
  }
  if (gOta.written != gOta.rawSize) {
    otaFail(OTA_E_INCOMPLETE, "short image");
    return false;
  }
  uint32_t crc = ~gOta.crcRun;
  if (crc != gOta.rawCrc) {
    otaFail(OTA_E_CRC, "crc mismatch");
    return false;
  }
  esp_err_t e = esp_ota_end(gOta.handle);
  gOta.handleOpen = false;
  gOta.handle = 0;
  if (e != ESP_OK) {
    otaFail(OTA_E_END, "esp_ota_end fail");
    return false;
  }
  gOta.state = OTA_ST_READY;
  gOta.error = OTA_E_OK;
  char d[48];
  snprintf(d, sizeof(d), "ready — apply boots %s",
           gOta.part && gOta.part->label ? gOta.part->label : "?");
  otaSetDetail(d);
  return true;
}

static bool otaApply() {
  if (gOta.state != OTA_ST_READY || !gOta.part) {
    otaFail(OTA_E_STATE, "not ready");
    return false;
  }
  if (esp_ota_set_boot_partition(gOta.part) != ESP_OK) {
    otaFail(OTA_E_END, "set boot fail");
    return false;
  }
  gOta.state = OTA_ST_REBOOT;
  otaSetDetail("rebooting new slot");
  return true;
}
