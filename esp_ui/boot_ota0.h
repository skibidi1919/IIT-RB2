#pragma once
/*
 * Dual-slot boot:
 *   OTA0 = updater (WiFi protobuf OTA). Always the slot a RESET returns to.
 *   OTA1 = uploaded app. Updater chainloads it after WiFi is up.
 * Hold BOOT (GPIO0) while OTA0 starts to stay on the updater (no chainload).
 */
#include <Arduino.h>
#include <string.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_app_desc.h"

#ifndef BOOT_OTA0_PIN
#define BOOT_OTA0_PIN 0
#endif

static const esp_partition_t *bootOtaSlot_(esp_partition_subtype_t sub) {
  return esp_partition_find_first(ESP_PARTITION_TYPE_APP, sub, NULL);
}

static bool bootOtaSelect(esp_partition_subtype_t sub) {
  const esp_partition_t *p = bootOtaSlot_(sub);
  if (!p) return false;
  return esp_ota_set_boot_partition(p) == ESP_OK;
}

/* Next RESET / power-cycle boots OTA0. Current app keeps running. */
static bool bootOtaPinNextResetTo0() { return bootOtaSelect(ESP_PARTITION_SUBTYPE_APP_OTA_0); }

static bool bootOta1HasApp() {
  const esp_partition_t *p = bootOtaSlot_(ESP_PARTITION_SUBTYPE_APP_OTA_1);
  if (!p) return false;
  esp_app_desc_t d;
  return esp_ota_get_partition_description(p, &d) == ESP_OK;
}

static bool bootOtaJump(esp_partition_subtype_t sub, const char *tag) {
  const esp_partition_t *p = bootOtaSlot_(sub);
  if (!p) return false;
  Serial.printf("boot %s (%s)\n", tag, p->label ? p->label : "?");
  Serial.flush();
  if (esp_ota_set_boot_partition(p) != ESP_OK) return false;
  delay(50);
  ESP.restart();
  return true;
}

static bool bootOta0Now() { return bootOtaJump(ESP_PARTITION_SUBTYPE_APP_OTA_0, "OTA0"); }
static bool bootOta1Now() { return bootOtaJump(ESP_PARTITION_SUBTYPE_APP_OTA_1, "OTA1"); }

static bool bootOta0BootHeld() {
  pinMode(BOOT_OTA0_PIN, INPUT_PULLUP);
  delay(5);
  return digitalRead(BOOT_OTA0_PIN) == LOW;
}

static void bootOta0PollButton() {
  static uint32_t t0;
  static bool down;
  pinMode(BOOT_OTA0_PIN, INPUT_PULLUP);
  bool held = digitalRead(BOOT_OTA0_PIN) == LOW;
  if (held) {
    if (!down) {
      down = true;
      t0 = millis();
    } else if (millis() - t0 >= 1500) {
      down = false;
      bootOta0Now();
    }
  } else {
    down = false;
  }
}
