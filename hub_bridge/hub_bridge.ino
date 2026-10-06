/*
 * Meowler laptop HUB — ESP32 / ESP32-S3 (Freenove CAM UART/CH343)
 *
 * Transparent framed protobuf bridge:
 *   Laptop USB (115200)  <-->  ESP-NOW  <-->  robot color_bridge
 *
 * Frame: 0xA5 | len | LinkMessage | xor
 * Forwards cmd (USB→NOW) and telem/log (NOW→USB) at full rate.
 *
 * Classic NodeMCU-32S:
 *   esp32:esp32:nodemcu-32s:UploadSpeed=115200
 * Freenove ESP32-S3 CAM (CH343 UART port — not native USB-CDC):
 *   esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=8M,PSRAM=opi,PartitionScheme=default_8MB
 */

#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

#include "meowler.pb.h"
#include "meow_frame.h"

static uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool espnowOk = false;
static MeowFrameParser usbParser;
static uint32_t nUsb = 0, nNow = 0, lastStatMs = 0;

static void sendNow(const uint8_t *data, size_t len) {
  if (!espnowOk || !len || len > 250) return;
  esp_now_send(broadcastAddr, data, len);
}

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len < 3 || data[0] != MEOW_FRAME_MAGIC) return;
  Serial.write(data, len);
  nNow++;
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
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  // Freenove S3 CH343: wait briefly; native-CDC boards also fine
  delay(400);
  while (millis() < 1500) {
    if (Serial) break;
    delay(10);
  }
  meow_parser_init(&usbParser);
  espnowOk = initEspNow();
  // Banner is ASCII; laptop framed parser skips until 0xA5
  Serial.println();
  Serial.println(F("HUB READY"));
  Serial.print(F("ESPNOW "));
  Serial.println(espnowOk ? F("OK") : F("FAIL"));
  Serial.print(F("MAC "));
  Serial.println(WiFi.macAddress());
  Serial.println(F("USB 115200 framed LinkMessage"));
  Serial.flush();
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t plen = 0;
    if (meow_parser_feed(&usbParser, (uint8_t)Serial.read(), &plen)) {
      meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
      if (meow_frame_decode(meowler_LinkMessage_fields, &msg, usbParser.buf, plen) &&
          msg.which_payload == meowler_LinkMessage_cmd_tag) {
        uint8_t frame[MEOW_FRAME_MAX_PAYLOAD + 4];
        frame[0] = MEOW_FRAME_MAGIC;
        frame[1] = plen;
        memcpy(frame + 2, usbParser.buf, plen);
        frame[2 + plen] = meow_xor_checksum(plen, usbParser.buf);
        sendNow(frame, 3 + plen);
        nUsb++;
      }
    }
  }

  // occasional hub health as DebugLog so laptop sees link alive
  uint32_t now = millis();
  if (now - lastStatMs > 2000) {
    lastStatMs = now;
    char buf[40];
    snprintf(buf, sizeof(buf), "hub usb=%lu now=%lu", (unsigned long)nUsb, (unsigned long)nNow);
    meowler_LinkMessage msg = meowler_LinkMessage_init_zero;
    msg.which_payload = meowler_LinkMessage_log_tag;
    msg.payload.log.t_ms = now;
    msg.payload.log.level = meowler_LogLevel_LOG_DEBUG;
    size_t n = strnlen(buf, 39);
    memcpy(msg.payload.log.text.bytes, buf, n);
    msg.payload.log.text.size = n;
    uint8_t frame[64];
    size_t fl = meow_frame_encode(meowler_LinkMessage_fields, &msg, frame, sizeof(frame));
    if (fl) Serial.write(frame, fl);
  }
}
