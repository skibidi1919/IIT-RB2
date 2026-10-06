# Board: Freenove ESP32-S3-WROOM N16R8

| Spec | Value |
|------|--------|
| Module | ESP32-S3-WROOM-1 |
| Flash | 16 MB |
| PSRAM | 8 MB **OPI / Octal** |
| USB | CH343 UART (COM port) + native USB (optional) |

## MicroPython firmware choice

Use the official **Octal SPIRAM** build (N16R8 = octal PSRAM):

```text
ESP32_GENERIC_S3-SPIRAM_OCT-*-v*.bin
```

Download page: https://micropython.org/download/ESP32_GENERIC_S3/

Pinned URL used by `scripts/flash_mpy.ps1` (update version as needed):

```text
https://micropython.org/resources/firmware/ESP32_GENERIC_S3-SPIRAM_OCT-20260824-v1.29.0.bin
```

Standard (non-OCT) `ESP32_GENERIC_S3` also auto-detects some PSRAM, but **SPIRAM_OCT is required** for Freenove N16R8 OPI PSRAM.

## Flash map note

Stock MicroPython images often expose ~8 MB of flash to the filesystem even on 16 MB modules. That is enough for `app/*.py`. A custom MicroPython build with a 16 MB partition table is optional (see `esp_mpy/README.md` source-build notes).

## Critical GPIOs (Meowler)

- **I2C:** SDA=21, SCL=47 — PCA9685 `0x40`, VL53L0X `0x29`
- **GPIO14:** onboard FLASH LED — **never I2C**
- **Avoid:** 35–37 (OPI PSRAM), be careful with strapping 0/3/45/46
- **Conveyor:** PCA **CH4** (not CH15)

Full table: `docs/ESP32_S3_PINOUT.md` (repo root) and `app/pinout.py`.

## Arduino vs MicroPython

| Stack | Path | Notes |
|-------|------|--------|
| Arduino | `esp_ui/` | Full OTA slots, IMU, production brain |
| MicroPython | `esp_mpy/` | This port — Python drivers + TCP protobuf |

Do not flash MicroPython over `esp_ui` unless you intend to replace the Arduino firmware (re-flash `esp_ui` with Arduino-CLI to restore).
