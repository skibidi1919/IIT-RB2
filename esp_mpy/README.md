# Meowler — MicroPython port (`esp_mpy`)

Python firmware for the Freenove **ESP32-S3-WROOM N16R8** (16 MB flash, OPI/octal PSRAM), alongside the Arduino stack in `esp_ui/`. **Do not delete or break `esp_ui`.**

| Layer | Role |
|-------|------|
| Official MicroPython `ESP32_GENERIC_S3` **SPIRAM_OCT** | Runtime (flashed once) |
| `app/*.py` | Robot drivers + WiFi TCP :3333 protobuf |

Pinout matches `esp_ui` / `docs/ESP32_S3_PINOUT.md` (conveyor = **PCA CH4**).

---

## Quick start (Windows, prebuilt firmware)

```powershell
cd esp_mpy
python -m pip install -r requirements-host.txt

# 1) Flash MicroPython once (downloads SPIRAM_OCT .bin into firmware/)
.\scripts\flash_mpy.ps1 -Port COM5

# 2) WiFi secrets
copy app\secrets.py.example app\secrets.py
# edit WIFI_SSID / WIFI_PASS

# 3) Deploy app
.\scripts\deploy_app.ps1 -Port COM5 -WithSecrets
```

Robot joins WiFi, takes static `*.222`, listens **TCP :3333** with `uint32le | protobuf` frames (`proto/meowler.proto`).

Connect from `arm_ui` dashboard the same way as Arduino `esp_ui`.

---

## Firmware choice: prebuilt (preferred here)

This tree is written for **official prebuilt** MicroPython — no ESP-IDF uninstall, no source build required:

| Item | Value |
|------|--------|
| Board page | https://micropython.org/download/ESP32_GENERIC_S3/ |
| Variant | **SPIRAM_OCT** (N16R8 octal PSRAM) |
| Default file | `ESP32_GENERIC_S3-SPIRAM_OCT-20260824-v1.29.0.bin` |
| Flash address | `0x0` |

`scripts/flash_mpy.ps1` / `.bat` download that `.bin` into `esp_mpy/firmware/` and run `esptool`.

### Optional: build from source

Only if you need a custom partition (full 16 MB) or frozen modules. Prefer MicroPython’s **bundled/submoduled ESP-IDF** — do not wipe a system-wide IDF install.

```text
git clone --recurse-submodules https://github.com/micropython/micropython.git esp_mpy/vendor/micropython
# Follow https://docs.micropython.org/en/latest/develop/gettingstarted.html
# ESP32-S3:
cd esp_mpy/vendor/micropython
make -C mpy-cross
cd ports/esp32
make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT submodules
make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT
# flash: make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT PORT=COM5 deploy
```

`vendor/micropython` is **not** checked in by default (large). Use prebuilt unless you need a custom image.

---

## Deploy / OTA

| Method | When |
|--------|------|
| `mpremote` (`scripts/deploy_app.*`) | USB — day-to-day app updates |
| `ota_http.pull_app_manifest(url)` | Device already on WiFi; host serves `app/` over HTTP |
| WebREPL | Optional; enable separately on device |

Arduino dual-slot `.bin` OTA (`OtaCmd`) is **not** used — that path is `esp_ui` only. See `app/ota_http.py` and `app/mission_host.md`.

HTTP update example (on device REPL after WiFi is up):

```python
import ota_http
ota_http.pull_app_manifest("http://192.168.1.10:8000/app/")
ota_http.soft_reboot()
```

Host: `cd esp_mpy/app && python -m http.server 8000`

---

## App layout

```text
app/
  boot.py           # minimal boot
  main.py           # WiFi + TCP server + command loop
  pinout.py         # hardware constants
  secrets.py.example
  pca9685.py        # 150 Hz, arm CH0–2, conveyor CH4
  vl53.py           # VL53L0X continuous
  drive.py          # L298N + encoder poll
  color_tcs.py      # TCS3200 R/Y/G (from esp_ui/color_ryg.h)
  protocol.py       # uint32le + minimal protobuf codec
  ota_http.py       # HTTP / mpremote update notes
  mission_host.md   # missions stay on PC by default
```

### Behaviour (parity with `esp_ui`)

- Boot → I2C scan 21/47 → PCA + ToF → WiFi DHCP → learn gateway → static `*.222` → TCP :3333
- Commands: Drive, Arm, Conveyor, Stop, Center, Zero, MotorTest, GetTelem, ColorCal, RecCtrl
- Telemetry ~5 Hz: distance, color, pca/tof/wifi flags, encoders, conveyor
- Conveyor: continuous servo on **CH4**, stop = PWM full-off
- I2C recover on PCA miss
- Recording: host `arm_ui` preferred; on-device RecEvent stream supported

### Protocol

Primary: length-prefixed protobuf (`protocol.py`).  
Fallback: JSON lines, e.g. `{"op":"drive","left":100,"right":100}\n` (documented for debug; prefer protobuf for `arm_ui`).

---

## Hardware checklist (needs on-robot verification)

- [ ] PCA9685 @0x40, 150 Hz arm + CH4 conveyor
- [ ] VL53L0X @0x29 ranging
- [ ] L298N left/right + encoder counts
- [ ] TCS3200 R/Y/G lock vs ambient
- [ ] WiFi static `.222` + `arm_ui` Connect
- [ ] Drive hold timeout (400 ms)

IMU / Arduino OTA / soft BNO paths are **not** ported.
