# MEOW — Meowler robot documentation

Full hardware map for the robot brain on a **Freenove ESP32-S3-WROOM N16R8** board (camera module removed / unused). Firmware: `esp_ui/`. Control panel: `arm_ui/` → `http://127.0.0.1:5050`.

---

## Board

| Item | Value |
|------|--------|
| Board | Freenove ESP32-S3-WROOM **N16R8** (16 MB flash, 8 MB OPI PSRAM) |
| Role | Robot brain — drive, arm, conveyor, ToF, IMU, colour, WiFi TCP |
| Camera | **Off / removed** — cam GPIOs freed for robot I/O |
| USB | CH343 UART (often **COM5**) · serial debug @ 115200 |
| Sketch | `esp_ui/` |

### Arduino FQBN

```text
esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600
```

### Pins to avoid

| GPIO | Why |
|------|-----|
| **35–37** | OPI PSRAM (do not use) |
| **0 / 3 / 45 / 46** | Strapping — use only with care |

### Power commons

- Share **GND** across ESP32, L298N logic, PCA9685, VL53L0X, BNO08x, TCS3200
- Logic level **3.3 V** for I2C sensors (ESP32 is not 5 V tolerant on GPIO)
- Motor **VM / +12 V** from battery on L298N only — not into ESP 3V3

---

## GPIO summary (ESP32-S3)

| GPIO | Function |
|------|----------|
| **4 / 5** | Encoder M1 (C1 / C2) |
| **6 / 7** | Encoder M2 (C1 / C2) |
| **8** | L298N IN4 (M1) |
| **9** | L298N ENA PWM (M2) |
| **10** | L298N ENB PWM (M1) |
| **11 / 12** | L298N IN1 / IN2 (M2) |
| **13** | L298N IN3 (M1) |
| **14** | Board FLASH LED (not I2C) |
| **15 / 16** | TCS3200 S2 / S3 |
| **17 / 18** | TCS3200 OUT / LED |
| **21 / 47** | I2C0 SDA / SCL — PCA9685 + VL53 + BNO08x |
| **38** | BNO08x **3V3 power** (GPIO HIGH ≈ 3.3 V → IMU VIN) |
| **41 / 42** | Soft-I2C fallback for BNO (only if not on 21/47) |

---

## 1. Drive — L298N + N20 geared motors

Dual H-bridge. Pull **ENA** and **ENB** jumpers off so ESP PWM can control speed.  
**M1 = OUT3/4** (left cmd) · **M2 = OUT1/2** (right cmd).

### M1 — left (OUT3 / OUT4)

| L298N | ESP32 GPIO | Notes |
|-------|------------|--------|
| **ENB** | **10** | LEDC PWM 20 kHz, 8-bit |
| **IN3** | **13** | Direction |
| **IN4** | **8** | Direction |
| OUT3 / OUT4 | Motor M1 | Geared N20 |
| Logic VCC | 5 V (shared) | |
| GND | Common GND | |
| +12V / VM | Battery | Motor power |

### M2 — right (OUT1 / OUT2)

| L298N | ESP32 GPIO | Notes |
|-------|------------|--------|
| **ENA** | **9** | LEDC PWM 20 kHz, 8-bit |
| **IN1** | **11** | Direction |
| **IN2** | **12** | Direction |
| OUT1 / OUT2 | Motor M2 | Geared N20 |

Do **not** wire PCA PWM outputs into ENA/ENB/IN1–IN4.

---

## 2. Wheel encoders

Quadrature (or dual digital) channels per motor. `INPUT_PULLUP` in firmware.

| Signal | ESP32 GPIO |
|--------|------------|
| M1 C1 | **4** |
| M1 C2 | **5** |
| M2 C1 | **6** |
| M2 C2 | **7** |

Wheel diameter used in firmware: **43 mm** · steps/rev ≈ **600** → mm/step in telemetry.

---

## 3. PCA9685 — arm + conveyor PWM

16-channel PWM driver @ **≈150 Hz**. Pulse width **500–2500 µs** for MG90 (0–180°).

### PCA ↔ ESP32

| PCA9685 | ESP32 / power | Notes |
|---------|---------------|--------|
| **VCC** | **3.3 V** | Logic |
| **V+** | 5–6 V servo rail | Servo power (separate) |
| **GND** | Common GND | |
| **SDA** | **21** (`Wire`) | Shared with VL53 + BNO |
| **SCL** | **47** (`Wire`) | |
| **OE** | **GND** | Outputs enabled |
| ADDR | `0x40` (default) | Firmware scans `0x40`–`0x47` |

Firmware also falls back to Wire1 **38/39** if PCA is found there.

### PCA channels

| CH | Part | Type | Behaviour |
|----|------|------|-----------|
| **0** | Arm **base** | MG90 | Soft ramp → hold → relax → reassert → relax |
| **1** | Arm **height** | MG90 | Same as base |
| **2** | Arm **grip** | MG90 | Snap (no soft ramp) |
| **15** | **UM conveyor** | SG90 **360°** continuous | Speed −255…255 · PWM off = stop |

Unused CH3–CH14: free.

---

## 4. Arm — MG90 servos (via PCA)

| Joint | PCA CH | Signal | Power |
|-------|--------|--------|--------|
| Base | 0 | PCA PWM0 | Servo V+ / GND |
| Height | 1 | PCA PWM1 | Servo V+ / GND |
| Grip | 2 | PCA PWM2 | Servo V+ / GND |

Panel: base / height / grip degrees · Center / Open / Close.

---

## 5. Conveyor — SG90 360° (via PCA CH4)

| Item | Value |
|------|--------|
| Servo | Continuous rotation SG90-360 |
| PCA channel | **4** |
| Stop | PWM full-off (or ~1500 µs mid) |
| Drive | Pulse ~1000–2000 µs mapped from speed −255…255 |

Panel: Forward / Stop / Reverse + speed slider.

---

## 6. VL53L0X — Time-of-Flight distance

| VL53L0X | ESP32 / power | Notes |
|---------|---------------|--------|
| **VIN / VCC** | **3.3 V** (or 5 V if breakout regulates) | Match board |
| **GND** | Common GND | |
| **SDA** | **21** (`Wire`) | Shared with PCA + BNO |
| **SCL** | **47** (`Wire`) | |
| ADDR | **`0x29`** | Fixed |

Telemetry: `distance_mm`, `tof_disp_mm` (relative to origin when set).

---

## 7. BNO08x / BNO085 — IMU (PATH ECC)

I2C mode: **PS0 = GND**, **PS1 = GND**. Address **`0x4A`** (ADR low) or **`0x4B`** (ADR high).

**3V3-only modules:** board rail is often 5 V — do **not** feed 5 V into VIN. Firmware drives **GPIO38 HIGH** (~3.3 V) as IMU power.

### Wire it (same I2C as PCA + VL53)

| BNO08x | ESP32 |
|--------|--------|
| **VIN / VCC** | **GPIO 38** (firmware sets HIGH @ boot) |
| **GND** | Common GND |
| **SDA** | **21** (same as PCA + ToF) |
| **SCL** | **47** (same as PCA + ToF) |
| **PS0 / PS1** | **GND** (I2C mode) |
| ADR | GND → `0x4A` (or HIGH → `0x4B`) |
| INT / RST | nc |

### Soft-I2C fallback (only if not on 21/47)

| BNO08x | ESP32 |
|--------|--------|
| SDA | **41** |
| SCL | **42** |
| VIN | still **GPIO 38** |

GPIO38 can source ~10–20 mA — enough for one BNO; keep wiring short. Panel: yaw / pitch / roll (centidegrees) for PATH ECC later.

---

## 8. TCS3200 — colour sensor (R / Y / G)

Output frequency scaling fixed at **20%**: S0 high, S1 low (hardwired).

| TCS3200 | Connect | Notes |
|---------|---------|--------|
| **VCC** | **3.3 V** (or 5 V if OUT is safe / divided) | Prefer 3.3 V with ESP |
| **GND** | Common GND | |
| **S0** | **5 V** or **3.3 V** (HIGH) | 20% scale |
| **S1** | **GND** | 20% scale |
| **S2** | GPIO **15** | Filter select |
| **S3** | GPIO **16** | Filter select |
| **OUT** | GPIO **17** | Frequency / pulse in |
| **LED** | GPIO **18** | White LED on |

Firmware (`color_ryg.h`): chroma R/Y/G scores, presence gate, Final lock RED / YELLOW / GREEN. Blue filter is internal only. Panel: pulses, bars, Final + conf.

---

## 9. WiFi / control link

| Item | Value |
|------|--------|
| Mode | STA |
| Static IP | **Same subnet as gateway, always ends in `.222`** (DHCP learn → static) |
| Gateway / DNS | From the AP (DHCP), then reused for static config |
| Protocol | Length-prefixed **protobuf** (`proto/meowler.proto`) |
| Port | **TCP 3333** |
| Panel | `cd arm_ui` → `uv run dashboard` → Connect `<lan>.222:3333` |

Serial USB is **debug only** — not the control plane.

---

## 10. I2C bus map (as wired in firmware)

| Bus | SDA / SCL | Devices |
|-----|-----------|---------|
| **Wire** | **21 / 47** | PCA9685 `0x40` · VL53L0X `0x29` · BNO08x `0x4A`/`0x4B` |
| Soft | **41 / 42** | BNO fallback only |

All I2C logic: **3.3 V**, common **GND**, pull-ups to **3.3 V**. BNO **VIN** = **GPIO38 HIGH** (not 5 V).

---

## 11. Software map

| Path | Role |
|------|------|
| `esp_ui/` | ESP32-S3 firmware (this pinout) |
| `esp_ui/color_ryg.h` | TCS3200 R/Y/G detect + lock |
| `arm_ui/` | Flask dashboard + protobuf TCP client |
| `proto/meowler.proto` | Wire protocol |
| `docs/ESP32_S3_PINOUT.md` | Short pin cheat-sheet |
| `docs/COLOR.md` | Colour algorithm notes |

### Flash

```powershell
arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600" esp_ui
arduino-cli upload -p COM5 --fqbn "esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600" esp_ui
```

### Panel

```powershell
cd arm_ui
uv run dashboard
# http://127.0.0.1:5050 → Connect → <lan>.222:3333  (e.g. 192.168.29.222)
```

---

## 12. Part checklist

| # | Part | Interface | ESP / PCA pins |
|---|------|-----------|----------------|
| 1 | Freenove ESP32-S3 N16R8 | USB + WiFi | Brain |
| 2 | L298N | GPIO PWM + DIR | 8–13 |
| 3 | N20 motors ×2 | L298N OUT | M1 OUT3/4 · M2 OUT1/2 |
| 4 | Encoders ×2 | GPIO | 4/5 · 6/7 |
| 5 | PCA9685 | I2C `0x40` | SDA21 SCL47 |
| 6 | MG90 ×3 | PCA CH0–2 | Base / height / grip |
| 7 | SG90-360 | PCA CH4 | Conveyor |
| 8 | VL53L0X | I2C `0x29` | SDA21 SCL47 |
| 9 | BNO08x | I2C `0x4A`/`0x4B` | SDA21 SCL47 · VIN=GPIO38 |
| 10 | TCS3200 | GPIO | S2=15 S3=16 OUT=17 LED=18 · S0/S1 hardwired |

---

*Source of truth for pins: `esp_ui/esp_ui.ino`. Update this file if wiring changes.*
