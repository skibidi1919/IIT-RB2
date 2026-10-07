# Meowler — live status

Last updated: 2026-10-07 · brain = **Freenove ESP32-S3 N16R8** (`esp_ui`) · panel http://127.0.0.1:5050

## Active target: Freenove ESP32-S3 N16R8 (camera REMOVED)

| Subsystem | Status | Notes |
|-----------|--------|--------|
| Board | Port ready | `esp_ui/` · CH343 COM (often COM7) · **cam unplugged** |
| PCA9685 arm | Shared Wire | **SDA=21 SCL=47** @ `0x40` · CH0–2 MG90 · **CH4** conveyor |
| VL53L0X ToF | Shared Wire | **SDA=21 SCL=47** @ `0x29` |
| BNO08x IMU | Shared Wire | **SDA=21 SCL=47** @ `0x4A` · VIN=**GPIO38 HIGH** · PS0/PS1/ADR GND · RST/CS→3V3 |
| M1 / M2 | LEDC PWM | ENB10/IN3=13/IN4=8 · ENA9/IN1=11/IN2=12 |
| Encoders | Port ready | 4/5 · 6/7 · Ø43 mm |
| TCS3200 | Port ready | S0=5V S1=GND · S2=15 S3=16 OUT=17 LED=18 |
| Network | TCP :3333 | static **\*.222** on joined WiFi (SSID DarshIshaan) |

Full ESP pin table: **`docs/ESP32_S3_PINOUT.md`**

**Root cause of “M1 dead”:** M1 is physically on **OUT3/4** (ENB/IN3/IN4). Wire-swap proved the motor; firmware was talking to the other H-bridge half.

## Power / commons

- Share **GND**: Nano · PCA · L298N logic · VL53 · BNO · TCS
- L298N **+12V/VM** = battery; logic 5V shared (don’t double-feed if onboard regulator jumper is on)
- Pull **ENA** and **ENB** jumpers off the L298N
- Unplug **all PCA wires** from ENA/ENB/IN1–IN4

---

## Drive (working)

### M1 = OUT3 / OUT4 (cmd left)

| L298N | Nano |
|-------|------|
| ENB | **D10** (PWM) |
| IN3 | **A2** |
| IN4 | **A3** |
| OUT3 / OUT4 | M1 motor |

### M2 = OUT1 / OUT2 (cmd right)

| L298N | Nano |
|-------|------|
| ENA | **D9** (PWM) |
| IN1 | **A0** |
| IN2 | **A1** |
| OUT1 / OUT2 | M2 motor |

---

## Arm / ToF (working)

| Device | Nano / PCA |
|--------|------------|
| PCA SDA/SCL | A4 / A5 · addr `0x40` |
| PCA OE | GND |
| Servo base / height / grip | PCA CH0 / CH1 / CH2 |
| VL53 VIN/GND/SDA/SCL | 5V / GND / A4 / A5 · `0x29` |

---

## BNO08x / BNO085 (“BM008x”) — wire this

Soft-I2C (does **not** share A4/A5 with PCA/VL53). Firmware: `arm_ui/soft_bno_i2c.h`.

| BNO08x | Nano | Notes |
|--------|------|--------|
| VIN / VCC | **5V** (Adafruit 5V) or **3V3** if that board only | match breakout |
| GND | **GND** | required |
| SDA | **D7** | soft I2C |
| SCL | **D8** | soft I2C |
| ADR / DI | GND → **0x4A** · HIGH → **0x4B** | firmware tries both |
| PS0 / PS1 | GND (I2C mode) | board default often OK |
| INT | nc | unused |
| RST | nc | unused |
| 3Vo | nc | do not feed Nano |

Panel flag: `imu=1` when seen. Yaw/pitch/roll in telem (centidegrees).

---

## TCS3200 — attach next (from efficient Nano code)

Proven compact reader: `nano_drive/nano_drive.ino` (S0/S1 = 20% scale, pulseIn + EMA + sticky R/Y/G).

### Pin conflict

`nano_drive` used **D7/D8 for TCS S0/S1**.  
`arm_ui` now uses **D7/D8 for BNO**. Do **not** put both there.

### Recommended wiring on `arm_ui` (keeps BNO on D7/D8)

Hard-wire scale pins to the same 20% mode the efficient code uses (`S0=HIGH`, `S1=LOW`):

| TCS3200 | Connect |
|---------|---------|
| VCC | **5V** |
| GND | **GND** |
| **S0** | **5V** (hardwire) |
| **S1** | **GND** (hardwire) |
| **S2** | **D11** |
| **S3** | **D12** |
| **OUT** | **D2** |
| **LED** | **D13** |

Firmware still needs this TCS block ported into `arm_ui` (not wired into loop yet). Until then, sensor can sit wired; color telem won’t show.

### Alternate (full GPIO like `nano_drive`)

If you move BNO onto **hardware Wire A4/A5** (shared with PCA/VL53):

| TCS3200 | Nano |
|---------|------|
| S0 / S1 / S2 / S3 | D7 / D8 / D11 / D12 |
| OUT / LED | D2 / D13 |

That’s the old `nano_drive` map. Prefer only if soft-I2C BNO is dropped.

### ESP32 color path (full robot link)

On `color_bridge` (not this USB panel): S0=4 S1=2 S2=18 S3=19 OUT=5 LED=13 · BNO on Wire 21/22.

---

## Encoders (optional)

| Enc | Nano |
|-----|------|
| M1 C1 / C2 | D3 / D4 |
| M2 C1 / C2 | D5 / D6 |

Ø **43 mm** · default **600** steps/rev · `Z` zeros encoders + ToF displacement.

---

## Free / reserved Nano map (USB build)

| Pin | Use |
|-----|-----|
| D0/D1 | USB serial |
| D2 | TCS OUT (planned) |
| D3/D4 | Enc M1 |
| D5/D6 | Enc M2 |
| D7/D8 | **BNO soft SDA/SCL** |
| D9/D10 | M2 ENA / M1 ENB |
| D11/D12 | TCS S2/S3 (planned) |
| D13 | TCS LED (planned) |
| A0/A1 | M2 IN1/IN2 |
| A2/A3 | M1 IN3/IN4 |
| A4/A5 | Wire PCA + VL53 |

---

## Host / flash

```powershell
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old arm_ui
arduino-cli upload -p COM3 --fqbn arduino:avr:nano:cpu=atmega328old arm_ui
cd arm_ui
uv run --with flask --with pyserial python app.py
# http://127.0.0.1:5050
```

Docs: `docs/PINOUT.md` · `docs/PROTOCOL.md` · `docs/SETUP.md` · `README.md`

## Next

1. Wire **BNO08x** D7/D8 · confirm `imu=1`
2. Wire **TCS3200** with S0/S1 hardwired · port `nano_drive` color block into `arm_ui` + panel
3. Calibrate `STEPS_PER_REV` after a measured roll
4. Optional later: **Arduino UNO Q** for Wi-Fi remote (not a drop-in Nano; see below)

## Arduino UNO Q (research — not ported yet)

Dual board: **Qualcomm QRB2210** (Debian Linux, Wi-Fi 5, Bluetooth 5.1, 2 GB RAM, 16 GB eMMC) + **STM32U585** MCU (Arduino sketches on Zephyr). Remote: App Lab **network mode**, SSH, host Flask on Linux, MCU↔Linux **Bridge RPC**. USB-C needs **PD ~15 W**. Docs: https://docs.arduino.cc/hardware/uno-q

**Can we port `arm_ui`?** Yes, as a **rewrite + rewire**, not plug-and-play.

| Topic | Nano now | UNO Q |
|-------|----------|--------|
| Sketch CPU | ATmega328 5 V | STM32  **3.3 V GPIO** |
| I2C `Wire` | **A4/A5** | **D20 SDA / D21 SCL** (A4/A5 is `Wire` I2C2 / D18/D19) |
| PWM ENA/ENB | D9 / D10 | D9 / D10 still PWM (also D3,5,6,11; PWM **500 Hz** fixed) |
| Form | Nano headers | Classic **UNO** headers |
| Remote | Laptop COM3 | Linux Wi-Fi + Flask **on the Q** |

**Must-fix before swapping the robot over**

- Level-shift or run I2C at **3.3 V** (PCA pull-ups to 3.3 V, not 5 V). Don’t put 5 V on Q analog/I2C.
- L298N IN/EN: 3.3 V HIGH is usually OK; keep jumper 5 V logic isolated from MCU pins.
- TCS3200: run at **3.3 V** or divide **OUT** (5 V OUT will kill a pin).
- BNO: Qwiic `Wire1` is the clean 3.3 V I2C; or share header I2C at 3.3 V.
- Libraries: VL53 + 7Semi BNO need a Q/Zephyr test; drop AVR-only tricks.
- Don’t copy Nano FQBN. MCU sketch via IDE/CLI; panel Python on Linux.

**Sane remote layout:** STM32 = motors/I2C/encoders · Linux = Wi-Fi Flask control panel · Bridge RPC instead of USB serial. Keep the Nano as the local bench brain until that exists.
