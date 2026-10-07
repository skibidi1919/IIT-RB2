# Freenove ESP32-S3-WROOM N16R8 — Meowler (`esp_ui`)

**Camera OFF · PSRAM ON · protobuf TCP :3333**

## FQBN

```text
esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600
```

## I2C — put ALL three on one bus

Boot scan only finds devices that are wired. Preferred:

| Device | SDA | SCL | Addr | Notes |
|--------|-----|-----|------|-------|
| **PCA9685** | **21** | **47** | `0x40` | confirmed (MODE1 read) |
| **VL53L0X** | **21** | **47** | `0x29` | same Wire as PCA |
| **BNO08x** | **21** | **47** | `0x4A`/`0x4B` | PS0/PS1 → GND · **VIN = GPIO38 HIGH** |

**Must:** common **GND**, PCA **OE → GND**, I2C pull-ups to **3.3 V**.

**BNO power:** 3V3-only modules → **VIN → GPIO38** (firmware drives HIGH ≈ 3.3 V). Do **not** feed board 5 V into BNO VIN.

**Do not use GPIO14 for I2C** — onboard camera FLASH LED.

## Drive / color

| Function | GPIO |
|----------|------|
| M2 ENA / IN1 / IN2 | **9 / 11 / 12** |
| M1 ENB / IN3 / IN4 | **10 / 13 / 8** |
| Enc M1 / M2 | **4/5 · 6/7** |
| TCS S0/S1 | **5V / GND** (20%) |
| TCS S2/S3/OUT/LED | **15 / 16 / 17 / 18** |

## PCA9685 channels (@ 50 Hz, pulse 500–2500 µs)

| CH | Function |
|----|----------|
| 0 | Arm base **MG90** — move→hold→relax→reassert→relax |
| 1 | Arm height **MG90** — same |
| 2 | Arm grip **MG90** — same (snap path) |
| **4** | **UM conveyor — SG90 360°** (PWM off = stop) |

## WiFi

STA joins the configured SSID via **DHCP first**, reads the network **gateway**, then binds a static address on that same subnet with host octet **always `.222`**.

Examples: gateway `192.168.29.1` → robot **`192.168.29.222`** · gateway `192.168.137.1` → **`192.168.137.222`**.

TCP **`:3333`** · mDNS `meowler.local`

## Panel

```powershell
cd arm_ui
uv run dashboard
# Connect → <yourLAN>.222:3333  (e.g. 192.168.29.222)
```
