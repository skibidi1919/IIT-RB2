# Meowler — agent notes

Canonical docs: **`README.md`**, **`docs/PINOUT.md`**, **`docs/PROTOCOL.md`**, **`docs/SETUP.md`**.

## Flash (critical)

```text
FQBN=arduino:avr:nano:cpu=atmega328old
```

Never use the new Nano bootloader FQBN on this board.

## Active USB path

- Firmware + panel: `arm_ui/` @ **115200**
- Panel: `cd arm_ui && uv run dashboard` → http://127.0.0.1:5050
- Nano fallback: `arm_ui/` · Drive M1 OUT3/4 D10/A2/A3 · M2 D9/A0/A1
- **ESP32-S3 target:** `esp_ui/` Freenove N16R8 **camera removed** — see `docs/ESP32_S3_PINOUT.md`
- Telemetry fields: see `docs/PROTOCOL.md`

## Robot link (protobuf + ESP-NOW)

```text
Laptop --USB--> hub_bridge --ESP-NOW--> color_bridge --UART 57600--> nano_drive
```

| Board | Typical COM | Role |
|-------|-------------|------|
| Drive Nano | COM3 | L298N + PCA + VL53 (+ encoders) |
| Robot ESP32 | COM4 | TCS3200 + BNO Wire 21/22 |
| Hub ESP32-S3 | COM5 | Laptop USB bridge |

SoftSerial Nano↔ESP (when used): Nano D11 RX / D12 TX @ 57600.  
Libs: VL53L0X, 7Semi BNO08x. Proto: `proto/meowler.proto` → `scripts/gen_proto.ps1`.

### `nano_drive` pins (robot-link build)

| Function | Pin |
|----------|-----|
| SoftSerial RX/TX | D11 / D12 |
| M1 enc C1/C2 | D3 / D4 |
| M2 enc C1/C2 | D5 / D6 |
| I2C | A4 / A5 |

### `color_bridge` pins

| Function | GPIO |
|----------|------|
| TCS3200 S0/S1/S2/S3/OUT/LED | 4 / 2 / 18 / 19 / 5 / 13 |
| I2C SDA/SCL (BNO) | 21 / 22 |
| UART TX2→Nano / RX2←Nano | 17 / 16 |

## Layout

Active firmware/host at repo root; probes → `archive/diag/`; old UIs → `archive/host/`.
