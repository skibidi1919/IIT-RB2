# Meowler

Small differential-drive robot with a 3-DOF PCA9685 arm, VL53L0X ToF, optional BNO08x IMU, and L298N wheel drive.

## Active stack (USB dashboard)

| Piece | Path | Notes |
|-------|------|--------|
| Nano firmware | `arm_ui/` | Old bootloader FQBN, 115200 text protocol |
| Control panel | `arm_ui/app.py` | Flask @ http://127.0.0.1:5050 |
| Docs | `docs/` | [Pinout](docs/PINOUT.md), [Youth Challenge Rules](docs/YOUTH_CHALLENGE_RULES.md) |

```powershell
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old arm_ui
arduino-cli upload -p COM3 --fqbn arduino:avr:nano:cpu=atmega328old arm_ui
cd arm_ui
uv run --with flask --with pyserial python app.py
```

## Full robot link (ESP-NOW)

Laptop → `hub_bridge` → ESP-NOW → `color_bridge` → UART → `nano_drive`  
Host UI: `control_gui/` (port 5055). Schema: `proto/meowler.proto`.

## Repository layout

```
arm_ui/           Active Nano + control panel (USB)
esp_ui/           ESP32-S3 Arduino brain (protobuf TCP :3333)
esp_mpy/          MicroPython port of the ESP32-S3 brain (see esp_mpy/README.md)
nano_drive/       Drive Nano (protobuf / robot link)
color_bridge/     Robot ESP32 (color + BNO + ESP-NOW)
hub_bridge/       Laptop ESP32 hub
control_gui/      Full-robot Flask UI
robot_link/       Python framed LinkMessage client
robot_arm/        Legacy arm sketch
proto/            Shared protobuf
generated/        Generated nanopb bindings
docs/             Documentation
archive/diag/     One-off I2C / PCA / motor probes
archive/host/     Older Flask UIs (gui, motor_gui, color_gui)
scripts/ tools/   Flash helpers, proto codegen
third_party/      Vendored nanopb
```

## Critical flash note

Arduino Nano on this project **always** uses the **old** ATmega328 bootloader:

```text
FQBN = arduino:avr:nano:cpu=atmega328old
```

See `docs/SETUP.md` and `docs/PINOUT.md`.
