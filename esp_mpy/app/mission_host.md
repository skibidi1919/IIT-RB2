# Mission scripts: host vs on-device

## Default (recommended): host-side

Mission / recording / replay stay on the PC:

| Piece | Role |
|-------|------|
| `arm_ui/` Flask dashboard | Connect to `*.222:3333`, send Drive/Arm/Conveyor |
| `arm_ui` recording | Host packs RecEvent timeline (or uses binary `.rpm`) |
| Mission Python (`arm_ui/mission.py`) | Runs on PC; talks protobuf TCP to the robot |

The MicroPython firmware accepts `RecCtrl` (start/stop) and streams `RecEvent` for on-robot capture, but **host recording via `arm_ui` remains the primary path** — same as Arduino `esp_ui`.

## Optional: on-device `.ms` runner

Not shipped by default. If you want missions on the ESP:

1. Deploy a small `mission_run.py` that reads `/missions/*.ms` (JSON or line-oriented opcodes).
2. Map opcodes → the same handlers as `main.py` (`drive`, `arm`, `conveyor`, `stop`, `center`).
3. Trigger via TCP (`RecCtrl` / custom JSON) or a GPIO button.

Keep mission files small; prefer host execution when WiFi is available so you can edit without redeploying.

## Compatibility note

`arm_ui` OTA (dual `ota_0`/`ota_1` `.bin` slots) targets **Arduino `esp_ui` only**. Updating MicroPython apps uses `mpremote` / `ota_http.py` (see README).
