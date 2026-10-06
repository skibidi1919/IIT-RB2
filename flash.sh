#!/usr/bin/env bash
set -euo pipefail
export PATH="$HOME/bin:$PATH"

# This Nano always uses the old ATmega328 bootloader (see AGENT.md).
FQBN="${FQBN:-arduino:avr:nano:cpu=atmega328old}"
SKETCH_DIR="$(cd "$(dirname "$0")/robot_arm" && pwd)"

echo "Looking for Arduino serial port..."
arduino-cli board list

PORT="${1:-}"
if [[ -z "$PORT" ]]; then
  # Prefer USB/ACM serial devices discovered by arduino-cli
  PORT="$(arduino-cli board list --format json 2>/dev/null \
    | python3 -c '
import json,sys
try:
  data=json.load(sys.stdin)
except Exception:
  sys.exit(0)
for d in data.get("detected_ports", data if isinstance(data,list) else []):
  port=(d.get("port") or {})
  addr=port.get("address") or d.get("address") or ""
  if addr.startswith("/dev/ttyUSB") or addr.startswith("/dev/ttyACM"):
    print(addr); break
' || true)"
fi

if [[ -z "${PORT}" ]]; then
  # Fallback: first present device node
  for candidate in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
    if [[ -e "$candidate" ]]; then
      PORT="$candidate"
      break
    fi
  done
fi

if [[ -z "${PORT}" ]]; then
  cat <<'EOF'
No Arduino port found in WSL.

1) Plug in the Arduino Nano (USB cable that carries data).
2) In an elevated Windows PowerShell:
     winget install --id dorssel.usbipd-win -e
     usbipd list
     usbipd bind --busid <BUSID>
     usbipd attach --wsl --busid <BUSID>
3) Back in WSL, confirm:
     ls /dev/ttyUSB* /dev/ttyACM*
     arduino-cli board list
4) Flash again:
     ./flash.sh
     # or old Nano bootloader:
     FQBN=arduino:avr:nano:cpu=atmega328old ./flash.sh
EOF
  exit 1
fi

echo "Compiling ($FQBN)..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"

echo "Uploading to $PORT ..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"

echo "Done. Open serial monitor:"
echo "  arduino-cli monitor -p $PORT -c baudrate=115200"
