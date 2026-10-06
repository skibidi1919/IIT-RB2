#!/usr/bin/env bash
# Helper: list Windows USB devices and remind how to attach the Nano into WSL.
set -euo pipefail

echo "=== Windows USB devices (usbipd) ==="
if command -v usbipd.exe >/dev/null 2>&1; then
  usbipd.exe list
elif [[ -x "/mnt/c/Program Files/usbipd-win/usbipd.exe" ]]; then
  "/mnt/c/Program Files/usbipd-win/usbipd.exe" list
else
  cat <<'EOF'
usbipd-win is not installed yet.

Open PowerShell as Administrator on Windows and run:
  winget install --id dorssel.usbipd-win -e --accept-package-agreements --accept-source-agreements

Then plug in the Arduino Nano and run this script again:
  ./attach_nano.sh
EOF
  exit 1
fi

echo
echo "=== WSL serial devices ==="
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(none yet)"

cat <<'EOF'

To attach a device into WSL (Admin PowerShell once for bind):
  usbipd list
  usbipd bind --busid <BUSID>          # once
  usbipd attach --wsl --busid <BUSID>  # each session

Look for CH340 / FTDI / CP210x / Arduino in the Shared list.
Then flash from WSL:
  ./flash.sh
EOF
