# Meowler USB serial debug (2026-10-07)

## Verdict

**Not a missing driver.** The board’s USB data path was intermittent / not on the bus (`IsPresent=False`). When the Freenove **CH343 UART** enumerated as **COM7**, USB flash worked.

Drivers already correct when the device is plugged:

| Device | VID/PID | Driver service | Provider |
|--------|---------|----------------|----------|
| Freenove onboard hub | `1A86:8095` | `USBHUB3` | Microsoft |
| ESP32-S3 native USB | `303A:1001` | `usbccgp` + `usbser` (COM) | Microsoft |
| CH343 UART (optional) | `1A86:55D3` | (WCH) | — |

**Root cause: the USB device tree is not present on the bus** (`DEVPKEY_Device_IsPresent = False` for the whole chain). There is no `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM` entry — no COM port exists to open.

The board **is** alive on Wi‑Fi (`192.168.137.222:3333`), so power is fine; **USB data to the PC is not**.

## Evidence

- Live USB right now: Bluetooth, camera, keyboard/mouse only — **no** `VID_303A`, **no** `VID_1A86` hub.
- Ghost ESP composite parent: `USB\VID_1A86&PID_8095\...` (WCH hub on Freenove), location `Port_#0003.Hub_#0002`.
- Last ESP USB arrival: **2026-10-06 10:55:56**.
- Last WCH hub arrival: **2026-10-07 02:20:46**, then disappeared.
- Historical nodes: `Unknown USB Device (Device Descriptor Request Failed)` on root-hub ports — classic bad/partial USB enum (charge-only cable, brownout, or flaky port).

## What to do

1. Use a **data** USB cable (not charge-only).
2. Plug into the Freenove **USB** connector that feeds the onboard hub (same path that creates `1A86:8095` → `303A:1001`).
3. Prefer a **USB 2.0** port directly on the PC (avoid unpowered hubs).
4. Unplug external power briefly, plug USB first, then power — or hold **BOOT** while plugging if download mode is needed.
5. Until USB enumerates: flash with **Wi‑Fi OTA** (`ota-flash.bat 192.168.137.222`).

## Helper

```text
powershell -ExecutionPolicy Bypass -File tools\fix-usb-serial.ps1
```
