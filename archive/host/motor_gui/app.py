#!/usr/bin/env python3
"""N20 motor bench GUI — Flask + serial bridge."""

from __future__ import annotations

import glob
import threading
import time

import serial
from flask import Flask, jsonify, render_template, request
from serial.tools import list_ports

app = Flask(__name__)

_lock = threading.Lock()
_ser: serial.Serial | None = None
_state = {
    "connected": False,
    "port": None,
    "motors": [
        {"id": 0, "name": "M1 front A", "dir": "stop", "speed": 180},
        {"id": 1, "name": "M2 front A", "dir": "stop", "speed": 180},
        {"id": 2, "name": "M3 back B", "dir": "stop", "speed": 180},
        {"id": 3, "name": "M4 back B", "dir": "stop", "speed": 180},
    ],
    "last_line": "",
    "error": None,
}


def _is_useful_port(device: str, hwid: str = "", description: str = "") -> bool:
    name = device.rsplit("/", 1)[-1]
    if name.startswith(("ttyUSB", "ttyACM", "cu.", "COM")):
        return True
    blob = f"{hwid} {description}".upper()
    return any(tag in blob for tag in ("USB", "FTDI", "CH340", "CP210", "ARDUINO", "UART"))


def candidate_ports() -> list[dict]:
    ports = []
    seen = set()
    for p in list_ports.comports():
        if not _is_useful_port(p.device, p.hwid or "", p.description or "") :
            continue
        seen.add(p.device)
        ports.append(
            {
                "device": p.device,
                "description": p.description or "",
                "hwid": p.hwid or "",
            }
        )
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for device in sorted(glob.glob(pattern)):
            if device not in seen:
                ports.append(
                    {"device": device, "description": "serial device", "hwid": ""}
                )
    ports.sort(
        key=lambda p: (
            0 if p["device"].upper().startswith("COM") else 1,
            p["device"],
        )
    )
    return ports


def _parse_status(line: str) -> None:
    # STATUS m0=fwd@180 m1=stop@0 ...
    if not line.startswith("STATUS"):
        return
    parts = line.split()
    for part in parts[1:]:
        if not part.startswith("m") or "=" not in part:
            continue
        try:
            idx_s, rest = part[1:].split("=", 1)
            idx = int(idx_s)
            dir_s, spd_s = rest.split("@", 1)
            if 0 <= idx < 4:
                _state["motors"][idx]["dir"] = dir_s
                _state["motors"][idx]["speed"] = int(spd_s)
        except ValueError:
            continue


def _read_loop() -> None:
    global _ser
    buf = ""
    while True:
        with _lock:
            ser = _ser
        if ser is None or not ser.is_open:
            time.sleep(0.05)
            continue
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception as exc:  # noqa: BLE001
            with _lock:
                _state["error"] = str(exc)
                _state["connected"] = False
                if _ser is not None:
                    try:
                        _ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                _ser = None
            time.sleep(0.2)
            continue
        if not chunk:
            time.sleep(0.01)
            continue
        buf += chunk.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            with _lock:
                _state["last_line"] = line
                _parse_status(line)


def _send(cmd: str) -> None:
    with _lock:
        if _ser is None or not _ser.is_open:
            raise RuntimeError("not connected")
        _ser.write((cmd.strip() + "\n").encode("ascii"))
        _ser.flush()


threading.Thread(target=_read_loop, daemon=True).start()


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/status")
def api_status():
    with _lock:
        return jsonify(dict(_state))


@app.get("/api/ports")
def api_ports():
    return jsonify({"ports": candidate_ports()})


@app.post("/api/connect")
def api_connect():
    global _ser
    data = request.get_json(force=True, silent=True) or {}
    port = data.get("port") or "COM3"
    baud = int(data.get("baud") or 115200)
    with _lock:
        if _ser is not None:
            try:
                _ser.close()
            except Exception:  # noqa: BLE001
                pass
            _ser = None
        try:
            _ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2.0)
            _ser.reset_input_buffer()
            _state["connected"] = True
            _state["port"] = port
            _state["error"] = None
        except Exception as exc:  # noqa: BLE001
            _state["connected"] = False
            _state["port"] = None
            _state["error"] = str(exc)
            return jsonify({"ok": False, "error": str(exc)}), 400
    try:
        _send("STATUS")
    except Exception:  # noqa: BLE001
        pass
    return jsonify({"ok": True, "port": port})


@app.post("/api/disconnect")
def api_disconnect():
    global _ser
    with _lock:
        if _ser is not None:
            try:
                _ser.write(b"ALL stop\n")
            except Exception:  # noqa: BLE001
                pass
            try:
                _ser.close()
            except Exception:  # noqa: BLE001
                pass
            _ser = None
        _state["connected"] = False
        _state["port"] = None
    return jsonify({"ok": True})


@app.post("/api/motor")
def api_motor():
    data = request.get_json(force=True, silent=True) or {}
    try:
        mid = int(data["id"])
        direction = str(data.get("dir") or "stop").lower()
        speed = int(data.get("speed", 180))
    except (KeyError, TypeError, ValueError):
        return jsonify({"ok": False, "error": "bad args"}), 400
    if mid < 0 or mid > 3:
        return jsonify({"ok": False, "error": "id 0-3"}), 400
    if direction not in ("fwd", "rev", "stop", "forward", "reverse", "back"):
        return jsonify({"ok": False, "error": "bad dir"}), 400
    if direction in ("forward",):
        direction = "fwd"
    if direction in ("reverse", "back"):
        direction = "rev"
    speed = max(0, min(255, speed))
    try:
        _send(f"M {mid} {direction} {speed}")
        with _lock:
            _state["motors"][mid]["dir"] = direction
            _state["motors"][mid]["speed"] = 0 if direction == "stop" else speed
        return jsonify({"ok": True})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/all")
def api_all():
    data = request.get_json(force=True, silent=True) or {}
    direction = str(data.get("dir") or "stop").lower()
    speed = int(data.get("speed", 180))
    if direction in ("forward",):
        direction = "fwd"
    if direction in ("reverse", "back"):
        direction = "rev"
    if direction not in ("fwd", "rev", "stop"):
        return jsonify({"ok": False, "error": "bad dir"}), 400
    speed = max(0, min(255, speed))
    try:
        _send(f"ALL {direction} {speed}")
        with _lock:
            for m in _state["motors"]:
                m["dir"] = direction
                m["speed"] = 0 if direction == "stop" else speed
        return jsonify({"ok": True})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


if __name__ == "__main__":
    print("N20 motor GUI -> http://127.0.0.1:5051")
    app.run(host="127.0.0.1", port=5051, debug=False, threaded=True)
