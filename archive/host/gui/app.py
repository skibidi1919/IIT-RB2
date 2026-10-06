#!/usr/bin/env python3
"""Meowler robot-arm control GUI — Flask + nanopb serial bridge."""

from __future__ import annotations

import glob
import threading
import time

import serial
from flask import Flask, jsonify, render_template, request
from serial.tools import list_ports

import proto_link as link

app = Flask(__name__)

_lock = threading.Lock()
_ser: serial.Serial | None = None
_rx = bytearray()
_state = {
    "connected": False,
    "port": None,
    "base": 90,
    "height": 90,
    "grip": 90,
    "pca_ok": False,
    "moving": False,
    "distance_mm": 0,
    "tof_ok": False,
    "imu_ok": False,
    "yaw_deg": 0.0,
    "pitch_deg": 0.0,
    "roll_deg": 0.0,
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
        if not _is_useful_port(p.device, p.hwid or "", p.description or ""):
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


def _apply_status(st) -> None:
    _state["base"] = int(st.base)
    _state["height"] = int(st.height)
    _state["grip"] = int(st.grip)
    _state["pca_ok"] = bool(st.pca_ok)
    _state["moving"] = bool(st.moving)
    _state["distance_mm"] = int(getattr(st, "distance_mm", 0) or 0)
    _state["tof_ok"] = bool(getattr(st, "tof_ok", False))
    _state["imu_ok"] = bool(getattr(st, "imu_ok", False))
    _state["yaw_deg"] = float(getattr(st, "yaw_cdeg", 0) or 0) / 100.0
    _state["pitch_deg"] = float(getattr(st, "pitch_cdeg", 0) or 0) / 100.0
    _state["roll_deg"] = float(getattr(st, "roll_cdeg", 0) or 0) / 100.0
    dist = (
        f" dist={_state['distance_mm']}mm"
        if _state["tof_ok"]
        else " dist=—"
    )
    imu = (
        f" ypr={_state['yaw_deg']:.0f}/{_state['pitch_deg']:.0f}/{_state['roll_deg']:.0f}"
        if _state["imu_ok"]
        else " imu=—"
    )
    _state["last_line"] = (
        f"pca={'ok' if st.pca_ok else 'FAIL'} "
        f"tof={'ok' if _state['tof_ok'] else 'FAIL'} "
        f"imu={'ok' if _state['imu_ok'] else 'FAIL'} "
        f"base={st.base} height={st.height} grip={st.grip}"
        f"{dist}{imu}"
        f"{' moving' if st.moving else ''}"
    )


def _read_loop() -> None:
    global _ser
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
            time.sleep(0.1)
            continue
        if not chunk:
            time.sleep(0.01)
            continue
        with _lock:
            _rx.extend(chunk)
            while True:
                st, rest = link.try_parse_status(_rx)
                _rx[:] = rest
                if st is None:
                    break
                _apply_status(st)


threading.Thread(target=_read_loop, daemon=True).start()


def send_pb(**kwargs) -> None:
    frame = link.encode_command(**kwargs)
    with _lock:
        if _ser is None or not _ser.is_open:
            raise RuntimeError("Not connected")
        _ser.write(frame)
        _ser.flush()


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
        if _ser is not None and _ser.is_open:
            try:
                _ser.close()
            except Exception:  # noqa: BLE001
                pass
            _ser = None
        _rx.clear()

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)  # Nano resets on DTR
        ser.reset_input_buffer()
        with _lock:
            _ser = ser
            _state["connected"] = True
            _state["port"] = port
            _state["error"] = None
        send_pb(action=link.ACT_STATUS)
        time.sleep(0.05)
        return jsonify({"ok": True, "port": port})
    except Exception as exc:  # noqa: BLE001
        with _lock:
            _state["connected"] = False
            _state["port"] = None
            _state["error"] = str(exc)
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/disconnect")
def api_disconnect():
    global _ser
    with _lock:
        if _ser is not None:
            try:
                _ser.close()
            except Exception:  # noqa: BLE001
                pass
            _ser = None
        _state["connected"] = False
        _state["port"] = None
        _rx.clear()
    return jsonify({"ok": True})


@app.post("/api/set")
def api_set():
    data = request.get_json(force=True, silent=True) or {}
    axis = (data.get("axis") or "").lower()
    try:
        value = int(data.get("value"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "value must be int"}), 400
    value = max(0, min(180, value))
    speed = int(data.get("speed_dps") or link.DEFAULT_SPEED_DPS)

    kwargs: dict = {"speed_dps": speed}
    if axis in ("base",):
        kwargs["base"] = value
    elif axis in ("height",):
        kwargs["height"] = value
    elif axis in ("grip", "gripper"):
        kwargs["grip"] = value
    else:
        return jsonify({"ok": False, "error": "unknown axis"}), 400

    try:
        send_pb(**kwargs)
        return jsonify({"ok": True, "axis": axis, "value": value})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/all")
def api_all():
    data = request.get_json(force=True, silent=True) or {}
    try:
        b = max(0, min(180, int(data.get("base", 90))))
        h = max(0, min(180, int(data.get("height", 90))))
        g = max(0, min(180, int(data.get("grip", 90))))
        speed = int(data.get("speed_dps") or link.DEFAULT_SPEED_DPS)
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "invalid angles"}), 400
    try:
        send_pb(base=b, height=h, grip=g, speed_dps=speed)
        return jsonify({"ok": True, "base": b, "height": h, "grip": g})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/action")
def api_action():
    data = request.get_json(force=True, silent=True) or {}
    action = (data.get("action") or "").lower()
    speed = int(data.get("speed_dps") or link.DEFAULT_SPEED_DPS)
    mapping = {
        "c": link.ACT_CENTER,
        "center": link.ACT_CENTER,
        "s": link.ACT_STATUS,
        "status": link.ACT_STATUS,
        "demo": link.ACT_DEMO,
    }
    if action not in mapping:
        return jsonify({"ok": False, "error": "unknown action"}), 400
    try:
        send_pb(action=mapping[action], speed_dps=speed)
        return jsonify({"ok": True, "action": action})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


def main() -> None:
    print("Meowler GUI -> http://127.0.0.1:5050")
    app.run(host="127.0.0.1", port=5050, debug=False, threaded=True)


if __name__ == "__main__":
    main()
