#!/usr/bin/env python3
"""Meowler control GUI — laptop ↔ Nano USB (500000 nanopb) or hub ESP32."""

from __future__ import annotations

import glob
import sys
import threading
import time
from collections import deque
from pathlib import Path

import serial
from flask import Flask, jsonify, render_template, request
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_link"))

from proto_link import (  # noqa: E402
    ACT_ARM_CENTER,
    ACT_ARM_DEMO,
    ACT_CAL_COLOR,
    ACT_PING,
    ACT_STOP,
    encode_action,
    encode_arm,
    encode_drive,
    flags_dict,
    try_parse_link,
)

app = Flask(__name__)

COLOR_NAMES = {0: "NONE", 1: "RED", 2: "YELLOW", 3: "GREEN"}

_lock = threading.RLock()
_ser: serial.Serial | None = None
_stop = threading.Event()
_reader: threading.Thread | None = None
_rx_buf = bytearray()
_logs: deque[dict] = deque(maxlen=80)

_state: dict = {
    "connected": False,
    "port": None,
    "error": None,
    "seq": 0,
    "t_ms": 0,
    "color": "NONE",
    "conf": 0,
    "r_us": 0,
    "g_us": 0,
    "b_us": 0,
    "c_us": 0,
    "left": 0,
    "right": 0,
    "enc_l": 0,
    "enc_r": 0,
    "base": 90,
    "height": 90,
    "grip": 90,
    "distance_mm": 0,
    "yaw_cdeg": 0,
    "pitch_cdeg": 0,
    "roll_cdeg": 0,
    "flags": {
        "nano_ok": False,
        "pca_ok": False,
        "tof_ok": False,
        "imu_ok": False,
        "moving": False,
        "color_cal": False,
    },
    "last_rx_ms": 0,
}


def _is_useful_port(device: str, hwid: str = "", description: str = "") -> bool:
    name = device.rsplit("/", 1)[-1]
    if name.startswith(("ttyUSB", "ttyACM", "cu.", "COM")):
        return True
    blob = f"{hwid} {description}".upper()
    return any(tag in blob for tag in ("USB", "FTDI", "CH340", "CH343", "CP210", "UART", "ESP"))


def candidate_ports() -> list[dict]:
    ports: list[dict] = []
    seen: set[str] = set()
    for p in list_ports.comports():
        if not _is_useful_port(p.device, p.hwid or "", p.description or ""):
            continue
        seen.add(p.device)
        ports.append(
            {"device": p.device, "description": p.description or "", "hwid": p.hwid or ""}
        )
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for device in sorted(glob.glob(pattern)):
            if device not in seen:
                ports.append({"device": device, "description": "serial device", "hwid": ""})
    ports.sort(key=lambda p: (0 if p["device"].upper().startswith("COM") else 1, p["device"]))
    return ports


def _write(frame: bytes) -> bool:
    with _lock:
        ser = _ser
        if ser is None or not ser.is_open:
            _state["error"] = "not connected"
            return False
        try:
            ser.write(frame)
            ser.flush()
            _state["error"] = None
            return True
        except Exception as exc:  # noqa: BLE001
            _state["error"] = str(exc)
            return False


def _apply_telem(t) -> None:
    fl = flags_dict(int(t.flags))
    _state.update(
        {
            "seq": int(t.seq),
            "t_ms": int(t.t_ms),
            "color": COLOR_NAMES.get(int(t.color), str(t.color)),
            "conf": int(t.conf),
            "r_us": int(t.r_us),
            "g_us": int(t.g_us),
            "b_us": int(t.b_us),
            "c_us": int(t.c_us),
            "left": int(t.left),
            "right": int(t.right),
            "enc_l": int(t.enc_l),
            "enc_r": int(t.enc_r),
            "base": int(t.base),
            "height": int(t.height),
            "grip": int(t.grip),
            "distance_mm": int(t.distance_mm),
            "yaw_cdeg": int(t.yaw_cdeg),
            "pitch_cdeg": int(t.pitch_cdeg),
            "roll_cdeg": int(t.roll_cdeg),
            "flags": fl,
            "last_rx_ms": int(time.time() * 1000),
        }
    )


def _reader_loop() -> None:
    global _ser
    while not _stop.is_set():
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
            time.sleep(0.005)
            continue
        with _lock:
            _rx_buf.extend(chunk)
            while True:
                msg, _ = try_parse_link(_rx_buf)
                if msg is None:
                    break
                kind = msg.WhichOneof("payload")
                if kind == "telem":
                    _apply_telem(msg.telem)
                elif kind == "log":
                    text = bytes(msg.log.text).decode("utf-8", "replace")
                    _logs.appendleft(
                        {
                            "t_ms": int(msg.log.t_ms),
                            "level": int(msg.log.level),
                            "text": text,
                        }
                    )


DEFAULT_BAUD = 500000  # Nano USB high-speed LinkMessage


def connect(port: str, baud: int = DEFAULT_BAUD) -> None:
    global _ser, _reader
    disconnect()
    ser = serial.Serial(port, baud, timeout=0.02)
    time.sleep(0.25)
    ser.reset_input_buffer()
    with _lock:
        _ser = ser
        _state["connected"] = True
        _state["port"] = port
        _state["error"] = None
        _rx_buf.clear()
    _stop.clear()
    _reader = threading.Thread(target=_reader_loop, name="hub-reader", daemon=True)
    _reader.start()
    _write(encode_action(ACT_PING))


def disconnect() -> None:
    global _ser, _reader
    _stop.set()
    if _reader and _reader.is_alive():
        _reader.join(timeout=0.8)
    _reader = None
    with _lock:
        if _ser is not None:
            try:
                _ser.close()
            except Exception:  # noqa: BLE001
                pass
        _ser = None
        _state["connected"] = False
        _state["port"] = None


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/ports")
def api_ports():
    return jsonify({"ports": candidate_ports()})


@app.get("/api/state")
def api_state():
    with _lock:
        stale = False
        if _state["connected"] and _state["last_rx_ms"]:
            stale = (time.time() * 1000 - _state["last_rx_ms"]) > 1500
        payload = dict(_state)
        payload["flags"] = dict(_state["flags"])
        payload["stale"] = stale
        payload["logs"] = list(_logs)[:24]
    return jsonify(payload)


@app.post("/api/connect")
def api_connect():
    data = request.get_json(force=True, silent=True) or {}
    port = (data.get("port") or "").strip()
    if not port:
        return jsonify({"ok": False, "error": "port required"}), 400
    try:
        connect(port, int(data.get("baud") or DEFAULT_BAUD))
        return jsonify({"ok": True, "port": port, "baud": int(data.get("baud") or DEFAULT_BAUD)})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.post("/api/disconnect")
def api_disconnect():
    disconnect()
    return jsonify({"ok": True})


@app.post("/api/drive")
def api_drive():
    data = request.get_json(force=True, silent=True) or {}
    left = int(data.get("left", 0))
    right = int(data.get("right", 0))
    ok = _write(encode_drive(left, right))
    return jsonify({"ok": ok, "error": _state.get("error")})


@app.post("/api/arm")
def api_arm():
    data = request.get_json(force=True, silent=True) or {}
    ok = _write(
        encode_arm(
            base=data.get("base"),
            height=data.get("height"),
            grip=data.get("grip"),
            speed_dps=int(data.get("speed_dps") or 500),
            action=int(data.get("action") or 0),
        )
    )
    return jsonify({"ok": ok, "error": _state.get("error")})


@app.post("/api/action")
def api_action():
    data = request.get_json(force=True, silent=True) or {}
    name = (data.get("name") or "").lower()
    mapping = {
        "stop": ACT_STOP,
        "ping": ACT_PING,
        "cal": ACT_CAL_COLOR,
        "cal_color": ACT_CAL_COLOR,
        "center": ACT_ARM_CENTER,
        "arm_center": ACT_ARM_CENTER,
        "demo": ACT_ARM_DEMO,
        "arm_demo": ACT_ARM_DEMO,
    }
    if name not in mapping:
        return jsonify({"ok": False, "error": "unknown action"}), 400
    ok = _write(encode_action(mapping[name]))
    return jsonify({"ok": ok, "error": _state.get("error")})


if __name__ == "__main__":
    # Default hub port on this bench is often COM5
    app.run(host="127.0.0.1", port=5055, debug=False, threaded=True)
