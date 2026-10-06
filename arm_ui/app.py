#!/usr/bin/env python3
"""Meowler control panel — ESP32 WiFi TCP + length-prefixed protobuf."""

from __future__ import annotations

import re
import socket
import struct
import threading
import time
from collections import deque
from pathlib import Path

from flask import Flask, jsonify, render_template, request

from meowler_pb import meowler_pb2 as pb
from mission import (
    MissionError,
    MissionRunner,
    list_missions,
    load_mission,
    save_mission,
)

app = Flask(__name__)

_lock = threading.RLock()
_sock: socket.socket | None = None
_stop = threading.Event()
_reader: threading.Thread | None = None
_logs: deque[str] = deque(maxlen=50)
_state = {
    "connected": False,
    "mode": None,
    "host": None,
    "error": None,
    "base": 90,
    "height": 90,
    "grip": 90,
    "left": 0,
    "right": 0,
    "distance_mm": 0,
    "pca_ok": False,
    "tof_ok": False,
    "imu_ok": False,
    "wifi_ok": False,
    "enc_l": 0,
    "enc_r": 0,
    "wheel_l_mm": 0,
    "wheel_r_mm": 0,
    "tof_disp_mm": 0,
    "yaw_cdeg": 0,
    "pitch_cdeg": 0,
    "roll_cdeg": 0,
    "color": 0,  # 0 UNKNOWN 1 RED 2 YELLOW 3 GREEN
    "color_name": "—",
    "color_conf": 0,
    "color_r": 0,
    "color_g": 0,
    "color_b": 0,
    "color_rp": 0,
    "color_gp": 0,
    "color_bp": 0,
    "color_cp": 0,
    "conveyor": 0,
    "last_rx": "",
    "ack": None,
    "recording": False,
    "replaying": False,
    "record_name": None,
    "record_events": 0,
}

DEFAULT_HOST = "192.168.137.222"
DEFAULT_PORT = 3333
MAX_FRAME = 4096
RPM_DIR = Path(__file__).resolve().parent / "recordings"
_SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# Binary .rpm v2 — little-endian packed timeline
# Header 16B: magic"MRPM" ver u8 flags u8 name_len u16 created u32 count u32
# then name UTF-8; then count events: t_ms u32, op u8, payload
RPM_MAGIC = b"MRPM"
RPM_VERSION = 2
RPM_HDR = struct.Struct("<4sBBHII")  # 16 bytes
OP_DRIVE = 1
OP_ARM = 2
OP_STOP = 3
OP_CENTER = 4
OP_ZERO = 5
OP_CONV = 6
OP_MTEST = 7
_OP_NAME = {
    OP_DRIVE: "drive",
    OP_ARM: "arm",
    OP_STOP: "stop",
    OP_CENTER: "center",
    OP_ZERO: "zero",
    OP_CONV: "conveyor",
    OP_MTEST: "motor_test",
}
_OP_CODE = {v: k for k, v in _OP_NAME.items()}

_rec_lock = threading.Lock()
_recording = False
_rec_t0 = 0.0
_rec_events: list[dict] = []
_rec_name: str | None = None
_replaying = False
_replay_stop = threading.Event()
_replay_thread: threading.Thread | None = None


def _normalize_host_port(host: str, port: int) -> tuple[str, int]:
    """Parse host box: strip scheme/path, honor host:port, keep exact IP/hostname."""
    h = (host or "").strip()
    if not h:
        return DEFAULT_HOST, port
    # strip URL scheme / path
    if "://" in h:
        h = h.split("://", 1)[1]
    h = h.split("/", 1)[0].strip()
    # [ipv6]:port — skip; we use IPv4 / names
    if h.startswith("["):
        return h, port
    if h.count(":") == 1:
        left, right = h.rsplit(":", 1)
        if right.isdigit():
            h = left.strip()
            port = int(right)
    h = h.strip().rstrip(".")
    if not h:
        h = DEFAULT_HOST
    if port < 1 or port > 65535:
        port = DEFAULT_PORT
    return h, port


def _send(msg: pb.ClientToRobot, record: bool = True) -> bool:
    del record  # capture is streamed by the ESP32 as RecEvent
    data = msg.SerializeToString()
    frame = struct.pack("<I", len(data)) + data
    with _lock:
        sock = _sock
    if sock is None:
        _state["error"] = "not connected"
        return False
    try:
        sock.sendall(frame)
        _state["error"] = None
        return True
    except Exception as exc:  # noqa: BLE001
        _state["error"] = str(exc)
        _state["connected"] = False
        return False


def _safe_rpm_name(name: str) -> str | None:
    n = (name or "").strip()
    if n.lower().endswith(".rpm"):
        n = n[:-4]
    if not _SAFE_NAME.match(n):
        return None
    return n


def _rpm_path(name: str) -> Path:
    RPM_DIR.mkdir(parents=True, exist_ok=True)
    return RPM_DIR / f"{name}.rpm"


def _msg_to_event(msg: pb.ClientToRobot) -> dict | None:
    which = msg.WhichOneof("op")
    if which == "drive":
        return {"op": "drive", "left": int(msg.drive.left), "right": int(msg.drive.right)}
    if which == "arm":
        ev: dict = {"op": "arm"}
        if msg.arm.set_base:
            ev["base"] = int(msg.arm.base)
        if msg.arm.set_height:
            ev["height"] = int(msg.arm.height)
        if msg.arm.set_grip:
            ev["grip"] = int(msg.arm.grip)
        return ev
    if which == "stop":
        return {"op": "stop"}
    if which == "center":
        return {"op": "center"}
    if which == "zero":
        return {"op": "zero"}
    if which == "conveyor":
        return {"op": "conveyor", "speed": int(msg.conveyor.speed)}
    if which == "motor_test":
        return {"op": "motor_test"}
    return None


def _event_to_msg(ev: dict) -> pb.ClientToRobot | None:
    op = ev.get("op")
    msg = pb.ClientToRobot()
    if op == "drive":
        msg.drive.left = int(ev.get("left", 0))
        msg.drive.right = int(ev.get("right", 0))
        return msg
    if op == "arm":
        if "base" in ev:
            msg.arm.base = int(ev["base"])
            msg.arm.set_base = True
        if "height" in ev:
            msg.arm.height = int(ev["height"])
            msg.arm.set_height = True
        if "grip" in ev:
            msg.arm.grip = int(ev["grip"])
            msg.arm.set_grip = True
        return msg
    if op == "stop":
        msg.stop = True
        return msg
    if op == "center":
        msg.center = True
        return msg
    if op == "zero":
        msg.zero = True
        return msg
    if op == "conveyor":
        msg.conveyor.speed = int(ev.get("speed", 0))
        return msg
    if op == "motor_test":
        msg.motor_test = True
        return msg
    return None


def _event_payload_eq(a: dict, b: dict) -> bool:
    if a.get("op") != b.get("op"):
        return False
    keys = ("left", "right", "base", "height", "grip", "speed")
    return all(a.get(k) == b.get(k) for k in keys)


def _i16(v: int) -> int:
    return max(-32768, min(32767, int(v)))


def _u8(v: int) -> int:
    return max(0, min(255, int(v)))


def _pack_event(ev: dict) -> bytes:
    op_name = ev.get("op")
    code = _OP_CODE.get(op_name)  # type: ignore[arg-type]
    if code is None:
        raise ValueError(f"bad op {op_name!r}")
    t_ms = max(0, int(ev.get("t_ms", 0))) & 0xFFFFFFFF
    head = struct.pack("<IB", t_ms, code)
    if code == OP_DRIVE:
        return head + struct.pack("<hh", _i16(ev.get("left", 0)), _i16(ev.get("right", 0)))
    if code == OP_ARM:
        mask = 0
        body = bytearray()
        if "base" in ev:
            mask |= 1
            body.append(_u8(ev["base"]))
        if "height" in ev:
            mask |= 2
            body.append(_u8(ev["height"]))
        if "grip" in ev:
            mask |= 4
            body.append(_u8(ev["grip"]))
        return head + bytes([mask]) + bytes(body)
    if code == OP_CONV:
        return head + struct.pack("<h", _i16(ev.get("speed", 0)))
    return head


def _unpack_events(blob: bytes, count: int) -> list[dict]:
    events: list[dict] = []
    off = 0
    n = len(blob)
    for _ in range(count):
        if off + 5 > n:
            raise ValueError("truncated event")
        t_ms, code = struct.unpack_from("<IB", blob, off)
        off += 5
        name = _OP_NAME.get(code)
        if name is None:
            raise ValueError(f"unknown opcode {code}")
        ev: dict = {"t_ms": int(t_ms), "op": name}
        if code == OP_DRIVE:
            if off + 4 > n:
                raise ValueError("truncated drive")
            left, right = struct.unpack_from("<hh", blob, off)
            off += 4
            ev["left"] = int(left)
            ev["right"] = int(right)
        elif code == OP_ARM:
            if off + 1 > n:
                raise ValueError("truncated arm")
            mask = blob[off]
            off += 1
            for bit, key in ((1, "base"), (2, "height"), (4, "grip")):
                if mask & bit:
                    if off + 1 > n:
                        raise ValueError("truncated arm field")
                    ev[key] = int(blob[off])
                    off += 1
        elif code == OP_CONV:
            if off + 2 > n:
                raise ValueError("truncated conveyor")
            (speed,) = struct.unpack_from("<h", blob, off)
            off += 2
            ev["speed"] = int(speed)
        events.append(ev)
    return events


def _write_rpm(name: str, events: list[dict]) -> Path:
    name_b = name.encode("utf-8")
    if len(name_b) > 0xFFFF:
        raise ValueError("name too long")
    body = bytearray()
    for ev in events:
        body += _pack_event(ev)
    hdr = RPM_HDR.pack(
        RPM_MAGIC,
        RPM_VERSION,
        0,
        len(name_b),
        int(time.time()) & 0xFFFFFFFF,
        len(events),
    )
    path = _rpm_path(name)
    path.write_bytes(hdr + name_b + bytes(body))
    return path


def _read_rpm_header(path: Path) -> tuple[int, int, int, bytes]:
    """Return (version, created_unix, event_count, name_bytes)."""
    raw = path.read_bytes()
    if len(raw) < RPM_HDR.size:
        raise ValueError("too short")
    magic, ver, _flags, name_len, created, count = RPM_HDR.unpack_from(raw, 0)
    if magic != RPM_MAGIC:
        raise ValueError("not a binary meowler .rpm (bad magic)")
    if ver != RPM_VERSION:
        raise ValueError(f"unsupported .rpm version {ver}")
    end = RPM_HDR.size + name_len
    if end > len(raw):
        raise ValueError("truncated name")
    return ver, int(created), int(count), raw[RPM_HDR.size:end]


def _load_rpm(name: str) -> dict:
    path = _rpm_path(name)
    if not path.is_file():
        raise FileNotFoundError(name)
    raw = path.read_bytes()
    if len(raw) < RPM_HDR.size:
        raise ValueError("too short")
    magic, ver, _flags, name_len, created, count = RPM_HDR.unpack_from(raw, 0)
    if magic != RPM_MAGIC:
        raise ValueError("not a binary meowler .rpm (bad magic)")
    if ver != RPM_VERSION:
        raise ValueError(f"unsupported .rpm version {ver}")
    off = RPM_HDR.size
    name_b = raw[off : off + name_len]
    off += name_len
    if len(name_b) != name_len:
        raise ValueError("truncated name")
    events = _unpack_events(raw[off:], count)
    return {
        "format": "meowler.rpm",
        "version": ver,
        "name": name_b.decode("utf-8", errors="replace"),
        "created_unix": int(created),
        "events": events,
    }


def _replay_loop(events: list[dict]) -> None:
    global _replaying
    last_t = 0
    try:
        for ev in events:
            if _replay_stop.is_set():
                break
            t_ms = int(ev.get("t_ms", last_t))
            dt = max(0, t_ms - last_t) / 1000.0
            last_t = t_ms
            if dt > 0:
                _replay_stop.wait(dt)
            if _replay_stop.is_set():
                break
            msg = _event_to_msg(ev)
            if msg is None:
                continue
            _send(msg, record=False)
        _send(_stop_msg(), record=False)
    finally:
        with _rec_lock:
            _replaying = False
            _state["replaying"] = False
        _logs.appendleft("REPLAY done")


def _stop_msg() -> pb.ClientToRobot:
    msg = pb.ClientToRobot()
    msg.stop = True
    return msg


def _apply_telem(t: pb.Telemetry) -> None:
    _state["distance_mm"] = t.distance_mm
    _state["left"] = t.cmd_l
    _state["right"] = t.cmd_r
    _state["base"] = t.base
    _state["height"] = t.height
    _state["grip"] = t.grip
    _state["pca_ok"] = t.pca_ok
    _state["tof_ok"] = t.tof_ok
    _state["imu_ok"] = t.imu_ok
    _state["wifi_ok"] = t.wifi_ok
    _state["enc_l"] = t.enc_l
    _state["enc_r"] = t.enc_r
    _state["wheel_l_mm"] = t.wheel_l_mm
    _state["wheel_r_mm"] = t.wheel_r_mm
    _state["tof_disp_mm"] = t.tof_disp_mm
    _state["yaw_cdeg"] = t.yaw_cdeg
    _state["pitch_cdeg"] = t.pitch_cdeg
    _state["roll_cdeg"] = t.roll_cdeg
    code = int(t.color) if 0 <= int(t.color) <= 3 else 0
    names = ("—", "RED", "YELLOW", "GREEN")
    _state["color"] = code
    _state["color_name"] = names[code]
    _state["color_conf"] = int(t.color_conf)
    _state["color_r"] = int(t.color_r)
    _state["color_g"] = int(t.color_g)
    _state["color_b"] = int(t.color_b)
    _state["color_rp"] = int(t.color_rp)
    _state["color_gp"] = int(t.color_gp)
    _state["color_bp"] = int(t.color_bp)
    _state["color_cp"] = int(t.color_cp)
    _state["conveyor"] = t.conveyor
    _state["last_rx"] = "telem"


def _rec_event_to_dict(e) -> dict | None:
    op = int(e.op)
    t_ms = int(e.t_ms)
    if op == 1:
        return {"t_ms" : t_ms, "op": "drive", "left": int(e.a), "right": int(e.b)}
    if op == 2:
        ev: dict = {"t_ms": t_ms, "op": "arm"}
        mask = int(e.mask)
        if mask & 1:
            ev["base"] = int(e.a)
        if mask & 2:
            ev["height"] = int(e.b)
        if mask & 4:
            ev["grip"] = int(e.c)
        if mask == 0:
            ev["base"] = int(e.a)
            ev["height"] = int(e.b)
            ev["grip"] = int(e.c)
        return ev
    if op == 3:
        return {"t_ms": t_ms, "op": "stop"}
    if op == 4:
        return {"t_ms": t_ms, "op": "center"}
    if op == 5:
        return {"t_ms": t_ms, "op": "zero"}
    if op == 6:
        return {"t_ms": t_ms, "op": "conveyor", "speed": int(e.a)}
    if op == 7:
        return {"t_ms": t_ms, "op": "motor_test"}
    return None


def _handle_rec(e) -> None:
    global _recording, _rec_events, _rec_name
    op = int(e.op)
    finish_name = None
    finish_events: list[dict] | None = None
    with _rec_lock:
        if op == 0:
            _recording = True
            _rec_events = []
            nm = (e.name or "").strip() or _rec_name or "move"
            _rec_name = _safe_rpm_name(nm) or "move"
            _state["recording"] = True
            _state["record_name"] = _rec_name
            _state["record_events"] = 0
            _logs.appendleft(f"REC stream {_rec_name}.rpm (ESP32)")
            return
        if op == 255:
            finish_name = _rec_name or "move"
            finish_events = list(_rec_events)
            _recording = False
            _state["recording"] = False
            _state["record_events"] = len(finish_events)
        else:
            if not _recording:
                return
            ev = _rec_event_to_dict(e)
            if ev is None:
                return
            if _rec_events and _event_payload_eq(_rec_events[-1], ev):
                return
            _rec_events.append(ev)
            _state["record_events"] = len(_rec_events)
            return
    if finish_events is not None and finish_name:
        path = _write_rpm(finish_name, finish_events)
        _logs.appendleft(f"REC saved {path.name} events={len(finish_events)} (ESP stream)")


def _handle_server(msg: pb.RobotToClient) -> None:
    which = msg.WhichOneof("msg")
    if which == "hello":
        h = msg.hello
        ip = socket.inet_ntoa(struct.pack("!I", h.ip & 0xFFFFFFFF))
        line = f"HELLO {ip}:{h.port} pca={int(h.pca_ok)} tof={int(h.tof_ok)} imu={int(h.imu_ok)}"
        _logs.appendleft(line)
        _state["last_rx"] = line
        _state["pca_ok"] = h.pca_ok
        _state["tof_ok"] = h.tof_ok
        _state["imu_ok"] = h.imu_ok
        _state["wifi_ok"] = True
    elif which == "telem":
        _apply_telem(msg.telem)
    elif which == "ack":
        _state["ack"] = msg.ack
        if msg.ack != 0:
            _logs.appendleft(f"ACK err={msg.ack}")
    elif which == "log":
        lvl = ("DBG", "INF", "WRN", "ERR")
        i = int(msg.log.level)
        tag = lvl[i] if 0 <= i < len(lvl) else str(i)
        line = f"[{tag}] {msg.log.text}"
        _logs.appendleft(line)
        _state["last_rx"] = line
    elif which == "rec":
        _handle_rec(msg.rec)


def _reader_loop() -> None:
    buf = bytearray()
    while not _stop.is_set():
        with _lock:
            sock = _sock
        if sock is None:
            time.sleep(0.05)
            continue
        try:
            chunk = sock.recv(1024)
        except socket.timeout:
            continue
        except Exception as exc:  # noqa: BLE001
            with _lock:
                _state["error"] = str(exc)
                _state["connected"] = False
            time.sleep(0.2)
            continue
        if not chunk:
            with _lock:
                _state["error"] = "robot disconnected"
                _state["connected"] = False
            time.sleep(0.2)
            continue
        buf.extend(chunk)
        while len(buf) >= 4:
            (n,) = struct.unpack_from("<I", buf, 0)
            if n == 0 or n > MAX_FRAME:
                buf.clear()
                _logs.appendleft("bad frame length")
                break
            if len(buf) < 4 + n:
                break
            payload = bytes(buf[4 : 4 + n])
            del buf[: 4 + n]
            msg = pb.RobotToClient()
            try:
                msg.ParseFromString(payload)
            except Exception as exc:  # noqa: BLE001
                _logs.appendleft(f"protobuf decode: {exc}")
                continue
            with _lock:
                _handle_server(msg)


def connect_tcp(host: str, port: int = DEFAULT_PORT) -> None:
    global _sock, _reader
    disconnect()
    sock = socket.create_connection((host, port), timeout=6.0)
    sock.settimeout(0.05)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        pass
    with _lock:
        _sock = sock
        _state["connected"] = True
        _state["mode"] = "protobuf-tcp"
        _state["host"] = f"{host}:{port}"
        _state["error"] = None
        _state["wifi_ok"] = True
    _stop.clear()
    _reader = threading.Thread(target=_reader_loop, daemon=True)
    _reader.start()


def disconnect() -> None:
    global _sock, _reader
    try:
        msg = pb.ClientToRobot()
        msg.stop = True
        _send(msg)
    except Exception:  # noqa: BLE001
        pass
    _stop.set()
    if _reader and _reader.is_alive():
        _reader.join(timeout=0.6)
    _reader = None
    with _lock:
        if _sock is not None:
            try:
                _sock.close()
            except Exception:  # noqa: BLE001
                pass
        _sock = None
        _state["connected"] = False
        _state["host"] = None
        _state["mode"] = None


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/ports")
def api_ports():
    return jsonify({"default_host": DEFAULT_HOST, "default_port": DEFAULT_PORT, "proto": "protobuf"})


@app.get("/api/state")
def api_state():
    with _lock:
        payload = dict(_state)
        payload["logs"] = list(_logs)[:24]
    return jsonify(payload)


@app.post("/api/connect")
def api_connect():
    data = request.get_json(force=True, silent=True) or {}
    raw_host = data.get("host")
    if raw_host is None or str(raw_host).strip() == "":
        raw_host = DEFAULT_HOST
    try:
        port = int(data.get("port") or DEFAULT_PORT)
    except (TypeError, ValueError):
        port = DEFAULT_PORT
    host, port = _normalize_host_port(str(raw_host), port)
    try:
        connect_tcp(host, port)
        _logs.appendleft(f"CONNECT {host}:{port}")
        return jsonify({"ok": True, "mode": "protobuf-tcp", "host": host, "port": port})
    except Exception as exc:  # noqa: BLE001
        _logs.appendleft(f"CONNECT FAIL {host}:{port} — {exc}")
        return jsonify({"ok": False, "error": str(exc), "host": host, "port": port}), 500


@app.post("/api/disconnect")
def api_disconnect():
    disconnect()
    return jsonify({"ok": True})


@app.post("/api/arm")
def api_arm():
    data = request.get_json(force=True, silent=True) or {}
    msg = pb.ClientToRobot()
    arm = msg.arm
    if data.get("base") is not None and data.get("height") is None and data.get("grip") is None:
        arm.base = int(data["base"])
        arm.set_base = True
    elif data.get("height") is not None and data.get("base") is None and data.get("grip") is None:
        arm.height = int(data["height"])
        arm.set_height = True
    elif data.get("grip") is not None and data.get("base") is None and data.get("height") is None:
        arm.grip = int(data["grip"])
        arm.set_grip = True
    else:
        arm.base = int(data["base"] if data.get("base") is not None else _state["base"])
        arm.height = int(data["height"] if data.get("height") is not None else _state["height"])
        arm.grip = int(data["grip"] if data.get("grip") is not None else _state["grip"])
        arm.set_base = arm.set_height = arm.set_grip = True
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/center")
def api_center():
    msg = pb.ClientToRobot()
    msg.center = True
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/drive")
def api_drive():
    data = request.get_json(force=True, silent=True) or {}
    msg = pb.ClientToRobot()
    msg.drive.left = max(-255, min(255, int(data.get("left", 0))))
    msg.drive.right = max(-255, min(255, int(data.get("right", 0))))
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/stop")
def api_stop():
    msg = pb.ClientToRobot()
    msg.stop = True
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/motor_test")
def api_motor_test():
    msg = pb.ClientToRobot()
    msg.motor_test = True
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/zero")
def api_zero():
    msg = pb.ClientToRobot()
    msg.zero = True
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/conveyor")
def api_conveyor():
    data = request.get_json(force=True, silent=True) or {}
    speed = max(-255, min(255, int(data.get("speed", 0))))
    msg = pb.ClientToRobot()
    msg.conveyor.speed = speed
    ok = _send(msg)
    if ok:
        _state["conveyor"] = speed
    return jsonify({"ok": ok, "speed": speed, "error": _state.get("error")})


@app.post("/api/color_cal")
def api_color_cal():
    """mode: 0=white 1=teach RED 2=YELLOW 3=GREEN 4=clear teach 5=factory"""
    data = request.get_json(force=True, silent=True) or {}
    mode = int(data.get("mode", 0))
    msg = pb.ClientToRobot()
    msg.color_cal.mode = mode
    ok = _send(msg)
    names = {
        0: "white-balance",
        1: "teach RED",
        2: "teach YELLOW",
        3: "teach GREEN",
        4: "clear teach",
        5: "factory reset",
    }
    _logs.appendleft(f"COLOR CAL {names.get(mode, mode)} ok={int(ok)}")
    return jsonify({"ok": ok, "mode": mode, "error": _state.get("error")})


def _list_rpms() -> list[dict]:
    RPM_DIR.mkdir(parents=True, exist_ok=True)
    out = []
    for p in sorted(RPM_DIR.glob("*.rpm"), key=lambda x: x.stat().st_mtime, reverse=True):
        n_ev = 0
        try:
            _ver, _created, n_ev, _name = _read_rpm_header(p)
        except Exception:  # noqa: BLE001
            pass
        out.append({"name": p.stem, "events": n_ev, "bytes": p.stat().st_size})
    return out


@app.get("/api/record/state")
def api_record_state():
    with _rec_lock:
        return jsonify({
            "recording": _recording,
            "replaying": _replaying,
            "name": _rec_name,
            "events": len(_rec_events) if _recording else _state.get("record_events", 0),
            "files": _list_rpms(),
        })


def _send_rec(action: int, name: str = "") -> bool:
    msg = pb.ClientToRobot()
    msg.rec.action = int(action)
    if name:
        msg.rec.name = name[:31]
    return _send(msg, record=False)


@app.post("/api/record/start")
def api_record_start():
    global _recording, _rec_t0, _rec_events, _rec_name
    data = request.get_json(force=True, silent=True) or {}
    name = _safe_rpm_name(str(data.get("name") or time.strftime("move-%Y%m%d-%H%M%S")))
    if not name:
        return jsonify({"ok": False, "error": "bad name"}), 400
    with _rec_lock:
        if _replaying:
            return jsonify({"ok": False, "error": "replaying"}), 409
        if not _state.get("connected"):
            return jsonify({"ok": False, "error": "not connected"}), 400
        _recording = True
        _rec_t0 = time.monotonic()
        _rec_events = []
        _rec_name = name
        _state["recording"] = True
        _state["record_name"] = name
        _state["record_events"] = 0
    ok = _send_rec(1, name)
    _logs.appendleft(f"REC start {name}.rpm → ESP32 stream")
    return jsonify({"ok": ok, "name": name, "error": _state.get("error")})


@app.post("/api/record/stop")
def api_record_stop():
    global _recording, _rec_events, _rec_name
    with _rec_lock:
        if not _recording:
            return jsonify({"ok": False, "error": "not recording"}), 400
        name = _rec_name or time.strftime("move-%Y%m%d-%H%M%S")
    _send_rec(2, name)
    deadline = time.monotonic() + 1.6
    while time.monotonic() < deadline:
        with _rec_lock:
            if not _recording:
                n = int(_state.get("record_events") or 0)
                return jsonify({"ok": True, "name": name, "events": n, "file": f"{name}.rpm"})
        time.sleep(0.05)
    with _rec_lock:
        events = list(_rec_events)
        name = _rec_name or name
        _recording = False
        _state["recording"] = False
    path = _write_rpm(name, events)
    _logs.appendleft(f"REC saved {path.name} events={len(events)} (timeout flush)")
    return jsonify({"ok": True, "name": name, "events": len(events), "file": path.name})


@app.get("/api/record/list")
def api_record_list():
    return jsonify({"files": _list_rpms()})


@app.post("/api/replay")
def api_replay():
    global _replaying, _replay_thread
    data = request.get_json(force=True, silent=True) or {}
    name = _safe_rpm_name(str(data.get("name") or ""))
    if not name:
        return jsonify({"ok": False, "error": "need name"}), 400
    try:
        rpm = _load_rpm(name)
    except FileNotFoundError:
        return jsonify({"ok": False, "error": "missing"}), 404
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400
    with _rec_lock:
        if _recording:
            return jsonify({"ok": False, "error": "recording"}), 409
        if _replaying:
            return jsonify({"ok": False, "error": "already replaying"}), 409
        if not _state.get("connected"):
            return jsonify({"ok": False, "error": "not connected"}), 400
        _replay_stop.clear()
        _replaying = True
        _state["replaying"] = True
        events = list(rpm.get("events") or [])
        _replay_thread = threading.Thread(target=_replay_loop, args=(events,), daemon=True)
        _replay_thread.start()
    _logs.appendleft(f"REPLAY {name}.rpm events={len(events)}")
    return jsonify({"ok": True, "name": name, "events": len(events)})


@app.post("/api/replay/stop")
def api_replay_stop():
    _replay_stop.set()
    _send(_stop_msg(), record=False)
    return jsonify({"ok": True})


def _play_rpm_blocking(name: str, stop_ev: threading.Event) -> None:
    """Replay a .rpm for mission scripts (blocks; respects stop_ev)."""
    rpm = _load_rpm(name)
    events = list(rpm.get("events") or [])
    last_t = 0
    for ev in events:
        if stop_ev.is_set():
            break
        t_ms = int(ev.get("t_ms", last_t))
        dt = max(0, t_ms - last_t) / 1000.0
        last_t = t_ms
        if dt > 0 and stop_ev.wait(dt):
            break
        if stop_ev.is_set():
            break
        msg = _event_to_msg(ev)
        if msg is not None:
            _send(msg, record=False)
    _send(_stop_msg(), record=False)


def _mission_record_start(name: str) -> None:
    global _recording, _rec_t0, _rec_events, _rec_name
    with _rec_lock:
        if _replaying or _recording:
            raise MissionError("busy recording/replaying")
        _recording = True
        _rec_t0 = time.monotonic()
        _rec_events = []
        _rec_name = name
        _state["recording"] = True
        _state["record_name"] = name
        _state["record_events"] = 0
    _send_rec(1, name)


def _mission_record_stop() -> None:
    global _recording, _rec_events, _rec_name
    with _rec_lock:
        if not _recording:
            return
        name = _rec_name or "move"
    _send_rec(2, name)
    deadline = time.monotonic() + 1.6
    while time.monotonic() < deadline:
        with _rec_lock:
            if not _recording:
                return
        time.sleep(0.05)
    with _rec_lock:
        events = list(_rec_events)
        name = _rec_name or name
        _recording = False
        _state["recording"] = False
    _write_rpm(name, events)


_mission = MissionRunner(
    send_msg=lambda m: _send(m, record=False),
    get_state=lambda: _state,
    play_rpm=_play_rpm_blocking,
    record_start=_mission_record_start,
    record_stop=_mission_record_stop,
    log=lambda s: _logs.appendleft(s),
)


@app.get("/api/mission/list")
def api_mission_list():
    return jsonify({"files": list_missions()})


@app.get("/api/mission/get")
def api_mission_get():
    name = str(request.args.get("name") or "")
    try:
        return jsonify({"ok": True, "name": name, "src": load_mission(name)})
    except MissionError as e:
        return jsonify({"ok": False, "error": str(e)}), 404


@app.post("/api/mission/save")
def api_mission_save():
    data = request.get_json(force=True, silent=True) or {}
    name = str(data.get("name") or "").strip()
    src = str(data.get("src") or "")
    try:
        path = save_mission(name, src)
        return jsonify({"ok": True, "name": path.stem})
    except MissionError as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.post("/api/mission/run")
def api_mission_run():
    data = request.get_json(force=True, silent=True) or {}
    src = data.get("src")
    name = str(data.get("name") or "").strip()
    if src is None and name:
        try:
            src = load_mission(name)
        except MissionError as e:
            return jsonify({"ok": False, "error": str(e)}), 404
    if not isinstance(src, str) or not src.strip():
        return jsonify({"ok": False, "error": "need src or name"}), 400
    if not _state.get("connected"):
        return jsonify({"ok": False, "error": "not connected"}), 400
    with _rec_lock:
        if _recording or _replaying:
            return jsonify({"ok": False, "error": "recording/replaying"}), 409
    if _mission.is_running():
        return jsonify({"ok": False, "error": "mission running"}), 409
    try:
        _mission.start(src)
    except MissionError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    _logs.appendleft("MISSION start")
    return jsonify({"ok": True})


@app.post("/api/mission/stop")
def api_mission_stop():
    _mission.stop()
    _replay_stop.set()
    _send(_stop_msg(), record=False)
    return jsonify({"ok": True})


@app.get("/api/mission/state")
def api_mission_state():
    return jsonify({
        "running": _mission.is_running(),
        "line": _mission.line,
        "error": _mission.last_error,
        "files": list_missions(),
    })


@app.post("/api/ota")
def api_ota():
    """action: 6=query 7=boot ota_0 8=boot ota_1 — full upload via ota_upload.py"""
    data = request.get_json(force=True, silent=True) or {}
    action = int(data.get("action", 6))
    if action not in (6, 7, 8):
        return jsonify({"ok": False, "error": "use ota_upload.py for begin/chunk/finish/apply"}), 400
    msg = pb.ClientToRobot()
    msg.ota.action = action
    ok = _send(msg, record=False)
    _logs.appendleft(f"OTA action={action} ok={int(ok)}")
    return jsonify({"ok": ok, "action": action, "error": _state.get("error")})


def main() -> None:
    app.run(host="0.0.0.0", port=5050, debug=False, threaded=True)


if __name__ == "__main__":
    main()
