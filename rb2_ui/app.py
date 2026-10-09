#!/usr/bin/env python3
"""RB2 Nano test panel — USB serial @ 115200."""
from __future__ import annotations

import argparse
import os
import threading
import time
from pathlib import Path

import serial
from flask import Flask, jsonify, render_template, request
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parent
app = Flask(__name__, template_folder=str(ROOT / "templates"))

_lock = threading.Lock()
_ser: serial.Serial | None = None
_log: list[str] = []
_telem = {
    "imu": 0,
    "yaw": 0.0,
    "rate": 0.0,
    "encL": 0,
    "encR": 0,
    "scaleL": 100,
    "scaleR": 100,
    "port": "",
    "ok": False,
}
_reader_stop = threading.Event()


def _push_log(line: str) -> None:
    _log.append(line)
    if len(_log) > 200:
        del _log[:50]


def _parse_telem(line: str) -> None:
    # TELEM imu yaw rate encL encR scaleL scaleR
    parts = line.split()
    if len(parts) < 8 or parts[0] != "TELEM":
        return
    try:
        _telem["imu"] = int(parts[1])
        _telem["yaw"] = float(parts[2])
        _telem["rate"] = float(parts[3])
        _telem["encL"] = int(parts[4])
        _telem["encR"] = int(parts[5])
        _telem["scaleL"] = int(parts[6])
        _telem["scaleR"] = int(parts[7])
    except ValueError:
        pass


def _reader() -> None:
    global _ser
    buf = ""
    while not _reader_stop.is_set():
        with _lock:
            ser = _ser
        if ser is None or not ser.is_open:
            time.sleep(0.2)
            continue
        try:
            raw = ser.read(ser.in_waiting or 1)
        except Exception as e:
            _push_log(f"! read {e}")
            time.sleep(0.3)
            continue
        if not raw:
            time.sleep(0.01)
            continue
        buf += raw.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            if line.startswith("TELEM "):
                _parse_telem(line)
            else:
                _push_log(line)


def send_cmd(cmd: str, wait: float = 0.05) -> str:
    cmd = cmd.strip() + "\n"
    with _lock:
        ser = _ser
        if ser is None or not ser.is_open:
            return "ERR not connected"
        try:
            ser.write(cmd.encode("ascii", errors="ignore"))
            ser.flush()
        except Exception as e:
            return f"ERR {e}"
    if wait:
        time.sleep(wait)
    return "OK"


def open_port(port: str, baud: int = 115200) -> str:
    global _ser
    close_port()
    try:
        ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
    except Exception as e:
        return f"ERR {e}"
    time.sleep(1.6)  # Nano reset after open
    with _lock:
        _ser = ser
    _telem["port"] = port
    _telem["ok"] = True
    _push_log(f"opened {port}")
    send_cmd("ping")
    return "OK"


def close_port() -> None:
    global _ser
    with _lock:
        ser = _ser
        _ser = None
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
    _telem["ok"] = False
    _telem["port"] = ""


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/ports")
def api_ports():
    ports = [{"device": p.device, "desc": p.description} for p in list_ports.comports()]
    return jsonify(ports)


@app.post("/api/connect")
def api_connect():
    data = request.get_json(force=True, silent=True) or {}
    port = str(data.get("port") or "").strip()
    if not port:
        return jsonify({"ok": False, "err": "no port"}), 400
    r = open_port(port)
    return jsonify({"ok": r == "OK", "msg": r})


@app.post("/api/disconnect")
def api_disconnect():
    close_port()
    return jsonify({"ok": True})


@app.get("/api/telem")
def api_telem():
    return jsonify({**_telem, "log": _log[-40:]})


@app.post("/api/cmd")
def api_cmd():
    data = request.get_json(force=True, silent=True) or {}
    cmd = str(data.get("cmd") or "").strip()
    if not cmd:
        return jsonify({"ok": False, "err": "empty"}), 400
    # Long moves — don't wait for completion (Nano blocks until done)
    wait = 0.02
    if cmd.split()[0] in ("cal", "fwd", "back", "turn", "dist", "prep"):
        wait = 0.05
    r = send_cmd(cmd, wait=wait)
    _push_log(f"> {cmd}")
    return jsonify({"ok": r.startswith("OK"), "msg": r})


def main() -> None:
    ap = argparse.ArgumentParser(description="RB2 test UI")
    ap.add_argument("--port", default=os.environ.get("RB2_PORT", "COM3"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=5055)
    ap.add_argument("--no-auto", action="store_true")
    args = ap.parse_args()

    _reader_stop.clear()
    t = threading.Thread(target=_reader, name="rb2-ser", daemon=True)
    t.start()

    if not args.no_auto and args.port:
        msg = open_port(args.port)
        print(msg, flush=True)

    print(f"RB2 UI  http://{args.host}:{args.http_port}", flush=True)
    try:
        app.run(host=args.host, port=args.http_port, threaded=True, use_reloader=False)
    finally:
        _reader_stop.set()
        close_port()


if __name__ == "__main__":
    main()
