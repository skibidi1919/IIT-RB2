#!/usr/bin/env python3
"""TCS3200 color sensor GUI — high-accuracy classifier + serial bridge."""

from __future__ import annotations

import glob
import re
import threading
import time
from collections import deque

import serial
from flask import Flask, jsonify, render_template, request
from serial.tools import list_ports

app = Flask(__name__)

_LINE_RE = re.compile(
    r"R\s*=\s*(\d+)\s+G\s*=\s*(\d+)\s+B\s*=\s*(\d+)(?:\s+C\s*=\s*(\d+))?"
    r"(?:\s*=>\s*([A-Z_]+))?",
    re.I,
)

_ser_lock = threading.RLock()
_ser: serial.Serial | None = None
_stop_reader = threading.Event()

# Distance bands from mean RGB pulse (longer = farther). Classification is not
# blocked in-band; ambient subtraction is what makes 5–20 cm accurate.
_AVG_CONTACT = 14.0  # < this ≈ contact / <~3 cm
_AVG_FAR = 160.0  # > this ≈ beyond ~20 cm
_AVG_LOST = 240.0  # no usable target

# White-LED spectral trim — keep empty-field nearly gray.
_TRIM_R = 1.05
_TRIM_G = 1.00
_TRIM_B = 0.82

# Soft ambient subtraction (f - k*amb) restores chroma at 5–20 cm.
# Default ambient ≈ empty-field on this LED+TCS3200 (R≈63 G≈53 B≈45).
_AMB_SUB = 0.88
_AMB_ALPHA = 0.12  # refine empty-field when gray frames appear
_DEFAULT_AMB_PULSE = (63.0, 53.0, 45.0)
_amb_f: list[float] | None = None  # ambient R/G/B frequencies

# Slightly longer median at distance (noisier), still fast
_PULSE_WINDOW = 4
_pulse_hist: deque[tuple[int, int, int, int]] = deque(maxlen=_PULSE_WINDOW)

_TARGETS = ("RED", "YELLOW", "GREEN")
_DISPLAY_HEX = {
    "RED": "#e53935",
    "YELLOW": "#fdd835",
    "GREEN": "#43a047",
}

# Fast lock: instant when obvious, ~250 ms when borderline.
_CONFIRM_S = 0.16
_SWITCH_S = 0.24
_LOCK_MISS_CLEAR_S = 0.35  # drop stale lock when colour washes out at range
_lock: str | None = None
_pending: str | None = None
_pending_t0 = 0.0
_pending_hits = 0
_miss_t0 = 0.0

_state = {
    "connected": False,
    "port": None,
    "r": 0,
    "g": 0,
    "b": 0,
    "c": 0,
    "label": "—",
    "sensed": "—",
    "pending": "—",
    "lock_progress": 0.0,
    "range_ok": False,
    "h": 0.0,
    "s": 0.0,
    "v": 0.0,
    "rgb": [0, 0, 0],
    "hex": "#000000",
    "pct": [0, 0, 0],
    "window": 0,
    "last_line": "",
    "error": None,
    "updated_ms": 0,
}


def _reset_filters() -> None:
    global _lock, _pending, _pending_t0, _pending_hits, _miss_t0, _amb_f
    _pulse_hist.clear()
    _lock = None
    _pending = None
    _pending_t0 = 0.0
    _pending_hits = 0
    _miss_t0 = 0.0
    # Always start from measured empty-field baseline so 5–20 cm works immediately
    ar, ag, ab = _DEFAULT_AMB_PULSE
    _amb_f = list(_raw_freqs(ar, ag, ab))


def _is_useful_port(device: str, hwid: str = "", description: str = "") -> bool:
    name = device.rsplit("/", 1)[-1]
    if name.startswith(("ttyUSB", "ttyACM", "cu.", "COM")):
        return True
    blob = f"{hwid} {description}".upper()
    return any(tag in blob for tag in ("USB", "FTDI", "CH340", "CP210", "SILICON", "UART"))


def candidate_ports() -> list[dict]:
    ports: list[dict] = []
    seen: set[str] = set()
    try:
        comports = list(list_ports.comports())
    except Exception:  # noqa: BLE001 — Windows/pyserial ctypes glitches
        comports = []
        # Fallback: bare COM names
        try:
            import serial.tools.list_ports_common as _lpc  # type: ignore

            for name in getattr(serial.tools.list_ports_windows, "comports", lambda: [])():  # type: ignore
                pass
        except Exception:  # noqa: BLE001
            pass
        for i in range(1, 30):
            device = f"COM{i}"
            ports.append({"device": device, "description": "serial port", "hwid": ""})
        return sorted(
            ports,
            key=lambda p: (0 if p["device"].upper() == "COM4" else 1, p["device"]),
        )

    for p in comports:
        try:
            device = p.device
            desc = p.description or ""
            hwid = p.hwid or ""
        except Exception:  # noqa: BLE001
            continue
        if not _is_useful_port(device, hwid, desc):
            continue
        seen.add(device)
        ports.append({"device": device, "description": desc, "hwid": hwid})

    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for device in sorted(glob.glob(pattern)):
            if device not in seen:
                ports.append(
                    {"device": device, "description": "serial device", "hwid": ""}
                )

    if not ports:
        # Last-resort COM list so UI still works if enumeration breaks
        for i in (4, 3, 5, 6, 7, 8, 9, 10):
            ports.append(
                {"device": f"COM{i}", "description": "manual entry", "hwid": ""}
            )

    ports.sort(
        key=lambda p: (
            0 if p["device"].upper() == "COM4" else 1,
            0 if p["device"].upper().startswith("COM") else 1,
            p["device"],
        )
    )
    return ports


def _freq(p: float) -> float:
    """Pulse width (µs) → relative frequency. Lower pulse = higher freq/intensity."""
    return 0.0 if p <= 0 else 1_000_000.0 / float(p)


def _push_pulses(r: int, g: int, b: int, c: int) -> tuple[float, float, float, float]:
    sample = (max(0, r), max(0, g), max(0, b), max(0, c))
    if sample[0] or sample[1] or sample[2]:
        # Flush history on a big jump so swapping cards isn't smeared
        if _pulse_hist:
            ar = sum(p[0] for p in _pulse_hist) / len(_pulse_hist)
            ag = sum(p[1] for p in _pulse_hist) / len(_pulse_hist)
            if abs(sample[0] - ar) > max(8.0, ar * 0.45) or abs(
                sample[1] - ag
            ) > max(8.0, ag * 0.45):
                _pulse_hist.clear()
        _pulse_hist.append(sample)
    if not _pulse_hist:
        return float(r), float(g), float(b), float(c)
    # Median per channel (better than mean for colour swaps)
    rs = sorted(p[0] for p in _pulse_hist)
    gs = sorted(p[1] for p in _pulse_hist)
    bs = sorted(p[2] for p in _pulse_hist)
    cs = sorted(p[3] for p in _pulse_hist)
    mid = len(rs) // 2
    return float(rs[mid]), float(gs[mid]), float(bs[mid]), float(cs[mid])


def _raw_freqs(rp: float, gp: float, bp: float) -> tuple[float, float, float]:
    return _freq(rp) * _TRIM_R, _freq(gp) * _TRIM_G, _freq(bp) * _TRIM_B


def _learn_ambient(fr: float, fg: float, fb: float, raw_spread: float) -> None:
    """Refine empty-field spectrum from low-chroma frames only."""
    global _amb_f
    if raw_spread > 0.11:
        return
    if _amb_f is None:
        _amb_f = [fr, fg, fb]
        return
    a = _AMB_ALPHA
    _amb_f[0] = (1.0 - a) * _amb_f[0] + a * fr
    _amb_f[1] = (1.0 - a) * _amb_f[1] + a * fg
    _amb_f[2] = (1.0 - a) * _amb_f[2] + a * fb


def _ensure_ambient() -> None:
    global _amb_f
    if _amb_f is None:
        ar, ag, ab = _DEFAULT_AMB_PULSE
        _amb_f = list(_raw_freqs(ar, ag, ab))


def _rgb_norm(
    rp: float, gp: float, bp: float
) -> tuple[float, float, float]:
    """
    Distance-stable chrominance: trim → soft ambient subtract → L1.
    Subtracting the empty-field spectrum is what makes 5–20 cm work;
    plain ratios wash out to gray/green beyond ~1–2 cm.
    """
    _ensure_ambient()
    fr, fg, fb = _raw_freqs(rp, gp, bp)
    tot0 = fr + fg + fb + 1e-9
    raw = (fr / tot0, fg / tot0, fb / tot0)
    raw_spread = max(raw) - min(raw)
    _learn_ambient(fr, fg, fb, raw_spread)

    fr = max(1e-6, fr - _AMB_SUB * _amb_f[0])
    fg = max(1e-6, fg - _AMB_SUB * _amb_f[1])
    fb = max(1e-6, fb - _AMB_SUB * _amb_f[2])

    tot = fr + fg + fb + 1e-9
    return fr / tot, fg / tot, fb / tot


def _rgb_to_hsv(pr: float, pg: float, pb: float) -> tuple[float, float, float]:
    mx, mn = max(pr, pg, pb), min(pr, pg, pb)
    v = mx
    d = mx - mn
    s = 0.0 if mx < 1e-6 else d / mx
    if s < 1e-4:
        return 0.0, 0.0, v
    if mx == pr:
        h = (60 * (((pg - pb) / d) % 6) + 360) % 360
    elif mx == pg:
        h = 60 * ((pb - pr) / d + 2)
    else:
        h = 60 * ((pr - pg) / d + 4)
    return h, s, v


def _sense_full(h: float, s: float, v: float) -> str:
    if v < 0.12:
        return "BLACK"
    if s < 0.08:
        return "WHITE" if v > 0.7 else "GRAY"
    if h < 15 or h >= 345:
        return "RED"
    if h < 35:
        return "ORANGE"
    if h < 70:
        return "YELLOW"
    if h < 100:
        return "LIME"
    if h < 160:
        return "GREEN"
    if h < 200:
        return "CYAN"
    if h < 260:
        return "BLUE"
    if h < 310:
        return "PURPLE"
    return "PINK"


def _dist3(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


def _decide_instant(
    pr: float, pg: float, pb: float, h: float, s: float
) -> tuple[str | None, bool]:
    """
    One-frame decision from RGB pulse ratios (distance-tolerant).
    Returns (label, certain) — certain locks immediately.
    """
    spread = max(pr, pg, pb) - min(pr, pg, pb)
    # Empty field after ambient-sub stays low-chroma; real targets pop up
    if s < 0.16 or spread < 0.09:
        return None, False

    rg = pr / max(pg, 1e-6)
    gr = pg / max(pr, 1e-6)
    rb = pr / max(pb, 1e-6)
    gb = pg / max(pb, 1e-6)

    # Dominance scores — works at 5–20 cm where absolute pulse lengths stretch
    r_dom = pr - max(pg, pb)
    g_dom = pg - max(pr, pb)
    y_pair = min(pr, pg) - pb
    y_bal = 1.0 - min(1.0, abs(pr - pg) / 0.28)

    score_r = max(0.0, r_dom) * 2.6 + max(0.0, pr - pg) * 0.9 + max(0.0, pr - pb) * 0.6
    if h < 28.0 or h >= 335.0:
        score_r += 0.35 * s
    elif h < 42.0:
        score_r += 0.12 * s

    score_g = max(0.0, g_dom) * 3.2 + max(0.0, pg - pr) * 1.2 + max(0.0, pg - pb) * 0.8
    if 75.0 <= h <= 165.0:
        score_g += 0.40 * s
    elif 55.0 <= h < 75.0:
        score_g += 0.10 * s

    score_y = max(0.0, y_pair) * 2.2 * max(0.30, y_bal)
    if 30.0 <= h <= 78.0:
        score_y += 0.32 * s
    if r_dom > 0.12:
        score_y *= 0.25
    if g_dom > 0.10:
        score_y *= 0.20
    if min(pr, pg) < 0.30:
        score_y *= 0.45

    # YELLOW first when R≈G and both beat B (close yellow often has R slightly > G)
    if (
        abs(pr - pg) <= 0.18
        and min(pr, pg) >= 0.28
        and pb <= 0.28
        and y_pair >= 0.06
        and 0.60 <= rg <= 1.60
        and 25.0 <= h <= 90.0
    ):
        return "YELLOW", True
    if (
        28.0 <= h <= 85.0
        and abs(pr - pg) <= 0.20
        and pb <= 0.30
        and y_pair >= 0.05
        and min(pr, pg) >= 0.26
    ):
        return "YELLOW", True

    # RED — R must clearly lead G (not just beat B), else yellow steals it
    if pr >= 0.42 and rg >= 1.45 and rb >= 1.25 and r_dom >= 0.12 and pg <= 0.30:
        return "RED", True
    if (h < 32.0 or h >= 335.0) and pr >= 0.40 and rg >= 1.30 and r_dom >= 0.10:
        return "RED", True

    if pg >= 0.42 and gr >= 1.35 and gb >= 1.25 and g_dom >= 0.10:
        return "GREEN", True
    if 80.0 <= h <= 160.0 and pg >= 0.40 and g_dom >= 0.08 and gr >= 1.25:
        return "GREEN", True

    # Soft score path for washed mid-range targets
    if s < 0.28 and spread < 0.16:
        return None, False
    if pb > pr and pb > pg and s < 0.30:
        return None, False

    best = "RED"
    best_score = score_r
    if score_y > best_score:
        best, best_score = "YELLOW", score_y
    if score_g > best_score:
        best, best_score = "GREEN", score_g

    if best_score < 0.22:
        return None, False

    # Require a clear winner so ambient/gray cannot nearest-neighbour to GREEN
    scores = {"RED": score_r, "YELLOW": score_y, "GREEN": score_g}
    second = max(v for k, v in scores.items() if k != best)
    if best_score - second < 0.06:
        return None, False

    certain = best_score >= 0.38 and (best_score - second) >= 0.12
    return best, certain


def _range_zone(avg_rgb: float) -> tuple[bool, str]:
    """
    Estimate standoff from mean RGB pulse. Classification is NOT blocked
    inside the loose band — only truly lost samples are rejected.
    Returns (usable_for_classify, ui_zone_label).
    """
    if avg_rgb <= 0 or avg_rgb > _AVG_LOST:
        return False, "lost"
    if avg_rgb < _AVG_CONTACT:
        return True, "near (<5cm)"
    if avg_rgb > _AVG_FAR:
        return True, "far (>20cm)"
    return True, "5-20cm"


def _update_lock(
    instant: str | None, certain: bool, usable: bool
) -> tuple[str, str, float]:
    """Hold lock. Clear matches lock instantly; fuzzy ones in ~0.2 s."""
    global _lock, _pending, _pending_t0, _pending_hits, _miss_t0
    now = time.time()

    if not usable:
        _pending = None
        _pending_t0 = 0.0
        _pending_hits = 0
        if _miss_t0 <= 0.0:
            _miss_t0 = now
        if _lock is not None and (now - _miss_t0) >= _LOCK_MISS_CLEAR_S:
            _lock = None
            _miss_t0 = 0.0
            return "—", "—", 0.0
        return (_lock or "—"), "—", 0.0

    if instant is None:
        _pending = None
        _pending_t0 = 0.0
        _pending_hits = 0
        if _miss_t0 <= 0.0:
            _miss_t0 = now
        # At 5–20 cm washout is common — don't freeze the last colour forever
        if _lock is not None and (now - _miss_t0) >= _LOCK_MISS_CLEAR_S:
            _lock = None
            _miss_t0 = 0.0
            return "—", "—", 0.0
        return (_lock or "—"), "—", 1.0 if _lock else 0.0

    _miss_t0 = 0.0

    if _lock is not None and instant == _lock:
        _pending = None
        _pending_t0 = 0.0
        _pending_hits = 0
        return _lock, "—", 1.0

    if instant != _pending:
        _pending = instant
        _pending_t0 = now
        _pending_hits = 1
        if _lock is None or certain:
            _lock = instant
            _pending = None
            _pending_t0 = 0.0
            _pending_hits = 0
            return _lock, "—", 1.0
        return _lock, instant, 0.0

    _pending_hits += 1
    need = _SWITCH_S if _lock is not None else _CONFIRM_S
    elapsed = now - _pending_t0
    progress = min(1.0, elapsed / need)
    if elapsed >= need or _pending_hits >= 2:
        _lock = instant
        _pending = None
        _pending_t0 = 0.0
        _pending_hits = 0
        return _lock, "—", 1.0

    return (_lock or "—"), instant, progress


def _classify(
    r: int, g: int, b: int, c: int
) -> tuple[
    str,
    str,
    str,
    float,
    bool,
    str,
    float,
    float,
    float,
    list[int],
    list[int],
    int,
    int,
    int,
    int,
]:
    rp, gp, bp, cp = _push_pulses(r, g, b, c)
    pr, pg, pb = _rgb_norm(rp, gp, bp)
    h, s, v = _rgb_to_hsv(pr, pg, pb)
    sensed = _sense_full(h, s, v)
    avg = (rp + gp + bp) / 3.0
    usable, zone = _range_zone(avg)

    if usable:
        instant, certain = _decide_instant(pr, pg, pb, h, s)
    else:
        instant, certain = None, False
    label, pending, progress = _update_lock(instant, certain, usable)
    # Green UI when inside intended band; near/far still classify
    range_ok = zone in ("5-20cm", "near (<5cm)")

    if label in _DISPLAY_HEX:
        hx = _DISPLAY_HEX[label]
        rgb = [int(hx[1:3], 16), int(hx[3:5], 16), int(hx[5:7], 16)]
    else:
        rgb = [
            int(max(0, min(255, round(pr * 255)))),
            int(max(0, min(255, round(pg * 255)))),
            int(max(0, min(255, round(pb * 255)))),
        ]
    pct = [int(round(pr * 100)), int(round(pg * 100)), int(round(pb * 100))]
    return (
        label,
        sensed,
        pending,
        progress,
        range_ok,
        zone,
        h,
        s,
        v,
        rgb,
        pct,
        int(round(rp)),
        int(round(gp)),
        int(round(bp)),
        int(round(cp)),
    )


def _apply_line(line: str) -> None:
    m = _LINE_RE.search(line)
    if not m:
        _state["last_line"] = line
        return
    r, g, b = int(m.group(1)), int(m.group(2)), int(m.group(3))
    c = int(m.group(4) or 0)
    (
        label,
        sensed,
        pending,
        progress,
        range_ok,
        zone,
        h,
        s,
        v,
        rgb,
        pct,
        sr,
        sg,
        sb,
        sc,
    ) = _classify(r, g, b, c)
    _state["r"] = sr
    _state["g"] = sg
    _state["b"] = sb
    _state["c"] = sc
    _state["label"] = label
    _state["sensed"] = sensed
    _state["pending"] = pending
    _state["lock_progress"] = round(progress, 2)
    _state["range_ok"] = range_ok
    _state["h"] = round(h, 1)
    _state["s"] = round(s, 3)
    _state["v"] = round(v, 3)
    _state["rgb"] = rgb
    _state["pct"] = pct
    _state["hex"] = (
        _DISPLAY_HEX[label]
        if label in _DISPLAY_HEX
        else f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"
    )
    _state["window"] = len(_pulse_hist)
    if pending and pending != "—" and label != pending:
        phase = f"locking {pending} {int(progress * 100)}%"
    elif label in _TARGETS:
        phase = f"locked {label}"
    else:
        phase = "waiting"
    _state["last_line"] = (
        f"{line}  | pct R{pct[0]} G{pct[1]} B{pct[2]}  "
        f"{sensed} | {phase} ({zone})"
    )
    _state["updated_ms"] = int(time.time() * 1000)


def _safe_close() -> None:
    global _ser
    with _ser_lock:
        ser = _ser
        _ser = None
        _state["connected"] = False
        _state["port"] = None
    if ser is None:
        return
    try:
        if getattr(ser, "is_open", False):
            ser.close()
    except Exception:  # noqa: BLE001
        pass


def _read_loop() -> None:
    buf = ""
    while not _stop_reader.is_set():
        with _ser_lock:
            ser = _ser
            open_ok = bool(ser is not None and getattr(ser, "is_open", False))
        if not open_ok:
            time.sleep(0.05)
            continue
        try:
            # Avoid in_waiting (ctypes/byref races on Win+py3.14). Blocking read w/ timeout.
            with _ser_lock:
                ser = _ser
                if ser is None or not ser.is_open:
                    chunk = b""
                else:
                    chunk = ser.read(256)
        except TypeError as exc:
            # byref() argument 1 must be _ctypes._CData, not None
            _state["error"] = f"serial handle lost ({exc})"
            _safe_close()
            buf = ""
            time.sleep(0.2)
            continue
        except Exception as exc:  # noqa: BLE001
            _state["error"] = str(exc)
            _safe_close()
            buf = ""
            time.sleep(0.2)
            continue

        if not chunk:
            time.sleep(0.01)
            continue
        buf += chunk.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if line:
                _apply_line(line)


def _send(cmd: str) -> None:
    with _ser_lock:
        if _ser is None or not _ser.is_open:
            raise RuntimeError("not connected")
        _ser.write((cmd.strip() + "\n").encode("ascii"))
        try:
            _ser.flush()
        except Exception:  # noqa: BLE001
            pass


threading.Thread(target=_read_loop, daemon=True, name="color-serial").start()


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/status")
def api_status():
    return jsonify(dict(_state))


@app.get("/api/ports")
def api_ports():
    try:
        return jsonify({"ports": candidate_ports()})
    except Exception as exc:  # noqa: BLE001
        return jsonify(
            {
                "ports": [
                    {"device": "COM4", "description": "fallback", "hwid": ""},
                    {"device": "COM3", "description": "fallback", "hwid": ""},
                ],
                "warning": str(exc),
            }
        )


@app.post("/api/connect")
def api_connect():
    global _ser
    data = request.get_json(force=True, silent=True) or {}
    port = str(data.get("port") or "COM4")
    baud = int(data.get("baud") or 115200)

    _safe_close()
    time.sleep(0.15)

    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.2
        ser.write_timeout = 1.0
        ser.dsrdtr = False
        ser.rtscts = False
        # Soft-reset ESP32 on open
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(0.05)
        ser.dtr = True
        ser.rts = False
        time.sleep(1.2)
        try:
            ser.reset_input_buffer()
        except Exception:  # noqa: BLE001
            pass
    except Exception as exc:  # noqa: BLE001
        _state["error"] = str(exc)
        _state["connected"] = False
        _state["port"] = None
        return jsonify({"ok": False, "error": str(exc)}), 400

    with _ser_lock:
        _ser = ser
        _reset_filters()
        _state["connected"] = True
        _state["port"] = port
        _state["error"] = None
        _state["label"] = "—"
        _state["sensed"] = "—"
        _state["pending"] = "—"
        _state["lock_progress"] = 0.0
        _state["range_ok"] = False
        _state["window"] = 0
        _state["last_line"] = "connected — hold colour 5–20 cm for ~2s to lock…"
        _state["updated_ms"] = int(time.time() * 1000)

    return jsonify({"ok": True, "port": port})


@app.post("/api/disconnect")
def api_disconnect():
    _safe_close()
    return jsonify({"ok": True})


if __name__ == "__main__":
    print("TCS3200 color GUI -> http://127.0.0.1:5052")
    app.run(host="127.0.0.1", port=5052, debug=False, threaded=True)
