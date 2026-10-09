#!/usr/bin/env python3
"""Meowler control panel — persistent ESP32 TCP :3333 + browser WebSocket."""

from __future__ import annotations

import atexit
import json
import re
import socket
import struct
import threading
import time
from pathlib import Path

from flask import Flask, jsonify, render_template, request
from flask_sock import Sock

from color_host import ColorFilter
from hardware_ready import (
    HW_TEST_PULSE_MS,
    clamp_hw_conveyor,
    clamp_hw_drive,
    is_motion_op,
    resolve_link_kind,
    run_preflight,
)
from meowler_pb import meowler_pb2 as pb
from robot_log import RobotLog
from rpm_format import (
    ARM_DEFAULTS,
    ARM_JOINTS,
    GRIP_MAX_DEG,
    REPLAYABLE_OPS,
    default_arm_pose,
    expand_arm_events,
    filter_replayable,
    load_rpm_bytes,
    sanitize_timeline,
    validate_conveyor,
    validate_drive,
    validate_joint,
    write_rpm_bytes,
)
from net_discover import (
    canonicalize_host,
    connect_hint,
    default_host_for_lan,
    discover as discover_robots,
    suggested_hosts,
)
from safety import (
    OpMode,
    allow_manual_motion,
    allow_record_start,
    allow_replay_start,
    mode_from_flags,
)
from ws_hub import WsHub

app = Flask(__name__)
sock = Sock(app)
_hub = WsHub()

_lock = threading.RLock()
_sock: socket.socket | None = None
_stop = threading.Event()
_reader: threading.Thread | None = None
_watchdog: threading.Thread | None = None
_keepalive: threading.Thread | None = None
_want_link = False
_link_host: str | None = None
_link_port = 3333
_reconnect_gate = threading.Event()
_conv_latch = 0  # latched belt speed; re-asserted while linked (survives soft stops)
_last_conv_reassert = 0.0
_log = RobotLog(maxlen=400)
_color = ColorFilter()
LOG_DIR = Path(__file__).resolve().parent / "logs"

_state: dict = {
    "connected": False,
    "mode": None,
    "op_mode": OpMode.DISCONNECTED.value,
    "link_kind": "DISCONNECTED",
    "dry_run": True,  # safe default until operator arms motors
    "origin_physical_verified": False,  # software cannot set True
    "origin_note": (
        "Arm origin = recorded joint angles only. "
        "Physical field pose cannot be verified — place robot on INITIAL mark manually."
    ),
    "host": None,
    "auto_reconnect": False,
    "reconnect_attempts": 0,
    "reconnect_note": None,
    "error": None,
    "warning": None,
    "fault": False,
    "estop": False,
    "base": 90,
    "height": 90,
    "grip": 90,
    "s13": 98,
    "s14": 90,
    "s15": 90,
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
    "color": 0,
    "color_name": "—",
    "color_conf": 0,
    "color_stable": False,
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
    "replay_paused": False,
    "ready_replay": False,
    "record_name": None,
    "record_events": 0,
    "record_duration_ms": 0,
    "replay_name": None,
    "replay_index": 0,
    "replay_total": 0,
    "replay_progress": 0.0,
    "last_telem_age_ms": None,
    "origin": None,
    "hw_test_active": False,
    "session_log": None,
    "tx_suppressed": 0,
}

DEFAULT_HOST = default_host_for_lan("192.168.137.222")
DEFAULT_PORT = 3333
MAX_FRAME = 4096
TELEM_SOFT_S = 3.0          # warn only (no latch / no command block)
TELEM_PROBE_S = 4.0         # get_telem ping before declaring dead
TELEM_DEAD_S = 8.0          # only then tear down + sticky reconnect
MAX_REC_EVENTS = 50_000
REC_DEDUP_S = 0.003         # 3ms — keep rapid retargets (was 15ms)
REC_DRIVE_KEYFRAME_S = 0.020  # resample held drive every 20ms for timing fidelity
RPM_DIR = Path(__file__).resolve().parent / "recordings"
_SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

_rec_lock = threading.Lock()
_recording = False
_rec_t0 = 0.0
_rec_events: list[dict] = []
_rec_name: str | None = None
_rec_origin: dict | None = None
_replaying = False
_replay_paused = False
_replay_stop = threading.Event()
_replay_pause_ev = threading.Event()
_replay_tx_lock = threading.Lock()
_replay_thread: threading.Thread | None = None
_replay_origin: dict | None = None
_last_telem_mono = 0.0
_last_cmd_sig: tuple | None = None
_last_cmd_mono = 0.0
_telem_fault = False
_last_color_log = (0, False)
_last_probe_mono = 0.0
_link_epoch = 0  # bumps on each new TCP session — ignore stale reader/watchdog


def _refresh_link_kind() -> None:
    _state["link_kind"] = resolve_link_kind(
        connected=bool(_state.get("connected")),
        host=_state.get("host"),
        dry_run=bool(_state.get("dry_run")),
    )


def _refresh_op_mode() -> None:
    _refresh_link_kind()
    _state["op_mode"] = mode_from_flags(
        connected=bool(_state.get("connected")),
        estop=bool(_state.get("estop")),
        fault=bool(_state.get("fault")),
        recording=_recording,
        replaying=_replaying,
        paused=_replay_paused,
        ready_replay=bool(_state.get("ready_replay")),
    ).value


def _session_log_path() -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    name = time.strftime("hw-%Y%m%d-%H%M%S.log")
    path = LOG_DIR / name
    _state["session_log"] = str(path)
    return path


def _log_hw(cat: str, msg: str) -> None:
    _log.add(cat, msg)
    path = _state.get("session_log")
    if not path:
        return
    try:
        with open(path, "a", encoding="utf-8") as f:
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            ms = int((time.time() % 1) * 1000)
            f.write(f"{stamp}.{ms:03d} [{cat}] {msg}\n")
    except OSError:
        pass


def _coerce_int(value, default: int = 0) -> int:
    """Accept int/float/numeric string; reject bool/garbage."""
    if isinstance(value, bool):
        raise ValueError("bool not allowed")
    if value is None:
        return int(default)
    return int(float(value))


def _set_fault(msg: str) -> None:
    _state["fault"] = True
    _state["error"] = msg
    _refresh_op_mode()


def _clear_fault() -> None:
    _state["fault"] = False
    if (_state.get("error") or "").startswith(("replay error", "EMERGENCY STOP", "fault:")):
        _state["error"] = None
    _refresh_op_mode()


def _normalize_host_port(host: str, port: int) -> tuple[str, int]:
    h = (host or "").strip()
    if not h:
        return DEFAULT_HOST, port
    if "://" in h:
        h = h.split("://", 1)[1]
    h = h.split("/", 1)[0].strip()
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


def _arm_pose_from_state() -> dict:
    def _joint(key: str) -> int:
        v = _state.get(key)
        return ARM_DEFAULTS.get(key, 90) if v is None else validate_joint(int(v), joint=key)

    return {k: _joint(k) for k in ARM_JOINTS}


def _arm_msg(pose: dict) -> pb.ClientToRobot:
    msg = pb.ClientToRobot()
    arm = msg.arm
    for key, set_attr in (
        ("base", "set_base"),
        ("height", "set_height"),
        ("grip", "set_grip"),
        ("s13", "set_s13"),
        ("s14", "set_s14"),
        ("s15", "set_s15"),
    ):
        setattr(arm, key, validate_joint(pose.get(key, 90), joint=key))
        setattr(arm, set_attr, True)
    return msg


def _stop_msg() -> pb.ClientToRobot:
    msg = pb.ClientToRobot()
    msg.stop = True
    return msg


def _msg_to_event(msg: pb.ClientToRobot) -> dict | None:
    which = msg.WhichOneof("op")
    if which == "drive":
        l, r = validate_drive(msg.drive.left, msg.drive.right)
        return {"op": "drive", "left": l, "right": r}
    if which == "arm":
        ev: dict = {"op": "arm"}
        for key, flag in (
            ("base", msg.arm.set_base),
            ("height", msg.arm.set_height),
            ("grip", msg.arm.set_grip),
            ("s13", msg.arm.set_s13),
            ("s14", msg.arm.set_s14),
            ("s15", msg.arm.set_s15),
        ):
            if flag:
                ev[key] = validate_joint(getattr(msg.arm, key), joint=key)
        return ev if len(ev) > 1 else None
    if which == "stop":
        return {"op": "stop"}
    if which == "center":
        return {"op": "center"}
    if which == "conveyor":
        return {"op": "conveyor", "speed": validate_conveyor(msg.conveyor.speed)}
    return None


def _event_to_msg(ev: dict) -> pb.ClientToRobot | None:
    op = ev.get("op")
    if op not in REPLAYABLE_OPS:
        return None
    msg = pb.ClientToRobot()
    if op == "drive":
        l, r = validate_drive(ev.get("left", 0), ev.get("right", 0))
        msg.drive.left = l
        msg.drive.right = r
        return msg
    if op == "arm":
        for key, flag in (
            ("base", "set_base"),
            ("height", "set_height"),
            ("grip", "set_grip"),
            ("s13", "set_s13"),
            ("s14", "set_s14"),
            ("s15", "set_s15"),
        ):
            if key in ev:
                setattr(msg.arm, key, validate_joint(ev[key], joint=key))
                setattr(msg.arm, flag, True)
        return msg
    if op == "stop":
        msg.stop = True
        return msg
    if op == "center":
        msg.center = True
        return msg
    if op == "conveyor":
        msg.conveyor.speed = validate_conveyor(ev.get("speed", 0))
        return msg
    return None


def _event_payload_eq(a: dict, b: dict) -> bool:
    if a.get("op") != b.get("op"):
        return False
    keys = ("left", "right", "base", "height", "grip", "s13", "s14", "s15", "speed")
    return all(a.get(k) == b.get(k) for k in keys)


def _event_sig(ev: dict) -> tuple:
    return (
        ev.get("op"),
        ev.get("left"),
        ev.get("right"),
        ev.get("base"),
        ev.get("height"),
        ev.get("grip"),
        ev.get("s13"),
        ev.get("s14"),
        ev.get("s15"),
        ev.get("speed"),
    )


def _rec_now_ms() -> int:
    """High-res host clock for recording (perf_counter, integer ms)."""
    if not _rec_t0:
        return 0
    return int(max(0, (time.perf_counter() - _rec_t0) * 1000))


def _host_record_append(ev: dict) -> None:
    """Authoritative host timeline — motion ops only, tight de-dup, host clock."""
    global _last_cmd_sig, _last_cmd_mono
    if ev.get("op") not in REPLAYABLE_OPS:
        return
    now = time.perf_counter()
    sig = _event_sig(ev)
    # Suppress identical spam within REC_DEDUP_S (bounce), not slower retargets
    if sig == _last_cmd_sig and (now - _last_cmd_mono) < REC_DEDUP_S:
        return
    with _rec_lock:
        if not _recording:
            return
        t_ms = int(max(0, (now - _rec_t0) * 1000))
        row = {"t_ms": t_ms, **{k: v for k, v in ev.items() if k != "t_ms"}}
        if _rec_events and _event_payload_eq(_rec_events[-1], row):
            last = _rec_events[-1]
            dt = t_ms - int(last.get("t_ms", 0))
            # Held non-zero drive: insert 20ms keyframes so replay timing stays exact
            if (
                row.get("op") == "drive"
                and (row.get("left") or row.get("right"))
                and dt >= int(REC_DRIVE_KEYFRAME_S * 1000)
            ):
                if len(_rec_events) >= MAX_REC_EVENTS:
                    _state["warning"] = f"recording capped at {MAX_REC_EVENTS} events"
                    return
                _rec_events.append(dict(row))
                _last_cmd_sig = sig
                _last_cmd_mono = now
                _state["record_events"] = len(_rec_events)
                _state["record_duration_ms"] = t_ms
                _hub.bump()
                return
            if row.get("op") == "drive" and (row.get("left") or row.get("right")):
                last["t_ms"] = t_ms
                _state["record_duration_ms"] = t_ms
            return
        if len(_rec_events) >= MAX_REC_EVENTS:
            _state["warning"] = f"recording capped at {MAX_REC_EVENTS} events"
            return
        _rec_events.append(row)
        _last_cmd_sig = sig
        _last_cmd_mono = now
        _state["record_events"] = len(_rec_events)
        _state["record_duration_ms"] = t_ms
    _hub.bump()
    _log.add("REC", f"+{ev.get('op')} t={t_ms}")


def _send(
    msg: pb.ClientToRobot,
    *,
    record: bool = True,
    force: bool = False,
) -> bool:
    global _sock
    which = msg.WhichOneof("op")
    if _replaying and not force and which not in ("stop", "rec"):
        _state["warning"] = "blocked during replay"
        return False

    # Dry-run: suppress actuator commands; always allow stop / rec / ota / color_cal
    if (
        _state.get("dry_run")
        and is_motion_op(which)
        and which != "stop"
    ):
        _state["tx_suppressed"] = int(_state.get("tx_suppressed") or 0) + 1
        _log_hw("DRY", f"suppress {which} (dry-run — no motor TX)")
        if record and _recording:
            ev = _msg_to_event(msg)
            if ev is not None:
                _host_record_append(ev)
        return True

    data = msg.SerializeToString()
    frame = struct.pack("<I", len(data)) + data
    with _lock:
        sock = _sock
    if sock is None:
        _state["error"] = "not connected"
        _state["connected"] = False
        _refresh_op_mode()
        return False
    try:
        sock.sendall(frame)
        if which != "rec":
            _state["error"] = None
        # Arm/drive spam must stay off the disk log — sync I/O was lagging TX
        if which not in ("arm", "drive", "conveyor", "get_telem"):
            kind = _state.get("link_kind") or "?"
            _log_hw("TX", f"{which or '?'} via {kind}")
        if record and _recording:
            ev = _msg_to_event(msg)
            if ev is not None:
                _host_record_append(ev)
        if which not in ("get_telem",):
            _hub.bump()
        return True
    except Exception as exc:  # noqa: BLE001
        _state["error"] = str(exc)
        _state["connected"] = False
        _log_hw("ERR", f"send failed: {exc}")
        with _lock:
            if _sock is not None:
                try:
                    _sock.close()
                except Exception:  # noqa: BLE001
                    pass
                _sock = None
        _on_link_lost("send failure")
        return False


def _write_rpm(name: str, events: list[dict]) -> Path:
    path = _rpm_path(name)
    clean = sanitize_timeline(events, max_events=MAX_REC_EVENTS)
    path.write_bytes(write_rpm_bytes(name, clean))
    return path


def _load_rpm(name: str) -> dict:
    path = _rpm_path(name)
    if not path.is_file():
        raise FileNotFoundError(name)
    data = load_rpm_bytes(path.read_bytes())
    data["events"] = sanitize_timeline(list(data.get("events") or []), max_events=MAX_REC_EVENTS)
    return data


def _read_rpm_header(path: Path) -> tuple[int, int, int, bytes]:
    raw = path.read_bytes()
    data = load_rpm_bytes(raw)
    return (
        int(data["version"]),
        int(data["created_unix"]),
        len(data["events"]),
        data["name"].encode("utf-8"),
    )


# MG90 settle timing
_SERVO_HOLD_S = 0.42
_SERVO_DEG_S = 0.0022
_SERVO_MIN_S = 0.28
_SERVO_MAX_S = 1.4


def _servo_settle_s(pose: dict, prev: dict) -> float:
    delta = max(
        *(
            abs(int(pose.get(k, 90)) - int(prev.get(k, pose.get(k, 90))))
            for k in ARM_JOINTS
        ),
        0,
    )
    if delta <= 0:
        return 0.0
    return max(_SERVO_MIN_S, min(_SERVO_MAX_S, _SERVO_HOLD_S + delta * _SERVO_DEG_S))


def _sleep_until(deadline: float, stop_ev: threading.Event) -> bool:
    while True:
        if stop_ev.is_set():
            return True
        while _replay_paused and not stop_ev.is_set():
            if stop_ev.wait(0.05):
                return True
        left = deadline - time.monotonic()
        if left <= 0:
            return False
        if stop_ev.wait(min(left, 0.05)):
            return True


def _block_while_paused(stop_ev: threading.Event) -> bool:
    """Hold between schedule wake and TX so pause/E-stop cannot race a drive send."""
    while _replay_paused and not stop_ev.is_set():
        if stop_ev.wait(0.05):
            return True
    return stop_ev.is_set()


def _restore_arm_origin(origin: dict | None, stop_ev: threading.Event) -> None:
    if not origin:
        return
    cur = _arm_pose_from_state()
    full = default_arm_pose()
    full.update({k: validate_joint(origin[k], joint=k) for k in ARM_JOINTS if k in origin})
    _send(_arm_msg(full), record=False, force=True)
    settle = _servo_settle_s(full, cur)
    if settle > 0:
        stop_ev.wait(settle)
    for k in ARM_JOINTS:
        _state[k] = full[k]
    _log.add(
        "REPLAY",
        f"restore arm {full['base']}/{full['height']}/{full['grip']} "
        f"+{full['s13']}/{full['s14']}/{full['s15']}",
    )


def _origin_from_events(events: list[dict], fallback: dict) -> dict:
    for ev in events:
        if ev.get("op") == "arm" and any(k in ev for k in ARM_JOINTS):
            pose = default_arm_pose()
            pose.update({k: validate_joint(fallback[k], joint=k) for k in ARM_JOINTS if k in fallback})
            pose.update({k: validate_joint(ev[k], joint=k) for k in ARM_JOINTS if k in ev})
            return pose
    out = default_arm_pose()
    out.update({k: validate_joint(fallback[k], joint=k) for k in ARM_JOINTS if k in fallback})
    return out


def _apply_arm_command(data: dict) -> tuple[pb.ClientToRobot | None, str | None]:
    """Build Arm protobuf from partial or full joint dict. Returns (msg, error)."""
    try:
        msg = pb.ClientToRobot()
        arm = msg.arm
        present = [k for k in ARM_JOINTS if data.get(k) is not None]
        if len(present) == 1:
            key = present[0]
            val = validate_joint(_coerce_int(data[key]), joint=key)
            setattr(arm, key, val)
            setattr(arm, f"set_{key}", True)
            _state[key] = val
        else:
            for key in ARM_JOINTS:
                raw = data[key] if data.get(key) is not None else _state.get(key, 90)
                val = validate_joint(_coerce_int(raw), joint=key)
                setattr(arm, key, val)
                setattr(arm, f"set_{key}", True)
                _state[key] = val
        _state["arm_cmd_mono"] = time.monotonic()
        return msg, None
    except (TypeError, ValueError) as exc:
        return None, f"invalid arm: {exc}"


def _run_timeline(
    events: list[dict],
    stop_ev: threading.Event,
    origin: dict | None = None,
) -> None:
    seed = origin or _arm_pose_from_state()
    filtered = sanitize_timeline(events, max_events=MAX_REC_EVENTS)
    expanded = expand_arm_events(filtered, seed)
    total = max(1, len(expanded))
    _state["replay_total"] = len(expanded)
    _state["replay_index"] = 0
    _state["replay_progress"] = 0.0

    # Critical: start from recorded origin before any motion
    _send(_stop_msg(), record=False, force=True)
    _restore_arm_origin(origin, stop_ev)
    if stop_ev.is_set():
        return

    t0 = time.monotonic()
    pause_offset = 0.0
    arm_free_at = t0
    arm_n = 0
    for i, ev in enumerate(expanded):
        if stop_ev.is_set():
            break
        if not _state.get("connected"):
            _log.add("ERR", "link lost mid-replay")
            stop_ev.set()
            break
        # Accumulate pause time so absolute schedule stays correct
        while _replay_paused and not stop_ev.is_set():
            paused_at = time.monotonic()
            if stop_ev.wait(0.05):
                break
            if _replay_paused:
                continue
            pause_offset += time.monotonic() - paused_at

        t_ms = int(ev.get("t_ms", 0))
        due = t0 + pause_offset + t_ms / 1000.0
        op = ev.get("op")
        _state["replay_index"] = i + 1
        _state["replay_progress"] = (i + 1) / total

        if op == "arm":
            if _sleep_until(max(due, arm_free_at), stop_ev):
                break
            if _block_while_paused(stop_ev):
                break
            pose = {
                "base": validate_joint(ev["base"], joint="base"),
                "height": validate_joint(ev["height"], joint="height"),
                "grip": validate_joint(ev["grip"], joint="grip"),
            }
            prev = ev.get("_prev") or pose
            if not _replay_send(_arm_msg(pose), stop_ev):
                if stop_ev.is_set() or _replay_paused:
                    if stop_ev.is_set():
                        break
                    continue
                stop_ev.set()
                break
            arm_free_at = time.monotonic() + _servo_settle_s(pose, prev)
            arm_n += 1
            _state["base"] = pose["base"]
            _state["height"] = pose["height"]
            _state["grip"] = pose["grip"]
            _log.add("REPLAY", f"arm {pose['base']}/{pose['height']}/{pose['grip']} @{t_ms}ms")
            continue

        if _sleep_until(due, stop_ev):
            break
        if _block_while_paused(stop_ev):
            break
        msg = _event_to_msg(ev)
        if msg is None:
            continue
        if not _replay_send(msg, stop_ev):
            if stop_ev.is_set() or _replay_paused:
                if stop_ev.is_set():
                    break
                continue
            stop_ev.set()
            break
        _log.add("REPLAY", f"{op} @{t_ms}ms")

    if arm_free_at > time.monotonic() and not stop_ev.is_set():
        _sleep_until(arm_free_at, stop_ev)
    _log.add("REPLAY", f"timeline arm={arm_n} total={len(expanded)}")
    _send(_stop_msg(), record=False, force=True)


def _replay_loop(events: list[dict], origin: dict, name: str) -> None:
    global _replaying, _replay_origin, _replay_paused
    try:
        _log.add(
            "REPLAY",
            f"start {name} n={len(events)} origin={origin['base']}/{origin['height']}/{origin['grip']}",
        )
        _run_timeline(events, _replay_stop, origin=origin)
        if _replay_stop.is_set():
            _send(_stop_msg(), record=False, force=True)
            _restore_arm_origin(origin, threading.Event())
            _log.add("REPLAY", "aborted — motors stopped, origin restore")
        else:
            _state["ready_replay"] = True
            _log.add("REPLAY", "complete")
    except Exception as exc:  # noqa: BLE001
        _set_fault(f"replay error: {exc}")
        _log.add("ERR", f"replay: {exc}")
        try:
            _send(_stop_msg(), record=False, force=True)
        except Exception:  # noqa: BLE001
            pass
    finally:
        with _rec_lock:
            _replaying = False
            _replay_paused = False
            _state["replaying"] = False
            _state["replay_paused"] = False
            _replay_origin = None
            _refresh_op_mode()
        _log.add("REPLAY", "idle")


def _apply_telem(t: pb.Telemetry) -> None:
    global _last_telem_mono, _telem_fault, _last_color_log
    _last_telem_mono = time.monotonic()
    _hub.bump()
    _telem_fault = False
    moving = abs(int(t.cmd_l)) > 8 or abs(int(t.cmd_r)) > 8
    _state["distance_mm"] = t.distance_mm
    _state["left"] = t.cmd_l
    _state["right"] = t.cmd_r
    _state["base"] = t.base
    _state["height"] = t.height
    _state["grip"] = t.grip
    _state["s13"] = getattr(t, "s13", _state.get("s13", 98))
    _state["s14"] = getattr(t, "s14", _state.get("s14", 90))
    _state["s15"] = getattr(t, "s15", _state.get("s15", 90))
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
    _state["color_rp"] = int(t.color_rp)
    _state["color_gp"] = int(t.color_gp)
    _state["color_bp"] = int(t.color_bp)
    _state["color_cp"] = int(t.color_cp)
    _state["conveyor"] = t.conveyor
    _state["last_rx"] = "telem"
    _state["last_telem_age_ms"] = 0

    # Live color only — never from recording; never crash on bad telem
    try:
        filt = _color.update(
            t.color,
            t.color_conf,
            t.color_r,
            t.color_b,  # yellow bar
            t.color_g,
            moving=moving,
            dist_mm=int(t.distance_mm) if t.distance_mm else None,
        )
        _state.update(filt)
        key = (int(filt.get("color") or 0), bool(filt.get("color_stable")))
        if key != _last_color_log and (key[1] or key[0] == 0):
            _last_color_log = key
            _log.add(
                "COLOR",
                f"{filt['color_name']} conf={filt['color_conf']} stable={int(filt['color_stable'])} "
                f"R={filt['color_r']} Y={filt['color_b']} G={filt['color_g']}",
            )
    except Exception as exc:  # noqa: BLE001
        _log.add("ERR", f"color filter: {exc}")
        _state["color"] = 0
        _state["color_name"] = "—"
        _state["color_stable"] = False


def _rec_event_to_dict(e) -> dict | None:
    op = int(e.op)
    t_ms = int(e.t_ms)
    if op == 1:
        return {"t_ms": t_ms, "op": "drive", "left": int(e.a), "right": int(e.b)}
    if op == 2:
        ev: dict = {"t_ms": t_ms, "op": "arm"}
        mask = int(e.mask)
        if mask & 1:
            ev["base"] = int(e.a)
        if mask & 2:
            ev["height"] = int(e.b)
        if mask & 4:
            ev["grip"] = int(e.c)
        if mask & 8:
            ev["s13"] = int(getattr(e, "d", 0))
        if mask & 16:
            ev["s14"] = int(getattr(e, "e", 0))
        if mask & 32:
            ev["s15"] = int(getattr(e, "f", 0))
        if mask == 0:
            ev["base"] = int(e.a)
            ev["height"] = int(e.b)
            ev["grip"] = int(e.c)
        return ev
    if op == 3:
        return {"t_ms": t_ms, "op": "stop"}
    if op == 4:
        return {"t_ms": t_ms, "op": "center"}
    if op == 6:
        return {"t_ms": t_ms, "op": "conveyor", "speed": int(e.a)}
    # op 5 zero / 7 motor_test intentionally ignored (not replayable)
    return None


def _handle_rec(e) -> None:
    """ESP RecEvent stream — gap-fill only; always remapped onto host clock."""
    global _rec_name
    op = int(e.op)
    with _rec_lock:
        if op == 0:
            nm = (e.name or "").strip() or _rec_name or "move"
            _rec_name = _safe_rpm_name(nm) or "move"
            _state["record_name"] = _rec_name
            _log.add("REC", f"ESP begin {_rec_name}")
            return
        if op == 255:
            _log.add("REC", "ESP end marker")
            return
        if not _recording or not _rec_t0:
            return
        ev = _rec_event_to_dict(e)
        if ev is None or ev.get("op") not in REPLAYABLE_OPS:
            return
        # Never trust ESP millis against host timeline — remap to perf_counter
        ev["t_ms"] = int(max(0, (time.perf_counter() - _rec_t0) * 1000))
        # Skip if host already captured the same payload recently
        for prev in _rec_events[-12:]:
            if _event_payload_eq(prev, ev):
                return
        if len(_rec_events) >= MAX_REC_EVENTS:
            return
        _rec_events.append(ev)
        _state["record_events"] = len(_rec_events)
        _state["record_duration_ms"] = int(ev["t_ms"])
    _hub.bump()


def _configure_tcp_sock(sock: socket.socket) -> None:
    """Persistent low-latency TCP: Nagle off + OS keepalive + larger buffers."""
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        pass
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except OSError:
        pass
    # Windows / Linux keepalive knobs (best-effort)
    for name, val in (
        ("TCP_KEEPIDLE", 5),
        ("TCP_KEEPINTVL", 1),
        ("TCP_KEEPCNT", 5),
    ):
        opt = getattr(socket, name, None)
        if opt is None:
            continue
        try:
            sock.setsockopt(socket.IPPROTO_TCP, opt, val)
        except OSError:
            pass
    if hasattr(socket, "SIO_KEEPALIVE_VALS"):
        try:
            # Windows: (on/off, idle_ms, interval_ms)
            sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 5000, 1000))  # type: ignore[attr-defined]
        except (OSError, AttributeError, ValueError):
            pass
    for opt, size in (
        (socket.SO_RCVBUF, 256 * 1024),
        (socket.SO_SNDBUF, 256 * 1024),
    ):
        try:
            sock.setsockopt(socket.SOL_SOCKET, opt, size)
        except OSError:
            pass
    sock.settimeout(0.05)


def _close_transport() -> None:
    """Tear down TCP reader/socket without clearing sticky reconnect intent."""
    global _sock, _reader, _watchdog, _link_epoch
    _stop.set()
    _link_epoch += 1
    if _reader and _reader.is_alive() and threading.current_thread() is not _reader:
        _reader.join(timeout=0.5)
    _reader = None
    _watchdog = None
    with _lock:
        if _sock is not None:
            try:
                _sock.shutdown(socket.SHUT_RDWR)
            except Exception:  # noqa: BLE001
                pass
            try:
                _sock.close()
            except Exception:  # noqa: BLE001
                pass
            _sock = None
        _state["connected"] = False
        _state["wifi_ok"] = False
        _state["mode"] = None
        _refresh_op_mode()
    _hub.bump()


def _on_link_lost(reason: str) -> None:
    global _recording, _replaying, _conv_latch
    _log.add("LINK", f"lost: {reason}")
    _replay_stop.set()
    # Safe stop attempt (may fail) — stop clears drive; belt needs explicit zero
    try:
        _send(_stop_msg(), record=False, force=True)
        conv = pb.ClientToRobot()
        conv.conveyor.speed = 0
        _send(conv, record=False, force=True)
    except Exception:  # noqa: BLE001
        pass
    _conv_latch = 0
    finish_name = None
    finish_events: list[dict] | None = None
    with _rec_lock:
        if _recording:
            finish_name = _rec_name or time.strftime("move-%Y%m%d-%H%M%S")
            finish_events = list(_rec_events)
            _recording = False
            _state["recording"] = False
        if _replaying:
            _state["error"] = f"disconnected during replay ({reason})"
    if finish_events is not None and finish_name:
        try:
            path = _write_rpm(finish_name, finish_events)
            _state["ready_replay"] = True
            _state["record_name"] = finish_name
            _log.add("REC", f"auto-saved {path.name} on disconnect n={len(finish_events)}")
        except Exception as exc:  # noqa: BLE001
            _log.add("ERR", f"auto-save failed: {exc}")
    _close_transport()
    if _want_link:
        with _lock:
            _state["auto_reconnect"] = True
            _state["warning"] = f"link lost ({reason}) — reconnecting…"
            _state["reconnect_note"] = reason
        _reconnect_gate.set()
        _ensure_keepalive()
    _refresh_op_mode()


def _ensure_keepalive() -> None:
    global _keepalive
    if _keepalive is not None and _keepalive.is_alive():
        return
    _keepalive = threading.Thread(target=_keepalive_loop, daemon=True, name="link-keep")
    _keepalive.start()


def _keepalive_loop() -> None:
    backoff = 1.0
    while True:
        # Wait until something asks us to retry, or poll slowly while sticky
        if not _want_link:
            _reconnect_gate.wait(timeout=1.0)
            _reconnect_gate.clear()
            backoff = 1.0
            with _lock:
                _state["auto_reconnect"] = False
                _state["reconnect_note"] = None
            continue
        if _state.get("connected") and _sock is not None:
            with _lock:
                _state["auto_reconnect"] = False
                note = _state.get("warning") or ""
                if note.startswith("link lost") or note.startswith("reconnecting"):
                    _state["warning"] = None
                _state["reconnect_note"] = None
            _reconnect_gate.wait(timeout=1.0)
            _reconnect_gate.clear()
            backoff = 1.0
            continue

        with _lock:
            _state["auto_reconnect"] = True
            _state["reconnect_note"] = f"scanning… (backoff {backoff:.0f}s)"

        hosts: list[str] = []
        if _link_host:
            hosts.append(canonicalize_host(_link_host))
        lan = default_host_for_lan("192.168.137.222")
        if lan not in hosts:
            hosts.append(lan)
        if "192.168.137.222" not in hosts:
            hosts.append("192.168.137.222")
        # Quick preferred-host probes before a full LAN sweep (cuts reconnect lag)
        port = _link_port or DEFAULT_PORT
        from net_discover import probe_tcp as _probe_tcp  # local import — avoid cycles

        for h in list(hosts):
            if _probe_tcp(h, port, timeout=1.25):
                break
        else:
            try:
                for hit in discover_robots(port=port):
                    h = hit["host"]
                    if h not in hosts:
                        hosts.append(h)
            except Exception:  # noqa: BLE001
                pass

        ok = False
        for h in hosts:
            if not _want_link:
                break
            try:
                connect_tcp(h, port, sticky=True)
                _log.add("LINK", f"RECONNECT {h}:{_link_port}")
                with _lock:
                    _state["auto_reconnect"] = False
                    _state["reconnect_note"] = None
                    _state["warning"] = None
                ok = True
                backoff = 1.0
                break
            except Exception as exc:  # noqa: BLE001
                with _lock:
                    _state["reconnect_attempts"] = int(_state.get("reconnect_attempts") or 0) + 1
                    _state["reconnect_note"] = f"{h}: {exc}"[:160]
        if ok:
            continue
        _reconnect_gate.wait(timeout=backoff)
        _reconnect_gate.clear()
        backoff = min(backoff * 1.5, 12.0)


def _handle_server(msg: pb.RobotToClient) -> None:
    which = msg.WhichOneof("msg")
    if which == "hello":
        h = msg.hello
        ip = socket.inet_ntoa(struct.pack("!I", h.ip & 0xFFFFFFFF))
        line = f"HELLO {ip}:{h.port} pca={int(h.pca_ok)} tof={int(h.tof_ok)} imu={int(h.imu_ok)}"
        _log.add("LINK", line)
        _state["last_rx"] = line
        _state["pca_ok"] = h.pca_ok
        _state["tof_ok"] = h.tof_ok
        _state["imu_ok"] = h.imu_ok
        _state["wifi_ok"] = True
        _state["connected"] = True
        _refresh_op_mode()
    elif which == "telem":
        _apply_telem(msg.telem)
    elif which == "ack":
        _state["ack"] = msg.ack
        if msg.ack != 0:
            _log.add("RX", f"ACK err={msg.ack}")
    elif which == "log":
        lvl = ("DBG", "INF", "WRN", "ERR")
        i = int(msg.log.level)
        tag = lvl[i] if 0 <= i < len(lvl) else str(i)
        line = f"{msg.log.text}"
        _log.add(tag if tag != "INF" else "RX", line)
        _state["last_rx"] = line
    elif which == "rec":
        _handle_rec(msg.rec)


def _reader_loop(epoch: int) -> None:
    buf = bytearray()
    while not _stop.is_set() and epoch == _link_epoch:
        with _lock:
            sock = _sock
        if sock is None:
            time.sleep(0.02)
            continue
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        except Exception as exc:  # noqa: BLE001
            if epoch != _link_epoch:
                return
            with _lock:
                _state["error"] = str(exc)
            _on_link_lost(str(exc))
            return
        if not chunk:
            if epoch != _link_epoch:
                return
            _on_link_lost("EOF")
            return
        buf.extend(chunk)
        while len(buf) >= 4:
            (n,) = struct.unpack_from("<I", buf, 0)
            if n == 0 or n > MAX_FRAME:
                buf.clear()
                _log.add("ERR", "bad frame length")
                break
            if len(buf) < 4 + n:
                break
            payload = bytes(buf[4 : 4 + n])
            del buf[: 4 + n]
            msg = pb.RobotToClient()
            try:
                msg.ParseFromString(payload)
            except Exception as exc:  # noqa: BLE001
                _log.add("ERR", f"protobuf decode: {exc}")
                continue
            with _lock:
                _handle_server(msg)


def _probe_telem() -> None:
    """Ask robot for telemetry without tearing down the persistent socket."""
    global _last_probe_mono
    now = time.monotonic()
    if now - _last_probe_mono < 0.4:
        return
    _last_probe_mono = now
    msg = pb.ClientToRobot()
    msg.get_telem = True
    _send(msg, record=False, force=True)


def _watchdog_loop(epoch: int) -> None:
    """Keep the TCP session alive — probe before reconnect (no flap)."""
    global _telem_fault, _last_conv_reassert
    while not _stop.is_set() and epoch == _link_epoch:
        time.sleep(0.2)
        if not _state.get("connected") or _sock is None:
            _telem_fault = False
            continue
        # Re-assert latched conveyor so PCA/I2C glitches don't leave the belt idle
        if _conv_latch != 0:
            now_c = time.monotonic()
            if now_c - _last_conv_reassert >= 0.22:
                _last_conv_reassert = now_c
                cmsg = pb.ClientToRobot()
                cmsg.conveyor.speed = int(_conv_latch)
                _send(cmsg, record=False, force=True)
        if _last_telem_mono <= 0:
            continue
        age = time.monotonic() - _last_telem_mono
        _state["last_telem_age_ms"] = int(age * 1000)
        if age > TELEM_PROBE_S:
            _probe_telem()
        if age > TELEM_DEAD_S:
            _log.add("WARN", f"telem dead {age:.1f}s after probes — drop link")
            _on_link_lost(f"telemetry dead {age:.1f}s")
            return
        if age > TELEM_SOFT_S:
            if not _telem_fault:
                _telem_fault = True
                _log.add("WARN", f"telem soft-timeout {age:.1f}s (link held — no E-stop)")
                _state["warning"] = f"telemetry timeout {age:.1f}s"
                _hub.bump()
        else:
            if _telem_fault:
                _telem_fault = False
                if (_state.get("warning") or "").startswith("telemetry timeout"):
                    _state["warning"] = None


def connect_tcp(host: str, port: int = DEFAULT_PORT, *, sticky: bool = True) -> None:
    """Open (or reuse) a persistent framed-protobuf TCP session to the robot."""
    global _sock, _reader, _watchdog, _last_telem_mono, _want_link, _link_host, _link_port
    global _link_epoch
    host = canonicalize_host(host)
    # Already on the right endpoint — keep the socket (no reconnect flap)
    with _lock:
        same = (
            _sock is not None
            and bool(_state.get("connected"))
            and _link_host == host
            and int(_link_port or 0) == int(port)
        )
    if same:
        _want_link = bool(sticky)
        if sticky:
            _ensure_keepalive()
        _log.add("LINK", f"KEEP {host}:{port} (persistent TCP already up)")
        _hub.bump()
        return

    _close_transport()
    if not _state.get("session_log"):
        _session_log_path()
    try:
        # 3s first — fail over to discover faster; sticky retry uses probe+longer path
        sock = socket.create_connection((host, port), timeout=3.0)
    except (TimeoutError, socket.timeout, OSError):
        try:
            sock = socket.create_connection((host, port), timeout=5.0)
        except (TimeoutError, socket.timeout, OSError) as exc:
            raise TimeoutError(connect_hint(f"{host}")) from exc
    _configure_tcp_sock(sock)
    _want_link = bool(sticky)
    _link_host = host
    _link_port = port
    _link_epoch += 1
    epoch = _link_epoch
    with _lock:
        _sock = sock
        _state["connected"] = True
        _state["mode"] = "protobuf-tcp"
        _state["host"] = f"{host}:{port}"
        _state["error"] = None
        _state["warning"] = None
        _state["fault"] = False
        _state["estop"] = False
        _state["wifi_ok"] = True
        _state["auto_reconnect"] = False
        _state["reconnect_note"] = None
        _state["origin_physical_verified"] = False
        _state["tx_suppressed"] = 0
        _last_telem_mono = time.monotonic()
        _color.reset()
        _refresh_op_mode()
    _stop.clear()
    _reader = threading.Thread(target=_reader_loop, args=(epoch,), daemon=True, name="pb-reader")
    _reader.start()
    _watchdog = threading.Thread(target=_watchdog_loop, args=(epoch,), daemon=True, name="watchdog")
    _watchdog.start()
    if sticky:
        _ensure_keepalive()
    # Motors must start stopped on every link-up
    _force_all_stop()
    _hub.bump()
    _log_hw(
        "LINK",
        f"CONNECT {host}:{port} kind={_state.get('link_kind')} dry_run={int(bool(_state.get('dry_run')))} sticky={int(sticky)} persistent=1",
    )


def disconnect(*, sticky: bool = False) -> None:
    """Close link. sticky=False (default) cancels auto-reconnect — intentional hang-up."""
    global _want_link
    if not sticky:
        _want_link = False
        with _lock:
            _state["auto_reconnect"] = False
            _state["reconnect_note"] = None
    _replay_stop.set()
    try:
        # Temporarily allow stop TX even if dry-run (stop is never suppressed)
        _force_all_stop()
    except Exception:  # noqa: BLE001
        pass
    _close_transport()
    with _lock:
        if not sticky:
            _state["host"] = None
        _state["hw_test_active"] = False
        _refresh_op_mode()
    _log_hw("LINK", "DISCONNECT — stop commanded" + (" (sticky)" if sticky else ""))


atexit.register(lambda: disconnect(sticky=False))


def _force_all_stop(*, _locked: bool = False) -> None:
    """Best-effort: stop flag + explicit zero drive + zero conveyor."""
    global _conv_latch

    def _body() -> None:
        global _conv_latch
        _send(_stop_msg(), record=False, force=True)
        drive = pb.ClientToRobot()
        drive.drive.left = 0
        drive.drive.right = 0
        _send(drive, record=False, force=True)
        conv = pb.ClientToRobot()
        conv.conveyor.speed = 0
        _send(conv, record=False, force=True)
        _send(_stop_msg(), record=False, force=True)
        _state["left"] = 0
        _state["right"] = 0
        _state["conveyor"] = 0
        _conv_latch = 0

    if _locked:
        _body()
    else:
        with _replay_tx_lock:
            _body()


def _replay_send(msg: pb.ClientToRobot, stop_ev: threading.Event) -> bool:
    """Send during replay only if not paused/stopped (atomic with pause/E-stop)."""
    with _replay_tx_lock:
        if stop_ev.is_set() or _replay_paused:
            return False
        return _send(msg, record=False, force=True)


def _do_estop(reason: str = "user") -> None:
    """Hard stop only — no latch, no command lock (E-stop mode removed)."""
    global _recording, _replay_paused
    _state["estop"] = False
    _replay_stop.set()
    _replay_paused = False
    _state["replay_paused"] = False
    _force_all_stop()
    with _rec_lock:
        if _recording:
            name = _rec_name or time.strftime("move-%Y%m%d-%H%M%S")
            events = list(_rec_events)
            _recording = False
            _state["recording"] = False
            try:
                path = _write_rpm(name, events)
                _state["ready_replay"] = True
                _log.add("REC", f"saved on hard-stop {path.name}")
            except Exception as exc:  # noqa: BLE001
                _log.add("ERR", f"hard-stop save: {exc}")
    _log.add("STOP", f"hard-stop ({reason})")
    if (_state.get("error") or "").startswith("EMERGENCY STOP"):
        _state["error"] = None
    _refresh_op_mode()
    _hub.bump()


def _gate_motion() -> tuple[bool, str | None]:
    if not allow_manual_motion(
        connected=bool(_state.get("connected")),
        estop=False,
        replaying=_replaying,
    ):
        if _replaying:
            return False, "replaying"
        return False, "not connected"
    return True, None


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/api/ports")
def api_ports():
    host = default_host_for_lan(DEFAULT_HOST)
    return jsonify(
        {
            "default_host": host,
            "default_port": DEFAULT_PORT,
            "suggested_hosts": suggested_hosts(),
            "proto": "protobuf",
        }
    )


@app.get("/api/discover")
def api_discover():
    try:
        port = int(request.args.get("port") or DEFAULT_PORT)
    except (TypeError, ValueError):
        port = DEFAULT_PORT
    hits = discover_robots(port=port)
    return jsonify(
        {
            "ok": True,
            "port": port,
            "hosts": hits,
            "suggested_hosts": suggested_hosts(),
            "default_host": default_host_for_lan(DEFAULT_HOST),
        }
    )


def _snapshot_state() -> dict:
    with _lock:
        with _rec_lock:
            if _recording and _rec_t0:
                _state["record_duration_ms"] = _rec_now_ms()
        _refresh_op_mode()
        payload = dict(_state)
        payload["logs"] = _log.lines(48)
        payload["sticky_link"] = bool(_want_link)
        payload["ws_clients"] = _hub.client_count
        payload["robot_transport"] = "persistent-tcp"
        payload["ui_transport"] = "websocket" if _hub.client_count else "http"
    return payload


@app.get("/api/state")
def api_state():
    return jsonify(_snapshot_state())


@app.post("/api/connect")
def api_connect():
    data = request.get_json(force=True, silent=True) or {}
    raw_host = data.get("host")
    if raw_host is None or str(raw_host).strip() == "":
        raw_host = default_host_for_lan(DEFAULT_HOST)
    try:
        port = int(data.get("port") or DEFAULT_PORT)
    except (TypeError, ValueError):
        port = DEFAULT_PORT
    host, port = _normalize_host_port(str(raw_host), port)
    # Hotspot DHCP leftovers (.140) → static *.222 before first attempt
    host = canonicalize_host(host)
    auto = bool(data.get("auto_discover", True))
    # Always try known HSP static first — UI often still has a stale host cached
    preferred = ["192.168.137.222", default_host_for_lan(DEFAULT_HOST), host]
    ordered: list[str] = []
    for h in preferred:
        h = canonicalize_host(h)
        if h and h not in ordered:
            ordered.append(h)
    tried: list[str] = []
    last_exc: Exception | None = None
    for cand in ordered:
        tried.append(cand)
        try:
            connect_tcp(cand, port)
            _log.add("LINK", f"CONNECT {cand}:{port}")
            return jsonify(
                {
                    "ok": True,
                    "mode": "protobuf-tcp",
                    "host": cand,
                    "port": port,
                    "discovered": cand != host,
                }
            )
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            _log.add("ERR", f"CONNECT FAIL {cand}:{port} — {exc}")
    if auto:
        try:
            for hit in discover_robots(port=port):
                alt = canonicalize_host(hit["host"])
                if alt in tried:
                    continue
                tried.append(alt)
                try:
                    connect_tcp(alt, port)
                    _log.add("LINK", f"CONNECT {alt}:{port} (auto-discover)")
                    return jsonify(
                        {
                            "ok": True,
                            "mode": "protobuf-tcp",
                            "host": alt,
                            "port": port,
                            "discovered": True,
                        }
                    )
                except Exception as exc:  # noqa: BLE001
                    last_exc = exc
                    continue
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
    err = str(last_exc) if last_exc else connect_hint(host)
    return (
        jsonify(
            {
                "ok": False,
                "error": err,
                "host": host,
                "port": port,
                "tried": tried,
                "discovered_hosts": [],
                "suggested_hosts": suggested_hosts(),
            }
        ),
        500,
    )


@app.post("/api/disconnect")
def api_disconnect():
    disconnect(sticky=False)
    _log.add("LINK", "DISCONNECT")
    return jsonify({"ok": True, "auto_reconnect": False})


@app.post("/api/reconnect")
def api_reconnect():
    """Force an immediate sticky reconnect attempt (keeps auto-reconnect armed)."""
    global _want_link, _link_host, _link_port
    data = request.get_json(force=True, silent=True) or {}
    raw = data.get("host") or _link_host or default_host_for_lan(DEFAULT_HOST)
    try:
        port = int(data.get("port") or _link_port or DEFAULT_PORT)
    except (TypeError, ValueError):
        port = DEFAULT_PORT
    host, port = _normalize_host_port(str(raw), port)
    host = canonicalize_host(host)
    _want_link = True
    _link_host = host
    _link_port = port
    with _lock:
        _state["auto_reconnect"] = True
        _state["reconnect_note"] = "manual reconnect"
        _state["host"] = f"{host}:{port}"
        _state["reconnect_attempts"] = int(_state.get("reconnect_attempts") or 0)
    _ensure_keepalive()
    _reconnect_gate.set()
    try:
        connect_tcp(host, port, sticky=True)
        return jsonify({"ok": True, "host": host, "port": port, "sticky_link": True})
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc), "host": host, "port": port, "auto_reconnect": True, "sticky_link": True}), 500


@app.post("/api/sticky")
def api_sticky():
    """Arm or cancel sticky auto-reconnect without requiring a live socket."""
    global _want_link, _link_host, _link_port
    data = request.get_json(force=True, silent=True) or {}
    enabled = bool(data.get("enabled"))
    if "host" in data and str(data.get("host") or "").strip():
        host, port = _normalize_host_port(str(data.get("host")), int(data.get("port") or _link_port or DEFAULT_PORT))
        _link_host = canonicalize_host(host)
        _link_port = port
    _want_link = enabled
    with _lock:
        _state["auto_reconnect"] = bool(enabled and not _state.get("connected"))
        if enabled:
            _state["reconnect_note"] = _state.get("reconnect_note") or "sticky armed"
            if _link_host:
                _state["host"] = f"{_link_host}:{_link_port or DEFAULT_PORT}"
        else:
            _state["reconnect_note"] = None
            _state["auto_reconnect"] = False
    if enabled:
        _ensure_keepalive()
        _reconnect_gate.set()
    _log.add("LINK", f"STICKY {'on' if enabled else 'off'} host={_link_host}:{_link_port}")
    return jsonify(
        {
            "ok": True,
            "sticky_link": enabled,
            "auto_reconnect": bool(_state.get("auto_reconnect")),
            "host": _link_host,
            "port": _link_port,
        }
    )


@app.post("/api/estop")
def api_estop():
    """Hard-stop from UI chord S+O only — never latches / never blocks later cmds."""
    _do_estop(reason="chord-SO")
    return jsonify({"ok": True, "op_mode": _state["op_mode"], "estop": False})


@app.post("/api/estop/clear")
def api_estop_clear():
    """No-op — there is no latch to clear."""
    _state["estop"] = False
    _refresh_op_mode()
    return jsonify({"ok": True, "op_mode": _state["op_mode"], "estop": False})


@app.post("/api/arm")
def api_arm():
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    data = request.get_json(force=True, silent=True) or {}
    msg, aerr = _apply_arm_command(data)
    if aerr or msg is None:
        return jsonify({"ok": False, "error": aerr or "bad arm"}), 400
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/center")
def api_center():
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    msg = pb.ClientToRobot()
    msg.center = True
    for k, v in default_arm_pose().items():
        _state[k] = v
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/drive")
def api_drive():
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    data = request.get_json(force=True, silent=True) or {}
    try:
        l, r = validate_drive(_coerce_int(data.get("left", 0)), _coerce_int(data.get("right", 0)))
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": f"invalid drive: {exc}"}), 400
    msg = pb.ClientToRobot()
    msg.drive.left = l
    msg.drive.right = r
    _state["left"], _state["right"] = l, r
    return jsonify({"ok": _send(msg), "error": _state.get("error")})


@app.post("/api/stop")
def api_stop():
    msg = pb.ClientToRobot()
    msg.stop = True
    return jsonify({"ok": _send(msg, force=True), "error": _state.get("error")})


@app.post("/api/motor_test")
def api_motor_test():
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    msg = pb.ClientToRobot()
    msg.motor_test = True
    return jsonify({"ok": _send(msg, record=False), "error": _state.get("error")})


@app.post("/api/zero")
def api_zero():
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    msg = pb.ClientToRobot()
    msg.zero = True
    return jsonify({"ok": _send(msg, record=False), "error": _state.get("error")})


@app.post("/api/conveyor")
def api_conveyor():
    global _conv_latch, _last_conv_reassert
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    data = request.get_json(force=True, silent=True) or {}
    try:
        speed = validate_conveyor(_coerce_int(data.get("speed", 0)))
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": f"invalid conveyor: {exc}"}), 400
    msg = pb.ClientToRobot()
    msg.conveyor.speed = speed
    ok = _send(msg)
    if ok:
        _state["conveyor"] = speed
        _conv_latch = speed
        _last_conv_reassert = time.monotonic()
    return jsonify({"ok": ok, "speed": speed, "error": _state.get("error")})


@app.post("/api/color_cal")
def api_color_cal():
    ok_gate, err = _gate_motion()
    if not ok_gate:
        return jsonify({"ok": False, "error": err}), 409
    data = request.get_json(force=True, silent=True) or {}
    mode = int(data.get("mode", 0))
    msg = pb.ClientToRobot()
    msg.color_cal.mode = mode
    ok = _send(msg, record=False)
    names = {
        0: "white-balance",
        1: "teach RED",
        2: "teach YELLOW",
        3: "teach GREEN",
        4: "clear teach",
        5: "factory reset",
    }
    _log.add("COLOR", f"CAL {names.get(mode, mode)} ok={int(ok)}")
    _color.reset()
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
            "paused": _replay_paused,
            "name": _rec_name,
            "events": len(_rec_events) if _recording else _state.get("record_events", 0),
            "duration_ms": _state.get("record_duration_ms", 0),
            "files": _list_rpms(),
            "op_mode": _state.get("op_mode"),
        })


def _send_rec(action: int, name: str = "") -> bool:
    msg = pb.ClientToRobot()
    msg.rec.action = int(action)
    if name:
        msg.rec.name = name[:31]
    return _send(msg, record=False, force=True)


@app.post("/api/record/start")
def api_record_start():
    global _recording, _rec_t0, _rec_events, _rec_name, _rec_origin, _last_cmd_sig, _last_cmd_mono
    data = request.get_json(force=True, silent=True) or {}
    name = _safe_rpm_name(str(data.get("name") or time.strftime("move-%Y%m%d-%H%M%S")))
    if not name:
        return jsonify({"ok": False, "error": "bad name"}), 400

    pose = _arm_pose_from_state()
    # Claim recording under lock first (blocks double-start races)
    with _rec_lock:
        err = allow_record_start(
            connected=bool(_state.get("connected")),
            estop=bool(_state.get("estop")),
            recording=_recording,
            replaying=_replaying,
        )
        if err:
            return jsonify({"ok": False, "error": err}), 409
        _recording = True
        _state["recording"] = True
        _rec_name = name
        _rec_t0 = time.perf_counter()
        _last_cmd_sig = None
        _last_cmd_mono = 0.0
        _rec_origin = dict(pose)
        _rec_events = [{"t_ms": 0, "op": "arm", **{k: pose[k] for k in ARM_JOINTS}}]
        _state["record_name"] = name
        _state["record_events"] = 1
        _state["record_duration_ms"] = 0
        _state["ready_replay"] = False
        _state["origin"] = dict(pose)
        _state["warning"] = None
        _refresh_op_mode()

    ok = _send_rec(1, name)
    if not ok:
        with _rec_lock:
            _recording = False
            _state["recording"] = False
            _rec_events = []
            _refresh_op_mode()
        return jsonify({"ok": False, "error": _state.get("error") or "send failed"}), 502

    _log.add("REC", f"start {name}.rpm origin={pose['base']}/{pose['height']}/{pose['grip']}")
    return jsonify({"ok": True, "name": name, "origin": pose})


@app.post("/api/record/stop")
def api_record_stop():
    global _recording, _rec_events, _rec_name, _rec_origin
    with _rec_lock:
        if not _recording:
            return jsonify({"ok": False, "error": "not recording"}), 400
        name = _rec_name or time.strftime("move-%Y%m%d-%H%M%S")
        events = list(_rec_events)
        origin = dict(_rec_origin) if _rec_origin else _arm_pose_from_state()
        _recording = False
        _state["recording"] = False

    _send_rec(2, name)
    # Ensure stop recorded / motors off
    _send(_stop_msg(), record=False, force=True)

    clean = sanitize_timeline(events, max_events=MAX_REC_EVENTS)
    path = _write_rpm(name, clean)
    n = len(clean)
    _state["record_events"] = n
    _state["record_name"] = name
    _state["ready_replay"] = n > 0
    _state["origin"] = origin

    # Return robot to INITIAL origin after recording
    _restore_arm_origin(origin, threading.Event())
    _refresh_op_mode()
    _log.add("REC", f"saved {path.name} events={n}; restored origin")
    return jsonify({
        "ok": True,
        "name": name,
        "events": n,
        "file": path.name,
        "origin": origin,
    })


@app.get("/api/record/list")
def api_record_list():
    return jsonify({"files": _list_rpms()})


@app.post("/api/replay")
def api_replay():
    global _replaying, _replay_thread, _replay_origin, _replay_paused
    data = request.get_json(force=True, silent=True) or {}
    name = _safe_rpm_name(str(data.get("name") or ""))
    if not name:
        return jsonify({"ok": False, "error": "need name"}), 400
    confirm = bool(data.get("confirm", False))
    if not confirm:
        return jsonify({"ok": False, "error": "confirm required"}), 400
    # Operator must acknowledge physical placement (software cannot verify)
    if not bool(data.get("origin_ack", False)):
        return jsonify({
            "ok": False,
            "error": "origin_ack required — place robot on INITIAL field mark, then confirm",
        }), 400

    try:
        rpm = _load_rpm(name)
    except FileNotFoundError:
        return jsonify({"ok": False, "error": "missing"}), 404
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": f"invalid recording: {exc}"}), 400

    events = sanitize_timeline(list(rpm.get("events") or []), max_events=MAX_REC_EVENTS)
    if not events:
        return jsonify({"ok": False, "error": "empty / no replayable events"}), 400

    # Pre-flight — block autonomous replay on critical FAIL
    pf = run_preflight(_state, events=events)
    if not pf["ok_to_replay"]:
        _log_hw("PREFLT", f"BLOCKED critical_fail checks={[c['id'] for c in pf['checks'] if c['status']=='FAIL' and c['critical']]}")
        return jsonify({"ok": False, "error": "preflight failed", "preflight": pf}), 409

    if _state.get("link_kind") == "SIMULATED" and not bool(data.get("allow_sim", False)):
        return jsonify({
            "ok": False,
            "error": "SIMULATED link — pass allow_sim=true to run mock replay (not real hardware)",
            "link_kind": "SIMULATED",
        }), 409

    origin = _origin_from_events(events, _arm_pose_from_state())
    _state["origin"] = dict(origin)
    _log_hw(
        "REPLAY",
        f"preflight OK kind={_state.get('link_kind')} dry_run={int(bool(_state.get('dry_run')))} "
        f"events={len(events)} origin_ack=1 physical_pose=UNVERIFIED",
    )
    with _rec_lock:
        err = allow_replay_start(
            connected=bool(_state.get("connected")),
            estop=bool(_state.get("estop")),
            recording=_recording,
            replaying=_replaying,
            confirm=True,
        )
        if err:
            return jsonify({"ok": False, "error": err}), 409
        _replay_origin = origin
        _replay_stop.clear()
        _replay_paused = False
        _replaying = True
        _state["replaying"] = True
        _state["replay_paused"] = False
        _state["replay_name"] = name
        _state["ready_replay"] = False
        _state["replay_index"] = 0
        _state["replay_total"] = len(events)
        _state["replay_progress"] = 0.0
        _state["fault"] = False
        _refresh_op_mode()
        _replay_thread = threading.Thread(
            target=_replay_loop, args=(events, origin, name), daemon=True, name="replay"
        )
        _replay_thread.start()
    _log.add("REPLAY", f"queued {name}.rpm events={len(events)}")
    return jsonify({"ok": True, "name": name, "events": len(events), "origin": origin})


@app.post("/api/replay/stop")
def api_replay_stop():
    global _replay_paused
    _replay_stop.set()
    _replay_paused = False
    _state["replay_paused"] = False
    _send(_stop_msg(), record=False, force=True)
    _log.add("REPLAY", "stop requested")
    return jsonify({"ok": True})


@app.post("/api/replay/pause")
def api_replay_pause():
    global _replay_paused
    data = request.get_json(force=True, silent=True) or {}
    pause = bool(data.get("pause", True))
    with _rec_lock:
        if not _replaying:
            return jsonify({"ok": False, "error": "not replaying"}), 400
        with _replay_tx_lock:
            _replay_paused = pause
            _state["replay_paused"] = pause
            if pause:
                _force_all_stop(_locked=True)
        _refresh_op_mode()
    _log.add("REPLAY", "paused" if pause else "resumed")
    return jsonify({"ok": True, "paused": pause})


@app.post("/api/ota")
def api_ota():
    data = request.get_json(force=True, silent=True) or {}
    action = int(data.get("action", 6))
    if action not in (6, 7, 8):
        return jsonify({"ok": False, "error": "use ota_upload.py for begin/chunk/finish/apply"}), 400
    msg = pb.ClientToRobot()
    msg.ota.action = action
    ok = _send(msg, record=False, force=True)
    _log.add("OTA", f"action={action} ok={int(ok)}")
    return jsonify({"ok": ok, "action": action, "error": _state.get("error")})


@app.get("/api/preflight")
def api_preflight():
    name = _safe_rpm_name(str(request.args.get("name") or _state.get("record_name") or ""))
    events: list[dict] = []
    if name:
        try:
            events = list(_load_rpm(name).get("events") or [])
        except Exception as exc:  # noqa: BLE001
            return jsonify({"ok_to_replay": False, "error": str(exc), "checks": []}), 400
    report = run_preflight(_state, events=events)
    _log_hw("PREFLT", f"ok_to_replay={report['ok_to_replay']} warnings={report['warning_count']}")
    return jsonify(report)


@app.post("/api/dry_run")
def api_dry_run():
    data = request.get_json(force=True, silent=True) or {}
    if "enabled" not in data:
        return jsonify({"ok": False, "error": "need enabled bool"}), 400
    enabled = bool(data.get("enabled"))
    if not enabled and _state.get("link_kind") == "SIMULATED":
        # turning off dry-run while on localhost still isn't real hardware
        pass
    if not enabled and not _state.get("connected"):
        return jsonify({"ok": False, "error": "connect first before arming motors"}), 400
    if not enabled:
        # Arming motors — require explicit ack
        if not bool(data.get("arm_ack", False)):
            return jsonify({
                "ok": False,
                "error": "arm_ack required to disable dry-run (motors will move)",
            }), 400
        _force_all_stop()
    _state["dry_run"] = enabled
    _refresh_op_mode()
    _log_hw("SAFE", f"dry_run={int(enabled)} link_kind={_state.get('link_kind')}")
    return jsonify({
        "ok": True,
        "dry_run": enabled,
        "link_kind": _state.get("link_kind"),
    })


@app.get("/api/readiness")
def api_readiness():
    """Audit snapshot for hardware-test briefings — never claims physical success."""
    return jsonify({
        "link_kind": _state.get("link_kind"),
        "dry_run": bool(_state.get("dry_run")),
        "connected": bool(_state.get("connected")),
        "host": _state.get("host"),
        "origin_physical_verified": False,
        "origin_note": _state.get("origin_note"),
        "session_log": _state.get("session_log"),
        "tx_suppressed": _state.get("tx_suppressed"),
        "color_independent": True,
        "limits": {
            "drive": 255,
            "hw_test_drive": 120,
            "hw_test_conveyor": 100,
            "hw_test_pulse_ms": HW_TEST_PULSE_MS,
            "telem_soft_s": TELEM_SOFT_S,
            "telem_dead_s": TELEM_DEAD_S,
        },
        "comm_path": (
            "UI WebSocket -> panel; panel persistent TCP :3333 -> ESP32 "
            "(length-prefixed protobuf ClientToRobot / RobotToClient)"
        ),
    })


def _hw_pulse_drive(left: int, right: int, pulse_ms: int) -> None:
    def _run() -> None:
        _state["hw_test_active"] = True
        try:
            msg = pb.ClientToRobot()
            msg.drive.left = left
            msg.drive.right = right
            _send(msg, record=False)
            time.sleep(max(0.05, pulse_ms / 1000.0))
        finally:
            _force_all_stop()
            _state["hw_test_active"] = False
            _log_hw("HWTEST", f"pulse done L={left} R={right}")

    threading.Thread(target=_run, daemon=True, name="hw-pulse").start()


@app.post("/api/hw_test")
def api_hw_test():
    """Controlled single-action hardware bench tests with hard speed/time caps."""
    data = request.get_json(force=True, silent=True) or {}
    action = str(data.get("action") or "").strip().lower()
    if not _state.get("connected"):
        return jsonify({"ok": False, "error": "not connected"}), 400
    if _recording or _replaying:
        return jsonify({"ok": False, "error": "busy recording/replaying"}), 409

    pulse = int(data.get("pulse_ms") or HW_TEST_PULSE_MS)
    pulse = max(50, min(800, pulse))
    spd = abs(int(data.get("speed") or 90))
    left, right = 0, 0

    if action == "connection":
        age = _state.get("last_telem_age_ms")
        ok = bool(_state.get("connected")) and age is not None and int(age) < 1500
        _log_hw("HWTEST", f"connection ok={ok} age={age} kind={_state.get('link_kind')}")
        return jsonify({
            "ok": ok,
            "link_kind": _state.get("link_kind"),
            "dry_run": _state.get("dry_run"),
            "telem_age_ms": age,
            "note": "PASS here means link+telem only — not proof of motors",
        })

    if action == "color":
        _log_hw(
            "HWTEST",
            f"color name={_state.get('color_name')} conf={_state.get('color_conf')} "
            f"stable={_state.get('color_stable')} R={_state.get('color_r')} "
            f"Y={_state.get('color_b')} G={_state.get('color_g')}",
        )
        return jsonify({
            "ok": True,
            "color": _state.get("color"),
            "color_name": _state.get("color_name"),
            "color_conf": _state.get("color_conf"),
            "color_stable": _state.get("color_stable"),
            "bars": {
                "r": _state.get("color_r"),
                "y": _state.get("color_b"),
                "g": _state.get("color_g"),
            },
            "independent_of_recording": True,
        })

    if action == "stop" or action == "estop":
        # "estop" alias kept for old clients — same as stop (no latch)
        _force_all_stop()
        return jsonify({"ok": True, "action": action, "link_kind": _state.get("link_kind")})

    if action in ("forward", "backward", "left", "right", "drive"):
        if action == "forward":
            left = right = spd
        elif action == "backward":
            left = right = -spd
        elif action == "left":
            left, right = -spd, spd
        elif action == "right":
            left, right = spd, -spd
        else:
            left, right = int(data.get("left", 0)), int(data.get("right", 0))
        left, right = clamp_hw_drive(left, right)
        _log_hw(
            "HWTEST",
            f"{action} L={left} R={right} pulse={pulse}ms dry_run={int(bool(_state.get('dry_run')))} "
            f"kind={_state.get('link_kind')}",
        )
        _hw_pulse_drive(left, right, pulse)
        return jsonify({
            "ok": True,
            "action": action,
            "left": left,
            "right": right,
            "pulse_ms": pulse,
            "dry_run": _state.get("dry_run"),
            "link_kind": _state.get("link_kind"),
            "note": "DRY_RUN suppresses motor TX" if _state.get("dry_run") else "REAL pulse sent",
        })

    if action == "conveyor":
        speed = clamp_hw_conveyor(int(data.get("speed") or 80))
        msg = pb.ClientToRobot()
        msg.conveyor.speed = speed
        ok = _send(msg, record=False)

        def _stop_later() -> None:
            time.sleep(pulse / 1000.0)
            z = pb.ClientToRobot()
            z.conveyor.speed = 0
            _send(z, record=False)

        threading.Thread(target=_stop_later, daemon=True).start()
        return jsonify({"ok": ok, "speed": speed, "pulse_ms": pulse, "dry_run": _state.get("dry_run")})

    if action == "arm_center":
        msg = pb.ClientToRobot()
        msg.center = True
        ok = _send(msg, record=False)
        return jsonify({"ok": ok, "dry_run": _state.get("dry_run")})

    return jsonify({"ok": False, "error": f"unknown action {action}"}), 400


def _dispatch_ws_cmd(data: dict) -> dict:
    """Realtime command path from browser WebSocket (same semantics as REST)."""
    op = str(data.get("op") or data.get("t") or "").strip().lower()
    if op in ("ping", "hello"):
        return {"ok": True, "t": "pong", "robot": bool(_state.get("connected"))}
    if op == "estop":
        _do_estop(reason="chord-SO")
        return {"ok": True, "op_mode": _state.get("op_mode"), "estop": False}
    if op == "stop":
        ok = _send(_stop_msg(), record=True, force=True)
        return {"ok": ok, "error": _state.get("error")}
    if op == "drive":
        ok_gate, err = _gate_motion()
        if not ok_gate:
            return {"ok": False, "error": err}
        try:
            left, right = validate_drive(
                _coerce_int(data.get("left", 0)),
                _coerce_int(data.get("right", 0)),
            )
        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": f"invalid drive: {exc}"}
        msg = pb.ClientToRobot()
        msg.drive.left = left
        msg.drive.right = right
        _state["left"], _state["right"] = left, right
        return {"ok": _send(msg), "error": _state.get("error")}
    if op == "arm":
        ok_gate, err = _gate_motion()
        if not ok_gate:
            return {"ok": False, "error": err}
        msg, aerr = _apply_arm_command(data)
        if aerr or msg is None:
            return {"ok": False, "error": aerr or "bad arm"}
        return {"ok": _send(msg), "error": _state.get("error")}
    if op == "conveyor":
        global _conv_latch, _last_conv_reassert
        ok_gate, err = _gate_motion()
        if not ok_gate:
            return {"ok": False, "error": err}
        try:
            speed = validate_conveyor(_coerce_int(data.get("speed", 0)))
        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": f"invalid conveyor: {exc}"}
        msg = pb.ClientToRobot()
        msg.conveyor.speed = speed
        ok = _send(msg)
        if ok:
            _state["conveyor"] = speed
            _conv_latch = speed
            _last_conv_reassert = time.monotonic()
        return {"ok": ok, "error": _state.get("error")}
    if op == "center":
        ok_gate, err = _gate_motion()
        if not ok_gate:
            return {"ok": False, "error": err}
        msg = pb.ClientToRobot()
        msg.center = True
        for k, v in default_arm_pose().items():
            _state[k] = v
        return {"ok": _send(msg), "error": _state.get("error")}
    return {"ok": False, "error": f"unknown op {op!r}"}


@sock.route("/ws")
def ui_ws(ws) -> None:
    """Persistent browser channel: push state + accept realtime cmds."""
    # Cap clients — each WS holds a Werkzeug thread; leaks freeze /api/connect.
    if _hub.client_count >= 4:
        try:
            ws.send('{"t":"err","error":"too many ws clients — refresh one tab"}')
        except Exception:  # noqa: BLE001
            pass
        try:
            ws.close()
        except Exception:  # noqa: BLE001
            pass
        return
    _hub.register(ws)
    try:
        ws.send(
            json.dumps(
                {"t": "hello", "s": _snapshot_state(), "robot_transport": "persistent-tcp"},
                separators=(",", ":"),
                default=str,
            )
        )
        while True:
            try:
                # timeout reaps half-dead browser tabs (was infinite → CLOSE_WAIT storm)
                raw = ws.receive(timeout=20)
            except Exception:  # noqa: BLE001
                break
            if raw is None:
                break
            if isinstance(raw, (bytes, bytearray)):
                raw = raw.decode("utf-8", errors="replace")
            raw = (raw or "").strip()
            if not raw:
                continue
            try:
                data = json.loads(raw)
            except Exception:  # noqa: BLE001
                ws.send('{"ok":false,"error":"bad json"}')
                continue
            if not isinstance(data, dict):
                continue
            if data.get("t") == "ping" or data.get("op") == "ping":
                try:
                    ws.send('{"t":"pong"}')
                except Exception:  # noqa: BLE001
                    break
                continue
            if data.get("t") == "sub" or data.get("op") == "sub":
                ws.send(
                    json.dumps(
                        {"t": "state", "s": _snapshot_state()},
                        separators=(",", ":"),
                        default=str,
                    )
                )
                continue
            resp = _dispatch_ws_cmd(data)
            # Fire-and-forget motion cmds skip acks; control ops always reply
            op = str(data.get("op") or "")
            if data.get("ack") or op in ("ping", "hello", "estop", "stop"):
                resp.setdefault("t", "ack")
                try:
                    ws.send(json.dumps(resp, separators=(",", ":"), default=str))
                except Exception:  # noqa: BLE001
                    break
    finally:
        _hub.unregister(ws)


def main() -> None:
    _hub.start_publisher(_snapshot_state, min_hz=25.0, idle_hz=5.0)
    # threaded=True required so WS + REST + TCP reader coexist
    app.run(host="0.0.0.0", port=5050, debug=False, threaded=True)


if __name__ == "__main__":
    main()
