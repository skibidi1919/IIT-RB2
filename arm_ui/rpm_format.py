"""Binary Meowler .rpm v2 — pack/unpack timeline events (no I/O side effects)."""

from __future__ import annotations

import struct
import time
from typing import Any

RPM_MAGIC = b"MRPM"
RPM_VERSION = 2
RPM_HDR = struct.Struct("<4sBBHII")

OP_DRIVE = 1
OP_ARM = 2
OP_STOP = 3
OP_CENTER = 4
OP_ZERO = 5
OP_CONV = 6
OP_MTEST = 7

OP_NAME = {
    OP_DRIVE: "drive",
    OP_ARM: "arm",
    OP_STOP: "stop",
    OP_CENTER: "center",
    OP_ZERO: "zero",
    OP_CONV: "conveyor",
    OP_MTEST: "motor_test",
}
OP_CODE = {v: k for k, v in OP_NAME.items()}

# Ops that belong in a field replay (not sensor decisions, not destructive tests)
REPLAYABLE_OPS = frozenset({"drive", "arm", "stop", "center", "conveyor"})


def _i16(v: int) -> int:
    return max(-32768, min(32767, int(v)))


def _u8(v: int) -> int:
    return max(0, min(255, int(v)))


def pack_event(ev: dict[str, Any]) -> bytes:
    op_name = ev.get("op")
    code = OP_CODE.get(op_name)  # type: ignore[arg-type]
    if code is None:
        raise ValueError(f"bad op {op_name!r}")
    t_ms = max(0, int(ev.get("t_ms", 0))) & 0xFFFFFFFF
    head = struct.pack("<IB", t_ms, code)
    if code == OP_DRIVE:
        return head + struct.pack("<hh", _i16(ev.get("left", 0)), _i16(ev.get("right", 0)))
    if code == OP_ARM:
        mask = 0
        body = bytearray()
        for bit, key in (
            (1, "base"),
            (2, "height"),
            (4, "grip"),
            (8, "s13"),
            (16, "s14"),
            (32, "s15"),
        ):
            if key in ev:
                mask |= bit
                body.append(_u8(ev[key]))
        return head + bytes([mask]) + bytes(body)
    if code == OP_CONV:
        return head + struct.pack("<h", _i16(ev.get("speed", 0)))
    return head


def unpack_events(blob: bytes, count: int) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    off = 0
    n = len(blob)
    for _ in range(count):
        if off + 5 > n:
            raise ValueError("truncated event")
        t_ms, code = struct.unpack_from("<IB", blob, off)
        off += 5
        name = OP_NAME.get(code)
        if name is None:
            raise ValueError(f"unknown opcode {code}")
        ev: dict[str, Any] = {"t_ms": int(t_ms), "op": name}
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
            for bit, key in (
                (1, "base"),
                (2, "height"),
                (4, "grip"),
                (8, "s13"),
                (16, "s14"),
                (32, "s15"),
            ):
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


def write_rpm_bytes(name: str, events: list[dict[str, Any]], created: int | None = None) -> bytes:
    name_b = name.encode("utf-8")
    if len(name_b) > 0xFFFF:
        raise ValueError("name too long")
    body = bytearray()
    for ev in events:
        body += pack_event(ev)
    hdr = RPM_HDR.pack(
        RPM_MAGIC,
        RPM_VERSION,
        0,
        len(name_b),
        (created if created is not None else int(time.time())) & 0xFFFFFFFF,
        len(events),
    )
    return hdr + name_b + bytes(body)


def load_rpm_bytes(raw: bytes) -> dict[str, Any]:
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
    events = unpack_events(raw[off:], count)
    return {
        "format": "meowler.rpm",
        "version": ver,
        "name": name_b.decode("utf-8", errors="replace"),
        "created_unix": int(created),
        "events": events,
    }


def filter_replayable(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Drop motor_test / unknown; keep motion timeline only (no color)."""
    return [e for e in events if e.get("op") in REPLAYABLE_OPS]


def sanitize_timeline(events: list[dict[str, Any]], *, max_events: int = 50_000) -> list[dict[str, Any]]:
    """Sort by time, clamp timestamps, drop junk — safe for replay after corruption/merge."""
    cleaned: list[dict[str, Any]] = []
    for ev in filter_replayable(events):
        try:
            raw_t = int(ev.get("t_ms", 0))
        except (TypeError, ValueError):
            continue
        if raw_t < 0:
            continue  # corrupt / inverted clock — drop
        t_ms = min(raw_t, 86_400_000)  # ≤ 24h
        row = dict(ev)
        row["t_ms"] = t_ms
        op = row.get("op")
        if op == "drive":
            try:
                row["left"], row["right"] = validate_drive(row.get("left", 0), row.get("right", 0))
            except (TypeError, ValueError):
                continue
        elif op == "arm":
            joints = ("base", "height", "grip", "s13", "s14", "s15")
            for k in joints:
                if k in row:
                    try:
                        row[k] = validate_joint(row[k], joint=k)
                    except (TypeError, ValueError):
                        row.pop(k, None)
            if not any(k in row for k in joints):
                continue
        elif op == "conveyor":
            try:
                row["speed"] = validate_conveyor(row.get("speed", 0))
            except (TypeError, ValueError):
                continue
        cleaned.append(row)
    cleaned.sort(key=lambda e: (int(e["t_ms"]), e.get("op") or ""))
    if len(cleaned) > max_events:
        cleaned = cleaned[:max_events]
    return cleaned


ARM_JOINTS = ("base", "height", "grip", "s13", "s14", "s15")
# CH13 homes at 98°; others at 90°
ARM_DEFAULTS = {"base": 90, "height": 90, "grip": 90, "s13": 98, "s14": 90, "s15": 90}
# Gripper mechanical limit (degrees) — host + ESP both enforce
GRIP_MAX_DEG = 110
JOINT_MAX_DEG = {j: 180 for j in ARM_JOINTS}
JOINT_MAX_DEG["grip"] = GRIP_MAX_DEG


def default_arm_pose(deg: int | None = None) -> dict[str, int]:
    if deg is None:
        return dict(ARM_DEFAULTS)
    return {k: int(deg) for k in ARM_JOINTS}


def merge_arm_pose(pose: dict[str, int], ev: dict[str, Any]) -> dict[str, int]:
    out = dict(pose)
    for k in ARM_JOINTS:
        if k in ev:
            out[k] = int(ev[k])
    return out


def expand_arm_events(
    events: list[dict[str, Any]], seed: dict[str, int]
) -> list[dict[str, Any]]:
    pose = dict(default_arm_pose())
    pose.update(seed)
    out: list[dict[str, Any]] = []
    for ev in events:
        op = ev.get("op")
        if op == "arm":
            prev = dict(pose)
            pose = merge_arm_pose(pose, ev)
            if pose == prev and out:
                continue
            row = {"t_ms": int(ev.get("t_ms", 0)), "op": "arm", "_prev": prev}
            row.update({k: pose[k] for k in ARM_JOINTS})
            out.append(row)
        elif op == "center":
            prev = dict(pose)
            pose = default_arm_pose()  # includes s13=98
            row = {"t_ms": int(ev.get("t_ms", 0)), "op": "arm", "_prev": prev}
            row.update(pose)
            out.append(row)
        elif op in REPLAYABLE_OPS:
            out.append(ev)
    return out


def validate_drive(left: int, right: int) -> tuple[int, int]:
    return max(-255, min(255, int(left))), max(-255, min(255, int(right)))


def validate_joint(deg: int, joint: str | None = None) -> int:
    hi = JOINT_MAX_DEG.get(joint, 180) if joint else 180
    return max(0, min(hi, int(deg)))


def validate_conveyor(speed: int) -> int:
    return max(-255, min(255, int(speed)))
