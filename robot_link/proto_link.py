"""Laptop ↔ hub ESP32 framed LinkMessage (nanopb-compatible)."""

from __future__ import annotations

import time
from typing import Iterator, Optional

from meowler_pb import meowler_pb2

FRAME_MAGIC = 0xA5

# RobotAction
ACT_NONE = 0
ACT_STATUS = 1
ACT_STOP = 2
ACT_PING = 3
ACT_CAL_COLOR = 4
ACT_ARM_CENTER = 5
ACT_ARM_DEMO = 6

# flags bits in RobotTelemetry.flags
FLAG_NANO = 1
FLAG_PCA = 2
FLAG_TOF = 4
FLAG_IMU = 8
FLAG_MOVING = 16
FLAG_CAL = 32


def _xor_checksum(length: int, payload: bytes) -> int:
    x = length & 0xFF
    for b in payload:
        x ^= b
    return x & 0xFF


def encode_link(msg: meowler_pb2.LinkMessage) -> bytes:
    payload = msg.SerializeToString()
    length = len(payload)
    if length > 255:
        raise ValueError("link message too large")
    return bytes([FRAME_MAGIC, length]) + payload + bytes([_xor_checksum(length, payload)])


def encode_drive(left: int, right: int) -> bytes:
    msg = meowler_pb2.LinkMessage()
    # Force submessage presence for nanopb has_drive
    drive = msg.cmd.drive
    drive.left = max(-255, min(255, int(left)))
    drive.right = max(-255, min(255, int(right)))
    return encode_link(msg)


def encode_arm(
    base: int | None = None,
    height: int | None = None,
    grip: int | None = None,
    speed_dps: int = 500,
    action: int = 0,
) -> bytes:
    msg = meowler_pb2.LinkMessage()
    if base is not None:
        msg.cmd.arm.base = max(0, min(180, int(base)))
    if height is not None:
        msg.cmd.arm.height = max(0, min(180, int(height)))
    if grip is not None:
        msg.cmd.arm.grip = max(0, min(180, int(grip)))
    if speed_dps:
        msg.cmd.arm.speed_dps = int(speed_dps)
    if action:
        msg.cmd.arm.action = int(action)
    return encode_link(msg)


def encode_action(action: int) -> bytes:
    msg = meowler_pb2.LinkMessage()
    msg.cmd.action = int(action)
    return encode_link(msg)


def try_parse_link(buf: bytearray) -> tuple[Optional[meowler_pb2.LinkMessage], bytearray]:
    while True:
        try:
            start = buf.index(FRAME_MAGIC)
        except ValueError:
            buf.clear()
            return None, buf
        if start:
            del buf[:start]
        if len(buf) < 2:
            return None, buf
        length = buf[1]
        need = 2 + length + 1
        if len(buf) < need:
            return None, buf
        payload = bytes(buf[2 : 2 + length])
        chk = buf[2 + length]
        del buf[:need]
        if chk != _xor_checksum(length, payload):
            continue
        msg = meowler_pb2.LinkMessage()
        try:
            msg.ParseFromString(payload)
        except Exception:  # noqa: BLE001
            continue
        return msg, buf


def iter_link_messages(ser, timeout: float = 1.0) -> Iterator[meowler_pb2.LinkMessage]:
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            while True:
                msg, buf = try_parse_link(buf)
                if msg is None:
                    break
                yield msg
                deadline = time.time() + timeout
        else:
            time.sleep(0.005)


def read_telemetry(ser, timeout: float = 0.5) -> Optional[meowler_pb2.RobotTelemetry]:
    for msg in iter_link_messages(ser, timeout=timeout):
        if msg.WhichOneof("payload") == "telem":
            return msg.telem
    return None


def flags_dict(flags: int) -> dict[str, bool]:
    return {
        "nano_ok": bool(flags & FLAG_NANO),
        "pca_ok": bool(flags & FLAG_PCA),
        "tof_ok": bool(flags & FLAG_TOF),
        "imu_ok": bool(flags & FLAG_IMU),
        "moving": bool(flags & FLAG_MOVING),
        "color_cal": bool(flags & FLAG_CAL),
    }
