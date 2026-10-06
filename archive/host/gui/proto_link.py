"""nanopb-compatible framed protobuf link for Meowler Nano."""

from __future__ import annotations

import time
from typing import Optional

from meowler_pb import meowler_pb2

FRAME_MAGIC = 0xA5
ACT_NONE = 0
ACT_STATUS = 1
ACT_CENTER = 2
ACT_DEMO = 3

DEFAULT_SPEED_DPS = 500


def _xor_checksum(length: int, payload: bytes) -> int:
    x = length & 0xFF
    for b in payload:
        x ^= b
    return x & 0xFF


def encode_command(
    *,
    base: int | None = None,
    height: int | None = None,
    grip: int | None = None,
    speed_dps: int = DEFAULT_SPEED_DPS,
    action: int = ACT_NONE,
) -> bytes:
    msg = meowler_pb2.ArmCommand()
    if base is not None:
        msg.base = max(0, min(180, int(base)))
    if height is not None:
        msg.height = max(0, min(180, int(height)))
    if grip is not None:
        msg.grip = max(0, min(180, int(grip)))
    if speed_dps:
        msg.speed_dps = int(speed_dps)
    if action:
        msg.action = int(action)
    payload = msg.SerializeToString()
    length = len(payload)
    if length > 255:
        raise ValueError("command too large")
    return bytes([FRAME_MAGIC, length]) + payload + bytes([_xor_checksum(length, payload)])


def try_parse_status(buf: bytearray) -> tuple[Optional[meowler_pb2.ArmStatus], bytearray]:
    """Consume sync'd frames from buf; return (status|None, remaining)."""
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
        st = meowler_pb2.ArmStatus()
        try:
            st.ParseFromString(payload)
        except Exception:  # noqa: BLE001
            continue
        return st, buf


def read_status(ser, timeout: float = 0.4) -> Optional[meowler_pb2.ArmStatus]:
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            st, buf = try_parse_status(buf)
            if st is not None:
                return st
        else:
            time.sleep(0.01)
    return None
