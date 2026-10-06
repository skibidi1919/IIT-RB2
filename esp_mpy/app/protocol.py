"""
Length-prefixed Meowler protobuf (proto/meowler.proto).

Wire: uint32 little-endian length | protobuf payload
Messages: ClientToRobot ↔ RobotToClient (minimal encoder/decoder for fields we use).

JSON-line fallback (documented): if a TCP peer sends a line starting with '{',
main.py can parse JSON commands — see README. Default path is protobuf.
"""

import struct

MAX_FRAME = 4224


# ---- low-level protobuf helpers ----

def _encode_varint(n):
    if n < 0:
        # zigzag not used; encode as unsigned 64-bit two's complement for sint? — we use int32 zig-free
        n = n & 0xFFFFFFFFFFFFFFFF
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            break
    return bytes(out)


def _encode_key(field, wire):
    return _encode_varint((field << 3) | wire)


def _encode_varint_field(field, value):
    if value == 0:
        return b""  # proto3 default omit
    return _encode_key(field, 0) + _encode_varint(value & 0xFFFFFFFFFFFFFFFF)


def _encode_bool_field(field, value):
    if not value:
        return b""
    return _encode_key(field, 0) + _encode_varint(1)


def _encode_bytes_field(field, data):
    if not data:
        return b""
    return _encode_key(field, 2) + _encode_varint(len(data)) + data


def _encode_string_field(field, s):
    if not s:
        return b""
    data = s.encode("utf-8") if isinstance(s, str) else s
    return _encode_bytes_field(field, data)


def _encode_submessage(field, payload):
    if not payload:
        return b""
    return _encode_bytes_field(field, payload)


def _encode_sint32(n):
    """Protobuf int32 on wire is varint of the raw two's complement bits."""
    return n & 0xFFFFFFFF


def _decode_varint(buf, i):
    shift = 0
    result = 0
    while i < len(buf):
        b = buf[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, i
        shift += 7
        if shift > 63:
            break
    raise ValueError("bad varint")


def _to_signed32(n):
    n &= 0xFFFFFFFF
    if n >= 0x80000000:
        n -= 0x100000000
    return n


def _decode_fields(buf):
    """Yield (field_number, wire_type, value) where value is int or bytes."""
    i = 0
    n = len(buf)
    while i < n:
        key, i = _decode_varint(buf, i)
        field = key >> 3
        wire = key & 7
        if wire == 0:
            val, i = _decode_varint(buf, i)
            yield field, wire, val
        elif wire == 2:
            ln, i = _decode_varint(buf, i)
            val = buf[i : i + ln]
            i += ln
            yield field, wire, val
        elif wire == 5:
            val = struct.unpack_from("<I", buf, i)[0]
            i += 4
            yield field, wire, val
        elif wire == 1:
            val = struct.unpack_from("<Q", buf, i)[0]
            i += 8
            yield field, wire, val
        else:
            raise ValueError("unsupported wire %d" % wire)


# ---- encode RobotToClient ----

def encode_drive(left, right):
    return (
        _encode_varint_field(1, _encode_sint32(int(left)))
        + _encode_varint_field(2, _encode_sint32(int(right)))
    )


def encode_arm(base, height, grip, set_base=True, set_height=True, set_grip=True):
    return (
        _encode_varint_field(1, _encode_sint32(int(base)))
        + _encode_varint_field(2, _encode_sint32(int(height)))
        + _encode_varint_field(3, _encode_sint32(int(grip)))
        + _encode_bool_field(4, set_base)
        + _encode_bool_field(5, set_height)
        + _encode_bool_field(6, set_grip)
    )


def encode_conveyor(speed):
    return _encode_varint_field(1, _encode_sint32(int(speed)))


def encode_hello(ip_u32, port, pca_ok, tof_ok, imu_ok=False):
    body = (
        _encode_varint_field(1, ip_u32 & 0xFFFFFFFF)
        + _encode_varint_field(2, int(port))
        + _encode_bool_field(3, pca_ok)
        + _encode_bool_field(4, tof_ok)
        + _encode_bool_field(5, imu_ok)
    )
    return _encode_submessage(1, body)  # RobotToClient.hello = 1


def encode_telemetry(t):
    """t: dict with Telemetry field names."""
    parts = []
    mapping = (
        (1, "distance_mm", False),
        (2, "cmd_l", True),
        (3, "cmd_r", True),
        (4, "base", True),
        (5, "height", True),
        (6, "grip", True),
        (7, "pca_ok", "bool"),
        (8, "tof_ok", "bool"),
        (9, "imu_ok", "bool"),
        (10, "wifi_ok", "bool"),
        (11, "enc_l", True),
        (12, "enc_r", True),
        (13, "wheel_l_mm", True),
        (14, "wheel_r_mm", True),
        (15, "tof_disp_mm", True),
        (16, "yaw_cdeg", True),
        (17, "pitch_cdeg", True),
        (18, "roll_cdeg", True),
        (19, "color", False),
        (20, "color_conf", False),
        (21, "color_r", False),
        (22, "color_g", False),
        (23, "color_b", False),
        (24, "color_rp", False),
        (25, "color_gp", False),
        (26, "color_bp", False),
        (27, "color_cp", False),
        (28, "conveyor", True),
    )
    for field, key, kind in mapping:
        if key not in t:
            continue
        val = t[key]
        if kind == "bool":
            parts.append(_encode_bool_field(field, bool(val)))
        elif kind is True:
            parts.append(_encode_varint_field(field, _encode_sint32(int(val))))
        else:
            parts.append(_encode_varint_field(field, int(val) & 0xFFFFFFFF))
    body = b"".join(parts)
    return _encode_submessage(2, body)  # RobotToClient.telem = 2


def encode_ack(code=0):
    # uint32 ack = 3; proto3 omits 0 — host still expects a frame, so force write
    return _encode_key(3, 0) + _encode_varint(int(code) & 0xFFFFFFFF)


def encode_log(level, text):
    body = _encode_varint_field(1, int(level)) + _encode_string_field(2, text)
    return _encode_submessage(4, body)


def encode_rec_event(ev):
    parts = [
        _encode_varint_field(1, int(ev.get("t_ms", 0))),
        _encode_varint_field(2, int(ev.get("op", 0))),
        _encode_varint_field(3, _encode_sint32(int(ev.get("a", 0)))),
        _encode_varint_field(4, _encode_sint32(int(ev.get("b", 0)))),
        _encode_varint_field(5, _encode_sint32(int(ev.get("c", 0)))),
        _encode_varint_field(6, int(ev.get("mask", 0))),
        _encode_varint_field(7, int(ev.get("count", 0))),
        _encode_string_field(8, ev.get("name", "")),
    ]
    return _encode_submessage(5, b"".join(parts))


def frame(payload):
    return struct.pack("<I", len(payload)) + payload


# ---- decode ClientToRobot ----

def _decode_drive(buf):
    left = right = 0
    for f, w, v in _decode_fields(buf):
        if f == 1 and w == 0:
            left = _to_signed32(v)
        elif f == 2 and w == 0:
            right = _to_signed32(v)
    return {"op": "drive", "left": left, "right": right}


def _decode_arm(buf):
    out = {
        "op": "arm",
        "base": 0,
        "height": 0,
        "grip": 0,
        "set_base": False,
        "set_height": False,
        "set_grip": False,
    }
    for f, w, v in _decode_fields(buf):
        if w != 0:
            continue
        if f == 1:
            out["base"] = _to_signed32(v)
        elif f == 2:
            out["height"] = _to_signed32(v)
        elif f == 3:
            out["grip"] = _to_signed32(v)
        elif f == 4:
            out["set_base"] = bool(v)
        elif f == 5:
            out["set_height"] = bool(v)
        elif f == 6:
            out["set_grip"] = bool(v)
    return out


def _decode_conveyor(buf):
    speed = 0
    for f, w, v in _decode_fields(buf):
        if f == 1 and w == 0:
            speed = _to_signed32(v)
    return {"op": "conveyor", "speed": speed}


def _decode_color_cal(buf):
    mode = 0
    for f, w, v in _decode_fields(buf):
        if f == 1 and w == 0:
            mode = v
    return {"op": "color_cal", "mode": mode}


def _decode_rec(buf):
    action = 0
    name = ""
    for f, w, v in _decode_fields(buf):
        if f == 1 and w == 0:
            action = v
        elif f == 2 and w == 2:
            name = bytes(v).decode("utf-8", "ignore")
    return {"op": "rec", "action": action, "name": name}


def decode_client_to_robot(buf):
    """Return dict with 'op' key, or None if empty/unknown."""
    for f, w, v in _decode_fields(buf):
        if w == 2:
            if f == 1:
                return _decode_drive(v)
            if f == 2:
                return _decode_arm(v)
            if f == 8:
                return _decode_conveyor(v)
            if f == 9:
                return _decode_color_cal(v)
            if f == 10:
                return _decode_rec(v)
            if f == 11:
                return {"op": "ota", "raw": v}  # handled lightly in main
        elif w == 0:
            if f == 3:
                return {"op": "stop", "value": bool(v)}
            if f == 4:
                return {"op": "center", "value": bool(v)}
            if f == 5:
                return {"op": "zero", "value": bool(v)}
            if f == 6:
                return {"op": "motor_test", "value": bool(v)}
            if f == 7:
                return {"op": "get_telem", "value": bool(v)}
    return None


class FrameReader:
    """Incremental uint32le | payload reader."""

    def __init__(self, max_frame=MAX_FRAME):
        self.max_frame = max_frame
        self.reset()

    def reset(self):
        self._hdr = bytearray()
        self._need = 0
        self._body = bytearray()
        self._in_body = False

    def feed(self, data):
        """Feed bytes; yield complete payload bytearrays."""
        i = 0
        n = len(data)
        while i < n:
            if not self._in_body:
                take = min(4 - len(self._hdr), n - i)
                self._hdr.extend(data[i : i + take])
                i += take
                if len(self._hdr) < 4:
                    return
                self._need = struct.unpack("<I", self._hdr)[0]
                self._hdr = bytearray()
                self._body = bytearray()
                if self._need == 0 or self._need > self.max_frame:
                    self.reset()
                    continue
                self._in_body = True
            take = min(self._need - len(self._body), n - i)
            self._body.extend(data[i : i + take])
            i += take
            if len(self._body) >= self._need:
                payload = bytes(self._body)
                self.reset()
                yield payload


def ip_to_u32(ip_str):
    """IPv4 dotted → uint32 network byte order (big-endian packed as int)."""
    parts = [int(x) for x in ip_str.split(".")]
    if len(parts) != 4:
        return 0
    return (
        ((parts[0] & 0xFF) << 24)
        | ((parts[1] & 0xFF) << 16)
        | ((parts[2] & 0xFF) << 8)
        | (parts[3] & 0xFF)
    )
