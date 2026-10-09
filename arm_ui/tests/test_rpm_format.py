"""Recording format: order, timing, filter, round-trip."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rpm_format import (
    expand_arm_events,
    filter_replayable,
    load_rpm_bytes,
    pack_event,
    unpack_events,
    validate_drive,
    write_rpm_bytes,
)


def test_roundtrip_preserves_order_and_timing():
    events = [
        {"t_ms": 0, "op": "arm", "base": 90, "height": 90, "grip": 90},
        {"t_ms": 100, "op": "drive", "left": 180, "right": 180},
        {"t_ms": 500, "op": "drive", "left": 0, "right": 0},
        {"t_ms": 600, "op": "conveyor", "speed": 120},
        {"t_ms": 900, "op": "stop"},
    ]
    raw = write_rpm_bytes("unit", events, created=12345)
    loaded = load_rpm_bytes(raw)
    assert loaded["name"] == "unit"
    assert loaded["created_unix"] == 12345
    assert loaded["events"] == events


def test_filter_drops_motor_test_and_zero():
    events = [
        {"t_ms": 0, "op": "drive", "left": 1, "right": 1},
        {"t_ms": 1, "op": "motor_test"},
        {"t_ms": 2, "op": "zero"},
        {"t_ms": 3, "op": "arm", "base": 45},
    ]
    out = filter_replayable(events)
    assert [e["op"] for e in out] == ["drive", "arm"]


def test_expand_arm_partial_to_absolute():
    seed = {"base": 90, "height": 90, "grip": 90, "s13": 98, "s14": 90, "s15": 90}
    events = [
        {"t_ms": 0, "op": "arm", "base": 10},
        {"t_ms": 50, "op": "arm", "height": 20},
        {"t_ms": 75, "op": "arm", "s13": 45},
        {"t_ms": 100, "op": "center"},
    ]
    out = expand_arm_events(events, seed)
    assert out[0]["base"] == 10 and out[0]["height"] == 90 and out[0]["s13"] == 98
    assert out[1]["base"] == 10 and out[1]["height"] == 20
    assert out[2]["s13"] == 45 and out[2]["base"] == 10
    assert out[3]["base"] == 90 and out[3]["grip"] == 90 and out[3]["s13"] == 98 and out[3]["s15"] == 90


def test_pack_unpack_aux_servos():
    ev = {"t_ms": 12, "op": "arm", "s13": 30, "s14": 60, "s15": 90}
    blob = pack_event(ev)
    assert unpack_events(blob, 1) == [ev]


def test_validate_drive_clamps():
    assert validate_drive(999, -999) == (255, -255)


def test_pack_unpack_single():
    ev = {"t_ms": 42, "op": "drive", "left": -10, "right": 20}
    blob = pack_event(ev)
    assert unpack_events(blob, 1) == [ev]


def test_malformed_header_raises():
    try:
        load_rpm_bytes(b"XXXX" + b"\x00" * 20)
        assert False, "expected ValueError"
    except ValueError as e:
        assert "magic" in str(e).lower() or "short" in str(e).lower() or "not a" in str(e).lower()


def test_sanitize_drops_negative_time_and_sorts():
    from rpm_format import sanitize_timeline

    clean = sanitize_timeline([
        {"t_ms": 300, "op": "drive", "left": 1, "right": 1},
        {"t_ms": -1, "op": "stop"},
        {"t_ms": 50, "op": "arm", "base": 1, "height": 2, "grip": 3},
    ])
    assert [e["op"] for e in clean] == ["arm", "drive"]
    assert clean[0]["t_ms"] == 50
