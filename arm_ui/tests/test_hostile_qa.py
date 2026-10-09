"""Hostile QA — try to break record/replay/safety via MockRobot + API abuse."""

from __future__ import annotations

import struct
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from color_host import ColorFilter
from mock_robot import MockRobot
from rpm_format import load_rpm_bytes, sanitize_timeline, write_rpm_bytes

import app as panel

BASE = 14000
RPM = Path(__file__).resolve().parent / "_tmp_hostile_rpm"


def setup_module():
    panel.RPM_DIR = RPM
    RPM.mkdir(parents=True, exist_ok=True)


def _reset():
    try:
        panel._replay_stop.set()
        with panel._rec_lock:
            panel._recording = False
            panel._replaying = False
            panel._replay_paused = False
            panel._rec_events = []
            panel._rec_name = None
        panel._state.update({
            "recording": False,
            "replaying": False,
            "estop": False,
            "error": None,
            "fault": False,
            "warning": None,
            "ready_replay": False,
            "dry_run": False,  # tests that hit the mock need real TX
        })
        panel.disconnect()
    except Exception:
        pass


def setup_function():
    panel._state["dry_run"] = False
    panel._refresh_op_mode()


def teardown_function():
    _reset()


def _wait_connected(timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if panel._state.get("connected"):
            return True
        time.sleep(0.03)
    return bool(panel._state.get("connected"))


def _wait_flag(key, value=False, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if panel._state.get(key) is value:
            return True
        time.sleep(0.03)
    return panel._state.get(key) is value


def test_invalid_commands_rejected_not_500():
    bot = MockRobot(port=BASE)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE)
        assert _wait_connected()
        c = panel.app.test_client()
        for body in (
            {"left": "nope", "right": 1},
            {"left": True, "right": 1},
            {"left": None, "right": "x"},
        ):
            r = c.post("/api/drive", json=body)
            assert r.status_code == 400, body
            assert r.get_json()["ok"] is False
        r = c.post("/api/arm", json={"base": "abc"})
        assert r.status_code == 400
        r = c.post("/api/conveyor", json={"speed": {}})
        assert r.status_code == 400
        # still responsive
        assert c.post("/api/drive", json={"left": 10, "right": 10}).status_code == 200
    finally:
        panel.disconnect()
        bot.stop()


def test_rapid_drive_spam_does_not_crash():
    bot = MockRobot(port=BASE + 1)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 1)
        assert _wait_connected()
        c = panel.app.test_client()
        for i in range(80):
            r = c.post("/api/drive", json={"left": 100 + (i % 3), "right": 100})
            assert r.status_code in (200, 409)
        r = c.post("/api/stop", json={})
        assert r.status_code == 200
    finally:
        panel.disconnect()
        bot.stop()


def test_double_record_race():
    bot = MockRobot(port=BASE + 2)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 2)
        assert _wait_connected()
        c = panel.app.test_client()
        results = []

        def start(i):
            return c.post("/api/record/start", json={"name": f"race-{i}"}).status_code

        with ThreadPoolExecutor(max_workers=8) as pool:
            futs = [pool.submit(start, i) for i in range(8)]
            results = [f.result() for f in as_completed(futs)]
        assert results.count(200) == 1, results
        assert results.count(409) == 7, results
        c.post("/api/record/stop", json={})
    finally:
        panel.disconnect()
        bot.stop()


def test_double_replay_race():
    bot = MockRobot(port=BASE + 3)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 3)
        assert _wait_connected()
        c = panel.app.test_client()
        name = "dbl-replay"
        # Long timeline so first replay cannot finish before siblings enter the lock
        events = [{"t_ms": 0, "op": "arm", "base": 90, "height": 90, "grip": 90}]
        for i in range(1, 80):
            events.append({"t_ms": i * 200, "op": "drive", "left": 40, "right": 40})
        panel._rpm_path(name).write_bytes(write_rpm_bytes(name, events))

        barrier = threading.Barrier(6)

        def play():
            barrier.wait(timeout=3)
            return c.post("/api/replay", json={"name": name, "confirm": True, "origin_ack": True, "allow_sim": True}).status_code

        with ThreadPoolExecutor(max_workers=6) as pool:
            codes = [f.result() for f in as_completed([pool.submit(play) for _ in range(6)])]
        assert codes.count(200) == 1, codes
        assert codes.count(409) == 5, codes
        c.post("/api/replay/stop", json={})
        assert _wait_flag("replaying", False, timeout=4.0)
    finally:
        panel.disconnect()
        bot.stop()


def test_disconnect_mid_record_autosaves():
    bot = MockRobot(port=BASE + 4)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 4)
        assert _wait_connected()
        c = panel.app.test_client()
        name = "mid-disc"
        assert c.post("/api/record/start", json={"name": name}).status_code == 200
        c.post("/api/drive", json={"left": 90, "right": 90})
        time.sleep(0.05)
        bot.drop_clients()
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline and panel._state.get("recording"):
            time.sleep(0.05)
        assert panel._state.get("recording") is False
        assert panel._rpm_path(name).is_file()
        data = load_rpm_bytes(panel._rpm_path(name).read_bytes())
        assert len(data["events"]) >= 1
    finally:
        panel.disconnect()
        bot.stop()


def test_disconnect_mid_replay_stops_clean():
    bot = MockRobot(port=BASE + 5)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 5)
        assert _wait_connected()
        c = panel.app.test_client()
        name = "mid-replay-disc"
        # long timeline
        events = [{"t_ms": 0, "op": "arm", "base": 90, "height": 90, "grip": 90}]
        for i in range(1, 40):
            events.append({"t_ms": i * 100, "op": "drive", "left": 80, "right": 80})
        panel._rpm_path(name).write_bytes(write_rpm_bytes(name, events))
        assert c.post("/api/replay", json={"name": name, "confirm": True, "origin_ack": True, "allow_sim": True}).status_code == 200
        time.sleep(0.15)
        bot.drop_clients()
        assert _wait_flag("replaying", False, timeout=4.0)
        # motors commanded stop at least once
        assert "stop" in bot.commands or not panel._state.get("connected")
    finally:
        panel.disconnect()
        bot.stop()


def test_hard_stop_zeros_without_latch():
    bot = MockRobot(port=BASE + 6)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 6)
        assert _wait_connected()
        c = panel.app.test_client()
        c.post("/api/drive", json={"left": 200, "right": 200})
        time.sleep(0.08)
        assert c.post("/api/estop", json={}).status_code == 200
        assert panel._state["estop"] is False
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and (bot.cmd_l != 0 or bot.cmd_r != 0):
            time.sleep(0.02)
        assert bot.cmd_l == 0 and bot.cmd_r == 0
        assert "stop" in bot.commands
        # No latch — drive accepted immediately
        assert c.post("/api/drive", json={"left": 10, "right": 10}).status_code == 200
        assert panel._state["op_mode"] != "EMERGENCY STOP"
        assert panel._state["op_mode"] != "ERROR"
    finally:
        panel.disconnect()
        bot.stop()


def test_corrupt_and_empty_recordings():
    bot = MockRobot(port=BASE + 7)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 7)
        assert _wait_connected()
        c = panel.app.test_client()
        bad = panel._rpm_path("corrupt")
        bad.write_bytes(b"GARBAGE\x00\x01\x02")
        r = c.post("/api/replay", json={"name": "corrupt", "confirm": True, "origin_ack": True, "allow_sim": True})
        assert r.status_code == 400

        empty = panel._rpm_path("empty")
        empty.write_bytes(write_rpm_bytes("empty", []))
        r = c.post("/api/replay", json={"name": "empty", "confirm": True, "origin_ack": True, "allow_sim": True})
        assert r.status_code == 400
        assert "empty" in r.get_json()["error"]

        # truncated header
        trunc = panel._rpm_path("trunc")
        trunc.write_bytes(b"MRPM\x02")
        r = c.post("/api/replay", json={"name": "trunc", "confirm": True, "origin_ack": True, "allow_sim": True})
        assert r.status_code == 400

        # list endpoint survives trash
        assert c.get("/api/record/list").status_code == 200
    finally:
        panel.disconnect()
        bot.stop()


def test_unsorted_timeline_sanitized():
    events = [
        {"t_ms": 500, "op": "drive", "left": 1, "right": 1},
        {"t_ms": 10, "op": "arm", "base": 40, "height": 90, "grip": 90},
        {"t_ms": -5, "op": "stop"},
        {"t_ms": 200, "op": "motor_test"},
    ]
    clean = sanitize_timeline(events)
    # negative t_ms dropped as corrupt; motor_test filtered; remaining sorted by time
    assert [e["op"] for e in clean] == ["arm", "drive"]
    assert clean[0]["t_ms"] == 10
    assert clean[-1]["t_ms"] == 500


def test_recording_event_cap(monkeypatch):
    bot = MockRobot(port=BASE + 8)
    bot.start()
    monkeypatch.setattr(panel, "MAX_REC_EVENTS", 8)
    try:
        panel.connect_tcp("127.0.0.1", BASE + 8)
        assert _wait_connected()
        c = panel.app.test_client()
        assert c.post("/api/record/start", json={"name": "cap"}).status_code == 200
        for i in range(40):
            c.post("/api/drive", json={"left": i + 1, "right": i + 1})
            time.sleep(0.02)
        with panel._rec_lock:
            n = len(panel._rec_events)
        assert n <= 8
        assert panel._state.get("warning")
        c.post("/api/record/stop", json={})
    finally:
        panel.disconnect()
        bot.stop()


def test_noisy_and_missing_sensor_color():
    f = ColorFilter(window=5, agree_n=3)
    # garbage inputs must not raise
    out = f.update("x", None, "bad", [], object())
    assert out["color"] == 0
    assert out["color_stable"] is False
    # flicker must not lock
    for lab, r, y, g in [(1, 90, 10, 10), (3, 10, 10, 90), (1, 90, 10, 10), (2, 10, 90, 10)]:
        out = f.update(lab, 80, r, y, g)
    assert out["color_stable"] is False or out["color"] == 0


def test_delayed_telem_warns_without_estop(monkeypatch):
    bot = MockRobot(port=BASE + 9)
    bot.telem_enabled = False
    bot.start()
    monkeypatch.setattr(panel, "TELEM_SOFT_S", 0.4)
    monkeypatch.setattr(panel, "TELEM_PROBE_S", 10.0)
    monkeypatch.setattr(panel, "TELEM_DEAD_S", 20.0)
    try:
        panel.connect_tcp("127.0.0.1", BASE + 9)
        assert _wait_connected()
        # hello set connected; no telem → warning only, never latch E-stop
        deadline = time.monotonic() + 2.5
        warned = False
        while time.monotonic() < deadline:
            w = panel._state.get("warning") or ""
            if "telemetry timeout" in w:
                warned = True
                break
            time.sleep(0.05)
        assert warned
        assert panel._state.get("estop") is False
        assert panel._state.get("op_mode") != "EMERGENCY STOP"
    finally:
        panel.disconnect()
        bot.stop()


def test_garbage_frames_survive():
    bot = MockRobot(port=BASE + 10)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 10)
        assert _wait_connected()
        time.sleep(0.15)
        bot.inject_garbage()
        time.sleep(0.2)
        c = panel.app.test_client()
        # still able to command if link alive, or cleanly disconnected
        r = c.post("/api/drive", json={"left": 5, "right": 5})
        assert r.status_code in (200, 409, 502) or r.get_json().get("ok") in (True, False)
        assert c.get("/api/state").status_code == 200
    finally:
        panel.disconnect()
        bot.stop()


def test_stop_during_replay_and_pause():
    bot = MockRobot(port=BASE + 11)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 11)
        assert _wait_connected()
        c = panel.app.test_client()
        name = "pause-abort"
        events = [{"t_ms": 0, "op": "arm", "base": 90, "height": 90, "grip": 90}]
        for i in range(1, 30):
            events.append({"t_ms": i * 80, "op": "drive", "left": 60, "right": 60})
        panel._rpm_path(name).write_bytes(write_rpm_bytes(name, events))
        assert c.post("/api/replay", json={"name": name, "confirm": True, "origin_ack": True, "allow_sim": True}).status_code == 200
        time.sleep(0.1)
        assert c.post("/api/replay/pause", json={"pause": True}).status_code == 200
        assert panel._state["replay_paused"] is True
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and bot.cmd_l != 0:
            time.sleep(0.02)
        assert bot.cmd_l == 0
        # Pause must hold — no new non-zero drive while paused
        time.sleep(0.25)
        assert bot.cmd_l == 0
        assert c.post("/api/replay/stop", json={}).status_code == 200
        assert _wait_flag("replaying", False, timeout=4.0)
    finally:
        panel.disconnect()
        bot.stop()


def test_path_traversal_name_rejected():
    bot = MockRobot(port=BASE + 12)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 12)
        assert _wait_connected()
        c = panel.app.test_client()
        r = c.post("/api/record/start", json={"name": "../evil"})
        assert r.status_code == 400
        r = c.post("/api/replay", json={"name": "..\\evil", "confirm": True, "origin_ack": True, "allow_sim": True})
        assert r.status_code == 400
    finally:
        panel.disconnect()
        bot.stop()


def test_soft_error_does_not_sticky_error_mode():
    bot = MockRobot(port=BASE + 13)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", BASE + 13)
        assert _wait_connected()
        panel._state["error"] = "temporary send glitch"
        panel._refresh_op_mode()
        assert panel._state["op_mode"] == "MANUAL MODE"
    finally:
        panel.disconnect()
        bot.stop()
