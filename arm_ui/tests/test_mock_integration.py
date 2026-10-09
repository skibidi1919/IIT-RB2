"""End-to-end against MockRobot — connect, command, record, estop, replay gates."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mock_robot import MockRobot

import app as panel


PORT = 13334


def setup_module():
    panel.RPM_DIR = Path(__file__).resolve().parent / "_tmp_recordings"
    panel.RPM_DIR.mkdir(parents=True, exist_ok=True)


def _reset_panel():
    try:
        panel._replay_stop.set()
        with panel._rec_lock:
            panel._recording = False
            panel._replaying = False
            panel._replay_paused = False
            panel._rec_events = []
            panel._rec_name = None
        panel._state["recording"] = False
        panel._state["replaying"] = False
        panel._state["estop"] = False
        panel._state["error"] = None
        panel._state["ready_replay"] = False
        panel._state["dry_run"] = False
        panel.disconnect()
    except Exception:
        pass


def setup_function():
    panel._state["dry_run"] = False
    panel._refresh_op_mode()


def teardown_function():
    _reset_panel()


def test_connect_drive_estop_disconnect():
    bot = MockRobot(port=PORT)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", PORT)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not panel._state.get("connected"):
            time.sleep(0.05)
        assert panel._state["connected"]

        # wait for hello/telem
        time.sleep(0.2)
        assert panel._state.get("pca_ok")

        client = panel.app.test_client()
        r = client.post("/api/drive", json={"left": 100, "right": 100})
        assert r.get_json()["ok"] is True
        time.sleep(0.15)
        assert any(c == "drive" for c in bot.commands)

        r = client.post("/api/estop", json={})
        assert r.get_json()["ok"] is True
        # Chord hard-stop: zeros motors, never latches
        assert panel._state["estop"] is False
        assert "EMERGENCY" not in (panel._state.get("op_mode") or "")

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and (bot.cmd_l != 0 or bot.cmd_r != 0):
            time.sleep(0.02)
        assert bot.cmd_l == 0 and bot.cmd_r == 0

        # still commandable immediately
        r = client.post("/api/drive", json={"left": 50, "right": 50})
        assert r.status_code == 200
    finally:
        panel.disconnect()
        bot.stop()


def test_record_timeline_and_replay_confirm():
    bot = MockRobot(port=PORT + 1)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", PORT + 1)
        time.sleep(0.25)
        client = panel.app.test_client()

        name = "unit-test-move"
        r = client.post("/api/record/start", json={"name": name})
        assert r.status_code == 200, r.get_json()
        assert r.get_json()["ok"] is True

        client.post("/api/drive", json={"left": 120, "right": 120})
        time.sleep(0.08)
        client.post("/api/drive", json={"left": 0, "right": 0})
        client.post("/api/arm", json={"base": 60})
        time.sleep(0.05)

        r = client.post("/api/record/stop", json={})
        body = r.get_json()
        assert body["ok"] is True
        assert body["events"] >= 2
        path = panel._rpm_path(name)
        assert path.is_file()

        # replay without confirm
        r = client.post("/api/replay", json={"name": name})
        assert r.status_code == 400
        assert "confirm" in r.get_json()["error"]
        # confirm but no origin ack
        r = client.post("/api/replay", json={"name": name, "confirm": True})
        assert r.status_code == 400
        assert "origin_ack" in r.get_json()["error"]

        r = client.post("/api/replay", json={"name": name, "confirm": True, "origin_ack": True, "allow_sim": True})
        assert r.status_code == 200
        client.post("/api/replay/stop", json={})
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and panel._state.get("replaying"):
            time.sleep(0.05)
        assert panel._state["replaying"] is False
    finally:
        panel.disconnect()
        bot.stop()


def test_double_record_rejected():
    bot = MockRobot(port=PORT + 2)
    bot.start()
    try:
        panel.connect_tcp("127.0.0.1", PORT + 2)
        time.sleep(0.2)
        client = panel.app.test_client()
        assert client.post("/api/record/start", json={"name": "a"}).status_code == 200
        r = client.post("/api/record/start", json={"name": "b"})
        assert r.status_code == 409
        client.post("/api/record/stop", json={})
    finally:
        panel.disconnect()
        bot.stop()
