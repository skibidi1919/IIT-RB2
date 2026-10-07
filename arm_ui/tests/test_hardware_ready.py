"""Hardware readiness — dry-run, preflight, link kind, HW test limits."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hardware_ready import (
    HW_TEST_DRIVE_MAX,
    clamp_hw_drive,
    classify_host,
    resolve_link_kind,
    run_preflight,
)
from mock_robot import MockRobot

import app as panel

PORT = 15001


def setup_module():
    panel.RPM_DIR = Path(__file__).resolve().parent / "_tmp_hw_rpm"
    panel.RPM_DIR.mkdir(parents=True, exist_ok=True)


def _reset():
    try:
        panel._replay_stop.set()
        with panel._rec_lock:
            panel._recording = False
            panel._replaying = False
        panel._state["estop"] = False
        panel._state["fault"] = False
        panel._state["error"] = None
        panel._state["dry_run"] = True
        panel.disconnect()
    except Exception:
        pass


def teardown_function():
    _reset()


def test_classify_and_resolve_link_kind():
    assert classify_host("127.0.0.1:3333") == "SIMULATED"
    assert classify_host("192.168.29.222:3333") == "REAL_HARDWARE"
    assert resolve_link_kind(connected=True, host="127.0.0.1:3334", dry_run=True) == "DRY_RUN"
    assert resolve_link_kind(connected=True, host="127.0.0.1:3334", dry_run=False) == "SIMULATED"
    assert resolve_link_kind(connected=True, host="192.168.29.222:3333", dry_run=False) == "REAL_HARDWARE"


def test_clamp_hw_drive():
    assert clamp_hw_drive(255, -255) == (HW_TEST_DRIVE_MAX, -HW_TEST_DRIVE_MAX)


def test_dry_run_suppresses_drive_but_allows_stop():
    bot = MockRobot(port=PORT)
    bot.start()
    try:
        panel._state["dry_run"] = True
        panel.connect_tcp("127.0.0.1", PORT)
        time.sleep(0.2)
        c = panel.app.test_client()
        assert panel._state["link_kind"] == "DRY_RUN"
        before = list(bot.commands)
        assert c.post("/api/drive", json={"left": 100, "right": 100}).get_json()["ok"] is True
        time.sleep(0.1)
        # No new drive while dry-run (stop-on-connect may have sent stop)
        assert "drive" not in bot.commands[len(before):] or bot.cmd_l == 0
        assert c.post("/api/stop", json={}).status_code == 200
        assert panel._state.get("tx_suppressed", 0) >= 1
    finally:
        panel.disconnect()
        bot.stop()


def test_arm_motors_requires_ack():
    bot = MockRobot(port=PORT + 1)
    bot.start()
    try:
        panel._state["dry_run"] = True
        panel.connect_tcp("127.0.0.1", PORT + 1)
        time.sleep(0.15)
        c = panel.app.test_client()
        r = c.post("/api/dry_run", json={"enabled": False})
        assert r.status_code == 400
        r = c.post("/api/dry_run", json={"enabled": False, "arm_ack": True})
        assert r.status_code == 200
        assert panel._state["dry_run"] is False
        assert panel._state["link_kind"] == "SIMULATED"
    finally:
        panel.disconnect()
        bot.stop()


def test_preflight_blocks_when_disconnected():
    panel._state["connected"] = False
    panel._state["dry_run"] = True
    panel._refresh_op_mode()
    report = run_preflight(panel._state, events=[{"t_ms": 0, "op": "stop"}])
    assert report["ok_to_replay"] is False
    assert any(c["id"] == "link" and c["status"] == "FAIL" for c in report["checks"])
    assert "cannot verify" in report["origin_limitation"].lower()


def test_preflight_and_hw_test_connection():
    bot = MockRobot(port=PORT + 2)
    bot.start()
    try:
        panel._state["dry_run"] = True
        panel.connect_tcp("127.0.0.1", PORT + 2)
        time.sleep(0.25)
        c = panel.app.test_client()
        # seed origin for preflight
        panel._state["origin"] = {"base": 90, "height": 90, "grip": 90}
        from rpm_format import write_rpm_bytes
        name = "pf"
        events = [
            {"t_ms": 0, "op": "arm", "base": 90, "height": 90, "grip": 90},
            {"t_ms": 100, "op": "drive", "left": 40, "right": 40},
        ]
        panel._rpm_path(name).write_bytes(write_rpm_bytes(name, events))
        r = c.get(f"/api/preflight?name={name}")
        body = r.get_json()
        assert r.status_code == 200
        assert body["ok_to_replay"] is True
        assert any(c["id"] == "origin_pose" and c["status"] == "WARNING" for c in body["checks"])

        r = c.post("/api/hw_test", json={"action": "connection"})
        assert r.get_json()["ok"] is True
        assert r.get_json()["link_kind"] == "DRY_RUN"

        r = c.post("/api/hw_test", json={"action": "color"})
        assert r.get_json()["independent_of_recording"] is True
    finally:
        panel.disconnect()
        bot.stop()


def test_dry_run_replay_does_not_drive_motors():
    bot = MockRobot(port=PORT + 3)
    bot.start()
    try:
        panel._state["dry_run"] = True
        panel.connect_tcp("127.0.0.1", PORT + 3)
        time.sleep(0.2)
        c = panel.app.test_client()
        from rpm_format import write_rpm_bytes
        name = "dry-replay"
        events = [
            {"t_ms": 0, "op": "arm", "base": 90, "height": 90, "grip": 90},
            {"t_ms": 50, "op": "drive", "left": 80, "right": 80},
            {"t_ms": 120, "op": "stop"},
        ]
        panel._rpm_path(name).write_bytes(write_rpm_bytes(name, events))
        bot.commands.clear()
        r = c.post("/api/replay", json={
            "name": name, "confirm": True, "origin_ack": True, "allow_sim": True,
        })
        assert r.status_code == 200, r.get_json()
        time.sleep(0.8)
        c.post("/api/replay/stop", json={})
        time.sleep(0.3)
        # Dry-run may send stop/arm suppress — must not leave wheels spinning
        assert bot.cmd_l == 0 and bot.cmd_r == 0
        assert "drive" not in bot.commands or bot.cmd_l == 0
    finally:
        panel.disconnect()
        bot.stop()
