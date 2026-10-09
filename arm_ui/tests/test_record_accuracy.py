"""Host recording timeline: tight dedup + drive keyframes + host clock."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import app as panel


def test_drive_keyframes_while_held(monkeypatch):
    panel._recording = True
    panel._rec_t0 = time.perf_counter()
    panel._rec_events = []
    panel._last_cmd_sig = None
    panel._last_cmd_mono = 0.0
    monkeypatch.setattr(panel, "REC_DEDUP_S", 0.001)
    monkeypatch.setattr(panel, "REC_DRIVE_KEYFRAME_S", 0.015)

    panel._host_record_append({"op": "drive", "left": 100, "right": 100})
    time.sleep(0.02)
    panel._host_record_append({"op": "drive", "left": 100, "right": 100})
    time.sleep(0.02)
    panel._host_record_append({"op": "drive", "left": 100, "right": 100})
    assert len(panel._rec_events) >= 2
    assert panel._rec_events[-1]["t_ms"] >= panel._rec_events[0]["t_ms"]
    panel._recording = False


def test_rapid_arm_retargets_kept():
    panel._recording = True
    panel._rec_t0 = time.perf_counter()
    panel._rec_events = []
    panel._last_cmd_sig = None
    panel._last_cmd_mono = 0.0
    for i, ang in enumerate((10, 20, 30, 40)):
        panel._host_record_append({"op": "arm", "base": ang})
        time.sleep(0.004)
    bases = [e.get("base") for e in panel._rec_events if e.get("op") == "arm"]
    assert bases == [10, 20, 30, 40]
    panel._recording = False
