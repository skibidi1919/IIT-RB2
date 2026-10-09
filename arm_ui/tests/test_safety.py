"""State machine / safety gates."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from safety import OpMode, allow_record_start, allow_replay_start, mode_from_flags


def test_modes():
    assert mode_from_flags(
        connected=False, estop=False, error=False,
        recording=False, replaying=False, paused=False, ready_replay=False,
    ) == OpMode.DISCONNECTED
    # E-stop latch removed — estop flag must not lock the panel
    assert mode_from_flags(
        connected=True, estop=True, error=False,
        recording=False, replaying=False, paused=False, ready_replay=False,
    ) == OpMode.MANUAL
    assert mode_from_flags(
        connected=True, estop=False, error=False,
        recording=True, replaying=False, paused=False, ready_replay=False,
    ) == OpMode.RECORDING
    assert mode_from_flags(
        connected=True, estop=False, error=False,
        recording=False, replaying=True, paused=True, ready_replay=False,
    ) == OpMode.PAUSED
    assert mode_from_flags(
        connected=True, estop=False, error=False,
        recording=False, replaying=False, paused=False, ready_replay=True,
    ) == OpMode.READY_REPLAY


def test_record_guards():
    assert allow_record_start(connected=False, estop=False, recording=False, replaying=False)
    assert allow_record_start(connected=True, estop=True, recording=False, replaying=False) is None
    assert allow_record_start(connected=True, estop=False, recording=True, replaying=False)
    assert allow_record_start(connected=True, estop=False, recording=False, replaying=True)
    assert allow_record_start(connected=True, estop=False, recording=False, replaying=False) is None


def test_replay_requires_confirm():
    assert allow_replay_start(
        connected=True, estop=False, recording=False, replaying=False, confirm=False
    ) == "confirm required"
    assert allow_replay_start(
        connected=True, estop=False, recording=False, replaying=False, confirm=True
    ) is None
    assert allow_replay_start(
        connected=True, estop=False, recording=False, replaying=True, confirm=True
    ) == "already replaying"
