"""Safety gates for physical robot control — modes, E-stop, command policy."""

from __future__ import annotations

from enum import Enum


class OpMode(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    MANUAL = "MANUAL MODE"
    RECORDING = "RECORDING"
    READY_REPLAY = "READY FOR REPLAY"
    REPLAYING = "REPLAYING"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    ESTOP = "EMERGENCY STOP"


# Commands allowed while recording (motion timeline only)
RECORDABLE_OPS = frozenset({"drive", "arm", "stop", "center", "conveyor"})

# Blocked during replay (unless estop/stop)
BLOCKED_DURING_REPLAY = frozenset({
    "drive", "arm", "center", "zero", "conveyor", "motor_test", "rec", "color_cal",
})


def mode_from_flags(
    *,
    connected: bool,
    estop: bool = False,
    fault: bool = False,
    error: bool = False,  # deprecated soft flag; ignored for mode (use fault)
    recording: bool,
    replaying: bool,
    paused: bool,
    ready_replay: bool,
) -> OpMode:
    """Soft command `error` strings must not sticky-lock ERROR — only `fault`."""
    del error  # soft errors stay in UI text only
    del estop  # E-stop latch removed — never sticky-lock the panel
    if not connected:
        return OpMode.DISCONNECTED
    if fault:
        return OpMode.ERROR
    if recording:
        return OpMode.RECORDING
    if replaying and paused:
        return OpMode.PAUSED
    if replaying:
        return OpMode.REPLAYING
    if ready_replay:
        return OpMode.READY_REPLAY
    return OpMode.MANUAL


def allow_manual_motion(*, connected: bool, estop: bool = False, replaying: bool) -> bool:
    del estop
    return connected and not replaying


def allow_record_start(*, connected: bool, estop: bool = False, recording: bool, replaying: bool) -> str | None:
    del estop
    if not connected:
        return "not connected"
    if recording:
        return "already recording"
    if replaying:
        return "replaying"
    return None


def allow_replay_start(
    *,
    connected: bool,
    estop: bool = False,
    recording: bool,
    replaying: bool,
    confirm: bool,
) -> str | None:
    del estop
    if not connected:
        return "not connected"
    if recording:
        return "recording"
    if replaying:
        return "already replaying"
    if not confirm:
        return "confirm required"
    return None
