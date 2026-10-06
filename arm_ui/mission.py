"""Meowler mission script — host-side DSL over TCP (no ESP reflash for logic).

Example (arm_ui/missions/triage.ms):

    # RFGYC Senior AM — patient triage helper
    conveyor 160
    wait 800
    conveyor 0
    wait_color any 4000
    if_color red
      play drop_red
    elif_color yellow
      play drop_yellow
    elif_color green
      play drop_green
    end
    stop

Commands:
  drive L R          | stop | center | zero | motor_test
  arm B H G          | base N | height N | grip N
  conveyor S         | -255..255  (0 = stop)
  wait MS
  wait_color NAME MS | red|yellow|green|any  (any = first non-unknown)
  wait_tof MM MS     | until distance_mm <= MM
  play NAME          | replay recordings/NAME.rpm
  record_start NAME  | record_stop
  log TEXT
  if_color NAME ... elif_color NAME ... else ... end
  # comment
"""

from __future__ import annotations

import re
import threading
import time
from pathlib import Path
from typing import Any, Callable

COLOR = {"unknown": 0, "red": 1, "yellow": 2, "green": 3, "any": -1}
_SAFE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

MISSION_DIR = Path(__file__).resolve().parent / "missions"


class MissionError(Exception):
    pass


def _tok(line: str) -> list[str]:
    # keep quoted strings as one token
    parts: list[str] = []
    cur = []
    q = None
    for ch in line.strip():
        if q:
            if ch == q:
                parts.append("".join(cur))
                cur = []
                q = None
            else:
                cur.append(ch)
            continue
        if ch in "\"'":
            q = ch
            continue
        if ch.isspace():
            if cur:
                parts.append("".join(cur))
                cur = []
            continue
        cur.append(ch)
    if q:
        raise MissionError("unclosed quote")
    if cur:
        parts.append("".join(cur))
    return parts


def parse(src: str) -> list[tuple[str, list[str]]]:
    """Return list of (op, args) with if/elif/else/end kept as structured ops."""
    out: list[tuple[str, list[str]]] = []
    for i, raw in enumerate(src.splitlines(), 1):
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        try:
            t = _tok(s)
        except MissionError as e:
            raise MissionError(f"line {i}: {e}") from e
        op = t[0].lower()
        out.append((op, t[1:]))
    return out


class MissionRunner:
    def __init__(
        self,
        *,
        send_msg: Callable[[Any], bool],
        get_state: Callable[[], dict],
        play_rpm: Callable[[str, threading.Event], None],
        record_start: Callable[[str], None] | None = None,
        record_stop: Callable[[], None] | None = None,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self._send = send_msg
        self._state = get_state
        self._play = play_rpm
        self._rec_start = record_start
        self._rec_stop = record_stop
        self._log = log or (lambda m: None)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.running = False
        self.last_error: str | None = None
        self.line = 0

    def stop(self) -> None:
        self._stop.set()

    def is_running(self) -> bool:
        return self.running

    def start(self, src: str) -> None:
        if self.running:
            raise MissionError("already running")
        ops = parse(src)
        self._stop.clear()
        self.last_error = None
        self.running = True
        self._thread = threading.Thread(target=self._run, args=(ops,), daemon=True)
        self._thread.start()

    def _sleep(self, ms: float) -> bool:
        return not self._stop.wait(max(0.0, ms) / 1000.0)

    def _pb(self):
        from meowler_pb import meowler_pb2 as pb

        return pb

    def _cmd(self, build) -> None:
        pb = self._pb()
        msg = pb.ClientToRobot()
        build(msg)
        if not self._send(msg):
            raise MissionError("send failed (not connected?)")

    def _run(self, ops: list[tuple[str, list[str]]]) -> None:
        try:
            self._exec_block(ops, 0, len(ops))
            self._log("MISSION done")
        except MissionError as e:
            self.last_error = str(e)
            self._log(f"MISSION err: {e}")
        except Exception as e:  # noqa: BLE001
            self.last_error = str(e)
            self._log(f"MISSION crash: {e}")
        finally:
            try:
                self._cmd(lambda m: setattr(m, "stop", True))
            except Exception:  # noqa: BLE001
                pass
            self.running = False

    def _exec_block(self, ops: list[tuple[str, list[str]]], start: int, end: int) -> int:
        i = start
        while i < end:
            if self._stop.is_set():
                raise MissionError("stopped")
            op, args = ops[i]
            self.line = i + 1
            if op == "if_color":
                i = self._exec_if(ops, i, end)
                continue
            if op in ("elif_color", "else", "end"):
                raise MissionError(f"line {i+1}: unexpected {op}")
            self._exec_one(op, args)
            i += 1
        return i

    def _exec_if(self, ops: list[tuple[str, list[str]]], i: int, end: int) -> int:
        # collect branches until matching end
        branches: list[tuple[int | None, int, int]] = []  # (color|None=else, start, end)
        op, args = ops[i]
        if op != "if_color" or not args:
            raise MissionError(f"line {i+1}: if_color NAME")
        cur_color = self._color_arg(args[0])
        cur_start = i + 1
        j = i + 1
        depth = 0
        while j < end:
            o, a = ops[j]
            if o == "if_color":
                depth += 1
            elif o == "end":
                if depth == 0:
                    branches.append((cur_color, cur_start, j))
                    break
                depth -= 1
            elif depth == 0 and o == "elif_color":
                branches.append((cur_color, cur_start, j))
                if not a:
                    raise MissionError(f"line {j+1}: elif_color NAME")
                cur_color = self._color_arg(a[0])
                cur_start = j + 1
            elif depth == 0 and o == "else":
                branches.append((cur_color, cur_start, j))
                cur_color = None
                cur_start = j + 1
            j += 1
        else:
            raise MissionError(f"line {i+1}: if_color missing end")

        live = int(self._state().get("color") or 0)
        chosen = None
        for col, a, b in branches:
            if col is None or col == live or (col == -1 and live != 0):
                chosen = (a, b)
                break
        if chosen:
            self._exec_block(ops, chosen[0], chosen[1])
        return j + 1

    def _color_arg(self, name: str) -> int:
        k = name.lower()
        if k not in COLOR:
            raise MissionError(f"bad color {name!r}")
        return COLOR[k]

    def _exec_one(self, op: str, args: list[str]) -> None:
        if op == "wait":
            if not args:
                raise MissionError("wait MS")
            if not self._sleep(float(args[0])):
                raise MissionError("stopped")
            return
        if op == "wait_color":
            if len(args) < 2:
                raise MissionError("wait_color NAME MS")
            want = self._color_arg(args[0])
            timeout = float(args[1])
            t0 = time.monotonic()
            while (time.monotonic() - t0) * 1000 < timeout:
                if self._stop.is_set():
                    raise MissionError("stopped")
                c = int(self._state().get("color") or 0)
                if want == -1 and c != 0:
                    return
                if c == want:
                    return
                if not self._sleep(50):
                    raise MissionError("stopped")
            raise MissionError(f"wait_color {args[0]} timeout")
        if op == "wait_tof":
            if len(args) < 2:
                raise MissionError("wait_tof MM MS")
            mm = float(args[0])
            timeout = float(args[1])
            t0 = time.monotonic()
            while (time.monotonic() - t0) * 1000 < timeout:
                if self._stop.is_set():
                    raise MissionError("stopped")
                d = float(self._state().get("distance_mm") or 99999)
                if 0 < d <= mm:
                    return
                if not self._sleep(40):
                    raise MissionError("stopped")
            raise MissionError("wait_tof timeout")
        if op == "drive":
            if len(args) < 2:
                raise MissionError("drive L R")
            L, R = int(args[0]), int(args[1])
            self._cmd(lambda m: (setattr(m.drive, "left", L), setattr(m.drive, "right", R)))
            return
        if op == "stop":
            self._cmd(lambda m: setattr(m, "stop", True))
            return
        if op == "center":
            self._cmd(lambda m: setattr(m, "center", True))
            return
        if op == "zero":
            self._cmd(lambda m: setattr(m, "zero", True))
            return
        if op == "motor_test":
            self._cmd(lambda m: setattr(m, "motor_test", True))
            return
        if op == "arm":
            if len(args) < 3:
                raise MissionError("arm B H G")
            b, h, g = int(args[0]), int(args[1]), int(args[2])

            def build(m):
                m.arm.base = b
                m.arm.height = h
                m.arm.grip = g
                m.arm.set_base = m.arm.set_height = m.arm.set_grip = True

            self._cmd(build)
            return
        if op in ("base", "height", "grip"):
            if not args:
                raise MissionError(f"{op} N")
            v = int(args[0])

            def build(m, axis=op, val=v):
                if axis == "base":
                    m.arm.base = val
                    m.arm.set_base = True
                elif axis == "height":
                    m.arm.height = val
                    m.arm.set_height = True
                else:
                    m.arm.grip = val
                    m.arm.set_grip = True

            self._cmd(build)
            return
        if op == "conveyor":
            if not args:
                raise MissionError("conveyor S")
            sp = max(-255, min(255, int(args[0])))
            self._cmd(lambda m, s=sp: setattr(m.conveyor, "speed", s))
            return
        if op == "play":
            if not args or not _SAFE.match(args[0]):
                raise MissionError("play NAME")
            self._log(f"MISSION play {args[0]}.rpm")
            self._play(args[0], self._stop)
            return
        if op == "record_start":
            if not self._rec_start or not args or not _SAFE.match(args[0]):
                raise MissionError("record_start NAME")
            self._rec_start(args[0])
            return
        if op == "record_stop":
            if not self._rec_stop:
                raise MissionError("record_stop unavailable")
            self._rec_stop()
            return
        if op == "log":
            self._log("MISSION " + (" ".join(args) if args else ""))
            return
        raise MissionError(f"unknown op {op!r}")


def list_missions() -> list[dict]:
    MISSION_DIR.mkdir(parents=True, exist_ok=True)
    out = []
    for p in sorted(MISSION_DIR.glob("*.ms"), key=lambda x: x.name.lower()):
        out.append({"name": p.stem, "bytes": p.stat().st_size})
    return out


def load_mission(name: str) -> str:
    if not _SAFE.match(name):
        raise MissionError("bad name")
    p = MISSION_DIR / f"{name}.ms"
    if not p.is_file():
        raise MissionError("missing")
    return p.read_text(encoding="utf-8")


def save_mission(name: str, src: str) -> Path:
    if not _SAFE.match(name):
        raise MissionError("bad name")
    MISSION_DIR.mkdir(parents=True, exist_ok=True)
    p = MISSION_DIR / f"{name}.ms"
    p.write_text(src.replace("\r\n", "\n"), encoding="utf-8")
    return p
