"""Structured ring-buffer log for robot field debugging."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class LogEntry:
    t: float
    cat: str
    msg: str

    def format(self) -> str:
        ms = int((self.t % 1.0) * 1000)
        stamp = time.strftime("%H:%M:%S", time.localtime(self.t))
        return f"{stamp}.{ms:03d} [{self.cat}] {self.msg}"


class RobotLog:
    def __init__(self, maxlen: int = 200) -> None:
        self._lock = threading.Lock()
        self._entries: deque[LogEntry] = deque(maxlen=maxlen)

    def add(self, cat: str, msg: str) -> None:
        entry = LogEntry(time.time(), cat.upper()[:8], msg)
        with self._lock:
            self._entries.appendleft(entry)

    def lines(self, n: int = 40, cats: Iterable[str] | None = None) -> list[str]:
        with self._lock:
            items = list(self._entries)
        if cats:
            allow = {c.upper() for c in cats}
            items = [e for e in items if e.cat in allow]
        return [e.format() for e in items[:n]]

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()
