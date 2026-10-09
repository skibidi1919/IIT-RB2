"""WebSocket hub fan-out basics (no browser)."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ws_hub import WsHub


class FakeWs:
    def __init__(self) -> None:
        self.msgs: list[str] = []
        self.dead = False

    def send(self, raw: str) -> None:
        if self.dead:
            raise RuntimeError("closed")
        self.msgs.append(raw)


def test_hub_broadcast_and_drop_dead():
    hub = WsHub()
    a, b = FakeWs(), FakeWs()
    hub.register(a)
    hub.register(b)
    assert hub.client_count == 2
    hub.broadcast({"t": "state", "n": 1})
    assert len(a.msgs) == 1 and len(b.msgs) == 1
    b.dead = True
    hub.broadcast({"t": "state", "n": 2})
    assert hub.client_count == 1
    assert '"n":2' in a.msgs[-1]


def test_publisher_emits_on_bump():
    hub = WsHub()
    ws = FakeWs()
    hub.register(ws)
    n = {"i": 0}

    def snap():
        n["i"] += 1
        return {"ok": True, "i": n["i"]}

    hub.start_publisher(snap, min_hz=50.0, idle_hz=20.0)
    hub.bump()
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not ws.msgs:
        time.sleep(0.02)
    assert ws.msgs, "expected publisher push"
    hub.stop()
