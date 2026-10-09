"""Browser ↔ panel realtime fan-out (WebSocket). Robot link stays raw TCP :3333."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable


class WsHub:
    """Thread-safe WebSocket client set + dirty-flag publisher."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clients: set[Any] = set()
        self._dirty = threading.Event()
        self._seq = 0
        self._pub: threading.Thread | None = None
        self._stop = threading.Event()

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def register(self, ws: Any) -> None:
        with self._lock:
            self._clients.add(ws)
        self.bump()

    def unregister(self, ws: Any) -> None:
        with self._lock:
            self._clients.discard(ws)

    def bump(self) -> None:
        with self._lock:
            self._seq += 1
        self._dirty.set()

    def seq(self) -> int:
        with self._lock:
            return self._seq

    def broadcast(self, payload: dict[str, Any]) -> None:
        raw = json.dumps(payload, separators=(",", ":"), default=str)
        dead: list[Any] = []
        with self._lock:
            clients = list(self._clients)
        for ws in clients:
            try:
                ws.send(raw)
            except Exception:  # noqa: BLE001
                dead.append(ws)
        for ws in dead:
            self.unregister(ws)

    def start_publisher(
        self,
        snapshot_fn: Callable[[], dict[str, Any]],
        *,
        min_hz: float = 20.0,
        idle_hz: float = 4.0,
    ) -> None:
        if self._pub is not None and self._pub.is_alive():
            return
        self._stop.clear()
        min_dt = 1.0 / max(1.0, min_hz)
        idle_dt = 1.0 / max(0.5, idle_hz)

        def _loop() -> None:
            last_sent = 0.0
            last_seq = -1
            while not self._stop.is_set():
                triggered = self._dirty.wait(timeout=idle_dt)
                self._dirty.clear()
                if self.client_count == 0:
                    continue
                now = time.monotonic()
                seq = self.seq()
                if not triggered and seq == last_seq and (now - last_sent) < idle_dt:
                    continue
                if (now - last_sent) < min_dt and seq == last_seq:
                    time.sleep(min_dt - (now - last_sent))
                try:
                    snap = snapshot_fn()
                    snap["ws_seq"] = seq
                    snap["transport"] = "websocket"
                    self.broadcast({"t": "state", "s": snap})
                    last_sent = time.monotonic()
                    last_seq = seq
                except Exception:  # noqa: BLE001
                    time.sleep(0.05)

        self._pub = threading.Thread(target=_loop, daemon=True, name="ws-pub")
        self._pub.start()

    def stop(self) -> None:
        self._stop.set()
        self._dirty.set()
