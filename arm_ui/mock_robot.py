"""Lightweight mock Meowler robot — TCP :port length-prefixed protobuf.

Used for automated tests without hardware. Run:
  uv run python mock_robot.py --port 3334
"""

from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

from meowler_pb import meowler_pb2 as pb


class MockRobot:
    def __init__(self, host: str = "127.0.0.1", port: int = 3334) -> None:
        self.host = host
        self.port = port
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.base = 90
        self.height = 90
        self.grip = 90
        self.cmd_l = 0
        self.cmd_r = 0
        self.conveyor = 0
        self.color = 0
        self.color_conf = 0
        self.color_r = 0
        self.color_g = 0
        self.color_b = 0
        self.distance_mm = 420
        self.commands: list[str] = []
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        # Adversarial knobs
        self.telem_enabled = True
        self.telem_period_s = 0.05
        self.cmd_delay_s = 0.0
        self.garbage_on_connect = False
        self.pca_ok = True
        self.tof_ok = True
        self.imu_ok = True

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        # wait until listening
        for _ in range(50):
            try:
                s = socket.create_connection((self.host, self.port), timeout=0.1)
                s.close()
                return
            except OSError:
                time.sleep(0.05)

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()
        try:
            s = socket.create_connection((self.host, self.port), timeout=0.2)
            s.close()
        except OSError:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)

    def _serve(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(4)
        srv.settimeout(0.3)
        while not self._stop.is_set():
            try:
                conn, _addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                self._clients.append(conn)
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()
        srv.close()

    def _send_frame(self, conn: socket.socket, msg: pb.RobotToClient) -> None:
        data = msg.SerializeToString()
        conn.sendall(struct.pack("<I", len(data)) + data)

    def drop_clients(self) -> None:
        """Hard-disconnect all clients (simulates cable/Wi‑Fi drop)."""
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for c in clients:
            try:
                c.close()
            except OSError:
                pass

    def inject_garbage(self) -> None:
        """Send a malformed length-prefix frame to every client."""
        junk = struct.pack("<I", 0xFFFFFF) + b"\x00\x01\x02"
        with self._lock:
            clients = list(self._clients)
        for c in clients:
            try:
                c.sendall(junk)
            except OSError:
                pass

    def _client(self, conn: socket.socket) -> None:
        conn.settimeout(0.2)
        if self.garbage_on_connect:
            try:
                conn.sendall(b"\xff\xff\x00\x00NOTPROTO")
            except OSError:
                conn.close()
                return
        hello = pb.RobotToClient()
        hello.hello.ip = 0x7F000001
        hello.hello.port = self.port
        hello.hello.pca_ok = self.pca_ok
        hello.hello.tof_ok = self.tof_ok
        hello.hello.imu_ok = self.imu_ok
        try:
            self._send_frame(conn, hello)
        except OSError:
            conn.close()
            return
        buf = bytearray()
        last_telem = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            period = max(0.01, float(self.telem_period_s))
            if self.telem_enabled and now - last_telem >= period:
                last_telem = now
                try:
                    self._send_frame(conn, self._telem())
                except OSError:
                    break
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            while len(buf) >= 4:
                (n,) = struct.unpack_from("<I", buf, 0)
                if n > 4096 or n == 0:
                    buf.clear()
                    break
                if len(buf) < 4 + n:
                    break
                raw = bytes(buf[4 : 4 + n])
                del buf[: 4 + n]
                if self.cmd_delay_s > 0:
                    time.sleep(self.cmd_delay_s)
                self._handle(raw)
        try:
            conn.close()
        except OSError:
            pass
        with self._lock:
            if conn in self._clients:
                self._clients.remove(conn)

    def _telem(self) -> pb.RobotToClient:
        out = pb.RobotToClient()
        t = out.telem
        t.distance_mm = self.distance_mm
        t.cmd_l = self.cmd_l
        t.cmd_r = self.cmd_r
        t.base = self.base
        t.height = self.height
        t.grip = self.grip
        t.pca_ok = self.pca_ok
        t.tof_ok = self.tof_ok
        t.imu_ok = self.imu_ok
        t.wifi_ok = True
        t.color = self.color
        t.color_conf = self.color_conf
        t.color_r = self.color_r
        t.color_g = self.color_g
        t.color_b = self.color_b
        t.conveyor = self.conveyor
        return out

    def _handle(self, raw: bytes) -> None:
        msg = pb.ClientToRobot()
        msg.ParseFromString(raw)
        which = msg.WhichOneof("op")
        with self._lock:
            self.commands.append(which or "?")
        if which == "drive":
            self.cmd_l = int(msg.drive.left)
            self.cmd_r = int(msg.drive.right)
        elif which == "arm":
            if msg.arm.set_base:
                self.base = int(msg.arm.base)
            if msg.arm.set_height:
                self.height = int(msg.arm.height)
            if msg.arm.set_grip:
                self.grip = int(msg.arm.grip)
        elif which == "stop":
            self.cmd_l = self.cmd_r = 0
            self.conveyor = 0
        elif which == "center":
            self.base = self.height = self.grip = 90
        elif which == "conveyor":
            self.conveyor = int(msg.conveyor.speed)
        elif which == "rec":
            # echo begin/end markers as RecEvent for host
            pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=3334)
    args = ap.parse_args()
    bot = MockRobot(args.host, args.port)
    bot.start()
    print(f"mock robot listening on {args.host}:{args.port}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        bot.stop()


if __name__ == "__main__":
    main()
