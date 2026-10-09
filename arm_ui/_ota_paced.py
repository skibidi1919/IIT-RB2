#!/usr/bin/env python3
"""Paced OTA — wait for robot written offset so hotspot links don't drop finish."""
from __future__ import annotations

import binascii
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from meowler_pb import meowler_pb2 as pb  # noqa: E402

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.222"
PORT = 3333
BIN = Path(__file__).resolve().parents[1] / "esp_ui" / "build" / "esp_ui.ino.bin"
CHUNK = 512
VERSION = "gyro-man"


def frame(msg: pb.ClientToRobot) -> bytes:
    b = msg.SerializeToString()
    return struct.pack("<I", len(b)) + b


class Link:
    def __init__(self) -> None:
        self.sock: socket.socket | None = None
        self.buf = bytearray()
        self.connect()

    def connect(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = socket.create_connection((HOST, PORT), timeout=10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
        self.sock.settimeout(0.5)
        self.buf.clear()
        time.sleep(0.15)
        self.drain(0.3)

    def send(self, msg: pb.ClientToRobot) -> None:
        assert self.sock
        self.sock.sendall(frame(msg))

    def drain(self, secs: float) -> list[pb.RobotToClient]:
        assert self.sock
        msgs: list[pb.RobotToClient] = []
        t0 = time.time()
        while time.time() - t0 < secs:
            try:
                chunk = self.sock.recv(8192)
                if not chunk:
                    break
                self.buf.extend(chunk)
            except socket.timeout:
                if msgs:
                    break
                continue
            except OSError:
                break
            while len(self.buf) >= 4:
                n = struct.unpack_from("<I", self.buf, 0)[0]
                if n == 0 or n > 16384:
                    self.buf.clear()
                    break
                if len(self.buf) < 4 + n:
                    break
                m = pb.RobotToClient()
                m.ParseFromString(bytes(self.buf[4 : 4 + n]))
                del self.buf[: 4 + n]
                msgs.append(m)
        return msgs

    @staticmethod
    def last_ota(msgs: list[pb.RobotToClient]) -> pb.OtaStatus | None:
        st = None
        for m in msgs:
            if m.WhichOneof("msg") == "ota":
                st = m.ota
        return st

    def query(self) -> pb.OtaStatus | None:
        msg = pb.ClientToRobot()
        msg.ota.action = 6
        self.send(msg)
        return self.last_ota(self.drain(0.8))


def main() -> None:
    raw = BIN.read_bytes()
    crc = binascii.crc32(raw) & 0xFFFFFFFF
    print(f"upload {BIN.name} {len(raw)} crc=0x{crc:08x} -> {HOST}:{PORT}")

    link = Link()
    abort = pb.ClientToRobot()
    abort.ota.action = 4
    link.send(abort)
    link.drain(0.5)

    begin = pb.ClientToRobot()
    begin.ota.action = 1
    begin.ota.begin.raw_size = len(raw)
    begin.ota.begin.raw_crc32 = crc
    begin.ota.begin.packed_size = len(raw)
    begin.ota.begin.codec = 0
    begin.ota.begin.version = VERSION
    link.send(begin)

    st = None
    for _ in range(40):
        st = link.last_ota(link.drain(0.5)) or link.query()
        if st and st.state == 1:
            break
    if not st or st.state != 1:
        raise SystemExit(f"begin fail: {st}")
    print("begin ok", st.detail)

    off = 0
    t0 = time.perf_counter()
    fails = 0
    while off < len(raw):
        piece = raw[off : off + CHUNK]
        msg = pb.ClientToRobot()
        msg.ota.action = 2
        msg.ota.chunk.offset = off
        msg.ota.chunk.data = piece
        try:
            link.send(msg)
        except OSError as e:
            print("send fail", e, "— reconnect")
            time.sleep(1.0)
            link.connect()
            fails += 1
            if fails > 10:
                raise
            st = link.query()
            if not st or st.state != 1:
                raise SystemExit("lost OTA session")
            off = st.written
            print("resume at", off)
            continue
        off += len(piece)
        if off % (CHUNK * 4) == 0:
            link.drain(0.02)
        if off % (16 * 1024) == 0 or off == len(raw):
            st = link.query()
            if st is None:
                fails += 1
                time.sleep(0.15)
                continue
            if st.state == 4:
                raise SystemExit(f"robot error {st.error} {st.detail!r}")
            if st.written < off:
                for _ in range(40):
                    time.sleep(0.04)
                    st = link.query()
                    if st and st.written >= off:
                        break
                if st and st.written < off:
                    print(f"rewind host {off} -> robot {st.written}")
                    off = st.written
            dt = max(1e-3, time.perf_counter() - t0)
            rw = st.written if st else -1
            print(f"  {off}/{len(raw)} robot={rw} ({off / dt / 1024:.0f} KiB/s)", flush=True)

    for _ in range(60):
        st = link.query()
        if st and st.written >= len(raw) and st.state == 1:
            break
        time.sleep(0.1)
    else:
        raise SystemExit(f"short image written={getattr(st, 'written', None)}")

    print("finish...")
    fin = pb.ClientToRobot()
    fin.ota.action = 3
    try:
        link.send(fin)
    except OSError as e:
        print("finish send err", e)
        time.sleep(2.0)
        link.connect()

    st = None
    for i in range(100):
        try:
            st = link.last_ota(link.drain(0.6)) or link.query()
        except OSError:
            time.sleep(1.0)
            link.connect()
            continue
        if st and st.state in (2, 4, 5):
            break
        if i % 5 == 0:
            try:
                link.query()
            except OSError:
                link.connect()
    print(
        "after finish state=",
        getattr(st, "state", None),
        "err=",
        getattr(st, "error", None),
        "detail=",
        repr(getattr(st, "detail", None)),
        "written=",
        getattr(st, "written", None),
    )
    if not st or st.state != 2:
        raise SystemExit("not READY")

    ap = pb.ClientToRobot()
    ap.ota.action = 5
    link.send(ap)
    print("apply — wait reboot")
    time.sleep(5.0)
    for i in range(40):
        try:
            link.connect()
            for m in link.drain(1.5):
                if m.WhichOneof("msg") == "hello":
                    print(
                        "BACK hello imu=",
                        m.hello.imu_ok,
                        "pca=",
                        m.hello.pca_ok,
                        "tof=",
                        m.hello.tof_ok,
                    )
                    return
            print("connected, waiting hello", i)
        except OSError as e:
            print("wait", i, type(e).__name__)
            time.sleep(1.0)
    raise SystemExit("no hello after apply")


if __name__ == "__main__":
    main()
