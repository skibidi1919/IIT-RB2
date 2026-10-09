#!/usr/bin/env python3
"""Wait for robot, OTA cal-fix firmware, run motor_cal, verify short drive_dist."""
from __future__ import annotations

import binascii
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from meowler_pb import meowler_pb2 as pb

HOST = "192.168.137.222"
PORT = 3333
BIN = Path(r"C:\Users\KENTOH~1\AppData\Local\Temp\meowler-ota-calfix\esp_ui.ino.bin")
CHUNK = 512


def frame(msg: pb.ClientToRobot) -> bytes:
    d = msg.SerializeToString()
    return struct.pack("<I", len(d)) + d


def connect(timeout: float = 5.0) -> socket.socket:
    s = socket.create_connection((HOST, PORT), timeout=timeout)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def wait_up(seconds: float = 300.0) -> None:
    end = time.time() + seconds
    n = 0
    while time.time() < end:
        try:
            s = connect(1.5)
            s.close()
            print(f"robot up after {n} probes", flush=True)
            return
        except OSError:
            n += 1
            if n % 10 == 0:
                print(f"waiting… ({n})", flush=True)
            time.sleep(1.0)
    raise SystemExit("robot never came up")


def recv_until(sock: socket.socket, t: float) -> list[pb.RobotToClient]:
    buf = bytearray()
    out: list[pb.RobotToClient] = []
    sock.settimeout(0.25)
    end = time.time() + t
    while time.time() < end:
        try:
            c = sock.recv(8192)
            if not c:
                break
            buf.extend(c)
        except socket.timeout:
            continue
        except OSError:
            break
        while len(buf) >= 4:
            (n,) = struct.unpack_from("<I", buf, 0)
            if n == 0 or n > 16384:
                del buf[0]
                continue
            if len(buf) < 4 + n:
                break
            payload = bytes(buf[4 : 4 + n])
            del buf[: 4 + n]
            m = pb.RobotToClient()
            try:
                m.ParseFromString(payload)
            except Exception:
                continue
            out.append(m)
    return out


def logs_of(msgs: list[pb.RobotToClient]) -> list[str]:
    lines = []
    for m in msgs:
        if m.WhichOneof("msg") == "log":
            lines.append(m.log.text)
    return lines


def ota(sock: socket.socket, raw: bytes) -> None:
    crc = binascii.crc32(raw) & 0xFFFFFFFF
    ab = pb.ClientToRobot()
    ab.ota.action = 4
    sock.sendall(frame(ab))
    recv_until(sock, 0.8)
    time.sleep(0.3)
    begin = pb.ClientToRobot()
    begin.ota.action = 1
    begin.ota.begin.raw_size = len(raw)
    begin.ota.begin.raw_crc32 = crc
    begin.ota.begin.packed_size = len(raw)
    begin.ota.begin.codec = 0
    begin.ota.begin.version = "cal-fix"
    sock.sendall(frame(begin))
    recv_until(sock, 2.0)
    off = 0
    while off < len(raw):
        piece = raw[off : off + CHUNK]
        msg = pb.ClientToRobot()
        msg.ota.action = 2
        msg.ota.chunk.offset = off
        msg.ota.chunk.data = piece
        sock.sendall(frame(msg))
        off += len(piece)
        time.sleep(0.002)
        if off % (CHUNK * 64) == 0:
            print(f"  ota {off}/{len(raw)}", flush=True)
    fin = pb.ClientToRobot()
    fin.ota.action = 3
    sock.sendall(frame(fin))
    otas = [m.ota for m in recv_until(sock, 12.0) if m.WhichOneof("msg") == "ota"]
    print("finish", [(o.state, o.error, o.written, o.detail) for o in otas[-3:]], flush=True)
    if not otas or otas[-1].state != 2:
        raise SystemExit("OTA finish failed")
    ap = pb.ClientToRobot()
    ap.ota.action = 5
    sock.sendall(frame(ap))
    recv_until(sock, 2.0)
    print("OTA apply sent", flush=True)


def send_motor_cal(sock: socket.socket) -> list[str]:
    msg = pb.ClientToRobot()
    msg.motor_cal.action = 1  # run encoder match
    sock.sendall(frame(msg))
    # cal ~2.2s + settle + margin
    msgs = recv_until(sock, 6.0)
    return logs_of(msgs)


def send_drive_dist(sock: socket.socket, mm: int, speed: int = 160) -> list[str]:
    msg = pb.ClientToRobot()
    msg.drive_dist.mm = mm
    msg.drive_dist.speed = speed
    sock.sendall(frame(msg))
    # ~ mm / (speed * mm_per_step roughly) — pad generously
    wait = max(8.0, abs(mm) / 80.0 + 4.0)
    msgs = recv_until(sock, wait)
    return logs_of(msgs)


def main() -> None:
    if not BIN.is_file():
        raise SystemExit(f"missing bin {BIN}")
    raw = BIN.read_bytes()
    print(f"bin={BIN} size={len(raw)}", flush=True)

    print("waiting for robot…", flush=True)
    wait_up(420.0)

    print("OTA…", flush=True)
    sock = connect()
    try:
        recv_until(sock, 0.4)
        ota(sock, raw)
    finally:
        sock.close()

    print("waiting post-reboot…", flush=True)
    time.sleep(4.0)
    wait_up(120.0)
    time.sleep(1.5)

    sock = connect()
    try:
        recv_until(sock, 0.5)
        print("MOTOR_CAL…", flush=True)
        lines = send_motor_cal(sock)
        for t in lines:
            if "MOTOR_CAL" in t or "MOTOR_SCALE" in t:
                print(" ", t, flush=True)
        if not any("MOTOR_CAL done" in t for t in lines):
            # drain a bit more
            more = logs_of(recv_until(sock, 3.0))
            for t in more:
                if "MOTOR_CAL" in t or "MOTOR_SCALE" in t:
                    print(" ", t, flush=True)
            lines.extend(more)
        if any("too few counts" in t for t in lines):
            raise SystemExit("cal failed: too few encoder counts")
        if not any("MOTOR_CAL done" in t for t in lines):
            raise SystemExit("cal failed: no done log")

        print("DRIVE_DIST 500mm verify…", flush=True)
        dlines = send_drive_dist(sock, 500, 160)
        for t in dlines:
            if "DRIVE_DIST" in t:
                print(" ", t, flush=True)
        if not any("DRIVE_DIST done" in t for t in dlines):
            more = logs_of(recv_until(sock, 8.0))
            for t in more:
                if "DRIVE_DIST" in t:
                    print(" ", t, flush=True)
    finally:
        sock.close()
    print("OK", flush=True)


if __name__ == "__main__":
    main()
