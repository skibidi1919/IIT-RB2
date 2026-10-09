#!/usr/bin/env python3
"""Upload a raw firmware .bin to Meowler over protobuf TCP (A/B, no chainload).

Running app accepts OTA into the *inactive* slot:
  1. begin  — erase/open inactive slot
  2. chunk* — sequential raw bytes
  3. finish — CRC-32, validate image
  4. apply  — boot new slot (crash rolls back)

Usage:
  py -3 arm_ui/ota_upload.py --host 192.168.29.222 --bin path/to/app.ino.bin
  ota-flash.bat
"""

from __future__ import annotations

import argparse
import binascii
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from meowler_pb import meowler_pb2 as pb  # noqa: E402

CHUNK = 1024  # smaller chunks — hotspot drops large OTA bursts
DEFAULT_HOST = "192.168.137.222"
DEFAULT_PORT = 3333
FQBN = (
    "esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,"
    "PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600"
)
BUILD_PROPS = [
    "--build-property",
    "build.partitions=ota8m_16MB",
    "--build-property",
    "upload.maximum_size=8323072",
]


def frame(msg: pb.ClientToRobot) -> bytes:
    data = msg.SerializeToString()
    return struct.pack("<I", len(data)) + data


def recv_frames(sock: socket.socket, deadline: float) -> list[pb.RobotToClient]:
    out: list[pb.RobotToClient] = []
    buf = bytearray()
    sock.settimeout(0.2)
    while time.time() < deadline:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf.extend(chunk)
        except (socket.timeout, ConnectionResetError, ConnectionAbortedError, OSError):
            if out:
                break
            continue
        while len(buf) >= 4:
            (n,) = struct.unpack_from("<I", buf, 0)
            if n == 0 or n > 16384:
                buf.clear()
                break
            if len(buf) < 4 + n:
                break
            payload = bytes(buf[4 : 4 + n])
            del buf[: 4 + n]
            msg = pb.RobotToClient()
            try:
                msg.ParseFromString(payload)
            except Exception:
                continue
            out.append(msg)
    return out


def last_ota(msgs: list[pb.RobotToClient]) -> pb.OtaStatus | None:
    st = None
    for m in msgs:
        if m.WhichOneof("msg") == "ota":
            st = m.ota
    return st


def send(sock: socket.socket, msg: pb.ClientToRobot) -> None:
    sock.sendall(frame(msg))


def wait_ota(sock: socket.socket, timeout: float = 3.0) -> pb.OtaStatus | None:
    return last_ota(recv_frames(sock, time.time() + timeout))


def query_ota(sock: socket.socket) -> None:
    msg = pb.ClientToRobot()
    msg.ota.action = 6
    send(sock, msg)


def wait_ota_states(
    sock: socket.socket,
    want: set[int],
    timeout: float,
    query: bool = True,
) -> pb.OtaStatus | None:
    """Wait until OtaStatus.state is in want (or error=4)."""
    deadline = time.time() + timeout
    last: pb.OtaStatus | None = None
    while time.time() < deadline:
        if query:
            query_ota(sock)
        st = last_ota(recv_frames(sock, time.time() + 0.6))
        if st is not None:
            last = st
            if int(st.state) in want or int(st.state) == 4:
                return st
        time.sleep(0.15)
    return last


def compile_bin() -> Path:
    """Incremental compile into esp_ui/build (reuses objects; -j all cores)."""
    cli = Path(os.environ.get("LOCALAPPDATA", "")) / "arduino-cli" / "arduino-cli.exe"
    if not cli.is_file():
        raise SystemExit(f"arduino-cli not found: {cli}")
    part_src = ROOT / "esp_ui" / "partitions.csv"
    part_dst = (
        Path(os.environ.get("LOCALAPPDATA", ""))
        / "Arduino15"
        / "packages"
        / "esp32"
        / "hardware"
        / "esp32"
        / "3.3.12"
        / "tools"
        / "partitions"
        / "ota8m_16MB.csv"
    )
    if part_src.is_file() and part_dst.parent.is_dir():
        part_dst.write_bytes(part_src.read_bytes())
    sketch = ROOT / "esp_ui"
    out = sketch / "build"
    cache = out / ".cache"
    out.mkdir(parents=True, exist_ok=True)
    cache.mkdir(parents=True, exist_ok=True)
    print(f"compile -> {out} (cache {cache})")
    r = subprocess.run(
        [
            str(cli),
            "compile",
            "--fqbn",
            FQBN,
            "-j",
            "0",
            "--build-path",
            str(cache),
            "--output-dir",
            str(out),
            str(sketch),
            *BUILD_PROPS,
        ],
        check=False,
    )
    if r.returncode != 0:
        raise SystemExit("compile failed")
    bin_path = out / "esp_ui.ino.bin"
    if not bin_path.is_file():
        raise SystemExit(f"missing {bin_path}")
    return bin_path


def upload(host: str, port: int, bin_path: Path, version: str, apply: bool) -> None:
    raw = bin_path.read_bytes()
    crc = binascii.crc32(raw) & 0xFFFFFFFF
    print(f"{bin_path.name}: raw={len(raw)} crc=0x{crc:08x} -> inactive slot")

    sock = socket.create_connection((host, port), timeout=8.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
    sock.settimeout(120.0)
    try:
        recv_frames(sock, time.time() + 0.2)

        begin = pb.ClientToRobot()
        begin.ota.action = 1
        begin.ota.begin.raw_size = len(raw)
        begin.ota.begin.raw_crc32 = crc
        begin.ota.begin.packed_size = len(raw)
        begin.ota.begin.codec = 0  # raw
        begin.ota.begin.version = (version or bin_path.stem)[:23]
        send(sock, begin)
        st = wait_ota_states(sock, {1}, 40.0)
        if not st or st.state != 1:
            raise SystemExit(f"begin rejected: {st}")
        print(f"begin ok state={st.state} detail={st.detail!r}")

        t0 = time.perf_counter()
        off = 0
        last_log = 0
        while off < len(raw):
            piece = raw[off : off + CHUNK]
            msg = pb.ClientToRobot()
            msg.ota.action = 2
            msg.ota.chunk.offset = off
            msg.ota.chunk.data = piece
            send(sock, msg)
            off += len(piece)
            if off - last_log >= 256 * 1024 or off == len(raw):
                dt = max(1e-3, time.perf_counter() - t0)
                print(f"  sent {off}/{len(raw)} ({off / dt / 1024:.0f} KiB/s)", flush=True)
                last_log = off
            if off % (CHUNK * 8) == 0:
                recv_frames(sock, time.time() + 0.02)

        recv_frames(sock, time.time() + 0.3)
        fin = pb.ClientToRobot()
        fin.ota.action = 3
        send(sock, fin)
        print("finish (CRC + validate inactive slot)...")
        st = wait_ota_states(sock, {2, 5}, 90.0)
        if not st or st.state not in (2, 5):
            raise SystemExit(f"finish failed: {st}")
        print(f"READY written={st.written} err={st.error} detail={st.detail!r}")

        if not apply:
            print("skip apply - robot holds READY until apply")
            return

        ap = pb.ClientToRobot()
        ap.ota.action = 5
        send(sock, ap)
        st = wait_ota(sock, 2.0)
        print(f"apply -> reboot ({st})")
        print("done - wait for WiFi .222 then reconnect dashboard")
    finally:
        try:
            sock.close()
        except OSError:
            pass


def main() -> None:
    ap = argparse.ArgumentParser(description="Meowler protobuf OTA uploader")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--bin", type=Path, help="esp_ui.ino.bin path")
    ap.add_argument("--compile", action="store_true", help="compile esp_ui first")
    ap.add_argument("--version", default="", help="label stored in OtaBegin")
    ap.add_argument("--no-apply", action="store_true", help="flash slot but do not reboot")
    ap.add_argument("--boot-ota0", action="store_true", help="reboot robot into OTA0 updater")
    ap.add_argument("--boot-ota1", action="store_true", help="reboot robot into OTA1 app")
    args = ap.parse_args()

    if args.boot_ota0 or args.boot_ota1:
        sock = socket.create_connection((args.host, args.port), timeout=8.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        msg = pb.ClientToRobot()
        msg.ota.action = 7 if args.boot_ota0 else 8
        send(sock, msg)
        print("sent boot OTA0" if args.boot_ota0 else "sent boot OTA1")
        time.sleep(0.3)
        sock.close()
        return

    bin_path = args.bin
    if args.compile or not bin_path:
        bin_path = compile_bin()
    if not bin_path.is_file():
        raise SystemExit(f"bin not found: {bin_path}")

    upload(args.host, args.port, bin_path, args.version, apply=not args.no_apply)


if __name__ == "__main__":
    main()
