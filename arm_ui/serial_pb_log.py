#!/usr/bin/env python3
"""Read length-prefixed meowler.RobotToClient frames from ESP32 USB serial."""

from __future__ import annotations

import argparse
import struct
import sys
import time

import serial

from meowler_pb import meowler_pb2 as pb

MAX_FRAME = 4096
LEVELS = ("DBG", "INF", "WRN", "ERR")


def decode_frames(buf: bytearray):
    """Yield (RobotToClient, consumed) while complete frames exist; may clear junk."""
    while len(buf) >= 4:
        (n,) = struct.unpack_from("<I", buf, 0)
        if n == 0 or n > MAX_FRAME:
            # resync: drop one byte
            del buf[0]
            continue
        if len(buf) < 4 + n:
            return
        payload = bytes(buf[4 : 4 + n])
        del buf[: 4 + n]
        msg = pb.RobotToClient()
        try:
            msg.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001
            print(f"! decode error: {exc}", file=sys.stderr)
            continue
        yield msg


def format_msg(msg: pb.RobotToClient) -> str:
    which = msg.WhichOneof("msg")
    if which == "log":
        i = int(msg.log.level)
        tag = LEVELS[i] if 0 <= i < len(LEVELS) else str(i)
        return f"[{tag}] {msg.log.text}"
    if which == "hello":
        ip = msg.hello.ip
        dotted = ".".join(str((ip >> s) & 0xFF) for s in (24, 16, 8, 0))
        return (
            f"[HELLO] {dotted}:{msg.hello.port} "
            f"pca={int(msg.hello.pca_ok)} tof={int(msg.hello.tof_ok)} imu={int(msg.hello.imu_ok)}"
        )
    if which == "telem":
        t = msg.telem
        return (
            f"[TELEM] cmd={t.cmd_l}/{t.cmd_r} enc={t.enc_l}/{t.enc_r} "
            f"wifi={int(t.wifi_ok)} pca={int(t.pca_ok)} color={t.color}/{t.color_conf}"
        )
    if which == "ack":
        return f"[ACK] {msg.ack}"
    return f"[?] {which}"


def main() -> int:
    ap = argparse.ArgumentParser(description="Meowler protobuf serial log reader")
    ap.add_argument("-p", "--port", default="COM7", help="serial port (default COM7)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--telem", action="store_true", help="also print telemetry frames")
    ap.add_argument("--seconds", type=float, default=0, help="exit after N seconds (0=forever)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    # Avoid RTS/DTR download mode on ESP32-S3 native USB
    ser.dtr = False
    ser.rts = False
    time.sleep(0.15)
    ser.reset_input_buffer()
    print(f"listening {args.port} @ {args.baud} (protobuf frames)", flush=True)

    buf = bytearray()
    t0 = time.time()
    try:
        while True:
            if args.seconds and (time.time() - t0) >= args.seconds:
                break
            chunk = ser.read(1024)
            if chunk:
                buf.extend(chunk)
                for msg in decode_frames(buf):
                    which = msg.WhichOneof("msg")
                    if which == "telem" and not args.telem:
                        continue
                    print(format_msg(msg), flush=True)
            else:
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nbye", flush=True)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
