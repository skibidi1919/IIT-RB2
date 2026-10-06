#!/usr/bin/env python3
"""Live monitor for hub ESP32 — telemetry + debug logs."""

from __future__ import annotations

import argparse
import sys
import time

import serial

from proto_link import encode_action, flags_dict, iter_link_messages, ACT_PING


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="COM5", help="hub ESP32 USB port")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(encode_action(ACT_PING))
    print(f"listening on {args.port} … Ctrl+C to quit", flush=True)

    try:
        while True:
            for msg in iter_link_messages(ser, timeout=0.2):
                kind = msg.WhichOneof("payload")
                if kind == "telem":
                    t = msg.telem
                    fl = flags_dict(t.flags)
                    print(
                        f"TELEM#{t.seq} color={t.color} conf={t.conf} "
                        f"drive={t.left},{t.right} enc={t.enc_l},{t.enc_r} "
                        f"arm={t.base}/{t.height}/{t.grip} d={t.distance_mm}mm "
                        f"ypr={t.yaw_cdeg},{t.pitch_cdeg},{t.roll_cdeg} "
                        f"flags={fl}",
                        flush=True,
                    )
                elif kind == "log":
                    text = bytes(msg.log.text).decode("utf-8", "replace")
                    print(f"LOG[{msg.log.level}] {text}", flush=True)
    except KeyboardInterrupt:
        print("bye")
        return 0


if __name__ == "__main__":
    sys.exit(main())
