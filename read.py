#!/usr/bin/env python3
"""Read ESP32 USB serial (length-prefixed protobuf). No extra packages required
beyond pyserial — does not import the arm_ui venv (that hung Python 3.14).

  py -u read.py
  py -u read.py -p COM7
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    sys.stdout.reconfigure(line_buffering=True, errors="replace")
    sys.stderr.reconfigure(line_buffering=True, errors="replace")
except Exception:
    pass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pip install pyserial   then:  py -u read.py")

MAX_FRAME = 4224
LEVELS = ("DBG", "INF", "WRN", "ERR")


def pick_port(prefer: str) -> str:
    ports = list(list_ports.comports())
    names = [p.device for p in ports]
    if prefer.upper() in {n.upper() for n in names}:
        return prefer
    for p in ports:
        desc = (p.description or "").upper()
        if "CH343" in desc or "USB-ENHANCED-SERIAL" in desc or "CP210" in desc:
            print(f"{prefer} not found, using {p.device} ({p.description})", flush=True)
            return p.device
    if names:
        print(f"{prefer} not found, using {names[0]}", flush=True)
        return names[0]
    raise SystemExit(f"no serial ports (wanted {prefer}). Close Arduino Serial Monitor.")


def varint(buf: bytes, i: int):
    x = 0
    s = 0
    while i < len(buf):
        b = buf[i]
        i += 1
        x |= (b & 0x7F) << s
        if not (b & 0x80):
            return x, i
        s += 7
        if s > 63:
            return None, i
    return None, i


def fields(buf: bytes) -> dict[int, list]:
    i = 0
    out: dict[int, list] = {}
    n = len(buf)
    while i < n:
        key, i = varint(buf, i)
        if key is None:
            break
        fn, wt = key >> 3, key & 7
        if fn == 0:
            break
        if wt == 0:
            v, i = varint(buf, i)
            if v is None:
                break
            out.setdefault(fn, []).append(v)
        elif wt == 2:
            ln, i = varint(buf, i)
            if ln is None or i + ln > n:
                break
            out.setdefault(fn, []).append(buf[i : i + ln])
            i += ln
        elif wt == 5:
            if i + 4 > n:
                break
            i += 4
        elif wt == 1:
            if i + 8 > n:
                break
            i += 8
        else:
            break
    return out


def u32(v: int) -> int:
    return int(v) & 0xFFFFFFFF


def i32(v: int) -> int:
    v = u32(v)
    return v - 0x100000000 if v >= 0x80000000 else v


def fmt_msg(payload: bytes) -> str:
    try:
        f = fields(payload)
        if 4 in f and isinstance(f[4][0], (bytes, bytearray)):
            lg = fields(f[4][0])
            lvl = u32(lg.get(1, [1])[0])
            text = lg.get(2, [b""])[0]
            if isinstance(text, bytes):
                text = text.decode("utf-8", "replace")
            tag = LEVELS[lvl] if lvl < 4 else str(lvl)
            return f"[{tag}] {text}"
        if 1 in f and isinstance(f[1][0], (bytes, bytearray)):
            h = fields(f[1][0])
            return (
                f"[HELLO] pca={int(bool(h.get(3, [0])[0]))} "
                f"tof={int(bool(h.get(4, [0])[0]))} imu={int(bool(h.get(5, [0])[0]))}"
            )
        if 2 in f and isinstance(f[2][0], (bytes, bytearray)):
            t = fields(f[2][0])
            g = lambda n, d=0: t.get(n, [d])[0]
            return (
                f"[TELEM] tof={int(bool(g(8)))} dist={u32(g(1))}mm disp={i32(g(15))} "
                f"pca={int(bool(g(7)))} imu={int(bool(g(9)))} wifi={int(bool(g(10)))} "
                f"cmd={i32(g(2))}/{i32(g(3))} enc={i32(g(11))}/{i32(g(12))}"
            )
        if 3 in f:
            return f"[ACK] {u32(f[3][0])}"
        return f"[?] fields={list(f)}"
    except Exception:
        return f"[?] {len(payload)}B {payload[:12].hex()}"


def decode_frames(buf: bytearray):
    while len(buf) >= 4:
        (n,) = struct.unpack_from("<I", buf, 0)
        if n == 0 or n > MAX_FRAME:
            del buf[0]
            continue
        if len(buf) < 4 + n:
            return
        payload = bytes(buf[4 : 4 + n])
        del buf[: 4 + n]
        yield payload


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        raise SystemExit(f"cannot open {port}: {e}\nClose Arduino Serial Monitor / other read.py") from e
    ser.dtr = False
    ser.rts = False
    return ser


def main() -> int:
    ap = argparse.ArgumentParser(description="Read Meowler ESP32 USB serial")
    ap.add_argument("-p", "--port", default="COM7")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--no-telem", action="store_true")
    ap.add_argument("--hz", type=float, default=5.0)
    args = ap.parse_args()

    print("starting…", flush=True)
    port = pick_port(args.port)
    print(f"opening {port} @ {args.baud}", flush=True)
    ser = open_serial(port, args.baud)
    time.sleep(0.05)
    try:
        ser.write(b"s\n")
        ser.flush()
    except Exception:
        pass
    print(f"reading {port}  Ctrl+C to stop", flush=True)

    buf = bytearray()
    last_telem = 0.0
    min_dt = (1.0 / args.hz) if args.hz > 0 else 0.0
    got = 0
    t_open = time.monotonic()
    warned = False
    try:
        while True:
            chunk = ser.read(2048)
            if chunk:
                got += len(chunk)
                buf.extend(chunk)
                n_before = len(buf)
                for payload in decode_frames(buf):
                    line = fmt_msg(payload)
                    if line.startswith("[TELEM]"):
                        if args.no_telem:
                            continue
                        now = time.monotonic()
                        if min_dt and (now - last_telem) < min_dt:
                            continue
                        last_telem = now
                    print(line, flush=True)
                if len(buf) == n_before and len(buf) > 8:
                    # stuck: dump so silence isn't a black hole
                    print(f"[raw] {len(buf)} bytes waiting, head={buf[:8].hex()}", flush=True)
                    del buf[0]
            else:
                if not warned and got == 0 and (time.monotonic() - t_open) > 2.0:
                    warned = True
                    print(
                        "no bytes yet — board already booted (logs only at boot). "
                        "USB-flash so telem also comes on serial, or reset the ESP.",
                        flush=True,
                    )
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nbye", flush=True)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
