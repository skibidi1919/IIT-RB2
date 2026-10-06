"""
Practical MicroPython app OTA / update helpers.

Two layers (see README):
  (1) Flash MicroPython firmware once with esptool (scripts/flash_mpy.*).
  (2) Update app/*.py without reflashing:
        - mpremote (USB) — preferred for development
        - HTTP pull of a .tar or individual .py files (this module)
        - WebREPL (optional; enable separately)

Arduino-style dual-slot .bin OTA (OtaCmd protobuf) is NOT implemented here —
MicroPython uses a different firmware layout. Host arm_ui OTA expects esp_ui.
For MicroPython, deploy Python sources instead.
"""

import gc
import os


def _ensure_dir(path):
    parts = path.split("/")
    cur = ""
    for p in parts:
        if not p:
            continue
        cur = cur + "/" + p if cur else p
        try:
            os.mkdir(cur)
        except OSError:
            pass


def http_get(url, timeout_s=30):
    """Fetch URL body as bytes. Uses urequests if available, else raw socket."""
    try:
        import urequests

        r = urequests.get(url, timeout=timeout_s)
        try:
            data = r.content
        finally:
            r.close()
        return data
    except ImportError:
        pass

    # Minimal HTTP/1.0 GET
    try:
        import usocket as socket
        import ussl
    except ImportError:
        import socket

        ussl = None

    if not url.startswith("http://") and not url.startswith("https://"):
        raise ValueError("url must be http(s)")
    tls = url.startswith("https://")
    rest = url.split("://", 1)[1]
    host_path = rest.split("/", 1)
    hostport = host_path[0]
    path = "/" + (host_path[1] if len(host_path) > 1 else "")
    if ":" in hostport:
        host, port_s = hostport.split(":", 1)
        port = int(port_s)
    else:
        host = hostport
        port = 443 if tls else 80

    addr = socket.getaddrinfo(host, port, 0, socket.SOCK_STREAM)[0][-1]
    s = socket.socket()
    s.settimeout(timeout_s)
    s.connect(addr)
    if tls:
        if ussl is None:
            s.close()
            raise RuntimeError("TLS not available")
        s = ussl.wrap_socket(s, server_hostname=host)
    req = "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n" % (path, host)
    s.write(req.encode())
    buf = b""
    while True:
        chunk = s.read(1024)
        if not chunk:
            break
        buf += chunk
    s.close()
    if b"\r\n\r\n" not in buf:
        raise RuntimeError("bad HTTP response")
    hdr, body = buf.split(b"\r\n\r\n", 1)
    status = hdr.split(b"\r\n", 1)[0]
    if b"200" not in status:
        raise RuntimeError("HTTP %s" % status.decode())
    return body


def write_file(path, data):
    """Write bytes to path, creating parent dirs."""
    if "/" in path:
        _ensure_dir(path.rsplit("/", 1)[0])
    with open(path, "wb") as f:
        f.write(data)
    gc.collect()


def pull_file(url, dest_path):
    """Download one file over HTTP and write to dest_path on the device FS."""
    data = http_get(url)
    write_file(dest_path, data)
    return len(data)


def pull_app_manifest(base_url, files=None):
    """
    Download a list of app files from base_url.
    Example base_url: http://192.168.1.10:8000/app/
    files default: core meowler modules.
    """
    if files is None:
        files = (
            "boot.py",
            "main.py",
            "pinout.py",
            "pca9685.py",
            "vl53.py",
            "drive.py",
            "color_tcs.py",
            "protocol.py",
            "ota_http.py",
        )
    if not base_url.endswith("/"):
        base_url += "/"
    results = []
    for name in files:
        url = base_url + name
        n = pull_file(url, name)
        results.append((name, n))
        gc.collect()
    return results


def soft_reboot():
    import machine

    machine.reset()
