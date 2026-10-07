"""Find Meowler TCP endpoints on local LANs (hotspot *.222, SoftAP 192.168.4.1)."""

from __future__ import annotations

import concurrent.futures
import re
import socket
import subprocess
from typing import Iterable


DEFAULT_PORT = 3333
SOFTAP_HOST = "192.168.4.1"


def _arp_hosts(prefix: str = "192.168.137.") -> list[str]:
    """Collect IPv4 neighbors from `arp -a` (hotspot clients often appear here first)."""
    out: list[str] = []
    try:
        raw = subprocess.check_output(["arp", "-a"], text=True, errors="ignore", timeout=3.0)
    except (OSError, subprocess.SubprocessError):
        return out
    for m in re.finditer(r"\b(\d+\.\d+\.\d+\.\d+)\b", raw):
        ip = m.group(1)
        if ip.startswith(prefix) and not ip.endswith(".255") and not ip.endswith(".1") and ip not in out:
            out.append(ip)
    return out


def local_ipv4s() -> list[str]:
    found: list[str] = []

    def _add(ip: str) -> None:
        if ip and not ip.startswith("127.") and not ip.startswith("169.254.") and ip not in found:
            found.append(ip)

    # Windows: enumerate adapters (includes Mobile Hotspot 192.168.137.1)
    try:
        raw = subprocess.check_output(
            ["ipconfig"], text=True, errors="ignore", timeout=4.0
        )
        for m in re.finditer(r"IPv4 Address[^\d]*(\d+\.\d+\.\d+\.\d+)", raw):
            _add(m.group(1))
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            _add(info[4][0])
    except OSError:
        pass
    # Outbound route last (often Ethernet, not the hotspot NIC)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.connect(("8.8.8.8", 80))
        _add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return found


def suggested_hosts(host_octet: int = 222) -> list[str]:
    """Prefer *.222 on each local /24, SoftAP, then common fallbacks."""
    out: list[str] = []
    # Hotspot / Meowler subnets first
    for ip in local_ipv4s():
        if ip.startswith("192.168.137."):
            cand = "192.168.137.222"
            if cand not in out:
                out.insert(0, cand)
    for ip in local_ipv4s():
        parts = ip.split(".")
        if len(parts) != 4:
            continue
        cand = f"{parts[0]}.{parts[1]}.{parts[2]}.{host_octet}"
        if cand != ip and cand not in out:
            out.append(cand)
    for extra in (SOFTAP_HOST, "192.168.137.222", "192.168.29.222", "meowler.local"):
        if extra not in out:
            out.append(extra)
    return out


def default_host_for_lan(fallback: str = "192.168.137.222") -> str:
    locals_ = local_ipv4s()
    # Prefer Mobile Hotspot / known robot LANs over the internet NIC
    for ip in locals_:
        if ip.startswith("192.168.137."):
            return "192.168.137.222"
    for ip in locals_:
        if ip.startswith("192.168.4."):
            return SOFTAP_HOST
    for ip in locals_:
        if ip.startswith("192.168.29."):
            return "192.168.29.222"
    for ip in locals_:
        parts = ip.split(".")
        if len(parts) == 4:
            return f"{parts[0]}.{parts[1]}.{parts[2]}.222"
    return fallback


def probe_tcp(host: str, port: int = DEFAULT_PORT, timeout: float = 0.35) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _prefer_static_222(hosts: list[str]) -> list[str]:
    """Meowler STA always settles on *.222 — probe that before leftover DHCP ARPs."""
    preferred: list[str] = []
    rest: list[str] = []
    for h in hosts:
        if h.endswith(".222") or h == SOFTAP_HOST or h.endswith(".local"):
            if h not in preferred:
                preferred.append(h)
        else:
            if h not in rest:
                rest.append(h)
    return preferred + rest


def canonicalize_host(host: str) -> str:
    """Map hotspot DHCP leftovers (e.g. .140) to static *.222 on the same /24."""
    h = (host or "").strip()
    parts = h.split(".")
    if len(parts) == 4 and h.startswith("192.168.") and parts[3] != "222" and parts[3].isdigit():
        # Known Meowler subnets: always use static host octet
        if parts[0] == "192" and parts[1] == "168" and parts[2] in ("137", "29"):
            return f"{parts[0]}.{parts[1]}.{parts[2]}.222"
    return h


def discover(
    port: int = DEFAULT_PORT,
    hosts: Iterable[str] | None = None,
    timeout: float = 0.45,
    max_workers: int = 32,
) -> list[dict]:
    """Return open {host, port} hits, preferred order preserved."""
    cands = list(hosts) if hosts is not None else suggested_hosts()
    # Hotspot ARP neighbors after *.222 (DHCP .140 is often stale once static binds)
    for ip in _arp_hosts("192.168.137."):
        if ip not in cands:
            cands.append(ip)
    # Also sweep .2-.20 on hotspot subnet when we are the AP host
    for ip in local_ipv4s():
        if ip == "192.168.137.1":
            for n in range(2, 21):
                h = f"192.168.137.{n}"
                if h not in cands:
                    cands.append(h)
            break
    if "192.168.137.222" not in cands:
        cands.insert(0, "192.168.137.222")
    cands = _prefer_static_222(cands)

    hits: list[dict] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as pool:
        futs = {pool.submit(probe_tcp, h, port, timeout): h for h in cands}
        for fut in concurrent.futures.as_completed(futs):
            h = futs[fut]
            try:
                if fut.result():
                    hits.append({"host": h, "port": port})
            except Exception:  # noqa: BLE001
                pass

    order = {h: i for i, h in enumerate(cands)}
    hits.sort(key=lambda d: order.get(d["host"], 9999))
    return hits


def connect_hint(host: str, *, hotspot_clients: int | None = None) -> str:
    canon = canonicalize_host(host)
    bits = [f"timed out - tried {host}:{DEFAULT_PORT}"]
    if canon != host:
        bits.append(f"Use {canon} (static IP; DHCP {host} is stale).")
    else:
        bits.append("Robot not on this LAN.")
        bits.append("PC Mobile Hotspot should be SSID DarshIshaan / Darsh@3001 (2.4 GHz).")
        bits.append("Or join Wi-Fi Meowler and use 192.168.4.1 (SoftAP after firmware update).")
    if hotspot_clients is not None and hotspot_clients == 0:
        bits.append("Hotspot reports 0 clients - power the ESP or flash SoftAP firmware.")
    return " ".join(bits)
