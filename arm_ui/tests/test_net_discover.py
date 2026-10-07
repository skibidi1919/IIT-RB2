"""Unit tests for LAN host suggestion / discover helpers."""

from net_discover import canonicalize_host, connect_hint, default_host_for_lan, suggested_hosts


def test_suggested_hosts_includes_softap_and_hotspot():
    hosts = suggested_hosts()
    assert "192.168.4.1" in hosts
    assert "192.168.137.222" in hosts


def test_default_host_fallback():
    h = default_host_for_lan("192.168.137.222")
    assert h.endswith(".222") or h == "192.168.4.1"


def test_canonicalize_dhcp_leftover():
    assert canonicalize_host("192.168.137.140") == "192.168.137.222"
    assert canonicalize_host("192.168.137.222") == "192.168.137.222"


def test_connect_hint_mentions_hotspot():
    msg = connect_hint("192.168.137.222", hotspot_clients=0)
    assert "192.168.137.222" in msg
    assert "0 clients" in msg
    assert "Meowler" in msg
    assert "timed out - tried" in msg
    assert "static IP" in connect_hint("192.168.137.140")
