"""
Meowler MicroPython brain — WiFi STA → static *.222 → TCP :3333
Length-prefixed protobuf (protocol.py / proto/meowler.proto).
"""

import gc
import json
import sys
import time

import network
import socket
from machine import Pin

import pinout as P
import protocol as proto
from color_tcs import ColorTcs
from drive import Drive
from pca9685 import PCA9685, recover_i2c
from vl53 import VL53L0X

LOG_INFO = 1
LOG_WARN = 2
LOG_ERR = 3

MAX_CLIENTS = 3
TELEM_MS = 200


def _load_secrets():
    try:
        import secrets

        return secrets.WIFI_SSID, secrets.WIFI_PASS
    except ImportError:
        print("WARN: no secrets.py — copy secrets.py.example → secrets.py")
        return "YOUR_SSID", "YOUR_PASSWORD"


class Robot:
    def __init__(self):
        self.ssid, self.password = _load_secrets()
        self.wlan = network.WLAN(network.STA_IF)
        self.i2c = None
        self.pca = None
        self.tof = None
        self.drive = Drive(use_pwm=True)
        self.color = ColorTcs()
        self.pca_ok = False
        self.tof_ok = False
        self.wifi_ok = False
        self.dist_mm = 0
        self.tof_origin = 0
        self.tof_origin_set = False
        self.server = None
        self.clients = []  # list of (sock, FrameReader)
        self._learned = None  # (ip, gw, mask, dns) strings
        self._last_telem = 0
        self._last_tof = 0
        self._last_wifi = 0
        self._rec_on = False
        self._rec_t0 = 0
        self._rec_count = 0
        self._rec_name = "move"
        self.color.set_yield(self._poll_net_light)

    def log(self, level, text):
        print(text)
        payload = proto.encode_log(level, text)
        self._broadcast(payload)

    def _broadcast(self, payload):
        framed = proto.frame(payload)
        dead = []
        for i, (sock, _) in enumerate(self.clients):
            try:
                sock.sendall(framed)
            except OSError:
                dead.append(i)
        for i in reversed(dead):
            try:
                self.clients[i][0].close()
            except OSError:
                pass
            del self.clients[i]

    def _send(self, sock, payload):
        try:
            sock.sendall(proto.frame(payload))
            return True
        except OSError:
            return False

    def _poll_net_light(self):
        """Yield from color sensor — accept new clients only (no RX drain)."""
        self._accept_clients()
        self.drive.poll_encoders()

    # ---- I2C / sensors ----

    def begin_i2c(self):
        try:
            Pin(P.PIN_FLASH_LED, Pin.OUT, value=0)
        except Exception:
            pass
        self.i2c, addrs = recover_i2c()
        self.log(LOG_INFO, "I2C SDA=%d SCL=%d scan=%s" % (P.I2C_SDA, P.I2C_SCL, [hex(a) for a in addrs]))
        self.pca = PCA9685(self.i2c, P.PCA_ADDR)
        self.pca_ok = self.pca.begin()
        if not self.pca_ok:
            time.sleep_ms(200)
            self.i2c, addrs = recover_i2c()
            self.pca = PCA9685(self.i2c, P.PCA_ADDR)
            self.pca_ok = self.pca.begin()
        if self.pca_ok:
            self.pca.center()
            self.pca.set_conveyor(0)
            self.pca.hold_motors_off()
            self.log(LOG_INFO, "PCA OK @0x40 150Hz CH0/1/2 arm CH4 conveyor")
        else:
            self.log(LOG_WARN, "PCA MISSING @0x40 on 21/47")

        self.tof = VL53L0X(self.i2c, P.TOF_ADDR)
        self.tof_ok = self.tof.begin()
        if not self.tof_ok:
            time.sleep_ms(200)
            self.tof_ok = self.tof.begin()
        if self.tof_ok:
            self.dist_mm = self.tof.distance_mm
            self.log(LOG_INFO, "TOF OK @0x29")
        else:
            self.log(LOG_WARN, "TOF MISSING @0x29 on 21/47")

    def ensure_pca(self):
        if self.pca_ok and self.pca and self.pca.probe():
            return True
        self.i2c, _ = recover_i2c()
        self.pca = PCA9685(self.i2c, P.PCA_ADDR)
        self.pca_ok = self.pca.begin()
        return self.pca_ok

    # ---- WiFi ----

    def _ifconfig_tuple(self, ip, gw, mask, dns):
        self.wlan.ifconfig((ip, mask, gw, dns))

    def _learn_from_dhcp(self):
        ip, mask, gw, dns = self.wlan.ifconfig()
        if not gw or gw == "0.0.0.0":
            parts = ip.split(".")
            gw = "%s.%s.%s.1" % (parts[0], parts[1], parts[2])
        if not mask or mask == "0.0.0.0":
            mask = "255.255.255.0"
        if not dns or dns == "0.0.0.0":
            dns = gw
        # Static host always *.222 on gateway subnet (/24 common case)
        gparts = gw.split(".")
        static_ip = "%s.%s.%s.%d" % (gparts[0], gparts[1], gparts[2], P.WIFI_HOST_OCTET)
        if mask != "255.255.255.0":
            # Keep same network bits, force host octet
            iparts = [int(x) for x in ip.split(".")]
            mparts = [int(x) for x in mask.split(".")]
            host = [
                (iparts[i] & mparts[i]) | (P.WIFI_HOST_OCTET if i == 3 else 0)
                for i in range(4)
            ]
            # For non-/24, only force last octet when mask allows
            host[3] = (iparts[3] & mparts[3]) | (P.WIFI_HOST_OCTET & (~mparts[3] & 0xFF))
            static_ip = "%d.%d.%d.%d" % tuple(host)
        self._learned = (static_ip, gw, mask, dns)
        self.log(LOG_INFO, "WIFI learned gw=%s -> %s" % (gw, static_ip))

    def start_wifi(self):
        self.wlan.active(True)
        try:
            self.wlan.config(dhcp_hostname=P.HOSTNAME)
        except Exception:
            pass

        # Cached static reconnect
        if self._learned:
            sip, gw, mask, dns = self._learned
            self.wlan.disconnect()
            time.sleep_ms(150)
            self._ifconfig_tuple(sip, gw, mask, dns)
            self.wlan.connect(self.ssid, self.password)
            if self._wait_wifi(20000):
                self.wifi_ok = True
                self._bind_server()
                self.log(LOG_INFO, "WIFI ok tcp://%s:%d" % (self.wlan.ifconfig()[0], P.TCP_PORT))
                return

        # DHCP then static
        self.wlan.disconnect()
        time.sleep_ms(150)
        # Clear static so DHCP works (empty / zeros)
        try:
            self.wlan.ifconfig(("0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0"))
        except Exception:
            pass
        self.wlan.connect(self.ssid, self.password)
        self.log(LOG_INFO, "WIFI DHCP %s" % self.ssid)
        if not self._wait_wifi(25000):
            self.log(LOG_ERR, "WIFI FAIL (DHCP)")
            self.wifi_ok = False
            return
        self._learn_from_dhcp()
        sip, gw, mask, dns = self._learned
        self.wlan.disconnect()
        time.sleep_ms(150)
        self._ifconfig_tuple(sip, gw, mask, dns)
        self.wlan.connect(self.ssid, self.password)
        if not self._wait_wifi(20000):
            self.log(LOG_WARN, "WIFI static FAIL — DHCP fallback")
            try:
                self.wlan.ifconfig(("0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0"))
            except Exception:
                pass
            self.wlan.connect(self.ssid, self.password)
            if not self._wait_wifi(20000):
                self.log(LOG_ERR, "WIFI FAIL")
                self.wifi_ok = False
                return
        self.wifi_ok = True
        self._bind_server()
        self.log(LOG_INFO, "WIFI ok tcp://%s:%d" % (self.wlan.ifconfig()[0], P.TCP_PORT))

    def _wait_wifi(self, timeout_ms):
        t0 = time.ticks_ms()
        while not self.wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                return False
            time.sleep_ms(200)
        return True

    def service_wifi(self):
        if time.ticks_diff(time.ticks_ms(), self._last_wifi) < 400:
            return
        self._last_wifi = time.ticks_ms()
        if self.wlan.isconnected():
            if not self.wifi_ok:
                self.wifi_ok = True
                self._bind_server()
                self.log(LOG_INFO, "WIFI up %s" % self.wlan.ifconfig()[0])
            return
        if self.wifi_ok:
            self.wifi_ok = False
            self._close_clients()
            self.log(LOG_WARN, "WIFI down")
        self.start_wifi()

    def _bind_server(self):
        if self.server:
            try:
                self.server.close()
            except OSError:
                pass
            self.server = None
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", P.TCP_PORT))
        s.listen(MAX_CLIENTS)
        s.setblocking(False)
        self.server = s
        self.log(LOG_INFO, "TCP listen :%d" % P.TCP_PORT)

    def _close_clients(self):
        for sock, _ in self.clients:
            try:
                sock.close()
            except OSError:
                pass
        self.clients = []

    def _accept_clients(self):
        if not self.server:
            return
        try:
            conn, addr = self.server.accept()
        except OSError:
            return
        conn.setblocking(False)
        while len(self.clients) >= MAX_CLIENTS:
            old, _ = self.clients.pop(0)
            try:
                old.close()
            except OSError:
                pass
        self.clients.append((conn, proto.FrameReader()))
        self.log(LOG_INFO, "client %s" % str(addr))
        # Hello
        try:
            ip = self.wlan.ifconfig()[0]
        except Exception:
            ip = "0.0.0.0"
        hello = proto.encode_hello(
            proto.ip_to_u32(ip),
            P.TCP_PORT,
            self.pca_ok,
            self.tof_ok,
            False,
        )
        self._send(conn, hello)

    # ---- telemetry / commands ----

    def telem_dict(self):
        wl, wr = self.drive.wheel_mm()
        tof_disp = 0
        if self.tof_ok and self.tof_origin_set:
            tof_disp = int(self.tof_origin) - int(self.dist_mm)
        return {
            "distance_mm": int(self.dist_mm),
            "cmd_l": self.drive.cmd_l,
            "cmd_r": self.drive.cmd_r,
            "base": self.pca.base if self.pca else 90,
            "height": self.pca.height if self.pca else 90,
            "grip": self.pca.grip if self.pca else 90,
            "pca_ok": self.pca_ok,
            "tof_ok": self.tof_ok,
            "imu_ok": False,
            "wifi_ok": self.wifi_ok,
            "enc_l": self.drive.enc_l,
            "enc_r": self.drive.enc_r,
            "wheel_l_mm": wl,
            "wheel_r_mm": wr,
            "tof_disp_mm": tof_disp,
            "yaw_cdeg": 0,
            "pitch_cdeg": 0,
            "roll_cdeg": 0,
            "color": self.color.label,
            "color_conf": self.color.conf,
            "color_r": self.color.r100,
            "color_g": self.color.g100,
            "color_b": self.color.y100,  # yellow Δ% in color_b slot (esp_ui)
            "color_rp": self.color.rp,
            "color_gp": self.color.gp,
            "color_bp": self.color.bp,
            "color_cp": self.color.cp,
            "conveyor": self.pca.conveyor if self.pca else 0,
        }

    def broadcast_telem(self):
        self._broadcast(proto.encode_telemetry(self.telem_dict()))

    def _ack(self, sock, code=0):
        self._send(sock, proto.encode_ack(code))

    def _rec_note(self, op, a=0, b=0, c=0, mask=0):
        if not self._rec_on:
            return
        ev = {
            "t_ms": time.ticks_diff(time.ticks_ms(), self._rec_t0),
            "op": op,
            "a": a,
            "b": b,
            "c": c,
            "mask": mask,
            "count": self._rec_count,
            "name": self._rec_name,
        }
        self._rec_count += 1
        self._broadcast(proto.encode_rec_event(ev))

    def handle_cmd(self, sock, msg):
        if not msg:
            return
        op = msg.get("op")
        if op == "drive":
            self.drive.set(msg["left"], msg["right"])
            self._rec_note(1, msg["left"], msg["right"])
            self._ack(sock, 0)
        elif op == "arm":
            self.ensure_pca()
            b = self.pca.base if self.pca else 90
            h = self.pca.height if self.pca else 90
            g = self.pca.grip if self.pca else 90
            mask = 0
            if msg.get("set_base"):
                b = msg["base"]
                mask |= 1
            if msg.get("set_height"):
                h = msg["height"]
                mask |= 2
            if msg.get("set_grip"):
                g = msg["grip"]
                mask |= 4
            if msg.get("set_base") and msg.get("set_height") and msg.get("set_grip"):
                self.pca.apply_arm(b, h, g)
            else:
                self.pca.set_arm(
                    b if msg.get("set_base") else None,
                    h if msg.get("set_height") else None,
                    g if msg.get("set_grip") else None,
                )
            self._rec_note(2, b, h, g, mask)
            self._ack(sock, 0)
        elif op == "stop":
            self.drive.stop()
            if self.pca_ok:
                self.pca.set_conveyor(0)
            self._rec_note(3)
            self._ack(sock, 0)
        elif op == "center":
            self.ensure_pca()
            if self.pca:
                self.pca.center()
            self._rec_note(4)
            self._ack(sock, 0)
        elif op == "zero":
            self.drive.zero()
            self.tof_origin = self.dist_mm
            self.tof_origin_set = True
            self._rec_note(5)
            self._ack(sock, 0)
        elif op == "motor_test":
            self.drive.set(255, 0)
            self._rec_note(7)
            self._ack(sock, 0)
        elif op == "get_telem":
            self._send(sock, proto.encode_telemetry(self.telem_dict()))
        elif op == "conveyor":
            self.ensure_pca()
            if self.pca:
                self.pca.set_conveyor(msg["speed"])
            self._rec_note(6, msg["speed"])
            self._ack(sock, 0)
        elif op == "color_cal":
            self.color.calibrate(msg.get("mode", 0))
            self._ack(sock, 0)
        elif op == "rec":
            if msg.get("action") == 1:
                self._rec_on = True
                self._rec_t0 = time.ticks_ms()
                self._rec_count = 0
                self._rec_name = msg.get("name") or "move"
                self._broadcast(
                    proto.encode_rec_event(
                        {
                            "t_ms": 0,
                            "op": 0,
                            "count": 0,
                            "name": self._rec_name,
                        }
                    )
                )
            else:
                if self._rec_on:
                    self._broadcast(
                        proto.encode_rec_event(
                            {
                                "t_ms": time.ticks_diff(time.ticks_ms(), self._rec_t0),
                                "op": 255,
                                "count": self._rec_count,
                                "name": self._rec_name,
                            }
                        )
                    )
                self._rec_on = False
            self._ack(sock, 0)
        elif op == "ota":
            # MicroPython: use mpremote / ota_http, not esp_ui slot OTA
            self._send(sock, proto.encode_log(LOG_WARN, "OTA: use mpremote/ota_http (not bin slots)"))
            self._ack(sock, 1)
        else:
            self._ack(sock, 1)

    def handle_json_line(self, sock, line):
        """Optional JSON-line fallback: {"op":"drive","left":100,"right":100}"""
        try:
            msg = json.loads(line)
        except ValueError:
            return
        if "op" not in msg:
            return
        self.handle_cmd(sock, msg)

    def _service_clients(self):
        self._accept_clients()
        dead = []
        for i, (sock, rx) in enumerate(self.clients):
            try:
                data = sock.recv(1024)
            except OSError:
                continue
            if not data:
                # non-blocking: empty means no data, not always disconnect
                continue
            # JSON-line fallback when idle on framing and payload looks like JSON
            if (not rx._in_body) and (len(rx._hdr) == 0) and data.lstrip()[:1] == b"{":
                try:
                    text = data.decode("utf-8")
                except UnicodeError:
                    text = ""
                for line in text.splitlines():
                    line = line.strip()
                    if line.startswith("{"):
                        self.handle_json_line(sock, line)
                continue
            for payload in rx.feed(data):
                try:
                    msg = proto.decode_client_to_robot(payload)
                except ValueError:
                    msg = None
                self.handle_cmd(sock, msg)
        # prune dead sockets periodically via failed send in _broadcast

    def update_sensors(self):
        self.drive.poll_encoders()
        self.drive.service_hold(400)
        now = time.ticks_ms()
        if self.tof_ok and time.ticks_diff(now, self._last_tof) >= 50:
            self._last_tof = now
            mm = self.tof.update()
            if mm:
                self.dist_mm = mm
        self.color.update(self.dist_mm, self.tof_ok)
        if self.clients and time.ticks_diff(now, self._last_telem) >= TELEM_MS:
            self._last_telem = now
            self.broadcast_telem()

    def run(self):
        self.log(LOG_INFO, "MEOWLER esp_mpy boot")
        self.drive.stop()
        self.begin_i2c()
        self.start_wifi()
        self.log(LOG_INFO, "loop — TCP :%d protobuf" % P.TCP_PORT)
        while True:
            try:
                self.service_wifi()
                self._service_clients()
                self.update_sensors()
            except Exception as e:
                sys.print_exception(e)
                time.sleep_ms(100)
            time.sleep_ms(5)
            gc.collect()


def main():
    Robot().run()


if __name__ == "__main__":
    main()
