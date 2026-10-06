"""TCS3200 R/Y/G scoring — port of esp_ui/color_ryg.h."""

from machine import Pin, time_pulse_us
import time

import pinout as P

SAMPLES = 7
PULSE_TO = 25000
PERIOD_MS = 70
AMB_FAR_MM = 100
MIN_SCORE = 24
CLEAR_LEAD = 5
LOCK_N = 3
SWITCH_LEAD = 6
SWITCH_N = 4
UNLOCK_CONF = 10


def _inv(p):
    return 1000.0 / p if p else 0.0


def _hue_dist(h, c):
    d = abs(h - c)
    if d > 180.0:
        d = 360.0 - d
    return d


def _score_pct(sc):
    p = int(round(sc * 85.0))
    if p < 0:
        p = 0
    if p > 100:
        p = 100
    return p


class ColorTcs:
    def __init__(self):
        self.s2 = Pin(P.PIN_TCS_S2, Pin.OUT, value=0)
        self.s3 = Pin(P.PIN_TCS_S3, Pin.OUT, value=0)
        self.out = Pin(P.PIN_TCS_OUT, Pin.IN)
        self.led = Pin(P.PIN_TCS_LED, Pin.OUT, value=1)
        self.label = 0  # 0 none · 1 RED · 2 YELLOW · 3 GREEN
        self.conf = 0
        self.r100 = 0
        self.y100 = 0
        self.g100 = 0
        self.rp = 0
        self.gp = 0
        self.bp = 0
        self.cp = 0
        self._last_ms = 0
        self._amb_rp = self._amb_gp = self._amb_bp = self._amb_cp = 0.0
        self._amb_ready = False
        self._gain_r = 1.08
        self._gain_g = 1.35
        self._gain_b = 0.88
        self._ema_r = self._ema_g = self._ema_b = self._ema_c = 0.0
        self._ema_init = False
        self._lock = 0
        self._cand = 0
        self._streak = 0
        self._switch_streak = 0
        self._conf_ema = 0.0
        self._yield_fn = None

    def set_yield(self, fn):
        self._yield_fn = fn

    def _pump(self):
        if self._yield_fn:
            self._yield_fn()

    def calibrate(self, mode=0):
        # mode bits match ColorCal in meowler.proto (we reset ambient/EMA)
        self._amb_ready = False
        self._ema_init = False
        if mode == 4 or mode == 5:
            self._lock = self._cand = self._streak = self._switch_streak = 0
            self._conf_ema = 0.0
            self.label = 0
            self.conf = 0
        if mode == 5:
            self._gain_r, self._gain_g, self._gain_b = 1.08, 1.35, 0.88

    def _pulse(self, level, timeout_us):
        try:
            t = time_pulse_us(self.out, level, timeout_us)
            return t if t > 0 else 0
        except OSError:
            return 0

    def _median_pulse(self):
        buf = []
        for _ in range(SAMPLES):
            t = self._pulse(0, PULSE_TO)
            if not t:
                t = self._pulse(1, PULSE_TO)
            if t:
                buf.append(t)
            self._pump()
        if not buf:
            return 0
        buf.sort()
        return buf[len(buf) // 2]

    def _read_ch(self, s2, s3):
        self.s2.value(1 if s2 else 0)
        self.s3.value(1 if s3 else 0)
        time.sleep_ms(3)
        self._pump()
        self._pulse(0, 5000)
        self._pump()
        return self._median_pulse()

    def _learn_ambient(self, far):
        c_gain = (
            (self._amb_cp / float(self.cp))
            if (self.cp > 0 and self._amb_cp > 1.0)
            else 1.0
        )
        empty = (not self._lock) and c_gain < 1.04
        if not self._amb_ready or far or empty:
            a = 0.40 if not self._amb_ready else (0.06 if far else 0.03)
            if self._amb_ready:
                self._amb_rp = self._amb_rp * (1 - a) + self.rp * a
                self._amb_gp = self._amb_gp * (1 - a) + self.gp * a
                self._amb_bp = self._amb_bp * (1 - a) + self.bp * a
                self._amb_cp = self._amb_cp * (1 - a) + self.cp * a
            else:
                self._amb_rp = float(self.rp)
                self._amb_gp = float(self.gp)
                self._amb_bp = float(self.bp)
                self._amb_cp = float(self.cp)
            self._amb_ready = True
            if not self._lock:
                ri, gi, bi = _inv(self.rp), _inv(self.gp), _inv(self.bp)
                peak = max(ri, gi, bi)
                if peak > 1e-6:
                    ga = 0.08
                    self._gain_r = (1 - ga) * self._gain_r + ga * (peak / max(ri, 1e-6))
                    self._gain_g = (1 - ga) * self._gain_g + ga * (peak / max(gi, 1e-6))
                    self._gain_b = (1 - ga) * self._gain_b + ga * (peak / max(bi, 1e-6))
                    self._gain_r = min(1.45, max(0.85, self._gain_r))
                    self._gain_g = min(1.80, max(1.05, self._gain_g))
                    self._gain_b = min(1.15, max(0.70, self._gain_b))

    def _score_chroma(self):
        if not (self.rp or self.gp or self.bp):
            return False, 0, 0, 0
        ri = _inv(self.rp) * self._gain_r
        gi = _inv(self.gp) * self._gain_g
        bi = _inv(self.bp) * self._gain_b
        ci = _inv(self.cp)
        peak = max(ri, gi, bi)
        if peak < 1e-9:
            peak = 1.0
        R, G, B = ri / peak, gi / peak, bi / peak
        if not self._ema_init:
            self._ema_r, self._ema_g, self._ema_b, self._ema_c = R, G, B, ci
            self._ema_init = True
        else:
            a = 0.40
            self._ema_r = a * R + (1 - a) * self._ema_r
            self._ema_g = a * G + (1 - a) * self._ema_g
            self._ema_b = a * B + (1 - a) * self._ema_b
            self._ema_c = a * ci + (1 - a) * self._ema_c

        mx = max(self._ema_r, self._ema_g, self._ema_b)
        mn = min(self._ema_r, self._ema_g, self._ema_b)
        v = mx
        d = mx - mn
        s = 0.0 if mx < 1e-6 else d / mx
        avg_pulse = (self.rp + self.gp + self.bp) / 3.0
        c_gain = (
            (self._amb_cp / float(self.cp))
            if (self.cp > 0 and self._amb_cp > 1.0)
            else 1.0
        )
        weak = (
            self._ema_c < 0.009
            or avg_pulse > 185.0
            or v < 0.10
            or s < 0.08
        )
        if weak or c_gain < 1.12:
            return False, 0, 0, 0

        h = 0.0
        if s >= 1e-4:
            if mx == self._ema_r:
                # match C fmodf((g-b)/d + 6, 6)
                h = 60.0 * (((self._ema_g - self._ema_b) / d + 6.0) % 6.0)
            elif mx == self._ema_g:
                h = 60.0 * (((self._ema_b - self._ema_r) / d) + 2.0)
            else:
                h = 60.0 * (((self._ema_r - self._ema_g) / d) + 4.0)

        r_dom = self._ema_r - max(self._ema_g, self._ema_b)
        g_dom = self._ema_g - max(self._ema_r, self._ema_b)
        y_pair = min(self._ema_r, self._ema_g) - self._ema_b
        y_bal = 1.0 - min(1.0, abs(self._ema_r - self._ema_g) / 0.42)
        if y_bal < 0:
            y_bal = 0

        sR = max(0.0, r_dom) * 2.5 + max(0.0, self._ema_r - self._ema_g) * 0.8 + max(
            0.0, self._ema_r - self._ema_b
        ) * 0.5
        if h < 22.0 or h >= 340.0:
            sR += 0.40 * s
        elif h < 38.0:
            sR += 0.18 * s

        sG = max(0.0, g_dom) * 3.1 + max(0.0, self._ema_g - self._ema_r) * 1.15 + max(
            0.0, self._ema_g - self._ema_b
        ) * 0.7
        if 65.0 <= h <= 175.0:
            sG += 0.55 * s * max(0.0, 1.0 - _hue_dist(h, 125.0) / 70.0)
        elif 50.0 <= h < 65.0:
            sG += 0.18 * s

        sY = max(0.0, y_pair) * 2.8 * max(0.30, y_bal)
        if 28.0 <= h <= 90.0:
            sY += 0.50 * s * max(0.0, 1.0 - _hue_dist(h, 52.0) / 42.0)
        if abs(self._ema_r - self._ema_g) < 0.22 and y_pair > 0.03:
            sY += 0.22
        if r_dom > 0.16:
            sY *= 0.45
        if g_dom > 0.14:
            sY *= 0.35
        if min(self._ema_r, self._ema_g) < 0.48:
            sY *= 0.50

        sat = (s - 0.05) / 0.45
        sat = 0.0 if sat < 0 else (1.0 if sat > 1 else sat)
        k = 0.38 + 0.62 * sat
        sR *= k
        sY *= k
        sG *= k

        r = _score_pct(sR)
        y = _score_pct(sY)
        g = _score_pct(sG)
        present = r >= 8 or y >= 8 or g >= 8
        return present, r, y, g

    @staticmethod
    def _rank3(r, y, g):
        leader, best, second = 0, 0, 0

        def put(lab, sc):
            nonlocal leader, best, second
            if sc > best:
                second = best
                best = sc
                leader = lab
            elif sc > second:
                second = sc

        put(1, r)
        put(2, y)
        put(3, g)
        return leader, best, second

    def _locked_score(self):
        if self._lock == 1:
            return self.r100
        if self._lock == 2:
            return self.y100
        if self._lock == 3:
            return self.g100
        return 0

    def _decide(self, leader, best, lead, present):
        if self._lock:
            sc = self._locked_score()
            still = present and (leader == self._lock)
            overtake = (
                present
                and leader
                and leader != self._lock
                and best >= MIN_SCORE
                and best >= sc + SWITCH_LEAD
                and lead >= CLEAR_LEAD
            )
            if overtake:
                self._switch_streak += 1
                if self._switch_streak >= SWITCH_N:
                    self._lock = leader
                    self._conf_ema = max(60.0, float(best))
                    self._switch_streak = 0
                    self._cand = self._streak = 0
                    return
            else:
                self._switch_streak = 0

            if still and sc >= MIN_SCORE:
                target = float(sc)
                if target < 20:
                    target = 20
                if target > 100:
                    target = 100
            elif present and sc >= 10:
                target = float(sc) * 0.4
            else:
                target = 0.0
            a = 0.30 if target < self._conf_ema else 0.45
            self._conf_ema = (1.0 - a) * self._conf_ema + a * target
            if self._conf_ema <= UNLOCK_CONF:
                self._lock = self._cand = self._streak = self._switch_streak = 0
                self._conf_ema = 0.0
                self._ema_init = False
            return

        self._switch_streak = 0
        if (not present) or leader == 0 or best < MIN_SCORE or lead < CLEAR_LEAD:
            self._cand = 0
            self._streak = 0
            self._conf_ema *= 0.5
            if self._conf_ema < 1.0:
                self._conf_ema = 0.0
            return

        if leader == self._cand:
            self._streak += 1
        else:
            self._cand = leader
            self._streak = 1
        self._conf_ema = float(self._streak) * (100.0 / float(LOCK_N))
        if self._conf_ema > 99:
            self._conf_ema = 99
        if self._streak >= LOCK_N:
            self._lock = leader
            self._conf_ema = max(60.0, float(best))
            self._streak = LOCK_N

    def update(self, dist_mm=0, tof_ok=False):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ms) < PERIOD_MS:
            return self.label, self.conf
        self._last_ms = now

        self.led.value(1)
        self.rp = self._read_ch(False, False)
        self._pump()
        self.bp = self._read_ch(False, True)
        self._pump()
        self.cp = self._read_ch(True, False)
        self._pump()
        self.gp = self._read_ch(True, True)
        self._pump()

        far = tof_ok and dist_mm >= AMB_FAR_MM
        self._learn_ambient(far)
        present, r, y, g = self._score_chroma()
        self.r100, self.y100, self.g100 = r, y, g
        leader = best = second = 0
        if present:
            leader, best, second = self._rank3(r, y, g)
        lead = int(best) - int(second)
        self._decide(leader, best, lead, present)
        self.label = self._lock
        self.conf = max(0, min(100, int(round(self._conf_ema))))
        return self.label, self.conf
