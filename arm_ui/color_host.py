"""Host-side R/Y/G color decision — live only, never from .rpm replay.

Pipeline (per telem sample):
  1. Clamp / reject garbage
  2. EMA-smooth R, Y, G bars (kills single-sample spikes)
  3. Rank smoothed scores; require absolute lead + lead ratio
  4. Vote with firmware label only when it agrees with bar winner
  5. Consecutive-agreement gate before LOCK
  6. Hysteresis to switch or unlock (prefer UNKNOWN over wrong)

Prefer UNKNOWN over a confident wrong call.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


# 0 UNKNOWN · 1 RED · 2 YELLOW · 3 GREEN
NAMES = ("UNKNOWN", "RED", "YELLOW", "GREEN")
LAB_RED, LAB_YEL, LAB_GRN = 1, 2, 3


def _clamp_pct(v: object) -> int:
    try:
        return max(0, min(100, int(v)))
    except (TypeError, ValueError):
        return 0


def _clamp_lab(v: object) -> int:
    try:
        n = int(v)
    except (TypeError, ValueError):
        return 0
    return n if 0 <= n <= 3 else 0


@dataclass
class ColorFilter:
    """Stability filter over firmware R/Y/G telem bars + optional fw label."""

    window: int = 7
    min_conf: int = 32          # min winning bar (alias: min_score)
    min_lead: int = 10          # absolute lead best−second
    min_lead_ratio: float = 0.16
    agree_n: int = 4            # samples needed to lock
    unlock_n: int = 3           # weak/disagree frames to drop lock
    switch_n: int = 5           # clear rival frames to switch lock
    ema_alpha: float = 0.38
    _hist: deque = field(default_factory=deque)
    label: int = 0
    conf: int = 0
    stable: bool = False
    _ema_r: float = 0.0
    _ema_y: float = 0.0
    _ema_g: float = 0.0
    _ema_ready: bool = False
    _cand: int = 0
    _streak: int = 0
    _unlock_streak: int = 0
    _switch_streak: int = 0
    _switch_lab: int = 0

    def __post_init__(self) -> None:
        self._hist = deque(maxlen=max(3, int(self.window)))

    def reset(self) -> None:
        self._hist.clear()
        self.label = 0
        self.conf = 0
        self.stable = False
        self._ema_r = self._ema_y = self._ema_g = 0.0
        self._ema_ready = False
        self._cand = 0
        self._streak = 0
        self._unlock_streak = 0
        self._switch_streak = 0
        self._switch_lab = 0

    def update(
        self,
        label: int,
        conf: int,
        r: int,
        y: int,
        g: int,
        *,
        moving: bool = False,
        dist_mm: int | None = None,
    ) -> dict:
        """Feed one telem sample. Returns filtered decision dict.

        Args match robot telem: label, color_conf, color_r, color_b(yellow), color_g.
        Optional dist_mm tightens gates when object is far (reflections / weak signal).
        """
        fw = _clamp_lab(label)
        conf_i = _clamp_pct(conf)
        r_i, y_i, g_i = _clamp_pct(r), _clamp_pct(y), _clamp_pct(g)

        a = self.ema_alpha * (0.72 if moving else 1.0)
        if not self._ema_ready:
            self._ema_r, self._ema_y, self._ema_g = float(r_i), float(y_i), float(g_i)
            self._ema_ready = True
        else:
            self._ema_r = a * r_i + (1 - a) * self._ema_r
            self._ema_y = a * y_i + (1 - a) * self._ema_y
            self._ema_g = a * g_i + (1 - a) * self._ema_g

        sr, sy, sg = int(round(self._ema_r)), int(round(self._ema_y)), int(round(self._ema_g))
        instant = self._rank_candidate(sr, sy, sg, moving=moving, dist_mm=dist_mm)

        # Firmware vote only reinforces an already-clear bar winner
        if instant != 0 and fw != 0 and fw != instant:
            # Disagreement → treat frame as uncertain (reflections / lag)
            instant = 0

        self._hist.append((instant, conf_i, sr, sy, sg, fw))
        self._advance_fsm(instant, conf_i, sr, sy, sg, moving=moving, dist_mm=dist_mm)
        return self._out(r_i, y_i, g_i, sr, sy, sg)

    def _thresholds(self, *, moving: bool, dist_mm: int | None) -> tuple[int, int, float, int]:
        min_score = self.min_conf + (10 if moving else 0)
        min_lead = self.min_lead + (4 if moving else 0)
        ratio = self.min_lead_ratio + (0.06 if moving else 0.0)
        need_agree = self.agree_n + (1 if moving else 0)
        if dist_mm is not None and dist_mm > 0:
            if dist_mm >= 180:
                min_score += 10
                min_lead += 4
                ratio += 0.05
                need_agree += 1
            elif dist_mm >= 120:
                min_score += 5
                min_lead += 2
                ratio += 0.03
        return min_score, min_lead, ratio, need_agree

    def _rank_candidate(
        self,
        r: int,
        y: int,
        g: int,
        *,
        moving: bool,
        dist_mm: int | None,
    ) -> int:
        min_score, min_lead, ratio, _ = self._thresholds(moving=moving, dist_mm=dist_mm)
        scores = ((LAB_RED, r), (LAB_YEL, y), (LAB_GRN, g))
        ranked = sorted(scores, key=lambda kv: kv[1], reverse=True)
        best_lab, best = ranked[0]
        second = ranked[1][1]
        lead = best - second
        if best < min_score:
            return 0
        if lead < min_lead:
            return 0
        if best > 0 and (lead / best) < ratio:
            return 0
        # Mid-pack reflection guard: all three high-ish and close → unknown
        if second >= min_score - 4 and lead < min_lead + 4:
            return 0
        return best_lab

    def _advance_fsm(
        self,
        instant: int,
        conf_i: int,
        r: int,
        y: int,
        g: int,
        *,
        moving: bool,
        dist_mm: int | None,
    ) -> None:
        _, _, _, need_agree = self._thresholds(moving=moving, dist_mm=dist_mm)
        scores = {LAB_RED: r, LAB_YEL: y, LAB_GRN: g}

        if self.label != 0:
            # Locked — hold unless clearly overtaken or signal collapses
            locked_score = scores.get(self.label, 0)
            if instant == self.label and locked_score >= max(18, self.min_conf // 2):
                self._unlock_streak = 0
                self._switch_streak = 0
                self._switch_lab = 0
                self.stable = True
                self.conf = max(self.conf - 2, min(100, max(conf_i, locked_score)))
                return

            if instant != 0 and instant != self.label:
                if instant == self._switch_lab:
                    self._switch_streak += 1
                else:
                    self._switch_lab = instant
                    self._switch_streak = 1
                need_sw = self.switch_n + (1 if moving else 0)
                if self._switch_streak >= need_sw:
                    self.label = instant
                    self.stable = True
                    self.conf = max(55, scores[instant])
                    self._cand = instant
                    self._streak = need_agree
                    self._switch_streak = 0
                    self._unlock_streak = 0
                else:
                    self.stable = True  # keep previous while undecided
                    self.conf = max(20, self.conf - 5)
                return

            # Weak / unknown while locked
            self._switch_streak = 0
            self._switch_lab = 0
            self._unlock_streak += 1
            self.conf = max(0, self.conf - 12)
            if self._unlock_streak >= self.unlock_n or self.conf <= 12:
                self.label = 0
                self.stable = False
                self.conf = 0
                self._cand = 0
                self._streak = 0
            else:
                self.stable = True
            return

        # Unlocked — build agreement streak
        self._unlock_streak = 0
        self._switch_streak = 0
        if instant == 0:
            self._cand = 0
            self._streak = 0
            self.stable = False
            self.label = 0
            self.conf = max(0, min(conf_i, max(r, y, g)) // 2)
            return

        if instant == self._cand:
            self._streak += 1
        else:
            self._cand = instant
            self._streak = 1

        self.conf = min(99, int(self._streak * (100 / max(1, need_agree))))
        if self._streak >= need_agree:
            self.label = instant
            self.stable = True
            self.conf = max(60, scores[instant])
        else:
            self.label = 0
            self.stable = False

    def _out(self, r: int, y: int, g: int, sr: int, sy: int, sg: int) -> dict:
        name = NAMES[self.label] if self.label else "UNKNOWN"
        return {
            "color": self.label,
            "color_name": name,
            "color_conf": int(self.conf),
            "color_stable": bool(self.stable and self.label != 0),
            "color_r": r,
            "color_g": g,
            "color_b": y,  # yellow bar (panel / telem convention)
            "color_ema_r": sr,
            "color_ema_y": sy,
            "color_ema_g": sg,
        }
