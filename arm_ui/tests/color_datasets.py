"""Simulated TCS3200 telem streams — R/Y/G bars + firmware label + conf.

Each sample: (fw_label, conf, r, y, g, moving [, dist_mm])
Expected:
  final   — required locked label at end (0 = UNKNOWN)
  forbid  — labels that must never appear as a stable lock
"""

from __future__ import annotations

import random
from typing import Any


def _noise(base: tuple[int, int, int], sigma: float, rng: random.Random) -> tuple[int, int, int]:
    out = []
    for v in base:
        n = int(round(v + rng.gauss(0, sigma)))
        out.append(max(0, min(100, n)))
    return out[0], out[1], out[2]


def _stream_clear(lab: int, bars: tuple[int, int, int], n: int = 12) -> list[tuple]:
    r, y, g = bars
    return [(lab, 75, r, y, g, False, 80) for _ in range(n)]


def _stream_noisy(lab: int, bars: tuple[int, int, int], n: int = 16, seed: int = 1) -> list[tuple]:
    rng = random.Random(seed)
    out = []
    for i in range(n):
        rr, yy, gg = _noise(bars, 9.0, rng)
        # occasional spike toward wrong channel
        if i % 5 == 4:
            spike = list((rr, yy, gg))
            wrong = (lab % 3)  # 0→y, 1→g, 2→r index offset
            idx = {1: 1, 2: 2, 3: 0}[lab]
            spike[idx] = min(100, spike[idx] + 18)
            rr, yy, gg = spike[0], spike[1], spike[2]
        fw = lab if rng.random() > 0.15 else 0
        out.append((fw, max(40, 70 + rng.randint(-15, 10)), rr, yy, gg, i % 3 == 0, 90))
    return out


def _stream_ambiguous(n: int = 14, seed: int = 2) -> list[tuple]:
    rng = random.Random(seed)
    out = []
    for i in range(n):
        # R≈Y, weak G — classic false-red risk
        r = 42 + rng.randint(-4, 4)
        y = 40 + rng.randint(-4, 4)
        g = 18 + rng.randint(-3, 3)
        fw = rng.choice([0, 1, 2])
        out.append((fw, 35 + rng.randint(0, 20), r, y, g, False, 100))
    return out


def _stream_rapid(n: int = 18, seed: int = 3) -> list[tuple]:
    rng = random.Random(seed)
    palette = [
        (1, (78, 18, 14)),
        (2, (28, 76, 20)),
        (3, (14, 18, 74)),
    ]
    out = []
    for i in range(n):
        lab, bars = palette[i % 3]
        rr, yy, gg = _noise(bars, 5.0, rng)
        out.append((lab, 70, rr, yy, gg, True, 85))
    return out


def _stream_low_conf(n: int = 12, seed: int = 4) -> list[tuple]:
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        r = 22 + rng.randint(0, 8)
        y = 20 + rng.randint(0, 8)
        g = 19 + rng.randint(0, 8)
        out.append((rng.choice([0, 1, 2, 3]), 12 + rng.randint(0, 10), r, y, g, False, 200))
    return out


def _stream_reflection(n: int = 12, seed: int = 5) -> list[tuple]:
    """Bright specular: all channels elevated, small lead."""
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        base = 55 + rng.randint(-3, 3)
        r = base + rng.randint(0, 6)
        y = base + rng.randint(0, 6)
        g = base + rng.randint(0, 5)
        out.append((1, 60, r, y, g, False, 70))
    return out


DATASETS: dict[str, dict[str, Any]] = {
    "clear_red": {
        "expected_final": 1,
        "forbid_stable": (2, 3),
        "samples": _stream_clear(1, (82, 16, 12)),
    },
    "clear_yellow": {
        "expected_final": 2,
        "forbid_stable": (1, 3),
        "samples": _stream_clear(2, (30, 84, 18)),
    },
    "clear_green": {
        "expected_final": 3,
        "forbid_stable": (1, 2),
        "samples": _stream_clear(3, (12, 16, 80)),
    },
    "noisy_red": {
        "expected_final": 1,
        "forbid_stable": (2, 3),
        "samples": _stream_noisy(1, (76, 18, 14), seed=11),
    },
    "noisy_yellow": {
        "expected_final": 2,
        "forbid_stable": (1, 3),
        "samples": _stream_noisy(2, (28, 74, 16), seed=12),
    },
    "noisy_green": {
        "expected_final": 3,
        "forbid_stable": (1, 2),
        "samples": _stream_noisy(3, (14, 18, 72), seed=13),
    },
    "ambiguous_ry": {
        "expected_final": 0,
        "forbid_stable": (1, 2, 3),
        "samples": _stream_ambiguous(seed=21),
    },
    "ambiguous_reflection": {
        "expected_final": 0,
        "forbid_stable": (1, 2, 3),
        "samples": _stream_reflection(seed=22),
    },
    "rapid_ryg_cycle": {
        # Must not confidently lock a wrong lasting color; end UNKNOWN preferred
        "expected_final": 0,
        "forbid_stable": (),  # any brief lock of a true frame color is ok; wrong sticky checked separately
        "allow_transient_correct": True,
        "samples": _stream_rapid(seed=31),
    },
    "low_confidence": {
        "expected_final": 0,
        "forbid_stable": (1, 2, 3),
        "samples": _stream_low_conf(seed=41),
    },
    "clear_red_moving": {
        "expected_final": 1,
        "forbid_stable": (2, 3),
        "samples": [(1, 72, 80, 15, 12, True, 95) for _ in range(14)],
    },
    "clear_red_far": {
        "expected_final": 1,
        "forbid_stable": (2, 3),
        "samples": [(1, 68, 78, 17, 14, False, 190) for _ in range(16)],
    },
}


def feed(filter_obj, samples: list[tuple]) -> list[dict]:
    outs = []
    for s in samples:
        if len(s) >= 7:
            lab, conf, r, y, g, moving, dist = s[:7]
            outs.append(filter_obj.update(lab, conf, r, y, g, moving=moving, dist_mm=dist))
        else:
            lab, conf, r, y, g, moving = s[:6]
            outs.append(filter_obj.update(lab, conf, r, y, g, moving=moving))
    return outs
