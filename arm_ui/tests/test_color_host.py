"""Host color filter — stability, uncertainty, no single-sample trust."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from color_host import ColorFilter


def test_noisy_single_sample_uncertain():
    f = ColorFilter()
    out = f.update(1, 90, 80, 10, 10)
    assert out["color"] == 0
    assert out["color_stable"] is False
    assert out["color_name"] == "UNKNOWN"


def test_consistent_red_locks():
    f = ColorFilter()
    last = None
    for _ in range(8):
        last = f.update(1, 70, 78, 16, 12)
    assert last is not None
    assert last["color"] == 1
    assert last["color_stable"] is True
    assert last["color_name"] == "RED"


def test_close_scores_uncertain():
    f = ColorFilter()
    last = None
    for _ in range(8):
        last = f.update(1, 50, 40, 38, 10)  # R≈Y
    assert last is not None
    assert last["color_stable"] is False
    assert last["color"] == 0


def test_moving_needs_more_agreement():
    f = ColorFilter(agree_n=3)
    for _ in range(4):
        f.update(3, 80, 10, 15, 72, moving=False)
    still = f.update(3, 80, 10, 15, 72, moving=False)
    assert still["color_stable"] is True
    f.reset()
    # moving raises agree_n by 1 → 4 needed; 3 samples stay uncertain
    for _ in range(2):
        f.update(3, 80, 10, 15, 72, moving=True)
    moving = f.update(3, 80, 10, 15, 72, moving=True)
    assert moving["color_stable"] is False


def test_yellow_wins_on_bars():
    f = ColorFilter()
    last = None
    for _ in range(8):
        last = f.update(2, 65, 28, 80, 18)
    assert last["color"] == 2
    assert last["color_name"] == "YELLOW"


def test_fw_disagrees_with_bars_stays_unknown():
    f = ColorFilter()
    last = None
    # Bars say green, firmware insists red → uncertain frames
    for _ in range(8):
        last = f.update(1, 80, 12, 14, 78)
    assert last["color"] == 0
    assert last["color_stable"] is False


def test_hysteresis_holds_then_switches():
    f = ColorFilter(agree_n=3, switch_n=4)
    for _ in range(5):
        f.update(1, 70, 80, 15, 12)
    assert f.label == 1
    # brief yellow noise must not flip
    for _ in range(2):
        f.update(2, 70, 25, 78, 14)
    assert f.label == 1
    # sustained yellow switches
    for _ in range(6):
        f.update(2, 70, 25, 78, 14)
    assert f.label == 2
