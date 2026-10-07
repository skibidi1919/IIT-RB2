"""Accuracy report against simulated R/Y/G sensor datasets."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from color_host import ColorFilter, NAMES

from color_datasets import DATASETS, feed


def _eval_dataset(name: str, spec: dict) -> dict:
    f = ColorFilter()
    outs = feed(f, spec["samples"])
    final = outs[-1]
    final_lab = int(final["color"]) if final.get("color_stable") else 0
    expected = int(spec["expected_final"])
    forbid = tuple(spec.get("forbid_stable") or ())

    wrong_stable = 0
    any_forbidden = False
    for o in outs:
        if o.get("color_stable") and o.get("color"):
            lab = int(o["color"])
            if lab in forbid:
                any_forbidden = True
                wrong_stable += 1
            # For clear/noisy expected colors, locking the wrong R/Y/G is fatal
            if expected in (1, 2, 3) and lab != expected:
                wrong_stable += 1

    if expected == 0:
        ok = final_lab == 0 and not any_forbidden
    else:
        ok = final_lab == expected and not any_forbidden

    # Rapid cycle: never accumulate a forbidden sticky wrong; final unknown preferred
    if spec.get("allow_transient_correct"):
        ok = final_lab == 0 and wrong_stable == 0

    return {
        "name": name,
        "ok": ok,
        "expected": NAMES[expected] if expected else "UNKNOWN",
        "final": NAMES[final_lab] if final_lab else "UNKNOWN",
        "final_conf": int(final.get("color_conf") or 0),
        "final_stable": bool(final.get("color_stable")),
        "wrong_stable_frames": wrong_stable,
        "n": len(outs),
    }


def test_dataset_accuracy_report(capsys):
    rows = [_eval_dataset(n, s) for n, s in DATASETS.items()]
    passed = sum(1 for r in rows if r["ok"])
    total = len(rows)
    accuracy = 100.0 * passed / total if total else 0.0

    # Per-class breakdown
    clear = [r for r in rows if r["name"].startswith("clear_") or r["name"].startswith("noisy_")]
    amb = [r for r in rows if "ambiguous" in r["name"] or r["name"] == "low_confidence"]
    clear_ok = sum(1 for r in clear if r["ok"])
    amb_ok = sum(1 for r in amb if r["ok"])

    lines = [
        "",
        "=== Color sensing accuracy (simulated telem) ===",
        f"{'dataset':<24} {'exp':<8} {'got':<8} {'conf':>4}  result",
        "-" * 56,
    ]
    for r in rows:
        mark = "PASS" if r["ok"] else "FAIL"
        lines.append(
            f"{r['name']:<24} {r['expected']:<8} {r['final']:<8} {r['final_conf']:>4}  {mark}"
            + (f"  wrong_frames={r['wrong_stable_frames']}" if r["wrong_stable_frames"] else "")
        )
    lines += [
        "-" * 56,
        f"Overall accuracy: {passed}/{total} = {accuracy:.1f}%",
        f"Clear/noisy lock accuracy: {clear_ok}/{len(clear)}",
        f"Ambiguous/low-conf UNKNOWN rate: {amb_ok}/{len(amb)}",
        "Policy: prefer UNKNOWN over wrong color.",
        "",
    ]
    report = "\n".join(lines)
    print(report)

    assert accuracy >= 90.0, report
    assert clear_ok == len(clear), report
    assert amb_ok == len(amb), report
    failed = [r["name"] for r in rows if not r["ok"]]
    assert not failed, f"failed datasets: {failed}\n{report}"


def test_never_locks_on_first_sample():
    f = ColorFilter()
    o = f.update(1, 99, 95, 5, 5)
    assert o["color"] == 0
    assert o["color_stable"] is False
    assert o["color_name"] == "UNKNOWN"
