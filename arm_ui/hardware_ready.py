"""Hardware readiness — link kind, dry-run policy, preflight, HW test limits.

Does not claim physical success for simulator sessions.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class LinkKind(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    SIMULATED = "SIMULATED"          # localhost / mock robot
    DRY_RUN = "DRY_RUN"              # real or sim link, motors suppressed
    REAL_HARDWARE = "REAL_HARDWARE"  # live robot, motors armed


# Ops that move actuators — suppressed in dry-run (stop always allowed)
MOTION_OPS = frozenset({
    "drive", "arm", "center", "zero", "conveyor", "motor_test",
})

# Hardware bench test speed caps (never full-scale on first touch)
HW_TEST_DRIVE_MAX = 120
HW_TEST_CONV_MAX = 100
HW_TEST_PULSE_MS = 400


def classify_host(host: str | None) -> str:
    """Return SIMULATED vs REAL_HARDWARE candidate from host string (before dry-run)."""
    if not host:
        return LinkKind.DISCONNECTED.value
    h = host.split(":", 1)[0].strip().lower()
    if h in ("127.0.0.1", "localhost", "::1") or h.startswith("127."):
        return LinkKind.SIMULATED.value
    return LinkKind.REAL_HARDWARE.value


def resolve_link_kind(*, connected: bool, host: str | None, dry_run: bool) -> str:
    if not connected:
        return LinkKind.DISCONNECTED.value
    base = classify_host(host)
    if dry_run:
        return LinkKind.DRY_RUN.value
    return base


def is_motion_op(which: str | None) -> bool:
    return which in MOTION_OPS


@dataclass
class CheckResult:
    id: str
    label: str
    status: str  # PASS | FAIL | WARNING
    detail: str
    critical: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "status": self.status,
            "detail": self.detail,
            "critical": self.critical,
        }


def run_preflight(state: dict[str, Any], *, events: list[dict] | None = None) -> dict[str, Any]:
    """Software-verifiable checks only — never invents physical pose truth."""
    checks: list[CheckResult] = []

    connected = bool(state.get("connected"))
    checks.append(CheckResult(
        "link",
        "Robot TCP link",
        "PASS" if connected else "FAIL",
        state.get("host") or "not connected",
        critical=True,
    ))

    kind = state.get("link_kind") or LinkKind.DISCONNECTED.value
    if kind == LinkKind.SIMULATED.value:
        checks.append(CheckResult(
            "link_kind",
            "Link type",
            "WARNING",
            "SIMULATED — localhost mock; not physical robot",
            critical=False,
        ))
    elif kind == LinkKind.DRY_RUN.value:
        checks.append(CheckResult(
            "link_kind",
            "Link type",
            "WARNING",
            "DRY RUN — workflow runs; motor TX suppressed",
            critical=False,
        ))
    elif kind == LinkKind.REAL_HARDWARE.value:
        checks.append(CheckResult(
            "link_kind",
            "Link type",
            "PASS",
            "REAL HARDWARE — motors can move",
            critical=False,
        ))
    else:
        checks.append(CheckResult(
            "link_kind",
            "Link type",
            "FAIL",
            "Disconnected",
            critical=True,
        ))

    estop = bool(state.get("estop"))
    checks.append(CheckResult(
        "estop",
        "Emergency stop clear",
        "FAIL" if estop else "PASS",
        "latched" if estop else "clear",
        critical=True,
    ))

    fault = bool(state.get("fault"))
    checks.append(CheckResult(
        "fault",
        "No hard fault",
        "FAIL" if fault else "PASS",
        state.get("error") or "ok",
        critical=True,
    ))

    age = state.get("last_telem_age_ms")
    if not connected:
        telem_status, telem_detail = "FAIL", "no telem"
    elif age is None:
        telem_status, telem_detail = "WARNING", "waiting for first telem"
    elif int(age) > 1500:
        telem_status, telem_detail = "FAIL", f"stale telem {age}ms"
    elif int(age) > 600:
        telem_status, telem_detail = "WARNING", f"telem age {age}ms"
    else:
        telem_status, telem_detail = "PASS", f"telem age {age}ms"
    checks.append(CheckResult(
        "telem", "Telemetry fresh", telem_status, telem_detail, critical=True,
    ))

    checks.append(CheckResult(
        "pca",
        "PCA / servos reported",
        "PASS" if state.get("pca_ok") else "WARNING",
        "pca_ok" if state.get("pca_ok") else "pca not ok (arm may not respond)",
        critical=False,
    ))
    checks.append(CheckResult(
        "tof",
        "TOF reported",
        "PASS" if state.get("tof_ok") else "WARNING",
        f"range={state.get('distance_mm')}mm",
        critical=False,
    ))
    checks.append(CheckResult(
        "imu",
        "IMU reported",
        "PASS" if state.get("imu_ok") else "WARNING",
        "imu_ok" if state.get("imu_ok") else "imu not ok",
        critical=False,
    ))

    motors_zero = int(state.get("left") or 0) == 0 and int(state.get("right") or 0) == 0
    checks.append(CheckResult(
        "motors_idle",
        "Drive cmd idle (telem)",
        "PASS" if motors_zero else "WARNING",
        f"cmd L/R={state.get('left')}/{state.get('right')}",
        critical=False,
    ))

    # Origin — software can only know recorded arm keyframe, NOT floor pose
    origin = state.get("origin")
    checks.append(CheckResult(
        "origin_arm",
        "Recorded arm origin (software)",
        "PASS" if origin else "WARNING",
        (
            f"base/height/grip={origin.get('base')}/{origin.get('height')}/{origin.get('grip')}"
            if isinstance(origin, dict)
            else "no origin captured yet — will use first arm keyframe in .rpm"
        ),
        critical=False,
    ))
    checks.append(CheckResult(
        "origin_pose",
        "Physical field position verified",
        "WARNING",
        "CANNOT VERIFY — place robot on INITIAL field mark manually before replay",
        critical=False,
    ))

    # Recording content
    evs = list(events or [])
    if not evs:
        checks.append(CheckResult(
            "recording",
            "Replayable events loaded",
            "FAIL",
            "empty / no events",
            critical=True,
        ))
    else:
        has_color = any(e.get("op") == "color" for e in evs)
        checks.append(CheckResult(
            "recording",
            "Replayable events loaded",
            "PASS",
            f"{len(evs)} events, duration≈{evs[-1].get('t_ms', 0)}ms",
            critical=True,
        ))
        checks.append(CheckResult(
            "no_color_in_rpm",
            "Color decisions not in recording",
            "FAIL" if has_color else "PASS",
            "color ops found (must not replay)" if has_color else "motion timeline only",
            critical=True,
        ))
        # speed bound check
        over = [
            e for e in evs
            if e.get("op") == "drive"
            and (abs(int(e.get("left", 0))) > 255 or abs(int(e.get("right", 0))) > 255)
        ]
        checks.append(CheckResult(
            "speed_bounds",
            "Recorded drive within ±255",
            "FAIL" if over else "PASS",
            f"{len(over)} out-of-range" if over else "ok",
            critical=True,
        ))

    if bool(state.get("recording")) or bool(state.get("replaying")):
        checks.append(CheckResult(
            "idle",
            "Not already recording/replaying",
            "FAIL",
            "busy",
            critical=True,
        ))
    else:
        checks.append(CheckResult(
            "idle",
            "Not already recording/replaying",
            "PASS",
            "idle",
            critical=True,
        ))

    critical_fail = any(c.status == "FAIL" and c.critical for c in checks)
    warnings = sum(1 for c in checks if c.status == "WARNING")
    return {
        "ok_to_replay": not critical_fail,
        "critical_fail": critical_fail,
        "warning_count": warnings,
        "checks": [c.as_dict() for c in checks],
        "link_kind": kind,
        "origin_limitation": (
            "Software restores arm joint angles from the recording origin keyframe. "
            "It cannot verify the robot's physical (x,y,heading) on the field. "
            "Operator must place the robot on the INITIAL mark before replay."
        ),
    }


def clamp_hw_drive(left: int, right: int) -> tuple[int, int]:
    lo = -HW_TEST_DRIVE_MAX
    hi = HW_TEST_DRIVE_MAX
    return max(lo, min(hi, int(left))), max(lo, min(hi, int(right)))


def clamp_hw_conveyor(speed: int) -> int:
    return max(-HW_TEST_CONV_MAX, min(HW_TEST_CONV_MAX, int(speed)))
