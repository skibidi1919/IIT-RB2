# Meowler — Real Hardware Test Checklist

**Do not treat SIMULATED or DRY RUN results as proof that the physical robot worked.**

Link badges in the UI:

| Badge | Meaning |
|--------|---------|
| `SIMULATED` | Connected to localhost mock — not the ESP32 |
| `DRY_RUN` | Link up, but **motor commands are suppressed** |
| `REAL_HARDWARE` | Live robot IP, dry-run **off** — motors can move |

Default after connect: **Dry-run ON** (safe). Use **Arm motors…** only when the field is clear.

---

## A. Network / connection

1. Join Wi‑Fi **DarshIshaan** (PC must be on `192.168.29.x`, not `192.168.1.x`).
2. Confirm robot IP (default `192.168.29.222`).
3. Start dashboard: from `arm_ui`, `uv run dashboard` (port **5050**).
4. Connect in UI → badge should show **DRY_RUN** (or **REAL_HARDWARE** only after arming).
5. **Hardware test → Test link** → expect telem age OK.
6. Disconnect robot power/Wi‑Fi briefly → UI should show disconnect / E-stop path; motors commanded stop.
7. Reconnect → confirm stop-on-connect (wheels idle).

Log file: `arm_ui/logs/hw-YYYYMMDD-HHMMSS.log`

---

## B. Motor safety (bench, dry-run OFF)

Field clear. E-Stop finger-ready. **Arm motors…**

1. **Stop** — wheels idle.
2. **Fwd / Back / Left / Right pulse** — short capped move (≤120, ≤800 ms), then auto-stop.
3. While moving, hit **E-Stop** — motion stops immediately; badge **EMERGENCY STOP**.
4. **Clear E-Stop** only when safe.
5. Unplug TCP / kill Wi‑Fi mid-drive — robot must not keep driving (host E-stop + stop frames).

---

## C. Color sensor (live, not from .rpm)

1. **Test color** with nothing in view → **UNKNOWN** / unstable OK.
2. Present **RED**, **YELLOW**, **GREEN** blocks in turn → locked name matches; wait for “locked”.
3. Confirm UI color updates while **not** replaying a recording.
4. Record a short move → open `.rpm` / event list → **no color decisions** stored.

---

## D. Recording accuracy

1. Dry-run **ON** first: Record → drive pad + arm → Stop rec → file saved, events & duration shown.
2. Arm motors. Place robot on **INITIAL** mark.
3. Record the real routine (same field as game).
4. Stop recording → arm returns toward recorded joint origin (software).
5. Confirm log shows `REC` events with timestamps; drive left/right preserved.

---

## E. Pre-flight + origin (honest limits)

1. Select `.rpm` → **Pre-flight**.
2. Every row **PASS** / **FAIL** / **WARNING**.
3. **Physical field position** must show **WARNING — CANNOT VERIFY**.
4. Critical **FAIL** → Replay blocked (do not bypass).
5. Place robot on INITIAL mark yourself → confirm in Replay dialog.

---

## F. Dry-run full workflow

1. Dry-run **ON**.
2. Pre-flight OK → Replay (dry).
3. Progress advances; **wheels must not move**.
4. Log shows `DRY suppress …` for motion ops.
5. Abort / E-Stop still safe.

---

## G. Real replay

1. Dry-run **OFF** (armed). Pre-flight **PASS** (no critical fail).
2. Robot on INITIAL mark. Clearance confirmed.
3. Replay → watch timing/order; be ready to **Abort** / **E-Stop**.
4. After run: stop idle; note any lag vs recording.
5. Never claim success if badge was **SIMULATED**.

---

## H. Sign-off

| Item | Pass? |
|------|--------|
| Link REAL_HARDWARE when testing physics | |
| Dry-run verified before arming | |
| E-Stop verified under motion | |
| Link-loss safe stop verified | |
| Color live R/Y/G/UNKNOWN | |
| Recording has no color ops | |
| Pre-flight used before replay | |
| Operator placed INITIAL mark | |
| Session log saved under `arm_ui/logs/` | |

**Software cannot verify floor pose or heading.** Only you can confirm the robot is on the INITIAL mark.
