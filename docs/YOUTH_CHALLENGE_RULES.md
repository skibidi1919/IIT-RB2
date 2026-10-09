# Robotics for Good Youth Challenge 2026–2027 — Official Rules & Field Specs

Promoted by the **International Telecommunication Union (ITU)** in partnership with **make+learn**.

---

## 1. Match Overview & General Rules

- **Match Duration**: **2 minutes (120 seconds)**.
- **Operation**: **100% Autonomous** — zero human intervention or interpretation allowed once the match begins.
- **Team Composition**: Up to 8 team members; max 3 members in the active competition area.
- **Multi-Robot Rule**: Teams may use multiple robots operating simultaneously from the starting zone.
- **Robot Decoration**: Must align with **SDG 3: Good Health and Well-Being**.

---

## 2. Latest Rule Amendments (Last-Minute Updates)

1. **Lego / Block-Based Robots**: Permitted. Teams may use Lego Technic or similar block systems for frames, arm linkages, gears, and pulley mechanisms.
2. **Skewed Quarantine Walls**: The containment beams for the quarantine zone **do NOT need to form a strict 90° right angle**. Skewed/oblique wall placement is valid as long as the perimeter is enclosed and beams touch the field boundaries.

---

## 3. Official Field & Zone Dimensions

| Area / Component | Dimensions (mm) | Description / Notes |
|---|---|---|
| **Full Game Board Surface** | $2362 \pm 5 \times 1143 \pm 5\text{ mm}$ | Divided into 2 equal halves for simultaneous team play. |
| **Competition Field (Half)** | $1181 \pm 6 \times 1143 \pm 5\text{ mm}$ | White smooth/glossy surface with perimeter walls ($65\text{ mm}$ high). |
| **Robot Starting Zone** | $480 \times 280\text{ mm}$ | Located at bottom of the field, marked by $20\text{ mm}$ black tape. |
| **Recovery Zone (RZ)** | Inside Starting Zone | Area for low-urgency / green triage cases. |
| **Quarantine Zone** | $280 \times 280\text{ mm}$ | Lower-left corner of field; bounded by 2 fixed board walls & 2 tape lines. |
| **Laboratory Zone** | Adjacent to Quarantine | Wooden structure ($345 \times 150\text{ mm}$) with three $\varnothing 60\text{ mm}$ slots for sample discs. |
| **Healthcare Destination Zones** | Top of the field | Center: **Hospital (H)** ($500 \times 180\text{ mm}$)<br>Corners: **2 Primary Care Centres (PCC)** ($300 \times 180\text{ mm}$ each). |

---

## 4. Game Elements & Specifications

| Element | Qty | Material & Dimensions | Function / Destination |
|---|---|---|---|
| **Samples** | 3 | Wood, flat discs, $\varnothing 56\text{ mm} \times 5\text{ mm}$ | Extracted from Quarantine $\rightarrow$ Delivered to Laboratory slots. |
| **Containment Beams (Walls)** | 2 | Wood, $60 \times 20\text{ mm}$ cross-section<br>• Beam 1: $250 \times 60 \times 20\text{ mm}$<br>• Beam 2: $280 \times 60 \times 20\text{ mm}$ | Transported by Robot 2 $\rightarrow$ Placed upright on $20\text{ mm}$ edge to seal Quarantine Zone. |
| **Medical Kits** | 10 | Wood cubes, $25 \times 25 \times 20\text{ mm}$ (with red cross) | 6 to Hospital (H), 2 to Left PCC, 2 to Right PCC. |
| **Triaged Patients (Cylinders)** | 12 | Wood cylinders, $\varnothing 20 \times 20\text{ mm}$<br>• 4 Red (Critical)<br>• 4 Yellow (Observation)<br>• 4 Green (Low-risk) | • Red $\rightarrow$ Hospital (H)<br>• Yellow $\rightarrow$ Primary Care Centres (PCC)<br>• Green $\rightarrow$ Recovery Zone (RZ). |

---

## 5. Missions & Scoring Breakdown

### Mission 1: Containment and Sample Management

| Action | Points |
|---|---|
| Correctly placing 1 sample inside a laboratory slot | **+15 pts** |
| Correctly placing all 3 samples inside laboratory slots | **+5 pts** bonus |
| Correctly placing 1 containment beam upright & stable by itself | **+25 pts** |
| Completing the quarantine perimeter with both beams & field walls | **+20 pts** |
| **Maximum Mission 1 Score** | **70 pts** |

> **Crucial Rule for Walls**: Beams must be standing upright on their long side, stable by themselves, and **fully released by the robot**. No robot, arm, or mechanism may touch or support the beams at the end of the match.

### Mission 2: Healthcare Management

| Action | Points |
|---|---|
| Correctly placing 1 medical kit in a valid destination zone | **+3 pts** (up to 30) |
| Achieving correct kit distribution (6 in H, 2 in PCC-L, 2 in PCC-R) | **+20 pts** |
| Placing 1 triaged patient cylinder in correct zone | **+5 pts** (up to 60) |
| Placing all 4 Red cylinders in Hospital (H) | **+6 pts** |
| Distributing Yellow cylinders across PCCs | **+8 pts** |
| Placing all 4 Green cylinders in Recovery Zone (RZ) | **+6 pts** |

---

## 6. Penalties Reference

| Infraction | Penalty |
|---|---|
| Touching the robot while match is running | **-20 pts** |
| Manipulating field or game pieces during round | **-20 pts** |
| Unauthorised restart / modifying program during round | **-20 pts** |
| Robot leaves field completely | **-20 pts** |
| Sample left inside quarantine area (Senior) | **-5 pts** each |
| Sample outside quarantine & lab at match end (Senior) | **-3 pts** each |
| Destination zone left with no kits at end (Senior) | **-10 pts** |
| Triaged patient placed in incorrect zone (Senior) | **-3 pts** each |

---

## 7. Robot 2 (RB2) Mission Architecture & Integration

Robot 2 is dedicated to **Mission 1 — Step 2 (Containment Wall Construction)**:

```
[ Power On ]
     │
     ▼
[ 2.5s Start Delay ] (Hands clear of starting zone)
     │
     ▼
[ Drive Forward ] (Active Closed-Loop balance overcomes left-side elevation without drift)
     │ (Reaches ~2000 encoder counts / Quarantine Zone boundary)
     ▼
[ Lower Pulley Arms ] (PCA9685 CH0/CH1 servos smoothly sweep down to place 2 beams upright)
     │ (800ms settle delay)
     ▼
[ Disengage & Back Up ] (Reverses 500ms to guarantee zero physical contact with beams)
     │
     ▼
[ Retract Pulley Arms ] (Returns arms/pulleys to raised home position)
     │
     ▼
[ Parked in Quarantine Zone ] (All motors halt; remains parked for referee scoring)
```

### Wheel & Encoder Calibration Reference

- **Wheel Diameter**: $43\text{ mm}$ ($\text{Circumference} = \pi \times 43 \approx 135.1\text{ mm}$)
- **Resolution**: $600\text{ ticks/rev} \approx \mathbf{4.44\text{ counts/mm}}$ ($\approx 44.4\text{ counts/cm}$)
- **Firmware Location**: [`rb2/rb2.ino`](file:///c:/meowler/rb2/rb2.ino)
- **Tuning Constants**:
  - `QUARANTINE_ZONE_COUNTS`: `2000` (target travel distance)
  - `AUTON_DRIVE_SPEED`: `195` (torque for elevation ramp)
  - `ENC_BAL_KP` / `KI`: `0.14` / `0.006` (closed-loop left elevation compensation)
