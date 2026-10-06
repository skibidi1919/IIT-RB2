# Meowler pinout (`arm_ui` — current USB build)

## Arduino Nano

| Function | Pin |
|----------|-----|
| I2C SDA / SCL (PCA + VL53) | A4 / A5 |
| Encoder M1 C1 / C2 | D3 / D4 |
| Encoder M2 C1 / C2 | D5 / D6 |
| BNO08x soft-I2C SDA / SCL | D7 / D8 |
| USB serial | FTDI (COM3 typical) |

## PCA9685 @ `0x40` (100 Hz)

**OE → GND.** Share GND with Nano + L298N logic.

| CH | Function |
|----|----------|
| 0 | Base servo |
| 1 | Height / humerus servo |
| 2 | Gripper servo |
| 4–9 | held OFF — motors are **not** on PCA |

## M1 = L298N **OUT3 / OUT4** (Nano GPIO)

| L298N | Nano |
|-------|------|
| **ENB** | **D10** (PWM) — **pull ENB jumper** |
| **IN3** | **A2** |
| **IN4** | **A3** |
| OUT3 / OUT4 | **M1** motor |

Unplug PCA from ENB / IN3 / IN4.

## M2 = L298N **OUT1 / OUT2** (Nano GPIO)

| L298N | Nano |
|-------|------|
| **ENA** | **D9** (PWM) — **pull ENA jumper** |
| **IN1** | **A0** |
| **IN2** | **A1** |
| OUT1 / OUT2 | **M2** motor |

Unplug PCA from ENA / IN1 / IN2.

## L298N power

| L298N | Wire |
|-------|------|
| +12V / GND | Battery |
| 5V logic | Shared 5V (don’t double-feed if regulator jumper on) |

## VL53L0X @ `0x29`

VIN→5V, GND→GND, SDA→A4, SCL→A5. XSHUT high / floating.

## BNO08x (soft I2C)

| BNO | Nano |
|-----|------|
| VIN | 5V or 3V3 (per breakout) |
| GND | GND |
| SDA | **D7** |
| SCL | **D8** |
| Address | `0x4A` or `0x4B` |
| INT / RST / 3Vo | nc |

Bit-bang I2C via `arm_ui/soft_bno_i2c.h` (does not share Wire with PCA/VL53).

## TCS3200 (planned — `nano_drive` compact reader)

D7/D8 are BNO. Hardwire S0=5V, S1=GND (20% scale). Soft pins:

| TCS | Nano |
|-----|------|
| S2 / S3 | D11 / D12 |
| OUT / LED | D2 / D13 |

Full map if BNO moves to Wire: S0–S3 = D7/D8/D11/D12, OUT/LED = D2/D13 (legacy `nano_drive`).

## Encoders + odometry

- Quadrature polled (same LUT as `nano_drive`)
- Wheel diameter **43 mm**
- Default **600 steps/rev** (tune `STEPS_PER_REV` in `arm_ui.ino` if mm reads wrong)
- Distance mm = `steps × π × 43 / steps_per_rev`
- VL53 displacement = origin range − current range (`Z` zeros both)

## Robot link boards (optional)

See `AGENT.md` for `color_bridge` / `hub_bridge` / `nano_drive` SoftSerial and TCS3200 pins.
