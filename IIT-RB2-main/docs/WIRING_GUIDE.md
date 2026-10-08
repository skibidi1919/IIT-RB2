# Master Wiring & System Architecture Guide: 4WD N20 Rover

This document provides the complete hardware wiring specifications, pin assignments, electrical distribution, and communication bus topography for the 4WD N20 Rover based on the audited direct-pinout architecture.

---

## 1. System Architecture Overview

```mermaid
graph TD
    Batt[11.1V / 12V Li-ion Battery] -->|12V High Current| L298NA[L298N Driver A<br>Motors 1 & 2]
    Batt -->|12V High Current| L298NB[L298N Driver B<br>Motors 3 & 4]
    
    L298NA -->|5V Regulated Out| Shield5V[Sensor Shield 5V Bus]
    Batt -->|Common Ground| GNDBus[Common Ground Bus]
    
    subgraph Nano_Direct_Control [Arduino Nano Direct GPIO & PWM]
        Nano[Arduino Nano Controller]
        Nano -->|D6 PWM / D7 Dir / D8 Dir| L298NA
        Nano -->|D9 PWM / D12 Dir / D13 Dir| L298NA
        Nano -->|D10 PWM / A0 Dir / A1 Dir| L298NB
        Nano -->|D11 PWM / A2 Dir / A3 Dir| L298NB
    end

    subgraph I2C_Bus [I2C Communication Bus: A4 SDA / A5 SCL]
        Nano -->|I2C Master| PCA[PCA9685 16-Ch PWM Board<br>Addr: 0x40]
        Nano -->|I2C Master| TOF[VL53L0X Laser ToF<br>Addr: 0x29]
        Nano -->|I2C Master| IMU[BNO08x 9-DOF IMU<br>Addr: 0x4A]
    end

    L298NA -->|OUT1 / OUT2| M1[N20 Motor 1 - Front Left]
    L298NA -->|OUT3 / OUT4| M2[N20 Motor 2 - Front Right]
    L298NB -->|OUT1 / OUT2| M3[N20 Motor 3 - Rear Left]
    L298NB -->|OUT3 / OUT4| M4[N20 Motor 4 - Rear Right]
    
    M1 -->|C1 Pulse| D2_Pin[Nano D2: External Interrupt 0]
    M2 -->|C1 Pulse| D3_Pin[Nano D3: External Interrupt 1]
    M3 -->|C1 Pulse| D5_Pin[Nano D5: PCINT21 on Port D]
    M4 -->|C1 Pulse| D4_Pin[Nano D4: PCINT20 on Port D]
```

---

## 2. Master Arduino Nano Pin Assignment

| Arduino Nano Pin | Shield Row | Connected Subsystem / Signal | Hardware Function | Status |
| :--- | :--- | :--- | :--- | :--- |
| **D0 (RX)** | D0 (S) | **UNCONNECTED** | USB Serial Upload & Debug | ✅ Safe & Free |
| **D1 (TX)** | D1 (S) | **UNCONNECTED** | USB Serial Upload & Debug | ✅ Safe & Free |
| **D2** | D2 (S) | **Motor 1 Encoder Signal (C1)** | External Interrupt 0 (`INT0`) | ✅ Dedicated |
| **D3** | D3 (S) | **Motor 2 Encoder Signal (C1)** | External Interrupt 1 (`INT1`) | ✅ Dedicated |
| **D4** | D4 (S) | **Motor 4 Encoder Signal (C1)** | Pin Change Interrupt (`PCINT20`) | ✅ Dedicated |
| **D5** | D5 (S) | **Motor 3 Encoder Signal (C1)** | Pin Change Interrupt (`PCINT21`) | ✅ Dedicated |
| **D6** | D6 (S) | **Driver A: ENA (Motor 1 Speed)** | Hardware PWM (`Timer0A`) | ✅ Dedicated |
| **D7** | D7 (S) | **Driver A: IN1 (Motor 1 Dir A)** | Digital GPIO | ✅ Dedicated |
| **D8** | D8 (S) | **Driver A: IN2 (Motor 1 Dir B)** | Digital GPIO | ✅ Dedicated |
| **D9** | D9 (S) | **Driver A: ENB (Motor 2 Speed)** | Hardware PWM (`Timer1A`) | ✅ Dedicated |
| **D10** | D10 (S) | **Driver B: ENA (Motor 3 Speed)** | Hardware PWM (`Timer1B`) | ✅ Dedicated |
| **D11** | D11 (S) | **Driver B: ENB (Motor 4 Speed)** | Hardware PWM (`Timer2A`) | ✅ Dedicated |
| **D12** | D12 (S) | **Driver A: IN3 (Motor 2 Dir A)** | Digital GPIO | ✅ Dedicated |
| **D13** | D13 (S) | **Driver A: IN4 (Motor 2 Dir B)** | Digital GPIO (Onboard LED line) | ✅ Dedicated |
| **A0** | A0 (S) | **Driver B: IN1 (Motor 3 Dir A)** | Digital GPIO (`D14`) | ✅ Dedicated |
| **A1** | A1 (S) | **Driver B: IN2 (Motor 3 Dir B)** | Digital GPIO (`D15`) | ✅ Dedicated |
| **A2** | A2 (S) | **Driver B: IN3 (Motor 4 Dir A)** | Digital GPIO (`D16`) | ✅ Dedicated |
| **A3** | A3 (S) | **Driver B: IN4 (Motor 4 Dir B)** | Digital GPIO (`D17`) | ✅ Dedicated |
| **A4 (SDA)** | A4 (S) / SDA | **I2C Bus Data Line** | Hardware I2C (PCA9685, VL53L0X, BNO08x) | ✅ Shared Bus |
| **A5 (SCL)** | A5 (S) / SCL | **I2C Bus Clock Line** | Hardware I2C (PCA9685, VL53L0X, BNO08x) | ✅ Shared Bus |
| **A6** | A6 (S) | Spare Analog Input Only | Available (e.g. Battery Voltage Divider) | ⚪ Spare |
| **A7** | A7 (S) | Spare Analog Input Only | Available (e.g. Current Sense) | ⚪ Spare |
| **5V (V)** | All V pins | **+5V Logic Power Rail** | Powered from Driver A 5V Regulator | ✅ Connected |
| **GND (G)** | All G pins | **Common Ground Plane** | Common Ground for all logic & battery | ✅ Connected |

---

## 3. L298N Motor Driver Wiring

### Driver A (Motors 1 & 2: Front Left & Front Right)
| L298N-A Terminal | Target Motor | Arduino Nano Pin | Shield Header | Function / Signal |
| :--- | :--- | :--- | :--- | :--- |
| **ENA** | Motor 1 | **D6** | D6 (S) | Speed PWM (0–255) |
| **IN1** | Motor 1 | **D7** | D7 (S) | Direction Bit A |
| **IN2** | Motor 1 | **D8** | D8 (S) | Direction Bit B |
| **IN3** | Motor 2 | **D12** | D12 (S) | Direction Bit A |
| **IN4** | Motor 2 | **D13** | D13 (S) | Direction Bit B |
| **ENB** | Motor 2 | **D9** | D9 (S) | Speed PWM (0–255) |
| **OUT1 / OUT2** | Motor 1 | — | — | Front Left N20 Terminals |
| **OUT3 / OUT4** | Motor 2 | — | — | Front Right N20 Terminals |

### Driver B (Motors 3 & 4: Rear Left & Rear Right)
| L298N-B Terminal | Target Motor | Arduino Nano Pin | Shield Header | Function / Signal |
| :--- | :--- | :--- | :--- | :--- |
| **ENA** | Motor 3 | **D10** | D10 (S) | Speed PWM (0–255) |
| **IN1** | Motor 3 | **A0** | A0 (S) | Direction Bit A |
| **IN2** | Motor 3 | **A1** | A1 (S) | Direction Bit B |
| **IN3** | Motor 4 | **A2** | A2 (S) | Direction Bit A |
| **IN4** | Motor 4 | **A3** | A3 (S) | Direction Bit B |
| **ENB** | Motor 4 | **D11** | D11 (S) | Speed PWM (0–255) |
| **OUT1 / OUT2** | Motor 3 | — | — | Rear Left N20 Terminals |
| **OUT3 / OUT4** | Motor 4 | — | — | Rear Right N20 Terminals |

*Note on Driver B Channels:*
* `OUT1 / OUT2` driven by `IN1/IN2 (A0/A1)` connects to **Motor 3**.
* `OUT3 / OUT4` driven by `IN3/IN4 (A2/A3)` connects to **Motor 4**.

---

## 4. Encoder Connections (C1 Signals)

All 4 encoder 3-wire interfaces plug directly into the Sensor Shield 3-pin headers `(V, G, S)`:

| Physical Motor | VCC Pin | GND Pin | Signal Pin (C1) | Shield Header | Interrupt Architecture |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Motor 1 (Front Left)** | VCC | GND | C1 | **D2** (V, G, S) | External Interrupt 0 (`INT0`) |
| **Motor 2 (Front Right)**| VCC | GND | C1 | **D3** (V, G, S) | External Interrupt 1 (`INT1`) |
| **Motor 3 (Rear Left)** | VCC | GND | C1 | **D5** (V, G, S) | Pin Change Interrupt 2 (`PCINT21`) |
| **Motor 4 (Rear Right)**| VCC | GND | C1 | **D4** (V, G, S) | Pin Change Interrupt 2 (`PCINT20`) |

> **Interrupt Tip:** Both D4 and D5 reside on `PORT D` (`PIND4` and `PIND5`). They share the vector `ISR(PCINT2_vect)`. Within the ISR, the software performs instantaneous bit masking against the previous state (`lastPIND`) to count pulses independently.

---

## 5. Shared I2C Bus Connections

The PCA9685, VL53L0X, and BNO08x connect in parallel to the Arduino Nano hardware I2C lines (`A4` and `A5`):

| Device | SDA Pin | SCL Pin | VCC Pin | GND Pin | I2C Address |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **PCA9685 Board** | A4 Header | A5 Header | 5V Header | GND Header | `0x40` |
| **VL53L0X ToF Sensor** | A4 Header | A5 Header | 5V Header | GND Header | `0x29` |
| **BNO08x 9-DOF IMU** | A4 Header | A5 Header | 5V Header | GND Header | `0x4A` (or `0x4B`) |

---

## 6. Power Distribution & Safety Checklist

1. **High-Current Battery Rail:** Connect Li-ion battery positive lead (+) directly to the **+12V screw terminals** on both Driver A and Driver B.
2. **Logic Power Rail (5V):** Use the regulated **5V output terminal** of Driver A (with its onboard 5V regulator jumper closed) to feed the Sensor Shield **5V bus**.
3. **Common Ground:** Tie Li-ion battery (-), Driver A GND, Driver B GND, Sensor Shield GND, and PCA9685 GND into one unified ground plane.
4. **USB Flashing Safety:** D0 (RX) and D1 (TX) are 100% free of motor or sensor connections. USB sketch flashing will never conflict with peripheral lines.
5. **No Analog Output Bugs:** Standard ATmega328P pins A6 and A7 are analog-input only. Motor directions exclusively use true digital I/O lines (D7, D8, D12, D13, A0–A3).
