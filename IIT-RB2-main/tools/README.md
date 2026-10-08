# 🛠 Diagnostic & Testing Suite: 4WD N20 Rover (IIT-RB2)

> 📖 **Part of the [IIT-RB2 Documentation Suite](../docs/README.md)**  
> **Related Guides:** [Diagnostics & Calibration Guide](../docs/DIAGNOSTICS_AND_CALIBRATION.md) | [Hardware Wiring Guide](../docs/WIRING_GUIDE.md) | [Troubleshooting Guide](../docs/TROUBLESHOOTING.md)

This directory contains standalone, zero-external-dependency diagnostic firmware sketches for the **Arduino Nano (ATmega328P)**. Each tool is designed to isolate, verify, and calibrate specific hardware subsystems on the bench before deploying full autonomous controller code.

---

## 📊 Diagnostic Tools Matrix

| Tool Directory | Sketch File | Baud Rate | Target Setup | Primary Verification |
| :--- | :--- | :---: | :--- | :--- |
| **[`01_I2C_Scanner/`](01_I2C_Scanner/)** | [`01_I2C_Scanner.ino`](01_I2C_Scanner/01_I2C_Scanner.ino) | **9600** | Shared I2C Bus (`A4/A5`) | Automated detection of VL53L0X (`0x29`), PCA9685 (`0x40`), and BNO08x (`0x4A`/`0x4B`). |
| **[`02_PCA9685_Motor_Test/`](02_PCA9685_Motor_Test/)** | [`02_PCA9685_Motor_Test.ino`](02_PCA9685_Motor_Test/02_PCA9685_Motor_Test.ino) | **9600** | PCA9685 + Dual L298N | Bench verification for auxiliary 16-channel PWM breakout setups. |
| **[`03_Direct_Hardware_Diagnostic_Test/`](03_Direct_Hardware_Diagnostic_Test/)** | [`03_Direct_Hardware_Diagnostic_Test.ino`](03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino) | **115200** | Direct Nano Pinout | ⭐ **RECOMMENDED:** Full interactive test suite for motors, encoders, I2C, and combined 4WD motion. |

---

## 🚀 Uploading in Arduino IDE

For all diagnostic sketches in this directory:
1. Open the `.ino` file in **Arduino IDE** or **VS Code**.
2. Go to **Tools** -> **Board** -> **Arduino AVR Boards** -> **Arduino Nano**.
3. Go to **Tools** -> **Processor** -> **ATmega328P (Old Bootloader)**.
4. Select your **Port** (e.g., `COM3`, `COM4`).
5. Click **Upload** (`Ctrl+U`).
6. Open **Serial Monitor** at the tool's matching baud rate (**115200** for Tool 03, **9600** for Tools 01 & 02).

---

## 🔍 Detailed Tool Overviews

### 1. [`01_I2C_Scanner/01_I2C_Scanner.ino`](01_I2C_Scanner/01_I2C_Scanner.ino)
* **Objective:** Confirms electrical connectivity and bus communication with all I2C peripherals.
* **Bus Pins:** `A4` (SDA) and `A5` (SCL).
* **Expected Addresses:**
  * `0x29`: VL53L0X Time-of-Flight Laser Distance Sensor
  * `0x40`: PCA9685 16-Channel PWM Board (if installed)
  * `0x4A`: BNO08x 9-DOF AHRS IMU (with DI0/ADDR grounded; responds at `0x4B` if high)
* **Serial Monitor Settings:** 9600 Baud.

### 2. [`02_PCA9685_Motor_Test/02_PCA9685_Motor_Test.ino`](02_PCA9685_Motor_Test/02_PCA9685_Motor_Test.ino)
* **Objective:** Verifies motor driving via PCA9685 PWM channels for systems utilizing the external PWM board.
* **Serial Monitor Settings:** 9600 Baud.

### 3. [`03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino`](03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino) ⭐
* **Objective:** Interactive bench diagnostic program for the **audited direct-pinout architecture**.
* **Serial Monitor Settings:** 115200 Baud (Both NL & CR).
* **Menu Options:**
  * `[1]` Test Motor 1 (Front Left: D6 PWM, D7/D8 Dir) + count D2 INT0 encoder ticks
  * `[2]` Test Motor 2 (Front Right: D9 PWM, D12/D13 Dir) + count D3 INT1 encoder ticks
  * `[3]` Test Motor 3 (Rear Left: D10 PWM, A0/A1 Dir) + count D5 PCINT21 encoder ticks
  * `[4]` Test Motor 4 (Rear Right: D11 PWM, A2/A3 Dir) + count D4 PCINT20 encoder ticks
  * `[5]` Test 4WD Combined Motion (Forward, Backward, Pivot Left, Pivot Right)
  * `[6]` Live Encoder Ticks Monitor (Manual Wheel Spin Test)
  * `[7]` Real-time I2C Bus Scan
  * `[8]` Automated Full System Self-Test Sequence
  * `[S / SPACE]` Emergency Instant Stop

> 💡 For comprehensive calibration math and motor mechanical mirroring instructions, see **[docs/DIAGNOSTICS_AND_CALIBRATION.md](../docs/DIAGNOSTICS_AND_CALIBRATION.md)**.

---

## 📋 Direct Pinout Summary Reference

| Subsystem / Function | Arduino Nano Pin | Shield Header | Microcontroller Mode |
| :--- | :--- | :--- | :--- |
| **M1 (FL) Encoder C1** | `D2` | D2 (S) | External Interrupt 0 (`INT0`) |
| **M2 (FR) Encoder C1** | `D3` | D3 (S) | External Interrupt 1 (`INT1`) |
| **M4 (RR) Encoder C1** | `D4` | D4 (S) | Pin Change Interrupt 2 (`PCINT20`) |
| **M3 (RL) Encoder C1** | `D5` | D5 (S) | Pin Change Interrupt 2 (`PCINT21`) |
| **Driver A: M1 Speed (PWM)** | `D6` | D6 (S) | Timer 0 Hardware PWM |
| **Driver A: M1 Direction** | `D7`, `D8` | D7 (S), D8 (S) | Digital GPIO |
| **Driver A: M2 Speed (PWM)** | `D9` | D9 (S) | Timer 1 Hardware PWM |
| **Driver A: M2 Direction** | `D12`, `D13` | D12 (S), D13 (S) | Digital GPIO (D13 shared with LED) |
| **Driver B: M3 Speed (PWM)** | `D10` | D10 (S) | Timer 1 Hardware PWM |
| **Driver B: M3 Direction** | `A0`, `A1` | A0 (S), A1 (S) | Digital GPIO (`D14`, `D15`) |
| **Driver B: M4 Speed (PWM)** | `D11` | D11 (S) | Timer 2 Hardware PWM |
| **Driver B: M4 Direction** | `A2`, `A3` | A2 (S), A3 (S) | Digital GPIO (`D16`, `D17`) |
| **Shared I2C Bus** | `A4` (SDA), `A5` (SCL) | A4/A5 Headers | Hardware TWI (400 kHz) |
| **USB Telemetry / Flashing** | `D0` (RX), `D1` (TX) | D0/D1 Headers | 100% Free Hardware UART |
