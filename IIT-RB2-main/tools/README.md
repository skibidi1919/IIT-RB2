# Diagnostic & Testing Suite: 4WD N20 Rover (IIT-RB2)

This directory contains standalone, zero-external-dependency diagnostic firmware sketches for the Arduino Nano (ATmega328P). Each tool is designed to isolate and test specific hardware subsystems before running full autonomous flight code.

---

## 🛠️ Available Diagnostic Tools

### 1. [`01_I2C_Scanner/01_I2C_Scanner.ino`](file:///c:/Users/NPC29/Downloads/IIT-RB2-main/IIT-RB2-main/tools/01_I2C_Scanner/01_I2C_Scanner.ino)
- **Purpose**: Rapidly discovers all active devices on the shared I2C bus (`A4/SDA` and `A5/SCL`).
- **Expected Peripherals**:
  - `0x29`: VL53L0X Time-of-Flight Laser Distance Sensor
  - `0x40`: PCA9685 16-Channel PWM Controller (if attached)
  - `0x4A`: BNO08x 9-DOF AHRS IMU (with DI0/ADDR grounded)
- **Serial Monitor**: 9600 Baud.

---

### 2. [`02_PCA9685_Motor_Test/02_PCA9685_Motor_Test.ino`](file:///c:/Users/NPC29/Downloads/IIT-RB2-main/IIT-RB2-main/tools/02_PCA9685_Motor_Test/02_PCA9685_Motor_Test.ino)
- **Purpose**: Verification bench for setups utilizing the PCA9685 16-Channel PWM board to drive dual L298N drivers.
- **Serial Monitor**: 9600 Baud.

---

### 3. [`03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino`](file:///c:/Users/NPC29/Downloads/IIT-RB2-main/IIT-RB2-main/tools/03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino) ⭐ **RECOMMENDED**
- **Purpose**: Interactive, full-system verification suite for the **audited direct Arduino Nano pin mapping**.
- **Supported Tests**:
  - `[1]` Test Motor 1 (Front Left) Forward + Reverse + D2 Encoder Ticks
  - `[2]` Test Motor 2 (Front Right) Forward + Reverse + D3 Encoder Ticks
  - `[3]` Test Motor 3 (Rear Left) Forward + Reverse + D5 Encoder Ticks
  - `[4]` Test Motor 4 (Rear Right) Forward + Reverse + D4 Encoder Ticks
  - `[5]` Test All Motors Combined (Forward, Backward, Left Pivot, Right Pivot)
  - `[6]` Live Encoder Ticks Monitor (Manual Wheel Spin Test)
  - `[7]` I2C Bus Scan (Verify VL53L0X, BNO08x, and PCA9685)
  - `[8]` Run Full Automated Self-Test Sequence
  - `[S / SPACE]` Emergency Instant Stop
- **Serial Monitor**: 115200 Baud (New Line or Both NL & CR).

---

## 📋 Direct Pinout Summary Reference

| Function | Arduino Nano Pin | Hardware Subsystem |
| :--- | :--- | :--- |
| **M1 FL Encoder** | `D2` | External Interrupt 0 (`INT0`) |
| **M2 FR Encoder** | `D3` | External Interrupt 1 (`INT1`) |
| **M4 RR Encoder** | `D4` | Pin Change Interrupt 2 (`PCINT20`) |
| **M3 RL Encoder** | `D5` | Pin Change Interrupt 2 (`PCINT21`) |
| **Driver A (M1 PWM)** | `D6` | Timer 0 Hardware PWM |
| **Driver A (M1 IN1/IN2)** | `D7`, `D8` | GPIO Direction |
| **Driver A (M2 PWM)** | `D9` | Timer 1 Hardware PWM |
| **Driver A (M2 IN3/IN4)** | `D12`, `D13` | GPIO Direction |
| **Driver B (M3 PWM)** | `D10` | Timer 1 Hardware PWM |
| **Driver B (M3 IN1/IN2)** | `A0`, `A1` | GPIO Direction (`D14`, `D15`) |
| **Driver B (M4 PWM)** | `D11` | Timer 2 Hardware PWM |
| **Driver B (M4 IN3/IN4)** | `A2`, `A3` | GPIO Direction (`D16`, `D17`) |
| **I2C Bus** | `A4` (SDA), `A5` (SCL) | Shared Hardware TWI |
| **USB Upload / Debug** | `D0` (RX), `D1` (TX) | 100% Free UART |
