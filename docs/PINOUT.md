# Meowler Pinout (Updated 4-Motor Dual L298N & PCA9685 Build)

## Overview & Pin Summary (Arduino Nano / ATmega328P)

| Pin | Function / Target | Description |
|:---|:---|:---|
| **D0 (RX)** | USB Serial RX | FTDI / USB Telemetry |
| **D1 (TX)** | USB Serial TX | FTDI / USB Telemetry |
| **D2** | **M3 C1** | Motor 3 Encoder Channel 1 (Hardware INT0) |
| **D3** | **M4 C1** | Motor 4 Encoder Channel 1 (Hardware INT1 / PWM) |
| **D4** | **M2 C1** | Motor 2 Encoder Channel 1 |
| **D5** | **M1 C1** | Motor 1 Encoder Channel 1 |
| **D6** | **L298N-P2 ENA** | Motor Driver 2 Enable A (**PWM** - Timer0) |
| **D7** | **L298N-P2 IN1** | Motor Driver 2 Direction IN1 |
| **D8** | **L298N-P2 IN2** | Motor Driver 2 Direction IN2 |
| **D9** | **L298N-P2 ENB** | Motor Driver 2 Enable B (**PWM** - Timer1) |
| **D10** | **L298N-P1 ENA** | Motor Driver 1 Enable A (**PWM** - Timer1) |
| **D11** | **L298N-P1 ENB** | Motor Driver 1 Enable B (**PWM** - Timer2) |
| **D12** | **L298N-P2 IN3** | Motor Driver 2 Direction IN3 |
| **D13** | **L298N-P2 IN4** | Motor Driver 2 Direction IN4 (Shared with onboard LED) |
| **A0** | **L298N-P1 IN1** | Motor Driver 1 Direction IN1 |
| **A1** | **L298N-P1 IN2** | Motor Driver 1 Direction IN2 |
| **A2** | **L298N-P1 IN3** | Motor Driver 1 Direction IN3 |
| **A3** | **L298N-P1 IN4** | Motor Driver 1 Direction IN4 |
| **A4** | **PCA9685 SDA** | Hardware I2C Data |
| **A5** | **PCA9685 SCL** | Hardware I2C Clock |

---

## 1. L298N - P1 (Motor Driver 1: M1 & M2)

> **Important**: Remove ENA and ENB jumpers on the L298N board to enable PWM speed control from the Nano.

| L298N Pin | Arduino Nano Pin | Note |
|:---|:---|:---|
| **ENA** | **D10** | PWM Speed Control (Channel A / M1) |
| **IN1** | **A0** | Direction Control 1 |
| **IN2** | **A1** | Direction Control 2 |
| **IN3** | **A2** | Direction Control 3 |
| **IN4** | **A3** | Direction Control 4 |
| **ENB** | **D11** | PWM Speed Control (Channel B / M2) |

---

## 2. L298N - P2 (Motor Driver 2: M3 & M4)

> **Important**: Remove ENA and ENB jumpers on the L298N board to enable PWM speed control from the Nano.

| L298N Pin | Arduino Nano Pin | Note |
|:---|:---|:---|
| **ENA** | **D6** | PWM Speed Control (Channel A / M3) |
| **IN1** | **D7** | Direction Control 1 |
| **IN2** | **D8** | Direction Control 2 |
| **IN3** | **D12** | Direction Control 3 |
| **IN4** | **D13** | Direction Control 4 (*mirrors onboard LED*) |
| **ENB** | **D9** | PWM Speed Control (Channel B / M4) |

---

## 3. PCA9685 - P1 (PWM / Servo Driver @ `0x40`)

| PCA9685 Pin | Connection / Nano Pin | Note |
|:---|:---|:---|
| **GND** | **GND** | Shared logic ground |
| **VCC** | **5V / VCC** | Shared 5V logic power |
| **V+** | **ExtBTT** | External battery / high-current servo supply |
| **SDA** | **A4** (or A5 depending on silk) | Hardware I2C Data line |
| **SCL** | **A5** (or A4 depending on silk) | Hardware I2C Clock line |
| **OE** | **GND** | Tied to GND to enable outputs |

---

## 4. Motor Feedback / Encoders (C1)

| Motor Channel | Nano Pin | Interrupt Capability |
|:---|:---|:---|
| **M1 C1** | **D5** | PCINT (Pin Change Interrupt) |
| **M2 C1** | **D4** | PCINT (Pin Change Interrupt) |
| **M3 C1** | **D2** | **INT0** (Hardware External Interrupt) |
| **M4 C1** | **D3** | **INT1** (Hardware External Interrupt) |

---

## 5. Power & Wiring Best Practices

- **Shared Ground**: Ensure GND from Arduino Nano, L298N-P1, L298N-P2, and PCA9685 are all bonded together.
- **Motor / Servo Power (V+ / 12V)**:
  - Feed motor battery directly into L298N +12V power terminals.
  - Feed servo battery directly into PCA9685 `V+` terminal.
  - Do not power motors or servos from the Nano 5V pin.

