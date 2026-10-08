# 🔧 Field & Bench Troubleshooting Guide: 4WD N20 Rover

This document provides systematic diagnostics and solutions for electrical, mechanical, firmware, and telemetry issues on the **4WD N20 Rover (IIT-RB2)**.

---

## ⚡ Master Symptom-to-Solution Matrix

| Observed Symptom | Probable Root Cause | Recommended Fix | Guide Reference |
| :--- | :--- | :--- | :--- |
| **Rover spins in circles when forward is commanded.** | Right-side motor polarity is mechanically mirrored. | Swap the two motor wire leads on `OUT1`/`OUT2` or `OUT3`/`OUT4` of that driver. | [Section 1.1](#11-rover-spins-in-circles-or-veers-sharply) |
| **Arduino resets/reboots as soon as motors start moving.** | Battery voltage brownout or back-EMF inductive spike. | Check battery charge, verify common ground, and disconnect L298N 5V jumper when on USB. | [Section 2.1](#21-arduino-brownout-reset-when-motors-engage) |
| **I2C bus hangs or controller freezes on boot.** | Missing I2C pullup resistors or hung sensor without timeout guard. | Enable `Wire.setWireTimeout(3000, true)` and check SDA/SCL 4.7kΩ pullups. | [Section 3.1](#31-i2c-bus-freezes-or-hangs-boot-sequence) |
| **Encoder ticks stay at 0 when wheels spin.** | Loose C1 signal wire, reversed header polarity, or missing pullup. | Check Shield header `(V, G, S)`, verify pin assignments, and test with Tool `03` Option `6`. | [Section 4.1](#41-encoder-ticks-remain-at-0) |
| **`avrdude: stk500_getsync()` upload error in Arduino IDE.** | Wrong processor bootloader selected in IDE. | Select **Processor: ATmega328P (Old Bootloader)** at 57600 baud. | [Section 5.1](#51-upload-error-stk500_getsync-or-timeout) |
| **Web Serial port cannot be opened from Dashboard.** | COM port is held open by another program (Serial Monitor). | Close Arduino Serial Monitor, CURA, or PowerShell bridge before opening Web Serial. | [Section 5.2](#52-web-serial-or-bridge-access-denied) |
| **Telemetry console prints strange garbled symbols.** | Serial baud rate mismatch. | Switch Serial Monitor to **115200 baud** (Master code) or **9600 baud** (Simple code). | [Section 5.3](#53-garbled-or-unreadable-telemetry-text) |

---

## 1. Motor & Drive System Faults

### 1.1 Rover Spins in Circles or Veers Sharply
* **Cause:** Motors on opposite sides of a skid-steer chassis are mounted 180° mirrored relative to each other. If both sides are wired with identical terminal polarity, one side drives forward while the other drives backward.
* **Resolution:**
  1. Open [`tools/03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino`](../tools/03_Direct_Hardware_Diagnostic_Test/03_Direct_Hardware_Diagnostic_Test.ino) at **115200 baud**.
  2. Press `1` (Motor 1 - Front Left). Note if wheel moves in the forward vehicle direction.
  3. Press `2` (Motor 2 - Front Right). Note if wheel moves in the forward vehicle direction.
  4. Press `3` (Motor 3 - Rear Left). Note direction.
  5. Press `4` (Motor 4 - Rear Right). Note direction.
  6. For any wheel that rotated in reverse, unscrew that motor's 2-pin terminal on the L298N board, swap the two wire positions, and retighten.

### 1.2 Motor Hums or Whines at Low Throttle but Does Not Turn
* **Cause:** Small N20 DC gearmotors with high gear ratios (30:1, 50:1) have mechanical static friction ("stiction") in the gear teeth. PWM values below ~100 do not deliver sufficient torque to overcome gearbox resistance.
* **Resolution:**
  * In the web dashboard, ensure the throttle slider is set to **150 or higher**.
  * In `Robot_Master.ino`, default drive speeds are clamped between `140` and `220` PWM for smooth continuous rotation.

### 1.3 Front-Right Motor Twitches During Firmware Upload
* **Cause:** Pin `D13` is connected to Driver A `IN4` (Motor 2 direction bit) and is also physically wired to the Arduino Nano onboard `SCK/LED` circuit. During bootloader flashing or MCU reset, the bootloader pulses pin `D13` 3 times.
* **Resolution:**
  * This is normal hardware behavior and causes no damage.
  * Keep the robot elevated on a bench stand while flashing firmware so the wheels do not make contact with the table.

---

## 2. Power & Electrical Faults

### 2.1 Arduino Brownout Reset When Motors Engage
* **Cause:** When all 4 DC motors turn on simultaneously, stall inrush current can exceed 2.5 Amps. If the Arduino Nano shares an unregulated power line with the motors, the voltage sags below 4.5V, triggering the ATmega328P internal Brown-Out Detector (BOD) and rebooting the chip.
* **Resolution:**
  1. **Dual Power Rails:** Ensure the high-current battery line (11.1V/12V) feeds directly into the L298N motor driver `+12V` screw terminals.
  2. **Logic Power:** Feed the Sensor Shield `5V` rail exclusively from Driver A's onboard 5V regulator (with regulator jumper installed), OR keep the Arduino powered via USB during bench testing.
  3. **Common Ground:** Confirm that the battery negative terminal (-), Driver A GND, Driver B GND, Sensor Shield GND, and Arduino Nano GND are all firmly connected together.

### 2.2 USB 5V vs L298N 5V Regulator Conflict
* **Cause:** If the Arduino Nano is plugged into USB while the L298N 5V regulator is also connected to the shield 5V rail with the battery turned off, the PC's USB port will try to back-power the entire robot and motor drivers.
* **Resolution:**
  * **Rule:** Always turn ON the battery power switch BEFORE plugging in the USB cable, OR remove the 5V jumper on Driver A when powering strictly over USB.

---

## 3. Sensor & I2C Bus Faults

### 3.1 I2C Bus Freezes or Hangs Boot Sequence
* **Cause:** Standard Arduino `Wire.h` library by default contains infinite `while()` polling loops waiting for peripheral ACK bits. If a sensor loses power or noise corrupts a clock pulse, the MCU hangs indefinitely.
* **Resolution:**
  * `Robot_Master.ino` includes the hardware timeout safeguard:
    ```cpp
    #if defined(WIRE_HAS_TIMEOUT)
      Wire.setWireTimeout(3000, true); // 3ms timeout with auto-reset
    #endif
    ```
  * Verify that both `A4 (SDA)` and `A5 (SCL)` lines have 4.7kΩ pull-up resistors to 5V (standard on Sensor Shield V3 and sensor breakout modules).

### 3.2 BNO08x 9-DOF IMU Not Detected
* **Cause:** BNO08x sensor address pin configuration:
  * Address `0x4A`: When pin `DI0 / ADDR` is pulled LOW (GND).
  * Address `0x4B`: When pin `DI0 / ADDR` is left floating or tied HIGH (3.3V).
* **Resolution:**
  * Run `tools/01_I2C_Scanner/01_I2C_Scanner.ino` to see which address responds.
  * If the scanner reports `0x4B`, update line 41 of `Robot_Master.ino`:
    ```cpp
    #define BNO08X_ADDR 0x4B
    ```

### 3.3 VL53L0X Laser ToF Sensor Reports `8190 mm`
* **Cause:** The VL53L0X driver returns max distance (`8190 mm` or `65535`) when:
  1. The target is beyond the 2,000 mm physical laser range.
  2. The protective brown peel-off shipping film is still covering the sensor optical lenses.
  3. The optical sensor aperture is obstructed by chassis cabling.
* **Resolution:**
  * Remove any protective protective shipping tape from the tiny optical aperture on the sensor PCB.
  * Check that the sensor has a clear line of sight forward.

---

## 4. Encoder & Interrupt Faults

### 4.1 Encoder Ticks Remain at 0
* **Cause:** The encoder C1 signal is disconnected, mispinned, or not powered.
* **Resolution:**
  1. Check the 3-pin servo cable plugged into the Sensor Shield:
     * **G (Black/Brown):** Common Ground
     * **V (Red):** +5V Power Rail
     * **S (Yellow/White):** Encoder Signal C1
  2. Verify pin matching:
     * Motor 1 (FL) -> Shield **D2**
     * Motor 2 (FR) -> Shield **D3**
     * Motor 3 (RL) -> Shield **D5**
     * Motor 4 (RR) -> Shield **D4**
  3. Upload `tools/03_Direct_Hardware_Diagnostic_Test.ino`, select option `6` (Live Encoders Monitor), and turn each wheel by hand. Ticks must count up smoothly.

---

## 5. Serial Communication & Software Faults

### 5.1 Upload Error: `stk500_getsync()` or Timeout
* **Cause:** Arduino IDE is trying to flash at 115200 baud (Optiboot New Bootloader) instead of 57600 baud (Old Bootloader).
* **Resolution:**
  * In Arduino IDE: **Tools** -> **Processor** -> Select **ATmega328P (Old Bootloader)**.

### 5.2 Web Serial or Bridge Access Denied
* **Cause:** Only one software application on Windows can hold an open handle to a serial COM port at any time.
* **Resolution:**
  * Close the Arduino IDE Serial Monitor before clicking **Connect USB** on the web dashboard.
  * If using `serve_dashboard.ps1`, disconnect the web serial browser tab so the PowerShell bridge can access the port.

### 5.3 Garbled or Unreadable Telemetry Text
* **Cause:** Serial baud rate mismatch between Arduino sketch and Serial Monitor / Dashboard.
* **Resolution:**
  * Match your terminal baud rate to the sketch:
    * `Robot_Master.ino`: **115200 baud**
    * `03_Direct_Hardware_Diagnostic_Test.ino`: **115200 baud**
    * `01_I2C_Scanner.ino`: **9600 baud**
    * `02_PCA9685_Motor_Test.ino`: **9600 baud**
    * `Robot_Simple.ino`: **9600 baud**
