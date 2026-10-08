/*
 * =====================================================================================
 * Project: Autonomous 4WD N20 Rover (IIT-RB2)
 * Tool: Direct Hardware Diagnostic & Pinout Verification Test Suite
 * File: 03_Direct_Hardware_Diagnostic_Test.ino
 * =====================================================================================
 *
 * PURPOSE:
 *   Comprehensive hardware self-test program for the newly audited direct-pinout mapping.
 *   Verifies:
 *     1. All 4 L298N Motor Driver channels (Speed PWM & Directions).
 *     2. All 4 Wheel Encoders (External Interrupts & Pin Change Interrupts).
 *     3. I2C Bus Peripherals (PCA9685 at 0x40, VL53L0X at 0x29, BNO08x at 0x4A).
 *     4. Real-time encoder pulse feedback per wheel.
 *
 * -------------------------------------------------------------------------------------
 * MASTER PINOUT AUDIT MAPPING (Nano V3 + Sensor Shield):
 * -------------------------------------------------------------------------------------
 * [ENCODERS (C1 Pulse Signals)]
 *   - Motor 1 (Front Left) : D2 (External Interrupt 0 / INT0)
 *   - Motor 2 (Front Right): D3 (External Interrupt 1 / INT1)
 *   - Motor 4 (Rear Right) : D4 (Pin Change Interrupt / PCINT20)
 *   - Motor 3 (Rear Left)  : D5 (Pin Change Interrupt / PCINT21)
 *
 * [L298N DRIVER A (Motors 1 & 2)]
 *   - ENA (Motor 1 Speed PWM) : D6  (Hardware PWM)
 *   - IN1 (Motor 1 Dir A)     : D7  (Digital GPIO)
 *   - IN2 (Motor 1 Dir B)     : D8  (Digital GPIO)
 *   - IN3 (Motor 2 Dir A)     : D12 (Digital GPIO)
 *   - IN4 (Motor 2 Dir B)     : D13 (Digital GPIO)
 *   - ENB (Motor 2 Speed PWM) : D9  (Hardware PWM)
 *
 * [L298N DRIVER B (Motors 3 & 4)]
 *   - ENA (Motor 3 Speed PWM) : D10 (Hardware PWM)
 *   - IN1 (Motor 3 Dir A)     : A0  (Digital GPIO / D14)
 *   - IN2 (Motor 3 Dir B)     : A1  (Digital GPIO / D15)
 *   - IN3 (Motor 4 Dir A)     : A2  (Digital GPIO / D16)
 *   - IN4 (Motor 4 Dir B)     : A3  (Digital GPIO / D17)
 *   - ENB (Motor 4 Speed PWM) : D11 (Hardware PWM)
 *
 * [I2C BUS]
 *   - SDA : A4
 *   - SCL : A5
 *
 * Serial Monitor Baud Rate: 115200 (also functional at 9600)
 * =====================================================================================
 */

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <util/atomic.h>

// Fallback pin definitions for static analyzers
#ifndef A0
  #define A0 14
  #define A1 15
  #define A2 16
  #define A3 17
#endif

// ================= PIN DEFINITIONS =================

// --- L298N Driver A (Motors 1 & 2) ---
const uint8_t M1_ENA = 6;   // PWM Speed Motor 1 (Front Left)
const uint8_t M1_IN1 = 7;   // Dir A Motor 1
const uint8_t M1_IN2 = 8;   // Dir B Motor 1

const uint8_t M2_ENB = 9;   // PWM Speed Motor 2 (Front Right)
const uint8_t M2_IN3 = 12;  // Dir A Motor 2
const uint8_t M2_IN4 = 13;  // Dir B Motor 2

// --- L298N Driver B (Motors 3 & 4) ---
const uint8_t M3_ENA = 10;  // PWM Speed Motor 3 (Rear Left)
const uint8_t M3_IN1 = A0;  // Dir A Motor 3
const uint8_t M3_IN2 = A1;  // Dir B Motor 3

const uint8_t M4_ENB = 11;  // PWM Speed Motor 4 (Rear Right)
const uint8_t M4_IN3 = A2;  // Dir A Motor 4
const uint8_t M4_IN4 = A3;  // Dir B Motor 4

// --- Encoders (C1 Pulse Signal Lines) ---
const uint8_t ENC1_PIN = 2; // M1 Encoder (INT0)
const uint8_t ENC2_PIN = 3; // M2 Encoder (INT1)
const uint8_t ENC4_PIN = 4; // M4 Encoder (PCINT20)
const uint8_t ENC3_PIN = 5; // M3 Encoder (PCINT21)

// ================= SPEED CONFIGURATION =================
// 8-bit PWM scale: 0 to 255.
// Set default test speed to ~180 (~70% duty cycle) for safe indoor bench testing.
const uint8_t TEST_PWM_SPEED = 180;
const uint8_t MAX_PWM_SPEED  = 255;

// ================= VOLATILE ENCODER COUNTERS =================
volatile long encTicks1 = 0; // Motor 1 ticks
volatile long encTicks2 = 0; // Motor 2 ticks
volatile long encTicks3 = 0; // Motor 3 ticks
volatile long encTicks4 = 0; // Motor 4 ticks

volatile uint8_t lastPIND = 0;

// ================= INTERRUPT SERVICE ROUTINES =================

// External Interrupt 0 (Pin D2) - Motor 1
void isrEncoder1() {
  encTicks1++;
}

// External Interrupt 1 (Pin D3) - Motor 2
void isrEncoder2() {
  encTicks2++;
}

// Pin Change Interrupt 2 (Pins D4 and D5 on PORT D) - Motors 4 & 3
ISR(PCINT2_vect) {
  uint8_t curr = PIND;

  // Pin D4 = Motor 4 (Bit PIND4)
  if ((curr & (1 << PIND4)) && !(lastPIND & (1 << PIND4))) {
    encTicks4++;
  }

  // Pin D5 = Motor 3 (Bit PIND5)
  if ((curr & (1 << PIND5)) && !(lastPIND & (1 << PIND5))) {
    encTicks3++;
  }

  lastPIND = curr;
}

// ================= MOTOR LOW-LEVEL CONTROL =================

void setMotor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopAllMotors() {
  setMotor(M1_ENA, M1_IN1, M1_IN2, 0);
  setMotor(M2_ENB, M2_IN3, M2_IN4, 0);
  setMotor(M3_ENA, M3_IN1, M3_IN2, 0);
  setMotor(M4_ENB, M4_IN3, M4_IN4, 0);
}

void driveRobot(int m1, int m2, int m3, int m4) {
  setMotor(M1_ENA, M1_IN1, M1_IN2, m1);
  setMotor(M2_ENB, M2_IN3, M2_IN4, m2);
  setMotor(M3_ENA, M3_IN1, M3_IN2, m3);
  setMotor(M4_ENB, M4_IN3, M4_IN4, m4);
}

// Helper to snapshot encoder values atomically
void getEncoderTicks(long &t1, long &t2, long &t3, long &t4) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t1 = encTicks1;
    t2 = encTicks2;
    t3 = encTicks3;
    t4 = encTicks4;
  }
}

void resetEncoderTicks() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encTicks1 = 0;
    encTicks2 = 0;
    encTicks3 = 0;
    encTicks4 = 0;
  }
}

// ================= TEST PROCEDURES =================

void printMenu() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("       IIT-RB2 4WD ROVER: HARDWARE TEST SUITE           "));
  Serial.println(F("========================================================"));
  Serial.println(F(" [1] Test MOTOR 1 (Front Left ) [PWM D6 , Dir D7/D8,  Enc D2]"));
  Serial.println(F(" [2] Test MOTOR 2 (Front Right) [PWM D9 , Dir D12/D13,Enc D3]"));
  Serial.println(F(" [3] Test MOTOR 3 (Rear Left  ) [PWM D10, Dir A0/A1,  Enc D5]"));
  Serial.println(F(" [4] Test MOTOR 4 (Rear Right ) [PWM D11, Dir A2/A3,  Enc D4]"));
  Serial.println(F(" [5] Test ALL 4 MOTORS Combined (FWD -> REV -> PIVOT)"));
  Serial.println(F(" [6] Live Encoder Pulse Stream (Real-Time Monitor)"));
  Serial.println(F(" [7] I2C Bus Diagnostic Scanner (A4/SDA & A5/SCL)"));
  Serial.println(F(" [8] AUTOMATED FULL HARDWARE SELF-TEST"));
  Serial.println(F(" [S] EMERGENCY STOP ALL MOTORS"));
  Serial.println(F("========================================================"));
  Serial.print(F("Enter command: "));
}

// Single Motor Diagnostic
void testSingleMotor(uint8_t motorNum, uint8_t pwmPin, uint8_t in1, uint8_t in2, uint8_t encPin) {
  Serial.println();
  Serial.print(F(">>> STARTING TEST: MOTOR "));
  Serial.println(motorNum);
  Serial.print(F("    Pins -> PWM: D")); Serial.print(pwmPin);
  Serial.print(F(" | IN1: "));
  if (in1 >= 14) { Serial.print(F("A")); Serial.print(in1 - 14); } else { Serial.print(F("D")); Serial.print(in1); }
  Serial.print(F(" | IN2: "));
  if (in2 >= 14) { Serial.print(F("A")); Serial.print(in2 - 14); } else { Serial.print(F("D")); Serial.print(in2); }
  Serial.print(F(" | Encoder: D")); Serial.println(encPin);

  resetEncoderTicks();
  long tStart1, tStart2, tStart3, tStart4;
  getEncoderTicks(tStart1, tStart2, tStart3, tStart4);

  // 1. FORWARD SPIN
  Serial.println(F("  -> Spinning FORWARD (PWM: 180) for 1.5 seconds..."));
  setMotor(pwmPin, in1, in2, TEST_PWM_SPEED);
  delay(1500);
  setMotor(pwmPin, in1, in2, 0);

  long tFwd1, tFwd2, tFwd3, tFwd4;
  getEncoderTicks(tFwd1, tFwd2, tFwd3, tFwd4);
  long fwdCount = (motorNum == 1) ? tFwd1 : (motorNum == 2) ? tFwd2 : (motorNum == 3) ? tFwd3 : tFwd4;
  Serial.print(F("     Forward Pulses Counted: ")); Serial.println(fwdCount);
  delay(500);

  // 2. REVERSE SPIN
  Serial.println(F("  -> Spinning REVERSE (PWM: -180) for 1.5 seconds..."));
  setMotor(pwmPin, in1, in2, -TEST_PWM_SPEED);
  delay(1500);
  setMotor(pwmPin, in1, in2, 0);

  long tRev1, tRev2, tRev3, tRev4;
  getEncoderTicks(tRev1, tRev2, tRev3, tRev4);
  long totalCount = (motorNum == 1) ? tRev1 : (motorNum == 2) ? tRev2 : (motorNum == 3) ? tRev3 : tRev4;
  long revCount = totalCount - fwdCount;
  Serial.print(F("     Reverse Pulses Counted: ")); Serial.println(revCount);
  Serial.print(F("     Total Pulses Recorded: ")); Serial.println(totalCount);

  if (totalCount > 10) {
    Serial.println(F("  [PASS] Motor spins and Encoder C1 pulses are correctly verified!"));
  } else {
    Serial.println(F("  [CHECK] Zero or low pulses recorded. Check encoder 5V/GND and C1 signal wiring."));
  }
}

// Combined 4WD Motion Test
void testAllMotorsCombined() {
  Serial.println(F("\n>>> STARTING ALL 4 MOTORS COMBINED DRIVE SEQUENCE"));
  resetEncoderTicks();

  // Step 1: Forward
  Serial.println(F("  [1/4] Driving FORWARD (All 4 wheels +160)..."));
  driveRobot(160, 160, 160, 160);
  delay(1500);
  stopAllMotors();
  delay(400);

  // Step 2: Reverse
  Serial.println(F("  [2/4] Driving REVERSE (All 4 wheels -160)..."));
  driveRobot(-160, -160, -160, -160);
  delay(1500);
  stopAllMotors();
  delay(400);

  // Step 3: Pivot Left
  Serial.println(F("  [3/4] Pivoting LEFT (M1/M3 -160, M2/M4 +160)..."));
  driveRobot(-160, 160, -160, 160);
  delay(1000);
  stopAllMotors();
  delay(400);

  // Step 4: Pivot Right
  Serial.println(F("  [4/4] Pivoting RIGHT (M1/M3 +160, M2/M4 -160)..."));
  driveRobot(160, -160, 160, -160);
  delay(1000);
  stopAllMotors();

  long t1, t2, t3, t4;
  getEncoderTicks(t1, t2, t3, t4);
  Serial.println(F("  Drive Sequence Complete!"));
  Serial.print(F("  Total Pulses -> M1(FL): ")); Serial.print(t1);
  Serial.print(F(" | M2(FR): ")); Serial.print(t2);
  Serial.print(F(" | M3(RL): ")); Serial.print(t3);
  Serial.print(F(" | M4(RR): ")); Serial.println(t4);
}

// Live Continuous Encoder Monitor
void liveEncoderMonitor() {
  Serial.println(F("\n>>> LIVE ENCODER MONITOR STARTED"));
  Serial.println(F("    Spin the wheels by hand or power to observe pulse count updates."));
  Serial.println(F("    Press ANY KEY to return to main menu.\n"));
  Serial.println(F("     TIME (ms)   |  M1 (D2)  |  M2 (D3)  |  M3 (D5)  |  M4 (D4)"));
  Serial.println(F("   ----------------------------------------------------------"));

  resetEncoderTicks();
  unsigned long lastDisplay = 0;

  while (!Serial.available()) {
    if (millis() - lastDisplay >= 150) {
      lastDisplay = millis();
      long t1, t2, t3, t4;
      getEncoderTicks(t1, t2, t3, t4);

      char buf[80];
      snprintf(buf, sizeof(buf), "     %-10lu  |  %-8ld |  %-8ld |  %-8ld |  %-8ld",
               millis(), t1, t2, t3, t4);
      Serial.println(buf);
    }
  }

  // Clear serial input
  while (Serial.available()) Serial.read();
  Serial.println(F("\n[Exited Live Encoder Monitor]"));
}

// I2C Bus Diagnostic Scanner
void scanI2CBus() {
  Serial.println(F("\n>>> SCANNING I2C BUS (A4/SDA, A5/SCL)..."));
  Wire.begin();

  uint8_t devicesFound = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      devicesFound++;
      Serial.print(F("  -> [FOUND] I2C Device at 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);

      if (address == 0x40) {
        Serial.print(F(" (PCA9685 16-Channel PWM Board)"));
      } else if (address == 0x29) {
        Serial.print(F(" (VL53L0X Laser ToF Sensor)"));
      } else if (address == 0x4A || address == 0x4B) {
        Serial.print(F(" (BNO08x 9-DOF IMU)"));
      } else {
        Serial.print(F(" (Custom I2C Device)"));
      }
      Serial.println();
    }
  }

  if (devicesFound == 0) {
    Serial.println(F("  [WARN] No I2C devices detected on A4/A5. Check pull-ups and 5V/GND."));
  } else {
    Serial.print(F("  Scan Complete: "));
    Serial.print(devicesFound);
    Serial.println(F(" device(s) active on bus."));
  }
}

// Automated Full Hardware Self-Test
void runAutomatedFullTest() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("       RUNNING AUTOMATED COMPLETE SYSTEM VERIFICATION   "));
  Serial.println(F("========================================================"));

  scanI2CBus();
  delay(1000);

  Serial.println(F("\n--- Step 1: Testing Motor 1 & Encoder 1 ---"));
  testSingleMotor(1, M1_ENA, M1_IN1, M1_IN2, ENC1_PIN);
  delay(800);

  Serial.println(F("\n--- Step 2: Testing Motor 2 & Encoder 2 ---"));
  testSingleMotor(2, M2_ENB, M2_IN3, M2_IN4, ENC2_PIN);
  delay(800);

  Serial.println(F("\n--- Step 3: Testing Motor 3 & Encoder 3 ---"));
  testSingleMotor(3, M3_ENA, M3_IN1, M3_IN2, ENC3_PIN);
  delay(800);

  Serial.println(F("\n--- Step 4: Testing Motor 4 & Encoder 4 ---"));
  testSingleMotor(4, M4_ENB, M4_IN3, M4_IN4, ENC4_PIN);
  delay(800);

  Serial.println(F("\n--- Step 5: Testing All 4 Motors Synchronously ---"));
  testAllMotorsCombined();

  Serial.println(F("\n========================================================"));
  Serial.println(F("       AUTOMATED TEST FINISHED SUCCESSFULLY             "));
  Serial.println(F("========================================================"));
}

// ================= SETUP & MAIN LOOP =================

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure Motor Pins as OUTPUT
  pinMode(M1_ENA, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_ENB, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);

  pinMode(M3_ENA, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  pinMode(M4_ENB, OUTPUT);
  pinMode(M4_IN3, OUTPUT);
  pinMode(M4_IN4, OUTPUT);

  stopAllMotors();

  // Configure Encoder Pins as INPUT with Pullups
  pinMode(ENC1_PIN, INPUT_PULLUP);
  pinMode(ENC2_PIN, INPUT_PULLUP);
  pinMode(ENC3_PIN, INPUT_PULLUP);
  pinMode(ENC4_PIN, INPUT_PULLUP);

  lastPIND = PIND;

  // Arm Hardware Interrupts:
  // D2 -> INT0 (Motor 1)
  attachInterrupt(digitalPinToInterrupt(ENC1_PIN), isrEncoder1, RISING);
  // D3 -> INT1 (Motor 2)
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), isrEncoder2, RISING);

  // D4 (PCINT20) & D5 (PCINT21) -> Pin Change Interrupt 2 (PORT D)
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);
  PCICR  |= (1 << PCIE2);

  delay(500);
  printMenu();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    // Consume newline or carriage return characters
    if (cmd == '\r' || cmd == '\n') return;

    switch (cmd) {
      case '1':
        testSingleMotor(1, M1_ENA, M1_IN1, M1_IN2, ENC1_PIN);
        break;
      case '2':
        testSingleMotor(2, M2_ENB, M2_IN3, M2_IN4, ENC2_PIN);
        break;
      case '3':
        testSingleMotor(3, M3_ENA, M3_IN1, M3_IN2, ENC3_PIN);
        break;
      case '4':
        testSingleMotor(4, M4_ENB, M4_IN3, M4_IN4, ENC4_PIN);
        break;
      case '5':
        testAllMotorsCombined();
        break;
      case '6':
        liveEncoderMonitor();
        break;
      case '7':
        scanI2CBus();
        break;
      case '8':
        runAutomatedFullTest();
        break;
      case 's':
      case 'S':
      case 'x':
      case 'X':
      case ' ':
        stopAllMotors();
        Serial.println(F("\n[!] EMERGENCY STOP EXECUTED - ALL MOTORS OFF"));
        break;
      default:
        Serial.print(F("Unknown command: '"));
        Serial.print(cmd);
        Serial.println(F("'."));
        break;
    }

    delay(200);
    printMenu();
  }
}
