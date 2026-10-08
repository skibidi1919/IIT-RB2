#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <util/atomic.h>
#include <VL53L0X.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// Fallback pin and type definitions for IDE static analysis (clangd)
#ifndef A0
  #define A0 14
  #define A1 15
  #define A2 16
  #define A3 17
#endif

// ================= HARDWARE PIN DEFINITIONS =================
// --- Encoders (C1 Pulse Signal Lines) ---
const uint8_t ENC1_PIN = 2; // Motor 1 (Front Left)  - External Interrupt 0 (INT0)
const uint8_t ENC2_PIN = 3; // Motor 2 (Front Right) - External Interrupt 1 (INT1)
const uint8_t ENC4_PIN = 4; // Motor 4 (Rear Right)  - Pin Change Interrupt 2 (PCINT20)
const uint8_t ENC3_PIN = 5; // Motor 3 (Rear Left)   - Pin Change Interrupt 2 (PCINT21)

// --- L298N Motor Driver A (Motors 1 & 2) ---
const uint8_t M1_ENA   = 6;  // Motor 1 Speed (Hardware PWM)
const uint8_t M1_IN1   = 7;  // Motor 1 Direction A
const uint8_t M1_IN2   = 8;  // Motor 1 Direction B
const uint8_t M2_ENB   = 9;  // Motor 2 Speed (Hardware PWM)
const uint8_t M2_IN3   = 12; // Motor 2 Direction A
const uint8_t M2_IN4   = 13; // Motor 2 Direction B

// --- L298N Motor Driver B (Motors 3 & 4) ---
const uint8_t M3_ENA   = 10; // Motor 3 Speed (Hardware PWM)
const uint8_t M3_IN1   = A0; // Motor 3 Direction A
const uint8_t M3_IN2   = A1; // Motor 3 Direction B
const uint8_t M4_ENB   = 11; // Motor 4 Speed (Hardware PWM)
const uint8_t M4_IN3   = A2; // Motor 4 Direction A
const uint8_t M4_IN4   = A3; // Motor 4 Direction B

// --- I2C Bus & Sensor Addresses ---
#define PCA9685_ADDR   0x40  // Auxiliary PWM expansion (if attached to A4/A5)
#define BNO08X_ADDR    0x4A  // 9-DOF IMU address
#define VL53L0X_ADDR   0x29  // Laser Distance Sensor address

// ================= ROBOT TUNING CONSTANTS =================
const int MAX_SPEED    = 220; // 8-bit PWM clamp (0-255)
const int DEFAULT_SPD  = 160;
const int TURN_SPEED   = 160;
const int REVERSE_SPD  = 150;
const uint16_t STOP_DIST_MM = 180;
const unsigned long WATCHDOG_MS = 1500; // Auto-stop if no command received

VL53L0X lox;
BNO080  bno08x;

bool hasVL53L0X = false;
bool hasBNO08x  = false;

volatile long encTicks1 = 0; // Front Left
volatile long encTicks2 = 0; // Front Right
volatile long encTicks3 = 0; // Rear Left
volatile long encTicks4 = 0; // Rear Right

volatile uint8_t lastPIND = 0;

float currentHeadingYaw    = 0.0;
float currentPitch         = 0.0;
float currentRoll          = 0.0;
uint16_t currentDistanceMm = 8190;

int lastM1 = -9999;
int lastM2 = -9999;
int lastM3 = -9999;
int lastM4 = -9999;

// Operating Modes
enum OperatingMode {
  OP_MANUAL, // Responsive to dashboard WASD/Joystick/Commands
  OP_AUTO,   // Autonomous obstacle avoidance state machine
  OP_SPIN,   // 360-degree skid spin calibration
  OP_ESTOP   // Emergency stop locked
};

OperatingMode opMode = OP_MANUAL;
int activeThrottle = DEFAULT_SPD;
unsigned long lastCommandTime = 0;

enum NavState {
  STATE_FORWARD,
  STATE_STOP,
  STATE_BACKWARD,
  STATE_TURN
};

NavState navState = STATE_FORWARD;
unsigned long navTimer = 0;

// Serial Command Buffer
char serialBuffer[64];
uint8_t bufferIdx = 0;

// ================= MOTOR DRIVE ROUTINES =================

void setMotorRaw(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
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

void setWheels(int m1, int m2, int m3, int m4) {
  if (m1 == lastM1 && m2 == lastM2 && m3 == lastM3 && m4 == lastM4) {
    return;
  }
  lastM1 = m1;
  lastM2 = m2;
  lastM3 = m3;
  lastM4 = m4;

  setMotorRaw(M1_ENA, M1_IN1, M1_IN2, m1);
  setMotorRaw(M2_ENB, M2_IN3, M2_IN4, m2);
  setMotorRaw(M3_ENA, M3_IN1, M3_IN2, m3);
  setMotorRaw(M4_ENB, M4_IN3, M4_IN4, m4);
}

void stopRobot() {
  setWheels(0, 0, 0, 0);
}

void moveForward(int speed)  { setWheels( speed,  speed,  speed,  speed); }
void moveBackward(int speed) { setWheels(-speed, -speed, -speed, -speed); }
void turnLeft(int speed)     { setWheels(-speed,  speed, -speed,  speed); }
void turnRight(int speed)    { setWheels( speed, -speed,  speed, -speed); }

// ================= INTERRUPT SERVICE ROUTINES =================

// Motor 1 Encoder on D2 (INT0)
void isrEncoder1() {
  encTicks1++;
}

// Motor 2 Encoder on D3 (INT1)
void isrEncoder2() {
  encTicks2++;
}

// Motors 4 & 3 Encoders on D4 & D5 (Pin Change Interrupt 2 on PORT D)
ISR(PCINT2_vect) {
  uint8_t curr = PIND;

  // D4 is Motor 4 (Rear Right)
  if ((curr & (1 << PIND4)) && !(lastPIND & (1 << PIND4))) {
    encTicks4++;
  }

  // D5 is Motor 3 (Rear Left)
  if ((curr & (1 << PIND5)) && !(lastPIND & (1 << PIND5))) {
    encTicks3++;
  }

  lastPIND = curr;
}

// ================= AUTONOMOUS NAVIGATION =================

void handleNavigation() {
  unsigned long now = millis();
  switch (navState) {
    case STATE_FORWARD:
      if (hasVL53L0X && currentDistanceMm < STOP_DIST_MM) {
        stopRobot();
        navState = STATE_STOP;
        navTimer = now;
      } else {
        moveForward(activeThrottle);
      }
      break;

    case STATE_STOP:
      if (now - navTimer >= 200) {
        moveBackward(REVERSE_SPD);
        navState = STATE_BACKWARD;
        navTimer = now;
      }
      break;

    case STATE_BACKWARD:
      if (now - navTimer >= 400) {
        turnRight(TURN_SPEED);
        navState = STATE_TURN;
        navTimer = now;
      }
      break;

    case STATE_TURN:
      if (now - navTimer >= 500) {
        moveForward(activeThrottle);
        navState = STATE_FORWARD;
      }
      break;
  }
}

// ================= SERIAL COMMAND DISPATCHER =================

void processCommand(const char* cmd) {
  if (strlen(cmd) == 0) return;

  // 1. Skid-Steer Telemetry Drive: "M:left,right"
  if (strncmp(cmd, "M:", 2) == 0) {
    if (opMode == OP_ESTOP) {
      Serial.println(F("[WARN] Rover is in E-STOP! Send RESUME to re-arm."));
      return;
    }
    const char* comma = strchr(cmd + 2, ',');
    if (comma != NULL) {
      int leftPwm = atoi(cmd + 2);
      int rightPwm = atoi(comma + 1);
      setWheels(leftPwm, rightPwm, leftPwm, rightPwm);
      lastCommandTime = millis();
      // Optional subtle ACK
      if (leftPwm == 0 && rightPwm == 0) {
        Serial.println(F("[OK] Wheels Stopped"));
      }
    }
    return;
  }

  // 2. Throttle Update: "THROTTLE:val"
  if (strncmp(cmd, "THROTTLE:", 9) == 0) {
    int t = atoi(cmd + 9);
    if (t >= 60 && t <= 255) {
      activeThrottle = t;
      Serial.print(F("[OK] Throttle set to "));
      Serial.println(activeThrottle);
    }
    return;
  }

  // 3. Mode Selection: "MODE:..."
  if (strncmp(cmd, "MODE:", 5) == 0) {
    const char* m = cmd + 5;
    if (strcmp(m, "MANUAL") == 0) {
      opMode = OP_MANUAL;
      stopRobot();
      Serial.println(F("[OK] Mode: MANUAL TELEOP"));
    } else if (strcmp(m, "AUTO") == 0) {
      opMode = OP_AUTO;
      navState = STATE_FORWARD;
      Serial.println(F("[OK] Mode: AUTONOMOUS AI"));
    } else if (strcmp(m, "SPIN") == 0) {
      opMode = OP_SPIN;
      Serial.println(F("[OK] Mode: 360 BENCHMARK SPIN"));
    }
    return;
  }

  // 4. Emergency Stop / Stop
  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "s") == 0 || strcmp(cmd, "S") == 0 || strcmp(cmd, " ") == 0) {
    stopRobot();
    opMode = OP_ESTOP;
    Serial.println(F("[ESTOP] Emergency stop executed. Motors disabled."));
    return;
  }

  // 5. Resume from E-Stop
  if (strcmp(cmd, "RESUME") == 0) {
    opMode = OP_MANUAL;
    stopRobot();
    Serial.println(F("[OK] Safety interlock cleared. Manual teleop armed."));
    return;
  }

  // 6. Directional Commands
  if (strcmp(cmd, "F") == 0 || strcmp(cmd, "FORWARD") == 0) {
    if (opMode == OP_ESTOP) {
      Serial.println(F("[WARN] Rover is in E-STOP! Send RESUME first."));
    } else {
      moveForward(activeThrottle);
      lastCommandTime = millis();
      Serial.print(F("[OK] Driving Forward: "));
      Serial.println(activeThrottle);
    }
    return;
  }
  if (strcmp(cmd, "B") == 0 || strcmp(cmd, "BACKWARD") == 0) {
    if (opMode == OP_ESTOP) {
      Serial.println(F("[WARN] Rover is in E-STOP! Send RESUME first."));
    } else {
      moveBackward(activeThrottle);
      lastCommandTime = millis();
      Serial.print(F("[OK] Driving Backward: "));
      Serial.println(activeThrottle);
    }
    return;
  }
  if (strcmp(cmd, "L") == 0 || strcmp(cmd, "LEFT") == 0) {
    if (opMode == OP_ESTOP) {
      Serial.println(F("[WARN] Rover is in E-STOP! Send RESUME first."));
    } else {
      turnLeft(activeThrottle);
      lastCommandTime = millis();
      Serial.print(F("[OK] Pivot Left: "));
      Serial.println(activeThrottle);
    }
    return;
  }
  if (strcmp(cmd, "R") == 0 || strcmp(cmd, "RIGHT") == 0) {
    if (opMode == OP_ESTOP) {
      Serial.println(F("[WARN] Rover is in E-STOP! Send RESUME first."));
    } else {
      turnRight(activeThrottle);
      lastCommandTime = millis();
      Serial.print(F("[OK] Pivot Right: "));
      Serial.println(activeThrottle);
    }
    return;
  }

  // 7. Ping / Health
  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }

  // 8. Test sequences
  if (strcmp(cmd, "TEST:COMBINED") == 0 || strcmp(cmd, "5") == 0) {
    Serial.println(F("[TEST] Running 4WD Combined Drive Cycle..."));
    moveForward(160); delay(1000);
    moveBackward(160); delay(1000);
    turnLeft(160); delay(800);
    turnRight(160); delay(800);
    stopRobot();
    Serial.println(F("[TEST] 4WD Combined Drive Cycle Complete."));
    return;
  }

  if (strcmp(cmd, "TEST:1") == 0 || strcmp(cmd, "1") == 0) {
    Serial.println(F("[TEST] M1 Front Left (D6/D7/D8) Pulse..."));
    setMotorRaw(M1_ENA, M1_IN1, M1_IN2, 180); delay(600);
    setMotorRaw(M1_ENA, M1_IN1, M1_IN2, -180); delay(600);
    setMotorRaw(M1_ENA, M1_IN1, M1_IN2, 0);
    Serial.println(F("[TEST] M1 Complete."));
    return;
  }
  if (strcmp(cmd, "TEST:2") == 0 || strcmp(cmd, "2") == 0) {
    Serial.println(F("[TEST] M2 Front Right (D9/D12/D13) Pulse..."));
    setMotorRaw(M2_ENB, M2_IN3, M2_IN4, 180); delay(600);
    setMotorRaw(M2_ENB, M2_IN3, M2_IN4, -180); delay(600);
    setMotorRaw(M2_ENB, M2_IN3, M2_IN4, 0);
    Serial.println(F("[TEST] M2 Complete."));
    return;
  }
  if (strcmp(cmd, "TEST:3") == 0 || strcmp(cmd, "3") == 0) {
    Serial.println(F("[TEST] M3 Rear Left (D10/A0/A1) Pulse..."));
    setMotorRaw(M3_ENA, M3_IN1, M3_IN2, 180); delay(600);
    setMotorRaw(M3_ENA, M3_IN1, M3_IN2, -180); delay(600);
    setMotorRaw(M3_ENA, M3_IN1, M3_IN2, 0);
    Serial.println(F("[TEST] M3 Complete."));
    return;
  }
  if (strcmp(cmd, "TEST:4") == 0 || strcmp(cmd, "4") == 0) {
    Serial.println(F("[TEST] M4 Rear Right (D11/A2/A3) Pulse..."));
    setMotorRaw(M4_ENB, M4_IN3, M4_IN4, 180); delay(600);
    setMotorRaw(M4_ENB, M4_IN3, M4_IN4, -180); delay(600);
    setMotorRaw(M4_ENB, M4_IN3, M4_IN4, 0);
    Serial.println(F("[TEST] M4 Complete."));
    return;
  }

  Serial.print(F("[WARN] Unrecognized command: "));
  Serial.println(cmd);
}

void readSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (bufferIdx > 0) {
        serialBuffer[bufferIdx] = '\0';
        processCommand(serialBuffer);
        bufferIdx = 0;
      }
    } else {
      if (bufferIdx < sizeof(serialBuffer) - 1) {
        serialBuffer[bufferIdx++] = c;
      }
    }
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  delay(500);

  Serial.println(F("\n=============================================="));
  Serial.println(F("     4WD N20 ROVER: MASTER SYSTEM BOOT        "));
  Serial.println(F("=============================================="));

  // 1. Initialize Direct Motor Pins
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

  stopRobot();
  Serial.println(F("[OK] Direct Motor Controller GPIOs Ready."));

  // 2. Initialize Encoder Interrupts
  pinMode(ENC1_PIN, INPUT_PULLUP);
  pinMode(ENC2_PIN, INPUT_PULLUP);
  pinMode(ENC3_PIN, INPUT_PULLUP);
  pinMode(ENC4_PIN, INPUT_PULLUP);

  lastPIND = PIND;

  attachInterrupt(digitalPinToInterrupt(ENC1_PIN), isrEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), isrEncoder2, RISING);

  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21); // D4 & D5
  PCICR  |= (1 << PCIE2);
  Serial.println(F("[OK] 4x Encoders Interrupt Handlers Armed (D2, D3, D4, D5)."));

  // 3. Initialize I2C VL53L0X Laser ToF Sensor
  lox.setTimeout(80);
  if (lox.init()) {
    hasVL53L0X = true;
    lox.startContinuous();
    Serial.println(F("[OK] VL53L0X Time-of-Flight Sensor Detected (0x29)."));
  } else {
    Serial.println(F("[WARN] VL53L0X Not Found. Operating without ToF."));
  }

  // 4. Initialize I2C BNO08x 9-DOF IMU
  if (bno08x.begin(BNO08X_ADDR, Wire)) {
    hasBNO08x = true;
    bno08x.enableGameRotationVector(50);
    Serial.println(F("[OK] BNO08x 9-DOF AHRS IMU Initialized (0x4A)."));
  } else {
    Serial.println(F("[WARN] BNO08x IMU Not Found. Operating without Gyro."));
  }

  opMode = OP_MANUAL;
  lastCommandTime = millis();
  Serial.println(F("[READY] Manual Teleop armed. Awaiting Dashboard commands.\n"));
}

// ================= LOOP =================

void loop() {
  // 1. Process all incoming Serial commands immediately
  readSerialInput();

  // 2. Update Sensors (Non-blocking: polled every 50ms to keep main loop fast)
  static unsigned long lastSensorPoll = 0;
  if (millis() - lastSensorPoll >= 50) {
    lastSensorPoll = millis();

    if (hasVL53L0X) {
      uint16_t d = lox.readRangeContinuousMillimeters();
      if (lox.timeoutOccurred() || d == 65535) {
        currentDistanceMm = 8190;
      } else {
        currentDistanceMm = d;
      }
    }

    if (hasBNO08x) {
      if (bno08x.hasReset()) {
        bno08x.enableGameRotationVector(50);
      }
      if (bno08x.dataAvailable()) {
        currentHeadingYaw = bno08x.getYaw()   * 180.0 / PI;
        currentPitch      = bno08x.getPitch() * 180.0 / PI;
        currentRoll       = bno08x.getRoll()  * 180.0 / PI;
      }
    }
  }

  // 3. Mode-specific Motor Control
  if (opMode == OP_AUTO) {
    handleNavigation();
  } else if (opMode == OP_SPIN) {
    turnRight(activeThrottle);
  } else if (opMode == OP_MANUAL) {
    // Safety Watchdog: If rover was moving and no command arrived for WATCHDOG_MS, stop
    if ((lastM1 != 0 || lastM2 != 0 || lastM3 != 0 || lastM4 != 0) && (millis() - lastCommandTime > WATCHDOG_MS)) {
      stopRobot();
    }
  } else if (opMode == OP_ESTOP) {
    stopRobot();
  }

  // 4. Send Telemetry to Dashboard (10 Hz = every 100ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    long t1, t2, t3, t4;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      t1 = encTicks1;
      t2 = encTicks2;
      t3 = encTicks3;
      t4 = encTicks4;
    }

    Serial.print(F("Dist: ")); Serial.print(currentDistanceMm); Serial.print(F(" mm | "));
    Serial.print(F("Yaw: "));  Serial.print(currentHeadingYaw, 1); Serial.print(F(" deg | "));
    Serial.print(F("Ticks [FL:")); Serial.print(t1);
    Serial.print(F(" FR:")); Serial.print(t2);
    Serial.print(F(" RL:")); Serial.print(t3);
    Serial.print(F(" RR:")); Serial.print(t4);
    Serial.println(F("]"));
  }
}
