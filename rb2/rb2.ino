/*
 * RB2 — Youth Challenge Autonomous Robot 2
 * Platform: Arduino Nano (ATmega328P) + 2× L298N (4WD) + PCA9685 I2C Servo Driver
 *
 * Exact Verified Pinout:
 * =================================================================
 * 1. L298N - P1 (Left Motors M1 / M2)
 *    ENA = D10  (PWM Speed M1)
 *    IN1 = A0   (M1 Dir 1)
 *    IN2 = A1   (M1 Dir 2)
 *    IN3 = A2   (M2 Dir 1)
 *    IN4 = A3   (M2 Dir 2)
 *    ENB = D11  (PWM Speed M2)
 *
 * 2. L298N - P2 (Right Motors M3 / M4)
 *    ENA = D6   (PWM Speed M3)
 *    IN1 = D7   (M3 Dir 1)
 *    IN2 = D8   (M3 Dir 2)
 *    IN3 = D12  (M4 Dir 1)
 *    IN4 = D13  (M4 Dir 2)
 *    ENB = D9   (PWM Speed M4)
 *
 * 3. PCA9685 - P1 (I2C Servo Driver)
 *    GND = GND
 *    VCC = 5V
 *    V+  = ExtBTT (Battery Servo Rail)
 *    SDA = A4 (ATmega328 Hardware TWI SDA)
 *    SCL = A5 (ATmega328 Hardware TWI SCL)
 *    CH0 = Left Pulley / Arm Servo
 *    CH1 = Right Pulley / Arm Servo
 *
 * 4. Wheel Encoders (Feedback C1 Channels - Port D PCINT)
 *    M1 C1 = D5 (Left Front)
 *    M2 C1 = D4 (Left Rear)
 *    M3 C1 = D2 (Right Front)
 *    M4 C1 = D3 (Right Rear)
 * =================================================================
 */

#include <Wire.h>
#include <string.h>
#include <stdio.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600

#ifndef RB2_4WD
#define RB2_4WD 1
#endif

enum : int { M1 = 1, M2 = 2, M3 = 3, M4 = 4 };
enum : int { FORWARD = 1, BACKWARD = 2, STOP = 0 };

/* ---------------- Pin Definitions ---------------- */
// L298N - P1 (Left Side)
static const uint8_t P1_ENA = 10;
static const uint8_t P1_IN1 = A0;
static const uint8_t P1_IN2 = A1;
static const uint8_t P1_IN3 = A2;
static const uint8_t P1_IN4 = A3;
static const uint8_t P1_ENB = 11;

// L298N - P2 (Right Side)
static const uint8_t P2_ENA = 6;
static const uint8_t P2_IN1 = 7;
static const uint8_t P2_IN2 = 8;
static const uint8_t P2_IN3 = 12;
static const uint8_t P2_IN4 = 13;
static const uint8_t P2_ENB = 9;

// Encoder Feedback Pins (C1)
static const uint8_t PIN_M1_C1 = 5;
static const uint8_t PIN_M2_C1 = 4;
static const uint8_t PIN_M3_C1 = 2;
static const uint8_t PIN_M4_C1 = 3;

/* ---------------- Pulley / Arm Servos (PCA9685) ---------------- */
static const int SERVO_ARM_L = 0;   // Left pulley / arm servo channel
static const int SERVO_ARM_R = 1;   // Right pulley / arm servo channel

// Angles for Pulley Arm States (tune to your physical mechanism)
static const float ARM_HOME_POS_L = 20.0f;   // Raised / holding walls during transport
static const float ARM_HOME_POS_R = 160.0f;  // (Mirrored or matched based on servo horn mounting)
static const float ARM_DROP_POS_L = 140.0f;  // Lowered / dropping walls to ground
static const float ARM_DROP_POS_R = 40.0f;   // Lowered

/* ---------------- Rule Update & Specifications (ITU 2026-2027) ----------------
 * - Field Half Surface: 1181 mm x 1143 mm
 * - Starting Zone: 480 mm x 280 mm (marked with 20 mm black tape)
 * - Quarantine Zone: 280 mm x 280 mm (lower-left corner, bounded by 2 field walls)
 * - 2 Containment Beams (Walls): 
 *     Beam 1: 250 x 60 x 20 mm
 *     Beam 2: 280 x 60 x 20 mm
 *   *LATEST RULE UPDATE*: 
 *     1. Lego / block-based attachments & mechanisms are permitted.
 *     2. Beams can be SKEWED (need NOT be strictly 90°), maximizing placement tolerance.
 *   Scoring requirement: Placed standing upright on 20 mm edge, fully released (no robot contact at end).
 * - Match Duration: 2 minutes (120 seconds). Robot remains parked in Quarantine Zone until end.
 * ----------------------------------------------------------------------------- */

/* ---------------- Autonomous Configuration ---------------- */
// Auto-start on power-on (zero human intervention after start)
const bool AUTO_START_ON_BOOT = true;

// Countdown delay before wheels start rolling (gives time to place robot & clear hands)
const uint32_t AUTO_START_DELAY_MS = 2500; 

// Base driving speed (PWM 180-220 ensures sufficient torque across left-side elevation)
const int AUTON_DRIVE_SPEED = 195; 

// Wheel Specs: Ø43 mm -> Circumference ≈ 135.1 mm -> ~4.44 encoder counts per mm (~44.4 counts/cm)
// Travel distance from Starting Zone into alignment with the 280x280 mm Quarantine Zone boundary
// E.g., ~450 mm travel ≈ 2000 counts (fine-tune in pit depending on starting placement)
const uint32_t QUARANTINE_ZONE_COUNTS = 2000; 

// Left Elevation Compensation parameters
// Left side encounters elevation/ramp -> increased resistance -> closed loop boosts left motors
static const float ENC_BAL_KP = 0.14f;   // Proportional correction gain
static const float ENC_BAL_KI = 0.006f;  // Integral correction gain for sustained elevation drag
static const int CORR_MAX = 85;          // Max PWM correction differential

/* ---------------- Motor & Calibration State ---------------- */
static volatile uint32_t encCnt_[4];
static volatile uint8_t encPrevPortD_;
static uint32_t pathEncL0_ = 0, pathEncR0_ = 0;
static float encIntegralErr_ = 0;

static uint8_t motorScaleL = 100, motorScaleR = 100;

void setMotorScales(uint8_t l, uint8_t r) {
  motorScaleL = constrain(l, 50, 100);
  motorScaleR = constrain(r, 50, 100);
}

/* ---------------- Encoders (Port D Interrupt) ---------------- */
ISR(PCINT2_vect) {
  uint8_t now = (uint8_t)(PIND & 0x3C);
  uint8_t ch = (uint8_t)(now ^ encPrevPortD_);
  encPrevPortD_ = now;
  if (ch & (1 << 5)) encCnt_[0]++;
  if (ch & (1 << 4)) encCnt_[1]++;
  if (ch & (1 << 2)) encCnt_[2]++;
  if (ch & (1 << 3)) encCnt_[3]++;
}

static void setupEncoders_() {
  pinMode(PIN_M1_C1, INPUT_PULLUP);
  pinMode(PIN_M2_C1, INPUT_PULLUP);
  pinMode(PIN_M3_C1, INPUT_PULLUP);
  pinMode(PIN_M4_C1, INPUT_PULLUP);
  encPrevPortD_ = (uint8_t)(PIND & 0x3C);
  encCnt_[0] = encCnt_[1] = encCnt_[2] = encCnt_[3] = 0;
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  PCICR |= (1 << PCIE2);
}

void zeroEncoders() {
  noInterrupts();
  encCnt_[0] = encCnt_[1] = encCnt_[2] = encCnt_[3] = 0;
  interrupts();
}

uint32_t getEnc(int motorNumber) {
  uint8_t i = (uint8_t)(motorNumber - 1);
  if (i > 3) return 0;
  noInterrupts();
  uint32_t v = encCnt_[i];
  interrupts();
  return v;
}

static void encLR_(uint32_t *l, uint32_t *r) {
  noInterrupts();
  uint32_t a = encCnt_[0], b = encCnt_[1], c = encCnt_[2], d = encCnt_[3];
  interrupts();
#if RB2_4WD
  *l = a + c;
  *r = b + d;
#else
  *l = a;
  *r = b;
  (void)c;
  (void)d;
#endif
}

/* ---------------- Servo Helpers ---------------- */
void moveServo(int servoNum, float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulse);
}

void setArmsPosition(float angleL, float angleR) {
  moveServo(SERVO_ARM_L, angleL);
  moveServo(SERVO_ARM_R, angleR);
}

// Smoothly interpolate arms from one position to another so walls lower steadily
void sweepArms(float fromL, float toL, float fromR, float toR, int steps, int stepDelayMs) {
  for (int i = 0; i <= steps; i++) {
    float frac = (float)i / (float)steps;
    float curL = fromL + (toL - fromL) * frac;
    float curR = fromR + (toR - fromR) * frac;
    setArmsPosition(curL, curR);
    delay(stepDelayMs);
  }
}

void armsHome() {
  setArmsPosition(ARM_HOME_POS_L, ARM_HOME_POS_R);
}

/* ---------------- Motor Drive ---------------- */
static void applyChannel(int16_t spd, uint8_t inA, uint8_t inB, uint8_t en) {
  spd = constrain(spd, -255, 255);
  uint8_t mag = (uint8_t)(spd < 0 ? -spd : spd);
  if (spd > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(en, mag);
  } else if (spd < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(en, mag);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(en, 0);
  }
}

static int16_t scalePwm_(int16_t v, uint8_t pct) {
  if (v == 0 || pct >= 100) return v;
  int32_t mag = ((int32_t)abs(v) * (int32_t)pct + 50) / 100;
  if (mag > 255) mag = 255;
  return (v > 0) ? (int16_t)mag : (int16_t)(-mag);
}

void setDrive(int16_t left, int16_t right) {
  left = scalePwm_(left, motorScaleL);
  right = scalePwm_(right, motorScaleR);
  
  // Left side: M1 (P1 ENA/IN1/IN2) and M3 (P2 ENA/IN1/IN2)
  applyChannel(left, P1_IN1, P1_IN2, P1_ENA);
#if RB2_4WD
  applyChannel(left, P2_IN1, P2_IN2, P2_ENA);
#endif

  // Right side: M2 (P1 ENB/IN3/IN4) and M4 (P2 ENB/IN3/IN4)
  applyChannel(right, P1_IN3, P1_IN4, P1_ENB);
#if RB2_4WD
  applyChannel(right, P2_IN3, P2_IN4, P2_ENB);
#endif
}

void stopMotors() {
  applyChannel(0, P1_IN1, P1_IN2, P1_ENA);
  applyChannel(0, P1_IN3, P1_IN4, P1_ENB);
  applyChannel(0, P2_IN1, P2_IN2, P2_ENA);
  applyChannel(0, P2_IN3, P2_IN4, P2_ENB);
}

/* ---------------- Closed-Loop Straight Drive with Elevation Handling ---------------- */
void pathPrepare() {
  stopMotors();
  delay(40);
  zeroEncoders();
  encLR_(&pathEncL0_, &pathEncR0_);
  encIntegralErr_ = 0;
}

// Computes differential correction when one side (e.g. Left on elevation) slows down
static int encBalanceCorr_() {
  uint32_t el = 0, er = 0;
  encLR_(&el, &er);
  int32_t err = (int32_t)(el - pathEncL0_) - (int32_t)(er - pathEncR0_);
  
  // Accumulate integral for sustained drag/incline on one side
  encIntegralErr_ += ((float)err * 0.05f);
  encIntegralErr_ = constrain(encIntegralErr_, -500.0f, 500.0f);
  
  float corr = (float)err * ENC_BAL_KP + encIntegralErr_ * ENC_BAL_KI;
  return (int)constrain((int)lroundf(corr), -CORR_MAX, CORR_MAX);
}

static void driveStraightHold_(int speed, bool reverse) {
  int corr = encBalanceCorr_();
  // If Left lags (err < 0, corr < 0): Left gets speed - corr (speed + |corr|), Right gets speed - |corr|
  int l = constrain(speed - corr, 0, 255);
  int r = constrain(speed + corr, 0, 255);
  if (reverse) setDrive((int16_t)(-l), (int16_t)(-r));
  else setDrive((int16_t)l, (int16_t)r);
}

void driveDistCounts(int speed, uint32_t counts) {
  pathPrepare();
  uint32_t t0 = millis();
  while (millis() - t0 < 30000UL) {
    uint32_t el = 0, er = 0;
    encLR_(&el, &er);
    uint32_t avgDist = ((el - pathEncL0_) + (er - pathEncR0_)) / 2;
    if (avgDist >= counts) break;
    driveStraightHold_(speed, false);
    delay(8);
  }
  stopMotors();
}

/* ---------------- Youth Challenge Autonomous Routine ---------------- */
enum AutonState {
  AUTON_IDLE,
  AUTON_COUNTDOWN,
  AUTON_DRIVING_TO_QUARANTINE,
  AUTON_DROP_WALLS,
  AUTON_CLEAR_WALLS,
  AUTON_RETRACT_ARMS,
  AUTON_FINISHED_PARKED
};

static AutonState autonState_ = AUTON_IDLE;
static uint32_t stateTimer_ = 0;
static bool autonActive_ = false;

void startAutonomous() {
  Serial.println(F("=== AUTON START ==="));
  autonActive_ = true;
  autonState_ = AUTON_COUNTDOWN;
  stateTimer_ = millis();
  armsHome();
}

void runAutonomousStep() {
  if (!autonActive_) return;

  switch (autonState_) {
    case AUTON_COUNTDOWN: {
      if (millis() - stateTimer_ >= AUTO_START_DELAY_MS) {
        Serial.println(F("[AUTON] Driving through course to Quarantine Zone..."));
        pathPrepare();
        autonState_ = AUTON_DRIVING_TO_QUARANTINE;
        stateTimer_ = millis();
      }
      break;
    }

    case AUTON_DRIVING_TO_QUARANTINE: {
      uint32_t el = 0, er = 0;
      encLR_(&el, &er);
      uint32_t avgDist = ((el - pathEncL0_) + (er - pathEncR0_)) / 2;

      // Check if quarantine distance is reached or safety timeout (15s)
      if (avgDist >= QUARANTINE_ZONE_COUNTS || (millis() - stateTimer_ > 15000UL)) {
        stopMotors();
        Serial.println(F("[AUTON] In Quarantine Zone. Deploying pulley arms..."));
        autonState_ = AUTON_DROP_WALLS;
        stateTimer_ = millis();
      } else {
        // Closed loop maintains straight heading even while climbing left elevation
        driveStraightHold_(AUTON_DRIVE_SPEED, false);
        delay(8);
      }
      break;
    }

    case AUTON_DROP_WALLS: {
      stopMotors();
      delay(300);
      // Smoothly lower the pulley arms to deposit the 2 walls on the ground
      Serial.println(F("[AUTON] Lowering walls via pulley..."));
      sweepArms(ARM_HOME_POS_L, ARM_DROP_POS_L, ARM_HOME_POS_R, ARM_DROP_POS_R, 30, 30);
      delay(800); // Allow walls to settle on floor

      Serial.println(F("[AUTON] Backing up slightly to clear walls..."));
      autonState_ = AUTON_CLEAR_WALLS;
      stateTimer_ = millis();
      break;
    }

    case AUTON_CLEAR_WALLS: {
      // Back up gently for 500ms to disengage/leave the walls resting in the zone
      setDrive(-150, -150);
      delay(500);
      stopMotors();
      delay(200);

      Serial.println(F("[AUTON] Retracting pulley arms to original position..."));
      autonState_ = AUTON_RETRACT_ARMS;
      break;
    }

    case AUTON_RETRACT_ARMS: {
      // Smoothly return arms back to home/transport position
      sweepArms(ARM_DROP_POS_L, ARM_HOME_POS_L, ARM_DROP_POS_R, ARM_HOME_POS_R, 30, 20);
      delay(300);

      Serial.println(F("=== AUTON COMPLETE: PARKED IN QUARANTINE ZONE ==="));
      stopMotors();
      autonState_ = AUTON_FINISHED_PARKED;
      break;
    }

    case AUTON_FINISHED_PARKED: {
      // Keep motors stopped permanently; robot remains in quarantine zone
      stopMotors();
      break;
    }

    default:
      break;
  }
}

/* ---------------- Serial UI & Pit Testing ---------------- */
static char serLine_[48];
static uint8_t serLen_ = 0;
static int16_t uiDriveL_ = 0, uiDriveR_ = 0;
static uint32_t uiDriveMs_ = 0;
static bool uiDriveOn_ = false;
static uint32_t lastTelemMs_ = 0;

static void handleSerialLine_(char *line) {
  while (*line == ' ') line++;
  if (!line[0]) return;

  if (!strcmp(line, "ping")) { Serial.println(F("PONG")); return; }
  if (!strcmp(line, "start") || !strcmp(line, "run")) {
    startAutonomous();
    Serial.println(F("OK start"));
    return;
  }
  if (!strcmp(line, "stop")) {
    autonActive_ = false;
    autonState_ = AUTON_IDLE;
    uiDriveOn_ = false;
    stopMotors();
    Serial.println(F("OK stop"));
    return;
  }
  if (!strcmp(line, "arms_drop")) {
    sweepArms(ARM_HOME_POS_L, ARM_DROP_POS_L, ARM_HOME_POS_R, ARM_DROP_POS_R, 25, 25);
    Serial.println(F("OK arms_drop"));
    return;
  }
  if (!strcmp(line, "arms_home")) {
    sweepArms(ARM_DROP_POS_L, ARM_HOME_POS_L, ARM_DROP_POS_R, ARM_HOME_POS_R, 25, 25);
    Serial.println(F("OK arms_home"));
    return;
  }
  if (!strcmp(line, "enc")) {
    Serial.print(F("ENC "));
    Serial.print(getEnc(1)); Serial.print(' ');
    Serial.print(getEnc(2)); Serial.print(' ');
    Serial.print(getEnc(3)); Serial.print(' ');
    Serial.println(getEnc(4));
    return;
  }
  if (!strcmp(line, "telem") || !strcmp(line, "status")) {
    uint32_t el = 0, er = 0;
    encLR_(&el, &er);
    Serial.print(F("TELEM 0 0.00 0.00 "));
    Serial.print(el); Serial.print(' ');
    Serial.print(er); Serial.print(' ');
    Serial.print(motorScaleL); Serial.print(' ');
    Serial.println(motorScaleR);
    return;
  }
  if (line[0] == 'd' && (line[1] == ' ' || line[1] == '\t')) {
    int l = 0, r = 0;
    if (sscanf(line + 1, "%d %d", &l, &r) == 2) {
      uiDriveL_ = (int16_t)constrain(l, -255, 255);
      uiDriveR_ = (int16_t)constrain(r, -255, 255);
      uiDriveMs_ = millis();
      uiDriveOn_ = (uiDriveL_ != 0 || uiDriveR_ != 0);
      if (!uiDriveOn_) stopMotors();
      else setDrive(uiDriveL_, uiDriveR_);
      Serial.println(F("OK d"));
    }
    return;
  }
  if (line[0] == 's' && (line[1] == ' ' || line[1] == '\t')) {
    int ch = 0;
    float ang = 90;
    if (sscanf(line + 1, "%d %f", &ch, &ang) == 2) {
      moveServo(ch, ang);
      Serial.println(F("OK s"));
    }
    return;
  }
  if (!strncmp(line, "dist ", 5)) {
    int spd = 180;
    unsigned long counts = 1000;
    sscanf(line + 5, "%d %lu", &spd, &counts);
    uiDriveOn_ = false;
    driveDistCounts(constrain(spd, 0, 255), (uint32_t)counts);
    Serial.println(F("OK dist"));
    return;
  }
  Serial.println(F("ERR"));
}

static void pollSerial_() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serLine_[serLen_] = 0;
      if (serLen_ > 0) handleSerialLine_(serLine_);
      serLen_ = 0;
      continue;
    }
    if (serLen_ + 1 < sizeof(serLine_)) serLine_[serLen_++] = c;
  }
}

static void setupDrivePins_() {
  pinMode(P1_ENA, OUTPUT);
  pinMode(P1_ENB, OUTPUT);
  pinMode(P1_IN1, OUTPUT);
  pinMode(P1_IN2, OUTPUT);
  pinMode(P1_IN3, OUTPUT);
  pinMode(P1_IN4, OUTPUT);
  pinMode(P2_ENA, OUTPUT);
  pinMode(P2_ENB, OUTPUT);
  pinMode(P2_IN1, OUTPUT);
  pinMode(P2_IN2, OUTPUT);
  pinMode(P2_IN3, OUTPUT);
  pinMode(P2_IN4, OUTPUT);
  stopMotors();
}

/* ---------------- Setup & Loop ---------------- */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("=== RB2 Youth Challenge Autonomous Controller ==="));

  setupDrivePins_();
  setupEncoders_();

  Wire.begin();
  Wire.setClock(400000);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  
  // Set arms to transport/holding position
  armsHome();

  Serial.println(F("Hardware initialized."));

  if (AUTO_START_ON_BOOT) {
    Serial.println(F("Autonomous mode configured to auto-start."));
    startAutonomous();
  } else {
    Serial.println(F("Send 'start' over Serial to initiate autonomous run."));
  }
}

void loop() {
  pollSerial_();

  if (autonActive_) {
    runAutonomousStep();
  } else if (uiDriveOn_) {
    if (millis() - uiDriveMs_ > 400) {
      uiDriveOn_ = false;
      stopMotors();
    } else {
      setDrive(uiDriveL_, uiDriveR_);
    }
  }

  if (millis() - lastTelemMs_ > 200) {
    lastTelemMs_ = millis();
    uint32_t el = 0, er = 0;
    encLR_(&el, &er);
    Serial.print(F("TELEM 0 0.00 0.00 "));
    Serial.print(el); Serial.print(' ');
    Serial.print(er); Serial.print(' ');
    Serial.print(motorScaleL); Serial.print(' ');
    Serial.println(motorScaleR);
  }
  delay(2);
}

