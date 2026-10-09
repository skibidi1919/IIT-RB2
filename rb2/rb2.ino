/*
 * ============================================================================
 * RB2 — PURE AUTONOMOUS COMPETITION FIRMWARE
 * Competition: Robotics for Good Youth Challenge 2026–2027 (ITU & make+learn)
 * Mission: Containment Wall Placement (Mission 1 — Step 2)
 *
 * Compliance:
 *   - 100% Autonomous (Zero human intervention / Zero manual teleoperation).
 *   - Automatically executes on boot/power-on.
 *   - Completely stops and stays parked in Quarantine Zone until match ends.
 *
 * Pinout Configuration:
 *   - L298N - P1 (Left Motors M1/M2):  ENA=D10, IN1=A0, IN2=A1, IN3=A2, IN4=A3, ENB=D11
 *   - L298N - P2 (Right Motors M3/M4): ENA=D6,  IN1=D7, IN2=D8, IN3=D12, IN4=D13, ENB=D9
 *   - Wheel Encoders (Feedback C1):    M1=D5, M2=D4, M3=D2, M4=D3 (Port D PCINT)
 *   - PCA9685 Servo Driver:            SDA=A4, SCL=A5 @ 0x40
 *   - Pulley Servos:                   CH0 (Left Pulley), CH1 (Right Pulley)
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600

#ifndef RB2_4WD
#define RB2_4WD 1
#endif

/* ---------------- Pin Definitions ---------------- */
// L298N - P1 (Left Motors M1 & M2)
static const uint8_t P1_ENA = 10;
static const uint8_t P1_IN1 = A0;
static const uint8_t P1_IN2 = A1;
static const uint8_t P1_IN3 = A2;
static const uint8_t P1_IN4 = A3;
static const uint8_t P1_ENB = 11;

// L298N - P2 (Right Motors M3 & M4)
static const uint8_t P2_ENA = 6;
static const uint8_t P2_IN1 = 7;
static const uint8_t P2_IN2 = 8;
static const uint8_t P2_IN3 = 12;
static const uint8_t P2_IN4 = 13;
static const uint8_t P2_ENB = 9;

// Encoders (Port D PCINT)
static const uint8_t PIN_M1_C1 = 5;
static const uint8_t PIN_M2_C1 = 4;
static const uint8_t PIN_M3_C1 = 2;
static const uint8_t PIN_M4_C1 = 3;

// PCA9685 Pulley Servos
static const int SERVO_ARM_L = 0;
static const int SERVO_ARM_R = 1;

/* ---------------- Mission & Tuning Parameters ---------------- */
// Angles for Pulley Arm States
static const float ARM_HOME_POS_L = 20.0f;   // Raised / holding walls during transport
static const float ARM_HOME_POS_R = 160.0f;  // Raised
static const float ARM_DROP_POS_L = 140.0f;  // Lowered / dropping walls to mat
static const float ARM_DROP_POS_R = 40.0f;   // Lowered

// Timing & Distances
static const uint32_t START_COUNTDOWN_MS  = 2500;  // 2.5s safe placement delay before moving
static const int      DRIVE_SPEED_PWM     = 195;   // Motor speed (provides torque over ramp)
static const uint32_t QUARANTINE_COUNTS   = 2000;  // Distance to Quarantine Zone boundary (~450 mm)
static const uint32_t DRIVE_TIMEOUT_MS    = 15000; // Safety cutoff timeout

// Active Closed-Loop Left-Side Elevation Balance
static const float ENC_KP = 0.14f;   // Proportional correction gain
static const float ENC_KI = 0.006f;  // Integral correction gain for sustained ramp drag
static const int   CORR_MAX = 85;    // Max differential PWM limit

/* ---------------- Internal State ---------------- */
static volatile uint32_t encCnt_[4] = {0, 0, 0, 0};
static volatile uint8_t encPrevPortD_ = 0;
static uint32_t pathEncL0_ = 0, pathEncR0_ = 0;
static float encIntegralErr_ = 0.0f;

/* ---------------- Encoders Interrupt Handler ---------------- */
ISR(PCINT2_vect) {
  uint8_t now = (uint8_t)(PIND & 0x3C);
  uint8_t ch = (uint8_t)(now ^ encPrevPortD_);
  encPrevPortD_ = now;
  if (ch & (1 << 5)) encCnt_[0]++;
  if (ch & (1 << 4)) encCnt_[1]++;
  if (ch & (1 << 2)) encCnt_[2]++;
  if (ch & (1 << 3)) encCnt_[3]++;
}

static void setupEncoders() {
  pinMode(PIN_M1_C1, INPUT_PULLUP);
  pinMode(PIN_M2_C1, INPUT_PULLUP);
  pinMode(PIN_M3_C1, INPUT_PULLUP);
  pinMode(PIN_M4_C1, INPUT_PULLUP);
  encPrevPortD_ = (uint8_t)(PIND & 0x3C);
  encCnt_[0] = encCnt_[1] = encCnt_[2] = encCnt_[3] = 0;
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);
  PCICR |= (1 << PCIE2);
}

static void zeroEncoders() {
  noInterrupts();
  encCnt_[0] = encCnt_[1] = encCnt_[2] = encCnt_[3] = 0;
  interrupts();
}

static void readEncodersLR(uint32_t *l, uint32_t *r) {
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

/* ---------------- Servo Control ---------------- */
static void moveServo(int servoNum, float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulse);
}

static void setArmsPosition(float angleL, float angleR) {
  moveServo(SERVO_ARM_L, angleL);
  moveServo(SERVO_ARM_R, angleR);
}

static void sweepArms(float fromL, float toL, float fromR, float toR, int steps, int stepDelayMs) {
  for (int i = 0; i <= steps; i++) {
    float frac = (float)i / (float)steps;
    float curL = fromL + (toL - fromL) * frac;
    float curR = fromR + (toR - fromR) * frac;
    setArmsPosition(curL, curR);
    delay(stepDelayMs);
  }
}

static void armsHome() {
  setArmsPosition(ARM_HOME_POS_L, ARM_HOME_POS_R);
}

/* ---------------- Motor Control ---------------- */
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

static void setDrive(int16_t left, int16_t right) {
  applyChannel(left, P1_IN1, P1_IN2, P1_ENA);
  applyChannel(right, P1_IN3, P1_IN4, P1_ENB);
#if RB2_4WD
  applyChannel(left, P2_IN1, P2_IN2, P2_ENA);
  applyChannel(right, P2_IN3, P2_IN4, P2_ENB);
#endif
}

static void stopMotors() {
  applyChannel(0, P1_IN1, P1_IN2, P1_ENA);
  applyChannel(0, P1_IN3, P1_IN4, P1_ENB);
  applyChannel(0, P2_IN1, P2_IN2, P2_ENA);
  applyChannel(0, P2_IN3, P2_IN4, P2_ENB);
}

/* ---------------- Closed-Loop Elevation Compensation ---------------- */
static void preparePath() {
  stopMotors();
  delay(50);
  zeroEncoders();
  readEncodersLR(&pathEncL0_, &pathEncR0_);
  encIntegralErr_ = 0.0f;
}

static void driveStraightClosedLoop(int baseSpeed) {
  uint32_t el = 0, er = 0;
  readEncodersLR(&el, &er);
  int32_t err = (int32_t)(el - pathEncL0_) - (int32_t)(er - pathEncR0_);

  // When left side experiences elevation drag, err becomes negative -> boosts left motors
  encIntegralErr_ += ((float)err * 0.05f);
  encIntegralErr_ = constrain(encIntegralErr_, -400.0f, 400.0f);

  float corrVal = (float)err * ENC_KP + encIntegralErr_ * ENC_KI;
  int corr = (int)constrain((int)lroundf(corrVal), -CORR_MAX, CORR_MAX);

  int leftSpd  = constrain(baseSpeed - corr, 0, 255);
  int rightSpd = constrain(baseSpeed + corr, 0, 255);
  setDrive((int16_t)leftSpd, (int16_t)rightSpd);
}

/* ---------------- Complete Autonomous Mission ---------------- */
void executeAutonomousMission() {
  Serial.println(F("=== RB2 AUTONOMOUS MISSION STARTED ==="));

  // --- Step 1: Pre-Match Countdown (Hands off) ---
  Serial.println(F("[STEP 1/6] Countdown & Safety Delay (2.5s)..."));
  armsHome();
  delay(START_COUNTDOWN_MS);

  // --- Step 2: Drive Across Left Elevation to Quarantine Zone ---
  Serial.println(F("[STEP 2/6] Driving across elevation to Quarantine Zone..."));
  preparePath();
  uint32_t tStart = millis();

  while (true) {
    uint32_t el = 0, er = 0;
    readEncodersLR(&el, &er);
    uint32_t distCounts = ((el - pathEncL0_) + (er - pathEncR0_)) / 2;

    if (distCounts >= QUARANTINE_COUNTS) {
      Serial.println(F("[INFO] Reached Quarantine Zone distance."));
      break;
    }
    if (millis() - tStart >= DRIVE_TIMEOUT_MS) {
      Serial.println(F("[WARN] Drive timeout reached. Stopping."));
      break;
    }

    driveStraightClosedLoop(DRIVE_SPEED_PWM);
    delay(8);
  }

  stopMotors();
  delay(300);

  // --- Step 3: Deploy Pulley / Lower Walls Upright ---
  Serial.println(F("[STEP 3/6] Deploying pulley arms: Lowering 2 walls..."));
  sweepArms(ARM_HOME_POS_L, ARM_DROP_POS_L, ARM_HOME_POS_R, ARM_DROP_POS_R, 30, 30);
  delay(800); // Allow walls to settle stably on the mat

  // --- Step 4: Clear and Detach from Walls ---
  Serial.println(F("[STEP 4/6] Backing up 500ms to clear walls completely..."));
  setDrive(-150, -150);
  delay(500);
  stopMotors();
  delay(200);

  // --- Step 5: Retract Pulley Arms to Home ---
  Serial.println(F("[STEP 5/6] Retracting pulley arms back to transport position..."));
  sweepArms(ARM_DROP_POS_L, ARM_HOME_POS_L, ARM_DROP_POS_R, ARM_HOME_POS_R, 30, 20);
  delay(300);

  // --- Step 6: Parked & Locked in Quarantine Zone ---
  Serial.println(F("=== [STEP 6/6] MISSION COMPLETE: PARKED IN QUARANTINE ZONE ==="));
  stopMotors();
}

/* ---------------- Pin Initialization ---------------- */
static void setupPins() {
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
  delay(100);

  setupPins();
  setupEncoders();

  Wire.begin();
  Wire.setClock(400000);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Arm transport home
  armsHome();

  // Execute full autonomous mission directly on boot
  executeAutonomousMission();
}

void loop() {
  // Pure autonomous hold: All motors permanently stopped in Quarantine Zone
  stopMotors();
  delay(500);
}


