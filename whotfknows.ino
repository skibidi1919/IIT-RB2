#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150  
#define SERVOMAX 600

// =================================================================
// 1. PIN DEFINITIONS (Updated Dual L298N + PCA9685 + 4-Motor Setup)
// =================================================================

// --- L298N - P1 (Left Side: Motors 1 & 2) ---
const uint8_t L298N_P1_ENA = 10;  // PWM (Timer1) -> M1 Speed
const uint8_t L298N_P1_IN1 = A0;  // M1 Dir 1
const uint8_t L298N_P1_IN2 = A1;  // M1 Dir 2
const uint8_t L298N_P1_IN3 = A2;  // M2 Dir 1
const uint8_t L298N_P1_IN4 = A3;  // M2 Dir 2
const uint8_t L298N_P1_ENB = 11;  // PWM (Timer2) -> M2 Speed

// --- L298N - P2 (Right Side: Motors 3 & 4) ---
const uint8_t L298N_P2_ENA = 6;   // PWM (Timer0) -> M3 Speed
const uint8_t L298N_P2_IN1 = 7;   // M3 Dir 1
const uint8_t L298N_P2_IN2 = 8;   // M3 Dir 2
const uint8_t L298N_P2_IN3 = 12;  // M4 Dir 1
const uint8_t L298N_P2_IN4 = 13;  // M4 Dir 2 (mirrors Nano onboard LED)
const uint8_t L298N_P2_ENB = 9;   // PWM (Timer1) -> M4 Speed

// --- Motor Feedback / Encoder Pins (C1) ---
const uint8_t PIN_M1_C1 = 5;      // Motor 1 feedback
const uint8_t PIN_M2_C1 = 4;      // Motor 2 feedback
const uint8_t PIN_M3_C1 = 2;      // Motor 3 feedback (Hardware Interrupt INT0)
const uint8_t PIN_M4_C1 = 3;      // Motor 4 feedback (Hardware Interrupt INT1)

// --- Motor IDs ---
const int M1 = 1; // Left Front
const int M2 = 2; // Left Rear
const int M3 = 3; // Right Front
const int M4 = 4; // Right Rear

// --- Direction Constants ---
const int STOP = 0;
const int FORWARD = 1;
const int BACKWARD = 2;

// =================================================================
// 2. SENSORS & SERVO CONFIGURATION
// =================================================================

// MPU6050 Hardware Constants (I2C on A4/A5)
const int MPU_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;
const int GYRO_ZOUT_H = 0x47;

// Gyro Variables
float gyro_z_bias = 0;   
float current_angle = 0;
unsigned long last_time = 0;

// Encoder Tick Counters
volatile long enc_m3_ticks = 0;
volatile long enc_m4_ticks = 0;

// PCA9685 Servo Channels & Angles
int GRIPPER_SERVOS[] = {5, 6, 7};
int SERVO_OPEN_ANGLES[] = {80, 60, 85};
int SERVO_CLOSE_ANGLES[] = {45, 20, 120};

int BIN_SERVOS[] = {10, 9, 8};
int BIN_CATCH_ANGLE[] = {20, 120, 30};
int BIN_THROW_ANGLE[] = {105, 30, 120};

const int PULL_UP_SERVO = 12;
const int CONVEYER_SERVO = 13;

// Execution Control
bool autonomous_completed = false;

// =================================================================
// 3. ENCODER INTERRUPT HANDLERS
// =================================================================
void isr_encoder_m3() { enc_m3_ticks++; }
void isr_encoder_m4() { enc_m4_ticks++; }

// =================================================================
// 4. SERVO CONTROL FUNCTIONS (PCA9685)
// =================================================================
void moveServo(int servoNum, float angle) {
  int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulse);
  delay(100);
}

void close_grippers() {
  for(int i = 0; i < 3; i++) {
    moveServo(GRIPPER_SERVOS[i], SERVO_CLOSE_ANGLES[i]);
  }
  delay(300);
}

void open_grippers() {
  int order[] = {0, 2, 1};
  for(int i = 0; i < 3; i++) {
    moveServo(GRIPPER_SERVOS[order[i]], SERVO_OPEN_ANGLES[order[i]]);
    delay(100);
  }
  delay(300);
}

void catchBins() {
  for(int i = 0; i < 3; i++) {
    moveServo(BIN_SERVOS[i], BIN_CATCH_ANGLE[i]);
  }
  delay(100);
}

void throwBins() {
  for(int i = 0; i < 3; i++) {
    moveServo(BIN_SERVOS[i], BIN_THROW_ANGLE[i]);
  }
  delay(300);
}

void throwRedBin() {
  moveServo(BIN_SERVOS[2], BIN_THROW_ANGLE[2]);
  delay(100);
  moveServo(BIN_SERVOS[2], BIN_THROW_ANGLE[2] - 50);
  delay(100);
  moveServo(BIN_SERVOS[2], BIN_THROW_ANGLE[2]);
  delay(100);
  moveServo(BIN_SERVOS[2], BIN_THROW_ANGLE[2] - 50);
  delay(100);
  moveServo(BIN_SERVOS[2], BIN_THROW_ANGLE[2]);
}

void throwGreenBin() {
  moveServo(BIN_SERVOS[1], BIN_THROW_ANGLE[1]);
  delay(100);
  moveServo(BIN_SERVOS[1], BIN_THROW_ANGLE[1] + 50);
  delay(100);
  moveServo(BIN_SERVOS[1], BIN_THROW_ANGLE[1]);
  delay(100);
  moveServo(BIN_SERVOS[1], BIN_THROW_ANGLE[1] + 50);
  delay(100);
  moveServo(BIN_SERVOS[1], BIN_THROW_ANGLE[1]);
}

void throwYellowBin() {
  moveServo(BIN_SERVOS[0], BIN_THROW_ANGLE[0]);
  delay(100);
  moveServo(BIN_SERVOS[0], BIN_THROW_ANGLE[0] - 50);
  delay(100);
  moveServo(BIN_SERVOS[0], BIN_THROW_ANGLE[0]);
  delay(100);
  moveServo(BIN_SERVOS[0], BIN_THROW_ANGLE[0] - 50);
  delay(100);
  moveServo(BIN_SERVOS[0], BIN_THROW_ANGLE[0]);
}

void pullUpArms(int delayMillis) {
  moveServo(PULL_UP_SERVO, 100);
  delay(delayMillis);
  moveServo(PULL_UP_SERVO, 70);
  delay(300);
}

void pullDownArms(int delayMillis) {
  moveServo(PULL_UP_SERVO, 55);
  delay(delayMillis);
  moveServo(PULL_UP_SERVO, 70);
  delay(300);
}

void moveConveyorToDrop(int delayMillis) {
  moveServo(CONVEYER_SERVO, 50);
  delay(delayMillis);
  moveServo(CONVEYER_SERVO, 70);
  delay(300);
}

void rewindConveyor(int delayMillis) {
  moveServo(CONVEYER_SERVO, 100);
  delay(delayMillis);
  moveServo(CONVEYER_SERVO, 70);
  delay(300);
}

// =================================================================
// 5. GYRO SENSOR (MPU6050)
// =================================================================
int16_t read_raw_gyro_z() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  
  if (Wire.available() >= 2) {
    int16_t high = Wire.read();
    int16_t low = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

void calibrate_gyro() {
  long sum = 0;
  const int samples = 200;
  
  for (int i = 0; i < samples; i++) {
    sum += read_raw_gyro_z();
    delay(3);
  }
  gyro_z_bias = (float)sum / samples;
}

void update_heading() {
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  int16_t raw_gyro_z = read_raw_gyro_z();
  float unbiased_gyro_z = (float)raw_gyro_z - gyro_z_bias;
  float gyro_rate_z = -unbiased_gyro_z / 131.0; 
  current_angle += gyro_rate_z * dt;
}

// =================================================================
// 6. 4-MOTOR L298N DRIVETRAIN CONTROLLER
// =================================================================

/**
 * Control an individual motor on either L298N driver
 */
void setSingleMotor(int motorNumber, int direction, int speed) {
  speed = constrain(speed, 0, 255);
  uint8_t enPin = 0, in1Pin = 0, in2Pin = 0;

  switch(motorNumber) {
    case M1: enPin = L298N_P1_ENA; in1Pin = L298N_P1_IN1; in2Pin = L298N_P1_IN2; break;
    case M2: enPin = L298N_P1_ENB; in1Pin = L298N_P1_IN3; in2Pin = L298N_P1_IN4; break;
    case M3: enPin = L298N_P2_ENA; in1Pin = L298N_P2_IN1; in2Pin = L298N_P2_IN2; break;
    case M4: enPin = L298N_P2_ENB; in1Pin = L298N_P2_IN3; in2Pin = L298N_P2_IN4; break;
    default: return;
  }

  if (direction == FORWARD) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
  } else if (direction == BACKWARD) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, speed);
  } else { // STOP
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

/**
 * Drive Left (M1, M2) and Right (M3, M4) motor banks synchronously
 */
void setDrivetrain(int leftDir, int leftSpeed, int rightDir, int rightSpeed) {
  setSingleMotor(M1, leftDir, leftSpeed);
  setSingleMotor(M2, leftDir, leftSpeed);
  setSingleMotor(M3, rightDir, rightSpeed);
  setSingleMotor(M4, rightDir, rightSpeed);
}

void stopMotors() {
  setDrivetrain(STOP, 0, STOP, 0);
}

void moveForward(int speed, int durationMillis) {
  setDrivetrain(FORWARD, speed, FORWARD, speed);
  delay(durationMillis);
  stopMotors();
}

void moveBackward(int speed, int durationMillis) {
  setDrivetrain(BACKWARD, speed, BACKWARD, speed);
  delay(durationMillis);
  stopMotors();
}

void turnRight(int speed) {
  setDrivetrain(FORWARD, speed, BACKWARD, speed);
}

void turnLeft(int speed) {
  setDrivetrain(BACKWARD, speed, FORWARD, speed);
}

// =================================================================
// 7. GYRO-STABILIZED AUTONOMOUS NAVIGATION
// =================================================================

// Move Forward maintaining a straight heading
void moveForwardGyro(int speed, int duration_millis, int offset = 0) {
  last_time = micros();
  float target_angle = current_angle;
  unsigned long start_time = millis();
  float Kp = 8.0;
  
  while (millis() - start_time < (unsigned long)duration_millis) {
    update_heading();
    
    float error = current_angle - target_angle;
    int correction = (int)(error * Kp);
    
    int left_speed = constrain(speed + correction + offset, 0, 255);
    int right_speed = constrain(speed - correction - offset, 0, 255);
    
    setDrivetrain(FORWARD, left_speed, FORWARD, right_speed);
    delay(10);
  }
  stopMotors();
}

// Move Backward maintaining a straight heading
void moveBackwardGyro(int speed, int duration_millis, int motor_bias = 0) {
  last_time = micros();
  float target_angle = current_angle;
  unsigned long start_time = millis();
  float Kp = 6.0; 
  
  while (millis() - start_time < (unsigned long)duration_millis) {
    update_heading();
    
    float error = current_angle - target_angle;
    int correction = (int)(error * Kp);
    
    int left_speed = constrain(speed + correction - motor_bias, 0, 255); 
    int right_speed = constrain(speed - correction + motor_bias, 0, 255); 
    
    setDrivetrain(BACKWARD, left_speed, BACKWARD, right_speed);
    delay(10);
  }
  stopMotors();
}

void turnRightGyro(int speed, float offset = 0) {
  last_time = micros();
  float start_angle = current_angle;
  delay(100);
  update_heading();

  float target_angle = start_angle - 90.0 + offset;
  turnRight(speed);
  
  while (current_angle > target_angle) {
    update_heading();
  }
  
  // Active counter-pulse to brake inertia
  turnLeft(speed); 
  delay(40);
  stopMotors();
  delay(300);
}

void turnLeftGyro(int speed, float offset = 0) {
  last_time = micros();
  float start_angle = current_angle;
  delay(100);
  update_heading();
  
  float target_angle = start_angle + 90.0 - offset; 
  turnLeft(speed);
  
  while (current_angle < target_angle) {
    update_heading();
  }
  
  // Active counter-pulse to brake inertia
  turnRight(speed);
  delay(40); 
  stopMotors();
  delay(300);
}

// =================================================================
// 8. SETUP INITIALIZATION
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("[MEOWLER] Initializing 4-Motor Autonomous Youth Challenge..."));

  // L298N P1 Pins
  pinMode(L298N_P1_ENA, OUTPUT);
  pinMode(L298N_P1_IN1, OUTPUT);
  pinMode(L298N_P1_IN2, OUTPUT);
  pinMode(L298N_P1_IN3, OUTPUT);
  pinMode(L298N_P1_IN4, OUTPUT);
  pinMode(L298N_P1_ENB, OUTPUT);

  // L298N P2 Pins
  pinMode(L298N_P2_ENA, OUTPUT);
  pinMode(L298N_P2_IN1, OUTPUT);
  pinMode(L298N_P2_IN2, OUTPUT);
  pinMode(L298N_P2_IN3, OUTPUT);
  pinMode(L298N_P2_IN4, OUTPUT);
  pinMode(L298N_P2_ENB, OUTPUT);

  // Encoder Pins
  pinMode(PIN_M1_C1, INPUT_PULLUP);
  pinMode(PIN_M2_C1, INPUT_PULLUP);
  pinMode(PIN_M3_C1, INPUT_PULLUP);
  pinMode(PIN_M4_C1, INPUT_PULLUP);

  // Hardware interrupts on D2 and D3
  attachInterrupt(digitalPinToInterrupt(PIN_M3_C1), isr_encoder_m3, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_M4_C1), isr_encoder_m4, RISING);

  // Stop motors initially
  stopMotors();

  // Initialize PCA9685 PWM driver
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);

  // Wake up MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); 
  Wire.endTransmission(true);

  Serial.println(F("[MEOWLER] Calibrating MPU6050 Gyro... Keep robot static."));
  calibrate_gyro();
  Serial.println(F("[MEOWLER] Gyro calibrated. Ready for challenge."));

  // Initial Posture
  catchBins();
  open_grippers();
  pullUpArms(100);
}

// =================================================================
// 9. AUTONOMOUS MISSION ROUTINE (Youth Challenge)
// =================================================================
void runYouthChallengeAutonomous() {
  const int STEP_DELAY = 1000;
  const int FORWARD_OFFSET = 10;
  const int SPEED_FAST = 200;
  const int SPEED_MED = 175;
  const int SPEED_SLOW = 150;

  Serial.println(F("[AUTONOMOUS] Starting Youth Challenge Sequence..."));

  // Step 1: Countdown / Launch Delay
  delay(3000);

  // Step 2: Turn Right to align with Zone 1
  turnRightGyro(SPEED_MED, 10);
  
  // Step 3: Back up against wall reference
  moveBackwardGyro(SPEED_SLOW, 700);
  delay(STEP_DELAY);

  // Step 4: Advance towards object pick sector
  moveForwardGyro(SPEED_FAST, 950, FORWARD_OFFSET);
  delay(STEP_DELAY);

  // Step 5: Turn Left towards first item
  turnLeftGyro(SPEED_MED, 10);

  // Step 6: Minor reverse alignment
  moveBackwardGyro(SPEED_SLOW, 700);
  delay(STEP_DELAY);

  // Step 7: Approach item 1
  moveForwardGyro(SPEED_FAST, 1225, FORWARD_OFFSET);
  delay(STEP_DELAY);
  
  // Step 8-11: Pick Item 1 & load
  pullDownArms(2000);
  close_grippers(); 
  delay(STEP_DELAY);
  pullUpArms(2000);
  delay(STEP_DELAY);
  open_grippers(); 
  delay(STEP_DELAY);

  // Step 12: Move forward to item 2
  moveForwardGyro(SPEED_FAST, 1175, FORWARD_OFFSET); 
  delay(STEP_DELAY);

  // Step 13-16: Pick Item 2 & load
  pullDownArms(2000);
  close_grippers(); 
  delay(STEP_DELAY);
  pullUpArms(2000); 
  delay(STEP_DELAY);
  open_grippers();
  delay(STEP_DELAY);

  // Step 17: Drive forward across mid field
  moveForward(SPEED_FAST, 3400);
  delay(STEP_DELAY);

  // Step 18-20: Turn and traverse to secondary arena zone
  turnRightGyro(190, 10); 
  moveBackward(SPEED_FAST, 900); 
  delay(STEP_DELAY);
  moveForward(SPEED_FAST, 6300); 
  delay(STEP_DELAY);
  
  // Step 21-24: Align with drop bins
  turnRightGyro(190, 10);
  moveBackwardGyro(SPEED_SLOW, 900); 
  delay(STEP_DELAY);
  turnRightGyro(190, 10); 
  moveBackward(SPEED_SLOW, 1200); 
  delay(STEP_DELAY);

  // Step 25-29: Deliver items via conveyor belt
  moveForwardGyro(SPEED_FAST, 1500, FORWARD_OFFSET);  
  delay(STEP_DELAY); 
  moveConveyorToDrop(1000);
  moveForwardGyro(SPEED_FAST, 1500, FORWARD_OFFSET); 
  moveConveyorToDrop(2200); 
  moveForwardGyro(SPEED_FAST, 3000, FORWARD_OFFSET); 
  delay(STEP_DELAY);

  // Step 30-33: Approach Bin Sorting Zone
  turnLeftGyro(SPEED_MED, 10); 
  moveBackwardGyro(SPEED_FAST, 700); 
  delay(STEP_DELAY); 
  moveForwardGyro(SPEED_FAST, 1000, FORWARD_OFFSET);
  delay(STEP_DELAY);
  moveConveyorToDrop(1500);
  delay(500);

  // Step 34-36: Throw Yellow Bin
  turnLeftGyro(SPEED_MED, 10);
  delay(STEP_DELAY);
  turnLeftGyro(SPEED_MED, 10);
  delay(STEP_DELAY);
  throwYellowBin();
  delay(STEP_DELAY);
  catchBins();

  // Step 37-40: Navigate & Throw Red Bin
  turnRightGyro(190, 10);
  delay(STEP_DELAY);
  moveForwardGyro(SPEED_FAST, 700);
  delay(STEP_DELAY);
  turnLeftGyro(SPEED_MED, 10);
  delay(STEP_DELAY);
  throwRedBin();
  delay(STEP_DELAY);
  catchBins();

  // Step 41-44: Navigate & Throw Green Bin
  turnLeftGyro(SPEED_MED, 10);
  delay(STEP_DELAY);
  turnLeftGyro(SPEED_MED, 10);
  delay(STEP_DELAY);
  moveForwardGyro(SPEED_FAST, 3000);
  delay(STEP_DELAY);
  throwGreenBin();
  delay(STEP_DELAY);
  catchBins();
  
  // Step 45: Reset Conveyor and safely stop all actuators
  rewindConveyor(3000);
  stopMotors(); 

  Serial.println(F("[AUTONOMOUS] Youth Challenge Completed Successfully!"));
}

// =================================================================
// 10. MAIN LOOP
// =================================================================
void loop() {
  if (!autonomous_completed) {
    runYouthChallengeAutonomous();
    autonomous_completed = true;
  }
  // Idle after mission completion
  stopMotors();
  delay(1000);
}