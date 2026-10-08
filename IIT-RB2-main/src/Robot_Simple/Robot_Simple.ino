#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <VL53L0X.h>

// Fallback pin definitions for static analyzers
#ifndef A0
  #define A0 14
  #define A1 15
  #define A2 16
  #define A3 17
#endif

// ================= HARDWARE PIN DEFINITIONS =================
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

// Speeds (0-255)
const uint8_t DRIVE_SPEED = 160;
const uint8_t TURN_SPEED  = 160;
const uint16_t STOP_DIST  = 180; // Obstacle distance threshold in mm

VL53L0X sensor;
bool sensorReady = false;

int lastM1 = -9999;
int lastM2 = -9999;
int lastM3 = -9999;
int lastM4 = -9999;

void setMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
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

  setMotor(M1_ENA, M1_IN1, M1_IN2, m1);
  setMotor(M2_ENB, M2_IN3, M2_IN4, m2);
  setMotor(M3_ENA, M3_IN1, M3_IN2, m3);
  setMotor(M4_ENB, M4_IN3, M4_IN4, m4);
}

void stopMotors() {
  setWheels(0, 0, 0, 0);
}

void driveForward(int speed) {
  setWheels(speed, speed, speed, speed);
}

void driveBackward(int speed) {
  setWheels(-speed, -speed, -speed, -speed);
}

void turnRight(int speed) {
  setWheels(speed, -speed, speed, -speed);
}

void turnLeft(int speed) {
  setWheels(-speed, speed, -speed, speed);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  delay(1000);

  Serial.println(F("--- Simple Trial Rover Starting ---"));

  // Configure Direct Motor Pins
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

  stopMotors();
  Serial.println(F("[OK] Direct Motor Controller GPIOs Ready."));

  // Initialize VL53L0X
  sensor.setTimeout(80);
  if (sensor.init()) {
    sensorReady = true;
    sensor.startContinuous();
    Serial.println(F("[OK] VL53L0X Online"));
  } else {
    Serial.println(F("[WARN] VL53L0X Not Found. Will drive forward blindly!"));
  }
}

void loop() {
  uint16_t dist = 9999;

  if (sensorReady) {
    dist = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred() || dist == 65535) {
      dist = 9999;
    }
  }

  Serial.print(F("Distance: "));
  Serial.print(dist);
  Serial.println(F(" mm"));

  if (sensorReady && dist < STOP_DIST) {
    Serial.println(F("Obstacle ahead! Reversing..."));
    stopMotors();
    delay(100);
    driveBackward(DRIVE_SPEED);
    delay(400);

    Serial.println(F("Turning right..."));
    turnRight(TURN_SPEED);
    delay(500);

    stopMotors();
    delay(100);
  } else {
    driveForward(DRIVE_SPEED);
  }

  delay(50);
}
