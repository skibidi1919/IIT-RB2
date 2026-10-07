#include <Wire.h>
#include <VL53L0X.h>

#define PCA9685_ADDR 0x40

const uint16_t DRIVE_SPEED = 1500;
const uint16_t TURN_SPEED  = 1600;
const uint16_t STOP_DIST   = 180;

VL53L0X sensor;
bool sensorReady = false;

int lastM1 = -9999;
int lastM2 = -9999;
int lastM3 = -9999;
int lastM4 = -9999;

void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void pcaSetPin(uint8_t ch, uint16_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + 4 * ch);
  if (val == 0) {
    Wire.write(0); Wire.write(0);
    Wire.write(0); Wire.write(16);
  } else if (val >= 4095) {
    Wire.write(0); Wire.write(16);
    Wire.write(0); Wire.write(0);
  } else {
    Wire.write(0); Wire.write(0);
    Wire.write(val & 0xFF); Wire.write(val >> 8);
  }
  Wire.endTransmission();
}

void initMotors() {
  pcaWrite(0x00, 0x00);
  pcaWrite(0x01, 0x04);
  uint8_t prescale = (uint8_t)(25000000.0 / (4096.0 * 200.0) - 1.0 + 0.5);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
  uint8_t oldmode = Wire.read();
  pcaWrite(0x00, (oldmode & 0x7F) | 0x10);
  pcaWrite(0xFE, prescale);
  pcaWrite(0x00, oldmode);
  delay(5);
  pcaWrite(0x00, oldmode | 0xA0);
  stopMotors();
}

void setMotor(uint8_t pwmCh, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -2400, 2400);
  if (speed > 0) {
    pcaSetPin(in1, 4095);
    pcaSetPin(in2, 0);
    pcaSetPin(pwmCh, speed);
  } else if (speed < 0) {
    pcaSetPin(in1, 0);
    pcaSetPin(in2, 4095);
    pcaSetPin(pwmCh, -speed);
  } else {
    pcaSetPin(in1, 0);
    pcaSetPin(in2, 0);
    pcaSetPin(pwmCh, 0);
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
  setMotor(0, 1, 2, m1);
  setMotor(5, 3, 4, m2);
  setMotor(6, 7, 8, m3);
  setMotor(11, 9, 10, m4);
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

  initMotors();
  Serial.println(F("Motors Initialized"));

  sensor.setTimeout(80);
  if (sensor.init()) {
    sensorReady = true;
    sensor.startContinuous();
    Serial.println(F("VL53L0X Distance Sensor Ready"));
  } else {
    Serial.println(F("VL53L0X Not Found - Check Wiring"));
  }

  delay(1500);
}

void loop() {
  uint16_t dist = 8190;

  if (sensorReady) {
    uint16_t d = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred() || d == 65535) {
      dist = 8190;
    } else {
      dist = d;
    }
  }

  Serial.print(F("Distance: "));
  Serial.print(dist);
  Serial.println(F(" mm"));

  if (sensorReady && dist < STOP_DIST) {
    Serial.println(F("Obstacle Detected -> Backing Up & Turning"));
    stopMotors();
    delay(200);

    driveBackward(DRIVE_SPEED);
    delay(400);

    turnRight(TURN_SPEED);
    delay(500);

    stopMotors();
    delay(200);
  } else {
    driveForward(DRIVE_SPEED);
  }

  delay(100);
}
