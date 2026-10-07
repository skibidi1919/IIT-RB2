#include <Wire.h>
#include <util/atomic.h>
#include <VL53L0X.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define PCA9685_ADDR   0x40
#define BNO08X_ADDR    0x4A
#define BNO08X_RESET   12
#define BNO08X_INT     2

const uint8_t ENC1_PIN = 3;
const uint8_t ENC2_PIN = 4;
const uint8_t ENC3_PIN = 7;
const uint8_t ENC4_PIN = 8;
const uint16_t MAX_SPEED = 2400;

VL53L0X lox;
BNO080  bno08x;

bool hasVL53L0X = false;
bool hasBNO08x  = false;

volatile long encTicks1 = 0;
volatile long encTicks2 = 0;
volatile long encTicks3 = 0;
volatile long encTicks4 = 0;

volatile uint8_t lastPIND = 0;
volatile uint8_t lastPINB = 0;

float currentHeadingYaw    = 0.0;
float currentPitch         = 0.0;
float currentRoll          = 0.0;
uint16_t currentDistanceMm = 8190;

int lastM1 = -9999;
int lastM2 = -9999;
int lastM3 = -9999;
int lastM4 = -9999;

enum NavState {
  STATE_FORWARD,
  STATE_STOP,
  STATE_BACKWARD,
  STATE_TURN
};

NavState navState = STATE_FORWARD;
unsigned long navTimer = 0;

void pcaWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t pcaRead8(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
  return Wire.read();
}

void pcaSetPWM(uint8_t channel, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + 4 * channel);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void pcaSetPin(uint8_t channel, uint16_t val) {
  if (val == 0) {
    pcaSetPWM(channel, 0, 4096);
  } else if (val >= 4095) {
    pcaSetPWM(channel, 4096, 0);
  } else {
    pcaSetPWM(channel, 0, val);
  }
}

void initPCA9685(uint16_t freqHz = 200) {
  pcaWrite8(0x00, 0x00);
  pcaWrite8(0x01, 0x04);
  uint8_t prescale = (uint8_t)(25000000.0 / (4096.0 * freqHz) - 1.0 + 0.5);
  uint8_t oldmode = pcaRead8(0x00);
  pcaWrite8(0x00, (oldmode & 0x7F) | 0x10);
  pcaWrite8(0xFE, prescale);
  pcaWrite8(0x00, oldmode);
  delay(5);
  pcaWrite8(0x00, oldmode | 0xA0);
}

void setMotorRaw(uint8_t pwmCh, uint8_t in1Ch, uint8_t in2Ch, int speed) {
  speed = constrain(speed, -(int)MAX_SPEED, (int)MAX_SPEED);
  if (speed > 0) {
    pcaSetPin(in1Ch, 4095);
    pcaSetPin(in2Ch, 0);
    pcaSetPin(pwmCh, speed);
  } else if (speed < 0) {
    pcaSetPin(in1Ch, 0);
    pcaSetPin(in2Ch, 4095);
    pcaSetPin(pwmCh, -speed);
  } else {
    pcaSetPin(in1Ch, 0);
    pcaSetPin(in2Ch, 0);
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
  setMotorRaw(0, 1, 2, m1);
  setMotorRaw(5, 3, 4, m2);
  setMotorRaw(6, 7, 8, m3);
  setMotorRaw(11, 9, 10, m4);
}

void stopRobot() {
  setWheels(0, 0, 0, 0);
}

void moveForward(int speed)  { setWheels( speed,  speed,  speed,  speed); }
void moveBackward(int speed) { setWheels(-speed, -speed, -speed, -speed); }
void turnLeft(int speed)     { setWheels(-speed,  speed, -speed,  speed); }
void turnRight(int speed)    { setWheels( speed, -speed,  speed, -speed); }

void isrEncoder1() {
  encTicks1++;
}

ISR(PCINT2_vect) {
  uint8_t curr = PIND;
  if ((curr & (1 << PIND4)) && !(lastPIND & (1 << PIND4))) encTicks2++;
  if ((curr & (1 << PIND7)) && !(lastPIND & (1 << PIND7))) encTicks3++;
  lastPIND = curr;
}

ISR(PCINT0_vect) {
  uint8_t curr = PINB;
  if ((curr & (1 << PINB0)) && !(lastPINB & (1 << PINB0))) encTicks4++;
  lastPINB = curr;
}

void handleNavigation() {
  unsigned long now = millis();
  switch (navState) {
    case STATE_FORWARD:
      if (hasVL53L0X && currentDistanceMm < 180) {
        stopRobot();
        navState = STATE_STOP;
        navTimer = now;
      } else {
        moveForward(1600);
      }
      break;

    case STATE_STOP:
      if (now - navTimer >= 200) {
        moveBackward(1500);
        navState = STATE_BACKWARD;
        navTimer = now;
      }
      break;

    case STATE_BACKWARD:
      if (now - navTimer >= 400) {
        turnRight(1600);
        navState = STATE_TURN;
        navTimer = now;
      }
      break;

    case STATE_TURN:
      if (now - navTimer >= 500) {
        moveForward(1600);
        navState = STATE_FORWARD;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  delay(1000);

  Serial.println(F("\n=============================================="));
  Serial.println(F("     4WD N20 ROVER: MASTER SYSTEM BOOT        "));
  Serial.println(F("=============================================="));

  initPCA9685(200);
  stopRobot();
  Serial.println(F("[OK] PCA9685 Motor Controller Ready."));

  pinMode(ENC1_PIN, INPUT_PULLUP);
  pinMode(ENC2_PIN, INPUT_PULLUP);
  pinMode(ENC3_PIN, INPUT_PULLUP);
  pinMode(ENC4_PIN, INPUT_PULLUP);

  lastPIND = PIND;
  lastPINB = PINB;

  attachInterrupt(digitalPinToInterrupt(ENC1_PIN), isrEncoder1, RISING);
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT23);
  PCICR  |= (1 << PCIE2);
  PCMSK0 |= (1 << PCINT0);
  PCICR  |= (1 << PCIE0);
  Serial.println(F("[OK] 4x Encoders Interrupt Handlers Armed."));

  lox.setTimeout(80);
  if (lox.init()) {
    hasVL53L0X = true;
    lox.startContinuous();
    Serial.println(F("[OK] VL53L0X Time-of-Flight Sensor Detected."));
  } else {
    Serial.println(F("[WARN] VL53L0X Not Found. Operating without ToF."));
  }

  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(10);
  digitalWrite(BNO08X_RESET, LOW);
  delay(10);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(200);

  if (bno08x.begin(BNO08X_ADDR, Wire, BNO08X_INT)) {
    hasBNO08x = true;
    bno08x.enableGameRotationVector(50);
    Serial.println(F("[OK] BNO08x 9-DOF AHRS IMU Initialized."));
  } else {
    Serial.println(F("[WARN] BNO08x IMU Not Found. Operating without Gyro."));
  }

  Serial.println(F("System Ready. Starting Autonomous Obstacle Avoidance Loop.\n"));
}

void loop() {
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

  handleNavigation();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
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
