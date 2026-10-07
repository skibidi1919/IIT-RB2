/*
 * Step 2: PCA9685 16-Channel PWM & Dual L298N Motor Test
 * ------------------------------------------------------
 * Self-contained: Requires ZERO external libraries! (Uses standard Wire.h)
 *
 * Channel Mapping on PCA9685:
 *   Driver A (Front Motors 1 & 2):
 *     Ch 0: ENA (Motor 1 Speed PWM)
 *     Ch 1: IN1 (Motor 1 Dir A)
 *     Ch 2: IN2 (Motor 1 Dir B)
 *     Ch 3: IN3 (Motor 2 Dir A)
 *     Ch 4: IN4 (Motor 2 Dir B)
 *     Ch 5: ENB (Motor 2 Speed PWM)
 *
 *   Driver B (Rear Motors 3 & 4):
 *     Ch 6: ENA (Motor 3 Speed PWM)
 *     Ch 7: IN1 (Motor 3 Dir A)
 *     Ch 8: IN2 (Motor 3 Dir B)
 *     Ch 9: IN3 (Motor 4 Dir A)
 *     Ch 10: IN4 (Motor 4 Dir B)
 *     Ch 11: ENB (Motor 4 Speed PWM)
 *
 * Serial Monitor: 9600 Baud
 */

#include <Wire.h>

#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define MODE2        0x01
#define PRESCALE     0xFE
#define LED0_ON_L    0x06

// Voltage Safety Cap (12-bit max is 4095)
// For 6V N20 motors on ~12V battery: cap around 2200 (~55% duty cycle)
// For 12V N20 motors: can use up to 4095
const uint16_t MAX_PWM_12BIT = 2400;

// ================= ZERO-DEPENDENCY PCA9685 DRIVER =================
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
  Wire.write(LED0_ON_L + 4 * channel);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void pcaSetPin(uint8_t channel, uint16_t val) {
  if (val == 0) {
    pcaSetPWM(channel, 0, 4096);       // Bit 4 of OFF_H = Full OFF
  } else if (val >= 4095) {
    pcaSetPWM(channel, 4096, 0);       // Bit 4 of ON_H = Full ON
  } else {
    pcaSetPWM(channel, 0, val);        // Standard PWM duty cycle
  }
}

void initPCA9685(uint16_t freqHz = 200) {
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  pcaWrite8(MODE1, 0x00);              // Normal mode
  pcaWrite8(MODE2, 0x04);              // Push-pull (totem-pole) outputs for strong logic levels

  // Set PWM Frequency
  uint8_t prescale = (uint8_t)(25000000.0 / (4096.0 * freqHz) - 1.0 + 0.5);
  uint8_t oldmode = pcaRead8(MODE1);
  pcaWrite8(MODE1, (oldmode & 0x7F) | 0x10); // Sleep mode to write prescale
  pcaWrite8(PRESCALE, prescale);
  pcaWrite8(MODE1, oldmode);
  delay(5);
  pcaWrite8(MODE1, oldmode | 0xA0);          // Auto-increment enabled
}

// ================= MOTOR DRIVE FUNCTIONS =================
void setMotor(uint8_t pwmCh, uint8_t in1Ch, uint8_t in2Ch, int speed) {
  speed = constrain(speed, -(int)MAX_PWM_12BIT, (int)MAX_PWM_12BIT);

  if (speed > 0) {
    pcaSetPin(in1Ch, 4095); // Dir A = HIGH
    pcaSetPin(in2Ch, 0);    // Dir B = LOW
    pcaSetPin(pwmCh, speed);
  } else if (speed < 0) {
    pcaSetPin(in1Ch, 0);    // Dir A = LOW
    pcaSetPin(in2Ch, 4095); // Dir B = HIGH
    pcaSetPin(pwmCh, -speed);
  } else {
    pcaSetPin(in1Ch, 0);    // Stop / Coast
    pcaSetPin(in2Ch, 0);
    pcaSetPin(pwmCh, 0);
  }
}

void stopAllMotors() {
  for (int ch = 0; ch < 12; ch++) {
    pcaSetPin(ch, 0);
  }
}

// Convenience Wrappers for Individual Motors
void setM1(int speed) { setMotor(0, 1, 2, speed); }   // Front Left
void setM2(int speed) { setMotor(5, 3, 4, speed); }   // Front Right
void setM3(int speed) { setMotor(6, 7, 8, speed); }   // Rear Left
void setM4(int speed) { setMotor(11, 9, 10, speed); } // Rear Right

// ================= TEST SETUP & LOOP =================
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000);

  Serial.println(F("\n=============================================="));
  Serial.println(F("     PCA9685 + DUAL L298N MOTOR TESTBENCH     "));
  Serial.println(F("=============================================="));

  initPCA9685(200); // 200 Hz PWM for L298N
  stopAllMotors();
  Serial.println(F("PCA9685 Initialized. All motors stopped."));
  delay(1000);
}

void testSingleMotor(const char* name, void (*motorFunc)(int)) {
  Serial.print(F("Testing ")); Serial.print(name); Serial.println(F(" FORWARD..."));
  motorFunc(1800); // ~45% speed
  delay(1500);

  Serial.print(F("Testing ")); Serial.print(name); Serial.println(F(" REVERSE..."));
  motorFunc(-1800);
  delay(1500);

  Serial.print(F("Stopping ")); Serial.println(name);
  motorFunc(0);
  delay(800);
}

void loop() {
  Serial.println(F("\n--- Starting Motor Cycle Sequence ---"));

  testSingleMotor("Motor 1 (Front Left)",  setM1);
  testSingleMotor("Motor 2 (Front Right)", setM2);
  testSingleMotor("Motor 3 (Rear Left)",   setM3);
  testSingleMotor("Motor 4 (Rear Right)",  setM4);

  Serial.println(F("\n--- Testing All 4 Motors Driving Forward Together ---"));
  setM1(1800); setM2(1800); setM3(1800); setM4(1800);
  delay(2000);

  Serial.println(F("--- Stopping All Motors ---"));
  stopAllMotors();

  Serial.println(F("Sequence complete. Pausing 5 seconds...\n"));
  delay(5000);
}
