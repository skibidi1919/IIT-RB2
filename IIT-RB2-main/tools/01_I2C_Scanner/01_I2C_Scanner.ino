/*
 * Step 1: Automated I2C Bus Diagnostic Scanner
 * -------------------------------------------
 * Scans the I2C bus (A4 = SDA, A5 = SCL) and verifies detection of:
 *   - 0x29: VL53L0X Time-of-Flight Distance Sensor
 *   - 0x40: PCA9685 16-Channel 12-Bit PWM Driver
 *   - 0x4A (or 0x4B): BNO08x 9-DOF IMU / Gyroscope
 *
 * Baud Rate: 9600 baud in Arduino Serial Monitor.
 */

#include <Wire.h>

void setup() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  Serial.begin(9600);
  while (!Serial && millis() < 2000);

  Serial.println(F("\n================================================"));
  Serial.println(F("       N20 ROVER: I2C BUS HARDWARE SCANNER       "));
  Serial.println(F("================================================"));
  Serial.println(F("Scanning I2C Bus (A4=SDA, A5=SCL)...\n"));
}

void loop() {
  byte error, address;
  int nDevices = 0;
  bool foundVL53L0X = false;
  bool foundPCA9685 = false;
  bool foundBNO08x  = false;

  Serial.println(F("--- Beginning New Bus Scan ---"));

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("[OK] Found device at 7-bit address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.print(F(" : "));

      if (address == 0x29) {
        Serial.println(F("VL53L0X Time-of-Flight Distance Sensor"));
        foundVL53L0X = true;
      } else if (address == 0x40) {
        Serial.println(F("PCA9685 16-Channel 12-Bit PWM Controller"));
        foundPCA9685 = true;
      } else if (address == 0x4A || address == 0x4B) {
        Serial.println(F("BNO08x 9-DOF IMU (Gyroscope / Accelerometer)"));
        foundBNO08x = true;
      } else if (address == 0x70) {
        Serial.println(F("PCA9685 All-Call Broadcast Address (Normal)"));
      } else {
        Serial.println(F("Unknown / Auxiliary I2C Peripheral"));
      }

      nDevices++;
    } else if (error == 4) {
      Serial.print(F("[ERR] Unknown transmission error at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
    }
  }

  Serial.println(F("\n--- Diagnostic Health Summary ---"));
  Serial.print(F("PCA9685 (0x40): "));
  Serial.println(foundPCA9685 ? F("ONLINE [PASS]") : F("OFFLINE [FAIL - Check A4/A5 and 5V/GND]"));

  Serial.print(F("VL53L0X (0x29): "));
  Serial.println(foundVL53L0X ? F("ONLINE [PASS]") : F("OFFLINE [FAIL - Check A4/A5 and 5V/GND]"));

  Serial.print(F("BNO08x  (0x4A): "));
  Serial.println(foundBNO08x  ? F("ONLINE [PASS]") : F("OFFLINE [FAIL - Check A4/A5, DI0 pin to GND]"));

  Serial.print(F("Total Devices Active: "));
  Serial.println(nDevices);
  Serial.println(F("------------------------------------------------\n"));

  delay(4000); // Re-scan every 4 seconds
}
