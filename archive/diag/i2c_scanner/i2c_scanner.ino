#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(500);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println(F("I2CScanner"));
  Serial.print(F("SDA=")); Serial.print(SDA);
  Serial.print(F(" SCL=")); Serial.println(SCL);
}

void scanOnce(uint32_t hz) {
  Wire.setClock(hz);
  Serial.print(F("\nscan @ "));
  Serial.print(hz);
  Serial.println(F(" Hz"));

  uint8_t n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("FOUND 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      n++;
    } else if (err == 4) {
      Serial.print(F("ERR4 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }
  if (n == 0) Serial.println(F("No I2C devices found"));
  else {
    Serial.print(F("count="));
    Serial.println(n);
  }
}

void loop() {
  scanOnce(100000);
  delay(200);
  scanOnce(50000);
  delay(200);
  scanOnce(10000);
  Serial.println(F("---"));
  delay(2000);
}
