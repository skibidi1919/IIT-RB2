#include <Wire.h>

void scanBus() {
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("FOUND 0x"));
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      n++;
    }
  }
  if (!n) Serial.println(F("No devices"));
  else { Serial.print(F("count=")); Serial.println(n); }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("BUS CHECK"));
  Serial.print(F("SDA=")); Serial.print(SDA);
  Serial.print(F(" SCL=")); Serial.println(SCL);

  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  delay(5);
  Serial.print(F("float SDA=")); Serial.print(digitalRead(SDA));
  Serial.print(F(" SCL=")); Serial.println(digitalRead(SCL));

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(5);
  int sda = digitalRead(SDA);
  int scl = digitalRead(SCL);
  Serial.print(F("pullup SDA=")); Serial.print(sda);
  Serial.print(F(" SCL=")); Serial.println(scl);
  if (sda == 0 || scl == 0) Serial.println(F("FAULT stuck LOW"));
  else Serial.println(F("idle HIGH OK"));

  // bus recover
  for (int i = 0; i < 9; i++) {
    pinMode(SCL, OUTPUT); digitalWrite(SCL, LOW); delayMicroseconds(30);
    pinMode(SCL, INPUT_PULLUP); delayMicroseconds(30);
  }
  pinMode(SDA, OUTPUT); digitalWrite(SDA, LOW); delayMicroseconds(30);
  pinMode(SCL, INPUT_PULLUP); delayMicroseconds(30);
  pinMode(SDA, INPUT_PULLUP); delayMicroseconds(30);

  Wire.begin();
  Wire.setClock(50000);
  Serial.println(F("scan1"));
  scanBus();
  Wire.setClock(10000);
  Serial.println(F("scan2"));
  scanBus();
}

void loop() {}
