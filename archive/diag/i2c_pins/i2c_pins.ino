void setup() {
  Serial.begin(115200);
  Serial.println(F("\nI2C PINS"));
  Serial.print(F("\tSDA = ")); Serial.println(SDA);
  Serial.print(F("\tSCL = ")); Serial.println(SCL);
}

void loop() {}
