/*
 * Bit-bang I2C scanner across common Nano pin pairs.
 * Prints every address found on each pair.
 */

static const uint8_t PAIRS[][2] = {
  {A4, A5}, // official Wire
  {A5, A4}, // swapped
  {A2, A3},
  {A3, A2},
  {2, 3},
  {3, 2},
  {4, 5},
  {5, 4},
  {6, 7},
  {7, 6},
  {8, 9},
  {9, 8},
  {10, 11},
  {11, 10},
};

uint8_t sdaPin = A4;
uint8_t sclPin = A5;

void i2cDelay() { delayMicroseconds(20); }

void sclHigh() { pinMode(sclPin, INPUT_PULLUP); i2cDelay(); }
void sclLow()  { pinMode(sclPin, OUTPUT); digitalWrite(sclPin, LOW); i2cDelay(); }
void sdaHigh() { pinMode(sdaPin, INPUT_PULLUP); i2cDelay(); }
void sdaLow()  { pinMode(sdaPin, OUTPUT); digitalWrite(sdaPin, LOW); i2cDelay(); }
bool sdaRead() { pinMode(sdaPin, INPUT_PULLUP); return digitalRead(sdaPin); }

void i2cStart() {
  sdaHigh(); sclHigh();
  sdaLow();
  sclLow();
}

void i2cStop() {
  sdaLow();
  sclHigh();
  sdaHigh();
}

bool i2cWriteByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) sdaHigh();
    else sdaLow();
    sclHigh();
    sclLow();
  }
  sdaHigh();
  sclHigh();
  bool ack = !sdaRead();  // ACK = SDA low
  sclLow();
  return ack;
}

bool i2cProbe(uint8_t addr7) {
  i2cStart();
  bool ack = i2cWriteByte((uint8_t)(addr7 << 1));  // write mode
  i2cStop();
  return ack;
}

const __FlashStringHelper* pinName(uint8_t p) {
  switch (p) {
    case A0: return F("A0");
    case A1: return F("A1");
    case A2: return F("A2");
    case A3: return F("A3");
    case A4: return F("A4");
    case A5: return F("A5");
    case A6: return F("A6");
    case A7: return F("A7");
    default: return nullptr;
  }
}

void printPin(uint8_t p) {
  const __FlashStringHelper* n = pinName(p);
  if (n) Serial.print(n);
  else Serial.print(p);
}

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println(F("MULTI-PIN I2C SCANNER"));
}

void loop() {
  bool any = false;
  for (uint8_t i = 0; i < sizeof(PAIRS) / sizeof(PAIRS[0]); i++) {
    sdaPin = PAIRS[i][0];
    sclPin = PAIRS[i][1];
    sdaHigh();
    sclHigh();
    delay(5);

    uint8_t found[8];
    uint8_t n = 0;
    for (uint8_t addr = 1; addr < 127 && n < 8; addr++) {
      if (i2cProbe(addr)) {
        found[n++] = addr;
      }
    }

    Serial.print(F("SDA="));
    printPin(sdaPin);
    Serial.print(F(" SCL="));
    printPin(sclPin);
    Serial.print(F(" -> "));
    if (n == 0) {
      Serial.println(F("none"));
    } else {
      any = true;
      for (uint8_t j = 0; j < n; j++) {
        Serial.print(F("0x"));
        if (found[j] < 16) Serial.print('0');
        Serial.print(found[j], HEX);
        Serial.print(' ');
      }
      Serial.println();
      // Machine line for host parser
      Serial.print(F("HIT SDA="));
      Serial.print(sdaPin);
      Serial.print(F(" SCL="));
      Serial.print(sclPin);
      Serial.print(F(" ADDR="));
      for (uint8_t j = 0; j < n; j++) {
        if (j) Serial.print(',');
        Serial.print(F("0x"));
        Serial.print(found[j], HEX);
      }
      Serial.println();
    }
  }
  if (!any) Serial.println(F("NO_DEVICE_ANY_PAIR"));
  Serial.println(F("---"));
  delay(2500);
}
