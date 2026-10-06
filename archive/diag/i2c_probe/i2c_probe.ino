/*
 * Meowler — I2C bus probe ONLY (no arm/drive/protobuf)
 *
 * FQBN: arduino:avr:nano:cpu=atmega328old
 * Serial: 115200 8N1  (Serial Monitor)
 *
 * Wiring under test: Nano A4=SDA, A5=SCL
 * Expected: PCA9685 @0x40, VL53L0X @0x29 (all-call 0x70 ok)
 */

#include <Wire.h>

static const uint8_t ADDR_PCA = 0x40;
static const uint8_t ADDR_TOF = 0x29;
static const uint8_t ADDR_ALLCALL = 0x70;

static void busRecover() {
  pinMode(A4, INPUT_PULLUP);  // SDA
  pinMode(A5, OUTPUT);        // SCL
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(A5, HIGH);
    delayMicroseconds(5);
    digitalWrite(A5, LOW);
    delayMicroseconds(5);
  }
  pinMode(A4, OUTPUT);
  digitalWrite(A4, LOW);
  delayMicroseconds(5);
  digitalWrite(A5, HIGH);
  delayMicroseconds(5);
  digitalWrite(A4, HIGH);
  delayMicroseconds(5);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
}

static const char *wireErr(uint8_t e) {
  switch (e) {
    case 0: return "OK";
    case 1: return "too_long";
    case 2: return "NACK_addr";
    case 3: return "NACK_data";
    case 4: return "other";
    case 5: return "timeout";
    default: return "?";
  }
}

static uint8_t probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

static bool read8(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  val = Wire.read();
  return true;
}

static void printAddr(uint8_t a) {
  Serial.print(F("0x"));
  if (a < 16) Serial.print('0');
  Serial.print(a, HEX);
}

static void identify(uint8_t a) {
  if (a == ADDR_PCA) Serial.print(F("  PCA9685"));
  else if (a == ADDR_TOF) Serial.print(F("  VL53L0X"));
  else if (a == ADDR_ALLCALL) Serial.print(F("  PCA all-call"));
}

static uint8_t scanBus() {
  uint8_t n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    uint8_t e = probe(a);
    if (e == 0) {
      Serial.print(F("  FOUND "));
      printAddr(a);
      identify(a);
      Serial.println();
      n++;
    } else if (e == 5) {
      Serial.print(F("  TIMEOUT "));
      printAddr(a);
      Serial.println();
    }
  }
  return n;
}

static void checkLines() {
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  delay(2);
  Serial.print(F("float  SDA="));
  Serial.print(digitalRead(A4));
  Serial.print(F(" SCL="));
  Serial.println(digitalRead(A5));

  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(2);
  int sda = digitalRead(A4);
  int scl = digitalRead(A5);
  Serial.print(F("pullup SDA="));
  Serial.print(sda);
  Serial.print(F(" SCL="));
  Serial.println(scl);
  if (sda == 0 || scl == 0) Serial.println(F("FAULT: line stuck LOW"));
  else Serial.println(F("idle HIGH OK"));
}

static void probeKnown(uint32_t hz) {
  Serial.print(F("--- clock "));
  Serial.print(hz);
  Serial.println(F(" Hz ---"));
  Wire.setClock(hz);

  uint8_t ep = probe(ADDR_PCA);
  Serial.print(F("PCA 0x40 -> "));
  Serial.println(wireErr(ep));
  if (ep == 0) {
    uint8_t mode1 = 0, presc = 0;
    if (read8(ADDR_PCA, 0x00, mode1)) {
      Serial.print(F("  MODE1=0x"));
      Serial.println(mode1, HEX);
    } else {
      Serial.println(F("  MODE1 read FAIL"));
    }
    if (read8(ADDR_PCA, 0xFE, presc)) {
      Serial.print(F("  PRESCALE=0x"));
      Serial.print(presc, HEX);
      // freq ≈ 25e6 / (4096 * (prescale+1))
      uint32_t f = 25000000UL / (4096UL * ((uint32_t)presc + 1UL));
      Serial.print(F("  (~"));
      Serial.print(f);
      Serial.println(F(" Hz)"));
    }
  }

  uint8_t et = probe(ADDR_TOF);
  Serial.print(F("TOF 0x29 -> "));
  Serial.println(wireErr(et));
}

void setup() {
  Serial.begin(115200);
  delay(400);
  while (Serial.available()) Serial.read();

  Serial.println();
  Serial.println(F("======== MEOWLER I2C PROBE ========"));
  Serial.print(F("SDA=A4="));
  Serial.print(A4);
  Serial.print(F("  SCL=A5="));
  Serial.println(A5);

  checkLines();
  Serial.println(F("bus recover..."));
  busRecover();

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(50000, false);
#endif

  const uint32_t speeds[] = {100000UL, 50000UL, 10000UL};
  for (uint8_t i = 0; i < 3; i++) {
    Wire.setClock(speeds[i]);
    Serial.print(F("SCAN @ "));
    Serial.print(speeds[i]);
    Serial.println(F(" Hz"));
    uint8_t n = scanBus();
    Serial.print(F("  devices="));
    Serial.println(n);
    probeKnown(speeds[i]);
    Serial.println();
  }

  Serial.println(F("Done. Rescan every 3s in loop."));
  Serial.println(F("==================================="));
}

void loop() {
  delay(3000);
  busRecover();
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(50000, false);
#endif
  Serial.println(F("--- rescan 100k ---"));
  uint8_t n = scanBus();
  Serial.print(F("devices="));
  Serial.println(n);
  probeKnown(100000UL);
}
