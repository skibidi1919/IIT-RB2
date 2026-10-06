#include "meow_i2c.h"

namespace meow_i2c {

static Err g_err = OK;

void begin(uint32_t hz, uint32_t timeout_us) {
  // Clear any stuck transaction from a prior reset mid-transfer
  recover();
  Wire.begin();
  Wire.setClock(hz);
  // Do not auto-reset TWI on timeout — that can leave the next xfer wedged
  Wire.setWireTimeout(timeout_us, false);
  Wire.clearWireTimeoutFlag();
  g_err = OK;
}

void setClock(uint32_t hz) { Wire.setClock(hz); }

void setTimeout(uint32_t timeout_us) {
  Wire.setWireTimeout(timeout_us, false);
  Wire.clearWireTimeoutFlag();
}

Err lastError() { return g_err; }

const char *errName(Err e) {
  switch (e) {
    case OK: return "OK";
    case NACK_ADDR: return "NACK_ADDR";
    case NACK_DATA: return "NACK_DATA";
    case OTHER: return "OTHER";
    case TIMEOUT: return "TIMEOUT";
    case SHORT_READ: return "SHORT_READ";
    default: return "?";
  }
}

static Err mapWireEnd(uint8_t e) {
  switch (e) {
    case 0: return OK;
    case 2: return NACK_ADDR;
    case 3: return NACK_DATA;
    case 5: return TIMEOUT;
    default: return OTHER;
  }
}

void recover() {
  // A4=SDA A5=SCL on Nano — clock out a stuck slave, then STOP
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
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

bool probe(uint8_t addr7) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(addr7);
  uint8_t e = Wire.endTransmission();
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    g_err = TIMEOUT;
    recover();
    Wire.begin();
    return false;
  }
  g_err = mapWireEnd(e);
  return g_err == OK;
}

uint8_t scan(uint8_t *out, uint8_t max) {
  uint8_t n = 0;
  for (uint8_t a = 0x08; a < 0x78 && n < max; a++) {
    if (probe(a)) out[n++] = a;
  }
  return n;
}

bool writeReg(uint8_t addr7, uint8_t reg, uint8_t val) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  Wire.write(val);
  uint8_t e = Wire.endTransmission();
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    g_err = TIMEOUT;
    recover();
    Wire.begin();
    return false;
  }
  g_err = mapWireEnd(e);
  return g_err == OK;
}

bool writeRegs(uint8_t addr7, uint8_t reg, const uint8_t *data, uint8_t n) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  for (uint8_t i = 0; i < n; i++) Wire.write(data[i]);
  uint8_t e = Wire.endTransmission();
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    g_err = TIMEOUT;
    recover();
    Wire.begin();
    return false;
  }
  g_err = mapWireEnd(e);
  return g_err == OK;
}

bool readReg(uint8_t addr7, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  g_err = mapWireEnd(Wire.endTransmission(false));  // repeated start
  if (g_err != OK) {
    if (g_err == TIMEOUT) recover();
    return false;
  }
  uint8_t got = Wire.requestFrom((int)addr7, 1);
  if (got != 1) {
    g_err = Wire.getWireTimeoutFlag() ? TIMEOUT : SHORT_READ;
    if (g_err == TIMEOUT) {
      Wire.clearWireTimeoutFlag();
      recover();
    }
    return false;
  }
  val = Wire.read();
  g_err = OK;
  return true;
}

bool readRegs(uint8_t addr7, uint8_t reg, uint8_t *data, uint8_t n) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  g_err = mapWireEnd(Wire.endTransmission(false));
  if (g_err != OK) {
    if (g_err == TIMEOUT) recover();
    return false;
  }
  uint8_t got = Wire.requestFrom((int)addr7, (int)n);
  if (got != n) {
    g_err = Wire.getWireTimeoutFlag() ? TIMEOUT : SHORT_READ;
    if (g_err == TIMEOUT) {
      Wire.clearWireTimeoutFlag();
      recover();
    }
    return false;
  }
  for (uint8_t i = 0; i < n; i++) data[i] = Wire.read();
  g_err = OK;
  return true;
}

bool writeReg16BE(uint8_t addr7, uint8_t reg, uint16_t val) {
  uint8_t buf[2] = {(uint8_t)(val >> 8), (uint8_t)val};
  return writeRegs(addr7, reg, buf, 2);
}

bool readReg16BE(uint8_t addr7, uint8_t reg, uint16_t &val) {
  uint8_t buf[2];
  if (!readRegs(addr7, reg, buf, 2)) return false;
  val = (uint16_t)((buf[0] << 8) | buf[1]);
  return true;
}

}  // namespace meow_i2c
