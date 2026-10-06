/*
 * N20 + L298N serial control — Nano (old bootloader)
 *
 * L298N-A: IN1=8 IN2=7 IN3=4 IN4=2  ENA=3 ENB=5
 * L298N-B: IN1=10 IN2=12 IN3=13 IN4=A0 ENA=6 ENB=9
 *
 * Lines (115200):
 *   M <0-3> fwd|rev|stop [speed 0-255]
 *   ALL fwd|rev|stop [speed]
 *   STATUS
 */

struct Motor {
  uint8_t inA;
  uint8_t inB;
  uint8_t en;
  uint8_t speed;
  int8_t dir;  // -1 rev, 0 stop, +1 fwd
};

static Motor motors[4] = {
    {8, 7, 3, 0, 0},
    {4, 2, 5, 0, 0},
    {10, 12, 6, 0, 0},
    {13, A0, 9, 0, 0},
};

static char lineBuf[48];
static uint8_t lineLen = 0;

void applyMotor(uint8_t i) {
  if (i >= 4) return;
  Motor &m = motors[i];
  if (m.dir > 0) {
    digitalWrite(m.inA, HIGH);
    digitalWrite(m.inB, LOW);
    analogWrite(m.en, m.speed);
  } else if (m.dir < 0) {
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, HIGH);
    analogWrite(m.en, m.speed);
  } else {
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, LOW);
    analogWrite(m.en, 0);
  }
}

int8_t parseDir(const char *s) {
  if (!strcmp(s, "fwd") || !strcmp(s, "f") || !strcmp(s, "forward")) return 1;
  if (!strcmp(s, "rev") || !strcmp(s, "r") || !strcmp(s, "back") || !strcmp(s, "reverse")) return -1;
  if (!strcmp(s, "stop") || !strcmp(s, "s") || !strcmp(s, "off")) return 0;
  return 127;  // invalid
}

void printStatus() {
  Serial.print(F("STATUS"));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F(" m"));
    Serial.print(i);
    Serial.print('=');
    if (motors[i].dir > 0) Serial.print(F("fwd"));
    else if (motors[i].dir < 0) Serial.print(F("rev"));
    else Serial.print(F("stop"));
    Serial.print('@');
    Serial.print(motors[i].speed);
  }
  Serial.println();
}

void handleLine(char *line) {
  // trim
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;

  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "STATUS") || !strcmp(cmd, "status")) {
    printStatus();
    return;
  }

  if (!strcmp(cmd, "ALL") || !strcmp(cmd, "all")) {
    char *d = strtok(NULL, " \t");
    char *sp = strtok(NULL, " \t");
    if (!d) {
      Serial.println(F("ERR ALL needs dir"));
      return;
    }
    int8_t dir = parseDir(d);
    if (dir == 127) {
      Serial.println(F("ERR bad dir"));
      return;
    }
    int speed = 180;
    if (sp) speed = atoi(sp);
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;
    if (dir == 0) speed = 0;
    for (uint8_t i = 0; i < 4; i++) {
      motors[i].dir = dir;
      motors[i].speed = (uint8_t)speed;
      applyMotor(i);
    }
    printStatus();
    return;
  }

  if (!strcmp(cmd, "M") || !strcmp(cmd, "m")) {
    char *idS = strtok(NULL, " \t");
    char *d = strtok(NULL, " \t");
    char *sp = strtok(NULL, " \t");
    if (!idS || !d) {
      Serial.println(F("ERR M <id> <dir> [speed]"));
      return;
    }
    int id = atoi(idS);
    if (id < 0 || id > 3) {
      Serial.println(F("ERR id 0-3"));
      return;
    }
    int8_t dir = parseDir(d);
    if (dir == 127) {
      Serial.println(F("ERR bad dir"));
      return;
    }
    int speed = motors[id].speed;
    if (speed == 0) speed = 180;
    if (sp) speed = atoi(sp);
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;
    if (dir == 0) speed = 0;
    motors[id].dir = dir;
    motors[id].speed = (uint8_t)speed;
    applyMotor(id);
    printStatus();
    return;
  }

  Serial.println(F("ERR unknown cmd"));
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motors[i].inA, OUTPUT);
    pinMode(motors[i].inB, OUTPUT);
    pinMode(motors[i].en, OUTPUT);
    motors[i].dir = 0;
    motors[i].speed = 0;
    applyMotor(i);
  }
  delay(200);
  Serial.println(F("N20 READY"));
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen) handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }
}
