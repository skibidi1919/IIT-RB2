/*
 * TCS3200 high-accuracy color sense — NodeMCU-32 / ESP32
 * S0=4 S1=2 S2=18 S3=19 OUT=5 LED=13  VCC=3V3 GND=GND
 * Serial 115200 — always prints; send CAL for white balance.
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t PIN_S0 = 4;
static const uint8_t PIN_S1 = 2;
static const uint8_t PIN_S2 = 18;
static const uint8_t PIN_S3 = 19;
static const uint8_t PIN_OUT = 5;
static const uint8_t PIN_LED = 13;

static const uint8_t SAMPLES = 7;
static const float EMA_ALPHA = 0.40f;
static const uint32_t PULSE_TIMEOUT_US = 25000UL;

// Default spectral trim for white LED (green photodiode weak, blue strong)
static float gainR = 1.08f, gainG = 1.35f, gainB = 0.88f;
static bool calibrated = false;
static float emaR = 0, emaG = 0, emaB = 0, emaC = 0;
static bool emaInit = false;

static int cmpU32(const void *a, const void *b) {
  uint32_t x = *(const uint32_t *)a;
  uint32_t y = *(const uint32_t *)b;
  return (x > y) - (x < y);
}

static uint32_t medianPulse() {
  uint32_t buf[SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    // Prefer LOW period; short timeout so loop never stalls for seconds
    uint32_t t = pulseIn(PIN_OUT, LOW, PULSE_TIMEOUT_US);
    if (t == 0) t = pulseIn(PIN_OUT, HIGH, PULSE_TIMEOUT_US);
    if (t > 0) buf[n++] = t;
    yield();
  }
  if (n == 0) return 0;
  qsort(buf, n, sizeof(uint32_t), cmpU32);
  return buf[n / 2];
}

static uint32_t readChannel(bool s2, bool s3) {
  digitalWrite(PIN_S2, s2 ? HIGH : LOW);
  digitalWrite(PIN_S3, s3 ? HIGH : LOW);
  delay(2);
  pulseIn(PIN_OUT, LOW, 5000UL);  // discard first edge
  return medianPulse();
}

static float invPulse(uint32_t p) {
  if (p == 0) return 0.0f;
  return 1000.0f / (float)p;
}

static void rgbToHSV(float R, float G, float B, float *h, float *s, float *v) {
  float mx = fmaxf(R, fmaxf(G, B));
  float mn = fminf(R, fminf(G, B));
  *v = mx;
  float d = mx - mn;
  *s = (mx < 1e-6f) ? 0.0f : (d / mx);
  if (*s < 1e-4f) {
    *h = 0.0f;
    return;
  }
  if (mx == R) *h = 60.0f * fmodf(((G - B) / d) + 6.0f, 6.0f);
  else if (mx == G) *h = 60.0f * (((B - R) / d) + 2.0f);
  else *h = 60.0f * (((R - G) / d) + 4.0f);
}

// Only RED / YELLOW / GREEN. Sticky when ambiguous.
static const char *stickyName = nullptr;

static float hueDist(float h, float center) {
  float d = fabsf(h - center);
  if (d > 180.0f) d = 360.0f - d;
  return d;
}

static const char *nameFromRGBH(float R, float G, float B, float h, float s, float v,
                                float clearI, float avgPulse) {
  static uint8_t weakFrames = 0;
  const bool tooDark = (clearI < 0.010f || avgPulse > 160.0f || v < 0.10f);
  const bool tooGray = (s < 0.08f);
  if (tooDark || tooGray) {
    if (++weakFrames >= 8) {
      stickyName = nullptr;
      return "NONE";
    }
    return stickyName ? stickyName : "NONE";
  }
  weakFrames = 0;

  float rDom = R - fmaxf(G, B);
  float gDom = G - fmaxf(R, B);
  float yPair = fminf(R, G) - B;
  float yBal = 1.0f - fminf(1.0f, fabsf(R - G) / 0.40f);
  if (yBal < 0.0f) yBal = 0.0f;

  float scoreR = fmaxf(0.0f, rDom) * 2.4f + fmaxf(0.0f, R - G) * 0.7f + fmaxf(0.0f, R - B) * 0.5f;
  if (h < 22.0f || h >= 340.0f) scoreR += 0.40f * s;
  else if (h < 38.0f) scoreR += 0.18f * s;

  float scoreG = fmaxf(0.0f, gDom) * 3.0f + fmaxf(0.0f, G - R) * 1.1f + fmaxf(0.0f, G - B) * 0.7f;
  if (h >= 60.0f && h <= 180.0f)
    scoreG += 0.55f * s * fmaxf(0.0f, 1.0f - hueDist(h, 125.0f) / 75.0f);
  else if (h >= 50.0f && h < 60.0f)
    scoreG += 0.20f * s;

  float scoreY = fmaxf(0.0f, yPair) * 2.0f * fmaxf(0.35f, yBal);
  if (h >= 32.0f && h <= 78.0f)
    scoreY += 0.35f * s * fmaxf(0.0f, 1.0f - hueDist(h, 52.0f) / 35.0f);
  if (rDom > 0.10f) scoreY *= 0.30f;
  if (gDom > 0.08f) scoreY *= 0.15f;
  if (fminf(R, G) < 0.55f) scoreY *= 0.40f;

  float sat = (s - 0.06f) / 0.45f;
  if (sat < 0.0f) sat = 0.0f;
  if (sat > 1.0f) sat = 1.0f;
  float k = 0.35f + 0.65f * sat;
  scoreR *= k;
  scoreY *= k;
  scoreG *= k;

  const char *best = "RED";
  float bestScore = scoreR;
  if (scoreY > bestScore) { best = "YELLOW"; bestScore = scoreY; }
  if (scoreG > bestScore) { best = "GREEN"; bestScore = scoreG; }

  if (stickyName != nullptr) {
    float stickyScore = scoreR;
    if (strcmp(stickyName, "YELLOW") == 0) stickyScore = scoreY;
    else if (strcmp(stickyName, "GREEN") == 0) stickyScore = scoreG;
    float need = (strcmp(best, "GREEN") == 0) ? 0.04f : 0.07f;
    if (strcmp(best, stickyName) != 0 && (bestScore - stickyScore) < need) {
      best = stickyName;
      bestScore = stickyScore;
    }
  }

  if (bestScore < 0.18f) return stickyName ? stickyName : "NONE";
  // Strong green unlocks immediately
  if (strcmp(best, "GREEN") == 0 && bestScore >= 0.32f && G >= R && G >= B)
    stickyName = "GREEN";
  else
    stickyName = best;
  return stickyName;
}

static void calibrateWhite(uint32_t rp, uint32_t gp, uint32_t bp) {
  float ri = invPulse(rp);
  float gi = invPulse(gp);
  float bi = invPulse(bp);
  float peak = fmaxf(ri, fmaxf(gi, bi));
  if (peak < 1e-6f) {
    Serial.println(F("CAL FAIL"));
    return;
  }
  gainR = peak / fmaxf(ri, 1e-6f);
  gainG = peak / fmaxf(gi, 1e-6f);
  gainB = peak / fmaxf(bi, 1e-6f);
  calibrated = true;
  emaInit = false;
  stickyName = nullptr;
  Serial.println(F("CAL OK"));
}

static void pollSerialCmd() {
  static char buf[24];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      if (len && (!strcasecmp(buf, "CAL") || !strcasecmp(buf, "WHITE"))) {
        uint32_t r = readChannel(false, false);
        uint32_t b = readChannel(false, true);
        uint32_t g = readChannel(true, true);
        calibrateWhite(r, g, b);
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_S0, HIGH);
  digitalWrite(PIN_S1, LOW);
  digitalWrite(PIN_LED, HIGH);

  Serial.println();
  Serial.println(F("TCS3200 READY hi-acc"));
  Serial.println(F("Send CAL on white paper"));
  Serial.flush();
}

void loop() {
  pollSerialCmd();

  uint32_t rp = readChannel(false, false);
  uint32_t bp = readChannel(false, true);
  uint32_t cp = readChannel(true, false);
  uint32_t gp = readChannel(true, true);

  // Always emit a line so GUI never stalls
  if (rp == 0 && gp == 0 && bp == 0) {
    Serial.println(F("R=0 G=0 B=0 C=0  => NO_SIGNAL"));
    Serial.flush();
    delay(150);
    return;
  }

  float ri = invPulse(rp) * gainR;
  float gi = invPulse(gp) * gainG;
  float bi = invPulse(bp) * gainB;
  float ci = invPulse(cp);

  float peak = fmaxf(ri, fmaxf(gi, bi));
  if (peak < 1e-9f) peak = 1.0f;
  float R = ri / peak;
  float G = gi / peak;
  float B = bi / peak;

  if (!emaInit) {
    emaR = R; emaG = G; emaB = B; emaC = ci;
    emaInit = true;
  } else {
    emaR = EMA_ALPHA * R + (1.0f - EMA_ALPHA) * emaR;
    emaG = EMA_ALPHA * G + (1.0f - EMA_ALPHA) * emaG;
    emaB = EMA_ALPHA * B + (1.0f - EMA_ALPHA) * emaB;
    emaC = EMA_ALPHA * ci + (1.0f - EMA_ALPHA) * emaC;
  }

  float h, s, v;
  rgbToHSV(emaR, emaG, emaB, &h, &s, &v);
  float avgPulse = (rp + gp + bp) / 3.0f;
  const char *name = nameFromRGBH(emaR, emaG, emaB, h, s, v, emaC, avgPulse);
  float conf = s * (calibrated ? 1.0f : 0.85f);
  if (v < 0.15f) conf *= 0.5f;
  int confPct = (int)constrain((int)lroundf(conf * 100.0f), 0, 99);

  Serial.print(F("R="));
  Serial.print(rp);
  Serial.print(F(" G="));
  Serial.print(gp);
  Serial.print(F(" B="));
  Serial.print(bp);
  Serial.print(F(" C="));
  Serial.print(cp);
  Serial.print(F("  => "));
  Serial.print(name);
  Serial.print(F("  h="));
  Serial.print(h, 0);
  Serial.print(F(" s="));
  Serial.print(s, 2);
  Serial.print(F(" v="));
  Serial.print(v, 2);
  Serial.print(F("  conf="));
  Serial.print(confPct);
  Serial.println(calibrated ? F("% CAL") : F("%"));
  Serial.flush();

  delay(80);
}
