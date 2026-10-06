#pragma once
/*
 * TCS3200 — deep fix
 * Bars = NodeMCU chroma scores (R/Y/G 0–100). Blue internal only.
 * Presence gate kills ambient false RED. Hard lock until conf ≤10,
 * switch when another colour clearly leads.
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

struct ColorRygPins {
  uint8_t s2, s3, out, led;
};

struct ColorRygOut {
  uint8_t label;  // 0 none · 1 RED · 2 YELLOW · 3 GREEN
  uint8_t conf;
  uint8_t r100, g100, y100;
  uint16_t rp, gp, cp;
};

class ColorRyg {
 public:
  using YieldFn = void (*)();

  void begin(const ColorRygPins &pins) {
    pins_ = pins;
    pinMode(pins_.s2, OUTPUT);
    pinMode(pins_.s3, OUTPUT);
    pinMode(pins_.out, INPUT);
    pinMode(pins_.led, OUTPUT);
    digitalWrite(pins_.led, HIGH);
    out_ = {};
    ambReady_ = false;
    emaInit_ = false;
    lockLab_ = cand_ = streak_ = switchStreak_ = 0;
    confEma_ = 0;
    gainR_ = 1.08f;
    gainG_ = 1.35f;
    gainB_ = 0.88f;
  }

  void setYield(YieldFn fn) { yieldFn_ = fn; }
  void calibrate(uint32_t) {
    ambReady_ = false;
    emaInit_ = false;
  }

  ColorRygOut update(bool /*moving*/, uint16_t distMm, bool tofOk) {
    uint32_t now = millis();
    if (now - lastMs_ < PERIOD_MS) return out_;
    lastMs_ = now;

    digitalWrite(pins_.led, HIGH);
    rp_ = readCh_(false, false);
    pump_();
    bp_ = readCh_(false, true);
    pump_();
    cp_ = readCh_(true, false);
    pump_();
    gp_ = readCh_(true, true);
    pump_();

    out_.rp = (uint16_t)rp_;
    out_.gp = (uint16_t)gp_;
    out_.cp = (uint16_t)cp_;

    bool far = tofOk && distMm >= AMB_FAR_MM;
    learnAmbient_(far);

    uint8_t r = 0, y = 0, g = 0;
    bool present = scoreChroma_(&r, &y, &g);
    out_.r100 = r;
    out_.g100 = g;
    out_.y100 = y;

    uint8_t leader = 0, best = 0, second = 0;
    if (present)
      rank3_(r, y, g, &leader, &best, &second);
    int lead = (int)best - (int)second;

    decide_(leader, best, lead, present);
    out_.label = lockLab_;
    out_.conf = (uint8_t)constrain((int)lroundf(confEma_), 0, 100);

    if (logEnable_ && now - lastLogMs_ >= 300) {
      lastLogMs_ = now;
      const char *nm = lockLab_ == 1 ? "RED" : lockLab_ == 2 ? "YELLOW" : lockLab_ == 3 ? "GREEN" : "—";
      Serial.printf(
          "DEC=%s conf=%u present=%u | bars R=%u Y=%u G=%u lead=%d | µs R=%u G=%u B=%u C=%u | chroma R=%.2f G=%.2f B=%.2f\n",
          nm, (unsigned)out_.conf, (unsigned)present,
          (unsigned)r, (unsigned)y, (unsigned)g, lead,
          (unsigned)rp_, (unsigned)gp_, (unsigned)bp_, (unsigned)cp_,
          emaR_, emaG_, emaB_);
    }
    return out_;
  }

  void setLog(bool on) { logEnable_ = on; }
  bool logEnabled() const { return logEnable_; }

 private:
  bool logEnable_ = true;
  static const uint8_t SAMPLES = 7;
  static const uint32_t PULSE_TO = 25000UL;
  static const uint32_t PERIOD_MS = 70;
  static const uint16_t AMB_FAR_MM = 100;
  static const uint8_t MIN_SCORE = 24;   /* chroma bars — need real block */
  static const uint8_t CLEAR_LEAD = 5;
  static const uint8_t LOCK_N = 3;
  static const uint8_t SWITCH_LEAD = 6;
  static const uint8_t SWITCH_N = 4;
  static const uint8_t UNLOCK_CONF = 10;

  ColorRygPins pins_{};
  YieldFn yieldFn_ = nullptr;
  ColorRygOut out_{};
  uint32_t rp_ = 0, gp_ = 0, bp_ = 0, cp_ = 0;
  uint32_t lastMs_ = 0, lastLogMs_ = 0;
  float ambRp_ = 0, ambGp_ = 0, ambBp_ = 0, ambCp_ = 0;
  bool ambReady_ = false;
  float gainR_ = 1.08f, gainG_ = 1.35f, gainB_ = 0.88f;
  float emaR_ = 0, emaG_ = 0, emaB_ = 0, emaC_ = 0;
  bool emaInit_ = false;
  uint8_t lockLab_ = 0, cand_ = 0, streak_ = 0, switchStreak_ = 0;
  float confEma_ = 0;

  static float inv_(uint32_t p) { return p ? 1000.0f / (float)p : 0; }

  static float hueDist_(float h, float c) {
    float d = fabsf(h - c);
    if (d > 180.0f) d = 360.0f - d;
    return d;
  }

  static uint8_t scorePct_(float sc) {
    /* NodeMCU scores typically 0..~1.2 → map to 0..100 */
    int p = (int)lroundf(sc * 85.0f);
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return (uint8_t)p;
  }

  void learnAmbient_(bool far) {
    /* Empty-field trim: far, or unlocked with weak clear (no object) */
    float cGain = (cp_ > 0 && ambCp_ > 1.0f) ? (ambCp_ / (float)cp_) : 1.0f;
    bool empty = !lockLab_ && cGain < 1.04f;
    if (!ambReady_ || far || empty) {
      float a = ambReady_ ? (far ? 0.06f : 0.03f) : 0.40f;
      ambRp_ = ambReady_ ? ambRp_ * (1 - a) + rp_ * a : (float)rp_;
      ambGp_ = ambReady_ ? ambGp_ * (1 - a) + gp_ * a : (float)gp_;
      ambBp_ = ambReady_ ? ambBp_ * (1 - a) + bp_ * a : (float)bp_;
      ambCp_ = ambReady_ ? ambCp_ * (1 - a) + cp_ * a : (float)cp_;
      ambReady_ = true;
      if (!lockLab_) {
        float ri = inv_(rp_), gi = inv_(gp_), bi = inv_(bp_);
        float peak = fmaxf(ri, fmaxf(gi, bi));
        if (peak > 1e-6f) {
          float ga = 0.08f;
          gainR_ = (1 - ga) * gainR_ + ga * (peak / fmaxf(ri, 1e-6f));
          gainG_ = (1 - ga) * gainG_ + ga * (peak / fmaxf(gi, 1e-6f));
          gainB_ = (1 - ga) * gainB_ + ga * (peak / fmaxf(bi, 1e-6f));
          gainR_ = fminf(1.45f, fmaxf(0.85f, gainR_));
          gainG_ = fminf(1.80f, fmaxf(1.05f, gainG_));
          gainB_ = fminf(1.15f, fmaxf(0.70f, gainB_));
        }
      }
    }
  }

  /* Returns present. Fills R/Y/G bar scores 0–100 from NodeMCU chroma. */
  bool scoreChroma_(uint8_t *rOut, uint8_t *yOut, uint8_t *gOut) {
    *rOut = *yOut = *gOut = 0;
    if (!(rp_ || gp_ || bp_)) return false;

    float ri = inv_(rp_) * gainR_;
    float gi = inv_(gp_) * gainG_;
    float bi = inv_(bp_) * gainB_;
    float ci = inv_(cp_);
    float peak = fmaxf(ri, fmaxf(gi, bi));
    if (peak < 1e-9f) peak = 1;
    float R = ri / peak, G = gi / peak, B = bi / peak;

    if (!emaInit_) {
      emaR_ = R;
      emaG_ = G;
      emaB_ = B;
      emaC_ = ci;
      emaInit_ = true;
    } else {
      float a = 0.40f;
      emaR_ = a * R + (1 - a) * emaR_;
      emaG_ = a * G + (1 - a) * emaG_;
      emaB_ = a * B + (1 - a) * emaB_;
      emaC_ = a * ci + (1 - a) * emaC_;
    }

    float mx = fmaxf(emaR_, fmaxf(emaG_, emaB_));
    float mn = fminf(emaR_, fminf(emaG_, emaB_));
    float v = mx;
    float d = mx - mn;
    float s = mx < 1e-6f ? 0 : d / mx;
    float avgPulse = (rp_ + gp_ + bp_) / 3.0f;

    /* Presence: clear MUST brighten vs ambient — kills empty-field false locks */
    float cGain = (cp_ > 0 && ambCp_ > 1.0f) ? (ambCp_ / (float)cp_) : 1.0f;
    bool weak = (emaC_ < 0.009f || avgPulse > 185.0f || v < 0.10f || s < 0.08f);
    if (weak || cGain < 1.12f) return false;
    bool present = true;
    (void)present;

    float h = 0;
    if (s >= 1e-4f) {
      if (mx == emaR_) h = 60.0f * fmodf(((emaG_ - emaB_) / d) + 6.0f, 6.0f);
      else if (mx == emaG_) h = 60.0f * (((emaB_ - emaR_) / d) + 2.0f);
      else h = 60.0f * (((emaR_ - emaG_) / d) + 4.0f);
    }

    float rDom = emaR_ - fmaxf(emaG_, emaB_);
    float gDom = emaG_ - fmaxf(emaR_, emaB_);
    float yPair = fminf(emaR_, emaG_) - emaB_;
    float yBal = 1.0f - fminf(1.0f, fabsf(emaR_ - emaG_) / 0.42f);
    if (yBal < 0) yBal = 0;

    float sR = fmaxf(0.0f, rDom) * 2.5f + fmaxf(0.0f, emaR_ - emaG_) * 0.8f + fmaxf(0.0f, emaR_ - emaB_) * 0.5f;
    if (h < 22.0f || h >= 340.0f) sR += 0.40f * s;
    else if (h < 38.0f) sR += 0.18f * s;

    float sG = fmaxf(0.0f, gDom) * 3.1f + fmaxf(0.0f, emaG_ - emaR_) * 1.15f + fmaxf(0.0f, emaG_ - emaB_) * 0.7f;
    if (h >= 65.0f && h <= 175.0f)
      sG += 0.55f * s * fmaxf(0.0f, 1.0f - hueDist_(h, 125.0f) / 70.0f);
    else if (h >= 50.0f && h < 65.0f)
      sG += 0.18f * s;

    /* Yellow: NodeMCU yPair vs B — this is what worked before */
    float sY = fmaxf(0.0f, yPair) * 2.8f * fmaxf(0.30f, yBal);
    if (h >= 28.0f && h <= 90.0f)
      sY += 0.50f * s * fmaxf(0.0f, 1.0f - hueDist_(h, 52.0f) / 42.0f);
    if (fabsf(emaR_ - emaG_) < 0.22f && yPair > 0.03f) sY += 0.22f;
    if (rDom > 0.16f) sY *= 0.45f;
    if (gDom > 0.14f) sY *= 0.35f;
    if (fminf(emaR_, emaG_) < 0.48f) sY *= 0.50f;

    float sat = (s - 0.05f) / 0.45f;
    if (sat < 0) sat = 0;
    if (sat > 1) sat = 1;
    float k = 0.38f + 0.62f * sat;
    sR *= k;
    sY *= k;
    sG *= k;

    *rOut = scorePct_(sR);
    *yOut = scorePct_(sY);
    *gOut = scorePct_(sG);
    return (*rOut >= 8 || *yOut >= 8 || *gOut >= 8);
  }

  static void rank3_(uint8_t r, uint8_t y, uint8_t g, uint8_t *leader, uint8_t *best, uint8_t *second) {
    *leader = 0;
    *best = 0;
    *second = 0;
    auto put = [&](uint8_t lab, uint8_t sc) {
      if (sc > *best) {
        *second = *best;
        *best = sc;
        *leader = lab;
      } else if (sc > *second) {
        *second = sc;
      }
    };
    put(1, r);
    put(2, y);
    put(3, g);
  }

  uint8_t lockedScore_() const {
    if (lockLab_ == 1) return out_.r100;
    if (lockLab_ == 2) return out_.y100;
    if (lockLab_ == 3) return out_.g100;
    return 0;
  }

  void decide_(uint8_t leader, uint8_t best, int lead, bool present) {
    if (lockLab_) {
      uint8_t sc = lockedScore_();
      bool stillLead = present && (leader == lockLab_);
      bool overtake = present && leader != 0 && leader != lockLab_ && best >= MIN_SCORE &&
                      (int)best >= (int)sc + (int)SWITCH_LEAD && lead >= (int)CLEAR_LEAD;

      if (overtake) {
        if (++switchStreak_ >= SWITCH_N) {
          lockLab_ = leader;
          confEma_ = fmaxf(60.0f, (float)best);
          switchStreak_ = 0;
          cand_ = streak_ = 0;
          return;
        }
      } else {
        switchStreak_ = 0;
      }

      float target = 0;
      if (stillLead && sc >= MIN_SCORE) {
        target = (float)sc;
        if (target < 20) target = 20;
        if (target > 100) target = 100;
      } else if (present && sc >= 10) {
        target = (float)sc * 0.4f;
      } else {
        target = 0; /* no presence → conf falls to unlock */
      }

      float a = (target < confEma_) ? 0.30f : 0.45f;
      confEma_ = (1.0f - a) * confEma_ + a * target;

      if (confEma_ <= (float)UNLOCK_CONF) {
        lockLab_ = cand_ = streak_ = switchStreak_ = 0;
        confEma_ = 0;
        emaInit_ = false;
      }
      return;
    }

    switchStreak_ = 0;
    if (!present || leader == 0 || best < MIN_SCORE || lead < (int)CLEAR_LEAD) {
      cand_ = 0;
      streak_ = 0;
      confEma_ *= 0.5f;
      if (confEma_ < 1.0f) confEma_ = 0;
      return;
    }

    if (leader == cand_)
      streak_++;
    else {
      cand_ = leader;
      streak_ = 1;
    }
    confEma_ = (float)streak_ * (100.0f / (float)LOCK_N);
    if (confEma_ > 99) confEma_ = 99;

    if (streak_ >= LOCK_N) {
      lockLab_ = leader;
      confEma_ = fmaxf(60.0f, (float)best);
      streak_ = LOCK_N;
    }
  }

  void pump_() {
    if (yieldFn_)
      yieldFn_();
    else
      yield();
  }

  static int cmpU32_(const void *a, const void *b) {
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return (x > y) - (x < y);
  }

  uint32_t medianPulse_() {
    uint32_t buf[SAMPLES];
    uint8_t n = 0;
    for (uint8_t i = 0; i < SAMPLES; i++) {
      uint32_t t = pulseIn(pins_.out, LOW, PULSE_TO);
      if (!t) t = pulseIn(pins_.out, HIGH, PULSE_TO);
      if (t) buf[n++] = t;
      pump_();
    }
    if (!n) return 0;
    qsort(buf, n, sizeof(uint32_t), cmpU32_);
    return buf[n / 2];
  }

  uint32_t readCh_(bool s2, bool s3) {
    digitalWrite(pins_.s2, s2 ? HIGH : LOW);
    digitalWrite(pins_.s3, s3 ? HIGH : LOW);
    delay(3);
    pump_();
    (void)pulseIn(pins_.out, LOW, 5000UL);
    pump_();
    return medianPulse_();
  }
};
