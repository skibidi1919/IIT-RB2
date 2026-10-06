"""L298N differential drive + optional encoder poll (parity with esp_ui)."""

from machine import Pin, PWM
import math
import time

import pinout as P

MM_PER_STEP = (P.WHEEL_DIAM_MM * math.pi) / P.STEPS_PER_REV

# Quadrature LUT (same as esp_ui)
_ENC_LUT = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)


class Drive:
    def __init__(self, use_pwm=True):
        self.use_pwm = use_pwm
        self.cmd_l = 0
        self.cmd_r = 0
        self.last_drive_ms = 0
        self.enc_l = 0
        self.enc_r = 0
        self._prev_m1 = 0
        self._prev_m2 = 0
        self._in1 = Pin(P.PIN_IN1, Pin.OUT, value=0)
        self._in2 = Pin(P.PIN_IN2, Pin.OUT, value=0)
        self._in3 = Pin(P.PIN_IN3, Pin.OUT, value=0)
        self._in4 = Pin(P.PIN_IN4, Pin.OUT, value=0)
        self._ena = None
        self._enb = None
        self._ena_pin = Pin(P.PIN_ENA, Pin.OUT, value=0)
        self._enb_pin = Pin(P.PIN_ENB, Pin.OUT, value=0)
        if use_pwm:
            self._ena = PWM(self._ena_pin, freq=1000, duty=0)
            self._enb = PWM(self._enb_pin, freq=1000, duty=0)
        self._m1c1 = Pin(P.PIN_M1_C1, Pin.IN, Pin.PULL_UP)
        self._m1c2 = Pin(P.PIN_M1_C2, Pin.IN, Pin.PULL_UP)
        self._m2c1 = Pin(P.PIN_M2_C1, Pin.IN, Pin.PULL_UP)
        self._m2c2 = Pin(P.PIN_M2_C2, Pin.IN, Pin.PULL_UP)
        self._prev_m1 = (self._m1c1.value() << 1) | self._m1c2.value()
        self._prev_m2 = (self._m2c1.value() << 1) | self._m2c2.value()

    def _write_en(self, side, mag):
        # side: 'l' uses ENB, 'r' uses ENA
        duty = int(mag) & 0xFF
        # MicroPython ESP32 PWM duty is 0..1023 by default on some builds;
        # duty_u16 is more portable when available.
        pwm = self._enb if side == "l" else self._ena
        pin = self._enb_pin if side == "l" else self._ena_pin
        if self.use_pwm and pwm is not None:
            if hasattr(pwm, "duty_u16"):
                pwm.duty_u16(duty * 257)  # 0..65535
            else:
                # duty 0..1023
                pwm.duty((duty * 1023) // 255)
        else:
            pin.value(1 if duty else 0)

    def _clamp_mag(self, spd):
        if spd > 255:
            spd = 255
        if spd < -255:
            spd = -255
        mag = abs(spd)
        if mag > 0 and mag < 80:
            mag = 80
        return spd, mag

    def _apply_side(self, spd, in_a, in_b, side):
        spd, mag = self._clamp_mag(spd)
        if spd > 0:
            in_a.value(1)
            in_b.value(0)
            self._write_en(side, mag if mag else 255)
        elif spd < 0:
            in_a.value(0)
            in_b.value(1)
            self._write_en(side, mag if mag else 255)
        else:
            in_a.value(0)
            in_b.value(0)
            self._write_en(side, 0)

    def apply(self):
        # M1 left: IN3/IN4/ENB · M2 right: IN1/IN2/ENA
        self._apply_side(self.cmd_l, self._in3, self._in4, "l")
        self._apply_side(self.cmd_r, self._in1, self._in2, "r")

    def set(self, left, right):
        l = int(left)
        r = int(right)
        if l > 255:
            l = 255
        if l < -255:
            l = -255
        if r > 255:
            r = 255
        if r < -255:
            r = -255
        self.cmd_l = l
        self.cmd_r = r
        self.last_drive_ms = time.ticks_ms()
        self.apply()

    def stop(self):
        self.set(0, 0)

    def service_hold(self, timeout_ms=400):
        """Auto-stop if no fresh Drive cmd (matches esp_ui)."""
        if self.cmd_l == 0 and self.cmd_r == 0:
            return
        if time.ticks_diff(time.ticks_ms(), self.last_drive_ms) > timeout_ms:
            self.stop()

    def poll_encoders(self):
        """Software quadrature poll — call often from main loop."""
        m1 = (self._m1c1.value() << 1) | self._m1c2.value()
        idx = (self._prev_m1 << 2) | m1
        self.enc_l += _ENC_LUT[idx & 15]
        self._prev_m1 = m1
        m2 = (self._m2c1.value() << 1) | self._m2c2.value()
        idx = (self._prev_m2 << 2) | m2
        self.enc_r += _ENC_LUT[idx & 15]
        self._prev_m2 = m2

    def zero(self):
        self.enc_l = 0
        self.enc_r = 0

    def wheel_mm(self):
        return int(self.enc_l * MM_PER_STEP), int(self.enc_r * MM_PER_STEP)
