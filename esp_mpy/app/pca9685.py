"""PCA9685 @ 150 Hz — MG90 arm CH0/1/2 + SG90-360 conveyor CH4."""

from machine import Pin, SoftI2C
import time

import pinout as P

REG_MODE1 = 0x00
REG_MODE2 = 0x01
REG_LED0 = 0x06
REG_PRESCALE = 0xFE

# ≈150 Hz: osc 25 MHz → prescale = round(25e6/(4096*150)) - 1 = 40
PCA_PRESCALE = 40
SERVO_PERIOD_US = 6667  # 1e6/150


class PCA9685:
    def __init__(self, i2c, addr=None):
        self.i2c = i2c
        self.addr = addr if addr is not None else P.PCA_ADDR
        self.ok = False
        self.base = 90
        self.height = 90
        self.grip = 90
        self.conveyor = 0  # -255..255

    def probe(self):
        try:
            self.i2c.readfrom_mem(self.addr, REG_MODE1, 1)
            return True
        except OSError:
            return False

    def begin(self):
        if not self.probe():
            self.ok = False
            return False
        try:
            self._write8(REG_MODE1, 0x10)  # sleep
            time.sleep_ms(5)
            self._write8(REG_PRESCALE, PCA_PRESCALE)
            self._write8(REG_MODE1, 0x00)  # wake
            time.sleep_ms(5)
            self._write8(REG_MODE1, 0xA1)  # AI + ALLCALL
            self._write8(REG_MODE2, 0x04)  # OUTDRV
            time.sleep_ms(2)
            if not self.probe():
                self.ok = False
                return False
            self.ok = True
            return True
        except OSError:
            self.ok = False
            return False

    def _write8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def set_pwm(self, ch, on, off):
        if not self.ok or ch > 15:
            return
        reg = REG_LED0 + 4 * ch
        buf = bytes(
            [
                on & 0xFF,
                (on >> 8) & 0xFF,
                off & 0xFF,
                (off >> 8) & 0xFF,
            ]
        )
        try:
            self.i2c.writeto_mem(self.addr, reg, buf)
        except OSError:
            self.ok = False

    def full_off(self, ch):
        self.set_pwm(ch, 0, 0x1000)

    def us_to_ticks(self, us):
        if us > SERVO_PERIOD_US - 50:
            us = SERVO_PERIOD_US - 50
        ticks = (us * 4096) // SERVO_PERIOD_US
        if ticks > 4095:
            ticks = 4095
        return ticks

    def deg_to_ticks(self, deg):
        if deg < 0:
            deg = 0
        if deg > 180:
            deg = 180
        us = P.SERVO_US_MIN + (deg * (P.SERVO_US_MAX - P.SERVO_US_MIN)) // 180
        return self.us_to_ticks(us)

    def set_servo_deg(self, ch, deg):
        self.set_pwm(ch, 0, self.deg_to_ticks(deg))

    def set_arm(self, base=None, height=None, grip=None):
        """Snap MG90 PWM then briefly hold (parity with esp_ui soft=false path)."""
        if not self.ok:
            return
        if base is not None:
            self.base = max(0, min(180, int(base)))
            self.set_servo_deg(P.CH_BASE, self.base)
        if height is not None:
            self.height = max(0, min(180, int(height)))
            self.set_servo_deg(P.CH_HEIGHT, self.height)
        if grip is not None:
            self.grip = max(0, min(180, int(grip)))
            self.set_servo_deg(P.CH_GRIP, self.grip)

    def apply_arm(self, base, height, grip, hold_ms=350):
        """MOVE → hold → relax (PWM off) — matches esp_ui applyArm snap path."""
        if not self.ok:
            return
        self.set_arm(base, height, grip)
        time.sleep_ms(hold_ms)
        self.full_off(P.CH_BASE)
        self.full_off(P.CH_HEIGHT)
        self.full_off(P.CH_GRIP)

    def center(self):
        self.apply_arm(90, 90, 90)

    def set_conveyor(self, speed):
        """SG90-360 on CH4. speed -255..255; 0 = full PWM off (stop)."""
        spd = int(speed)
        if spd > 255:
            spd = 255
        if spd < -255:
            spd = -255
        if -20 < spd < 20:
            spd = 0
        self.conveyor = spd
        if not self.ok:
            return
        if spd == 0:
            self.full_off(P.CH_CONVEYOR)
            time.sleep_us(300)
            self.full_off(P.CH_CONVEYOR)
            return
        # map -255..255 → 1000..2000 µs
        us = P.CONV_US_MIN + ((spd + 255) * (P.CONV_US_MAX - P.CONV_US_MIN)) // 510
        ticks = self.us_to_ticks(us)
        self.set_pwm(P.CH_CONVEYOR, 0, ticks)
        time.sleep_us(300)
        self.set_pwm(P.CH_CONVEYOR, 0, ticks)

    def hold_motors_off(self):
        """Quiet unused PCA channels that might hum."""
        if not self.ok:
            return
        for ch in range(4, 10):
            self.full_off(ch)


def scan_bus(i2c):
    try:
        return list(i2c.scan())
    except OSError:
        return []


def open_i2c(sda=None, scl=None, freq=None):
    sda = P.I2C_SDA if sda is None else sda
    scl = P.I2C_SCL if scl is None else scl
    freq = P.I2C_FREQ if freq is None else freq
    # Ensure FLASH LED pin is not left driving I2C
    try:
        Pin(P.PIN_FLASH_LED, Pin.OUT, value=0)
    except Exception:
        pass
    # SoftI2C: any GPIO (HW I2C pin mux varies by MicroPython build)
    return SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=freq)


def recover_i2c():
    """Re-open Wire on 21/47 and return (i2c, addrs)."""
    i2c = open_i2c()
    time.sleep_ms(20)
    addrs = scan_bus(i2c)
    return i2c, addrs
