"""Minimal VL53L0X continuous ranging (addr 0x29) — enough for telem distance_mm."""

import time

import pinout as P

# ST VL53L0X register / sequence constants (subset of Pololu / Arduino driver)
_SYSRANGE_START = 0x00
_RESULT_RANGE_STATUS = 0x14
_MSRC_CONFIG_CONTROL = 0x60
_SYSTEM_SEQUENCE_CONFIG = 0x01
_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
_SYSTEM_INTERRUPT_CLEAR = 0x0B
_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89
_I2C_SLAVE_DEVICE_ADDRESS = 0x8A
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
_MSRC_CONFIG_TIMEOUT_MACROP = 0x46
_IDENTIFICATION_MODEL_ID = 0xC0


class VL53L0X:
    def __init__(self, i2c, addr=None):
        self.i2c = i2c
        self.addr = addr if addr is not None else P.TOF_ADDR
        self.ok = False
        self.distance_mm = 0
        self.stop_variable = 0
        self._timeout_ms = 200

    def _w(self, reg, data):
        if isinstance(data, int):
            data = bytes([data & 0xFF])
        self.i2c.writeto_mem(self.addr, reg, data)

    def _r(self, reg, n=1):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _w16(self, reg, val):
        self._w(reg, bytes([(val >> 8) & 0xFF, val & 0xFF]))

    def _r16(self, reg):
        b = self._r(reg, 2)
        return (b[0] << 8) | b[1]

    def probe(self):
        try:
            mid = self._r(_IDENTIFICATION_MODEL_ID, 1)[0]
            return mid == 0xEE
        except OSError:
            return False

    def begin(self):
        """Init + continuous mode. Returns True on success."""
        try:
            if not self.probe():
                # Some clones answer at 0x29 without ID match — still try init
                try:
                    self.i2c.readfrom(self.addr, 1)
                except OSError:
                    self.ok = False
                    return False

            # Data init (Pololu-style abbreviated path)
            self._w(0x88, 0x00)
            self._w(0x80, 0x01)
            self._w(0xFF, 0x01)
            self._w(0x00, 0x00)
            self.stop_variable = self._r(0x91, 1)[0]
            self._w(0x00, 0x01)
            self._w(0xFF, 0x00)
            self._w(0x80, 0x00)

            # Disable MSRC / TCC checks that hang some boards
            msrc = self._r(_MSRC_CONFIG_CONTROL, 1)[0]
            self._w(_MSRC_CONFIG_CONTROL, msrc | 0x12)

            # Signal rate limit ~0.25 MCPS
            self._w16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x0010)

            self._w(_SYSTEM_SEQUENCE_CONFIG, 0xFF)

            # Load default tuning (minimal set)
            self._load_default_tuning()

            # Set interrupt on new sample ready
            self._w(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
            gpio = self._r(_GPIO_HV_MUX_ACTIVE_HIGH, 1)[0]
            self._w(_GPIO_HV_MUX_ACTIVE_HIGH, gpio & ~0x10)
            self._w(_SYSTEM_INTERRUPT_CLEAR, 0x01)

            # Timing budget ~33 ms
            self._w(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
            self._perform_ref_calibration()

            self.start_continuous(50)
            time.sleep_ms(40)
            mm = self.read_range_mm()
            if mm > 0 and mm < 8000:
                self.distance_mm = mm
            self.ok = True
            return True
        except OSError:
            self.ok = False
            return False

    def _load_default_tuning(self):
        # Subset of ST recommended defaults used by Pololu driver
        pairs = (
            (0xFF, 0x01),
            (0x00, 0x00),
            (0xFF, 0x00),
            (0x09, 0x00),
            (0x10, 0x00),
            (0x11, 0x00),
            (0x24, 0x01),
            (0x25, 0xFF),
            (0x75, 0x00),
            (0xFF, 0x01),
            (0x4E, 0x2C),
            (0x48, 0x00),
            (0x30, 0x20),
            (0xFF, 0x00),
            (0x30, 0x09),
            (0x54, 0x00),
            (0x31, 0x04),
            (0x32, 0x03),
            (0x40, 0x83),
            (0x46, 0x25),
            (0x60, 0x00),
            (0x27, 0x00),
            (0x50, 0x06),
            (0x51, 0x00),
            (0x52, 0x96),
            (0x56, 0x08),
            (0x57, 0x30),
            (0x61, 0x00),
            (0x62, 0x00),
            (0x64, 0x00),
            (0x65, 0x00),
            (0x66, 0xA0),
            (0xFF, 0x01),
            (0x22, 0x32),
            (0x47, 0x14),
            (0x49, 0xFF),
            (0x4A, 0x00),
            (0xFF, 0x00),
            (0x7A, 0x0A),
            (0x7B, 0x00),
            (0x78, 0x21),
            (0xFF, 0x01),
            (0x23, 0x34),
            (0x42, 0x00),
            (0x44, 0xFF),
            (0x45, 0x26),
            (0x46, 0x05),
            (0x40, 0x40),
            (0x0E, 0x06),
            (0x20, 0x1A),
            (0x43, 0x40),
            (0xFF, 0x00),
            (0x34, 0x03),
            (0x35, 0x44),
            (0xFF, 0x01),
            (0x31, 0x04),
            (0x4B, 0x09),
            (0x4C, 0x05),
            (0x4D, 0x04),
            (0xFF, 0x00),
            (0x44, 0x00),
            (0x45, 0x20),
            (0x47, 0x08),
            (0x48, 0x28),
            (0x67, 0x00),
            (0x70, 0x04),
            (0x71, 0x01),
            (0x72, 0xFE),
            (0x76, 0x00),
            (0x77, 0x00),
            (0xFF, 0x01),
            (0x0D, 0x01),
            (0xFF, 0x00),
            (0x80, 0x01),
            (0x01, 0xF8),
            (0xFF, 0x01),
            (0x8E, 0x01),
            (0x00, 0x01),
            (0xFF, 0x00),
            (0x80, 0x00),
        )
        for reg, val in pairs:
            self._w(reg, val)

    def _perform_ref_calibration(self):
        self._w(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._perform_single_ref_calibration(0x40)
        self._w(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._perform_single_ref_calibration(0x00)
        self._w(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    def _perform_single_ref_calibration(self, vhv_init_byte):
        self._w(_SYSRANGE_START, 0x01 | vhv_init_byte)
        t0 = time.ticks_ms()
        while (self._r(_RESULT_RANGE_STATUS, 1)[0] & 0x01) == 0:
            if time.ticks_diff(time.ticks_ms(), t0) > self._timeout_ms:
                break
        self._w(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._w(_SYSRANGE_START, 0x00)

    def start_continuous(self, period_ms=50):
        self._w(0x80, 0x01)
        self._w(0xFF, 0x01)
        self._w(0x00, 0x00)
        self._w(0x91, self.stop_variable)
        self._w(0x00, 0x01)
        self._w(0xFF, 0x00)
        self._w(0x80, 0x00)
        if period_ms <= 0:
            self._w(_SYSRANGE_START, 0x02)  # back-to-back
        else:
            # Timed continuous — period in ms encoded via ST formula ≈ ms
            self._w16(0x04, period_ms)  # SYSTEM_INTERMEASUREMENT_PERIOD approx
            self._w(_SYSRANGE_START, 0x04)

    def read_range_mm(self):
        try:
            t0 = time.ticks_ms()
            while (self._r(_RESULT_RANGE_STATUS, 1)[0] & 0x01) == 0:
                if time.ticks_diff(time.ticks_ms(), t0) > self._timeout_ms:
                    return 0
            # range is bytes 10-11 of RESULT_RANGE_STATUS block
            buf = self._r(_RESULT_RANGE_STATUS + 10, 2)
            self._w(_SYSTEM_INTERRUPT_CLEAR, 0x01)
            mm = (buf[0] << 8) | buf[1]
            if mm >= 8190:
                return 0
            self.distance_mm = mm
            return mm
        except OSError:
            self.ok = False
            return 0

    def update(self):
        if not self.ok:
            return self.distance_mm
        return self.read_range_mm()
