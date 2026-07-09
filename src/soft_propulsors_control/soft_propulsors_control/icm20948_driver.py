"""
icm20948_driver.py — Direct smbus2 driver for the ICM-20948 9-DOF IMU
=====================================================================
A dependency-light replacement for adafruit_icm20x.  The Adafruit library
pulls in Blinka -> digitalio -> Jetson.GPIO at import time, and Jetson.GPIO
cannot identify newer boards (e.g. the Orin Nano "Super" engineering
reference kit, p3767-0005), so the whole import fails before any hardware is
touched.  This driver talks to the sensor directly over /dev/i2c-N via
smbus2 and exposes the same three properties the node consumes:

    .acceleration -> (ax, ay, az) in m/s^2
    .gyro         -> (gx, gy, gz) in rad/s
    .magnetic     -> (mx, my, mz) in microtesla (uT)

The on-die magnetometer is the AK09916, reached by putting the ICM-20948
into I2C bypass so the AK09916 appears as its own device at 0x0C on the
same bus.
"""

import math
import time


class ICM20948:
    # --- ICM-20948 register map (Bank 0 unless noted) ---
    _REG_BANK_SEL   = 0x7F
    _WHO_AM_I       = 0x00   # expect 0xEA
    _PWR_MGMT_1     = 0x06
    _PWR_MGMT_2     = 0x07
    _INT_PIN_CFG    = 0x0F
    _ACCEL_XOUT_H   = 0x2D   # 6 accel + 6 gyro bytes, big-endian, signed
    # Bank 2
    _GYRO_CONFIG_1  = 0x01
    _ACCEL_CONFIG   = 0x14
    _WHO_AM_I_VAL   = 0xEA

    # --- AK09916 magnetometer (separate I2C device, via bypass) ---
    _AK_ADDR        = 0x0C
    _AK_WIA2        = 0x01   # expect 0x09
    _AK_ST1         = 0x10   # bit0 = data ready
    _AK_HXL         = 0x11   # 6 bytes, little-endian, signed
    _AK_ST2         = 0x18   # must be read to complete a measurement
    _AK_CNTL2       = 0x31
    _AK_CNTL3       = 0x32
    _AK_WIA2_VAL    = 0x09

    # Scale factors for the ranges we configure below
    _ACCEL_FS_SEL   = 0x03   # +/-4 g  -> 8192 LSB/g
    _ACCEL_LSB_PER_G = 8192.0
    _GYRO_FS_SEL    = 0x07   # +/-2000 dps -> 16.4 LSB/dps, DLPF enabled
    _GYRO_LSB_PER_DPS = 16.4
    _MAG_UT_PER_LSB = 0.15   # AK09916: 0.15 uT/LSB
    _G_MS2          = 9.80665

    def __init__(self, bus, address=0x69):
        self._bus = bus
        self._addr = address
        self._bank = None

        if self._read(self._WHO_AM_I) != self._WHO_AM_I_VAL:
            raise RuntimeError(
                f"ICM-20948 WHO_AM_I mismatch at 0x{address:02X} "
                f"(got 0x{self._read(self._WHO_AM_I):02X}, expected 0xEA)")

        self._configure()

    # ------------------------------------------------------------------
    # Low-level bus helpers (with bank switching)
    # ------------------------------------------------------------------
    def _set_bank(self, bank):
        if bank != self._bank:
            self._bus.write_byte_data(self._addr, self._REG_BANK_SEL, bank << 4)
            self._bank = bank

    def _read(self, reg, bank=0):
        self._set_bank(bank)
        return self._bus.read_byte_data(self._addr, reg)

    def _read_block(self, reg, length, bank=0):
        self._set_bank(bank)
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    def _write(self, reg, value, bank=0):
        self._set_bank(bank)
        self._bus.write_byte_data(self._addr, reg, value)

    @staticmethod
    def _s16_be(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v >= 32768 else v

    @staticmethod
    def _s16_le(lo, hi):
        v = (hi << 8) | lo
        return v - 65536 if v >= 32768 else v

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------
    def _configure(self):
        # Reset, then wake with the best available clock source.
        self._write(self._PWR_MGMT_1, 0x80)          # device reset
        time.sleep(0.1)
        self._bank = None                            # bank resets to 0 on reset
        self._write(self._PWR_MGMT_1, 0x01)          # wake, auto clock
        time.sleep(0.01)
        self._write(self._PWR_MGMT_2, 0x00)          # accel + gyro enabled

        # Ranges + on-chip DLPF (Bank 2)
        self._write(self._GYRO_CONFIG_1, self._GYRO_FS_SEL, bank=2)
        self._write(self._ACCEL_CONFIG, self._ACCEL_FS_SEL, bank=2)
        self._set_bank(0)

        # I2C bypass so the AK09916 magnetometer is reachable at 0x0C.
        self._write(self._INT_PIN_CFG, 0x02)
        time.sleep(0.01)
        self._init_mag()

    def _init_mag(self):
        try:
            if self._bus.read_byte_data(self._AK_ADDR, self._AK_WIA2) != self._AK_WIA2_VAL:
                self._have_mag = False
                return
            self._bus.write_byte_data(self._AK_ADDR, self._AK_CNTL3, 0x01)  # soft reset
            time.sleep(0.01)
            self._bus.write_byte_data(self._AK_ADDR, self._AK_CNTL2, 0x08)  # continuous 100 Hz
            time.sleep(0.01)
            self._have_mag = True
        except OSError:
            self._have_mag = False

    # ------------------------------------------------------------------
    # Public sensor properties (units match adafruit_icm20x)
    # ------------------------------------------------------------------
    @property
    def acceleration(self):
        d = self._read_block(self._ACCEL_XOUT_H, 6)
        scale = self._G_MS2 / self._ACCEL_LSB_PER_G
        return (self._s16_be(d[0], d[1]) * scale,
                self._s16_be(d[2], d[3]) * scale,
                self._s16_be(d[4], d[5]) * scale)

    @property
    def gyro(self):
        # Accel block (0x2D) is immediately followed by the gyro block (0x33);
        # read all 12 bytes and take the gyro half.
        d = self._read_block(self._ACCEL_XOUT_H, 12)
        scale = (1.0 / self._GYRO_LSB_PER_DPS) * (math.pi / 180.0)
        return (self._s16_be(d[6], d[7]) * scale,
                self._s16_be(d[8], d[9]) * scale,
                self._s16_be(d[10], d[11]) * scale)

    @property
    def magnetic(self):
        if not getattr(self, '_have_mag', False):
            return (0.0, 0.0, 0.0)
        # ST1 gates data-ready; HXL..HZH are little-endian; ST2 must be read
        # to release the measurement register for the next sample.
        d = self._bus.read_i2c_block_data(self._AK_ADDR, self._AK_HXL, 6)
        self._bus.read_byte_data(self._AK_ADDR, self._AK_ST2)
        return (self._s16_le(d[0], d[1]) * self._MAG_UT_PER_LSB,
                self._s16_le(d[2], d[3]) * self._MAG_UT_PER_LSB,
                self._s16_le(d[4], d[5]) * self._MAG_UT_PER_LSB)
