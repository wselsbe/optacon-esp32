import pz_drive

# WARNING: Heavy I2C access (tight loops of _read_reg/_write_reg) during
# active FIFO playback may cause audio glitches. The DRV2665 FIFO holds
# 100 samples (12.5 ms at 8 kHz), so occasional reads are fine, but
# avoid burst access patterns.


class DRV2665:
    """DRV2665 piezo driver IC — I2C register interface.

    Delegates all I2C access to pz_drive C module.
    """

    # Registers
    REG_STATUS = 0x00
    REG_CTRL1  = 0x01
    REG_CTRL2  = 0x02
    REG_DATA   = 0x0B

    # Status bits
    FIFO_FULL  = 0x01
    FIFO_EMPTY = 0x02

    # CTRL1: input mode (bit 2) + gain (bits 1:0)
    INPUT_DIGITAL = 0x00
    INPUT_ANALOG  = 0x04
    GAIN_25  = 0x00
    GAIN_50  = 0x01
    GAIN_75  = 0x02
    GAIN_100 = 0x03

    # CTRL2 bits
    STANDBY        = 0x40
    TIMEOUT_5MS    = 0x00
    TIMEOUT_10MS   = 0x04
    TIMEOUT_15MS   = 0x08
    TIMEOUT_20MS   = 0x0C
    EN_OVERRIDE    = 0x02

    def __init__(self):
        # Verify device is present (pz_drive owns I2C bus)
        val = self._read_reg(self.REG_STATUS)
        if val < 0:
            raise RuntimeError("DRV2665 not responding")

    def _read_reg(self, reg):
        return pz_drive.i2c_read(reg)

    def _write_reg(self, reg, val):
        pz_drive.i2c_write(reg, val)

    def init_digital(self, gain=GAIN_100):
        """Configure for digital FIFO mode (datasheet 8.3.1)."""
        self._write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)
        self._write_reg(self.REG_CTRL1, self.INPUT_DIGITAL | gain)
        self._write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)

    def init_analog(self, gain=GAIN_100):
        """Configure for analog input mode (datasheet 8.3.1)."""
        self._write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)
        self._write_reg(self.REG_CTRL1, self.INPUT_ANALOG | gain)
        self._write_reg(self.REG_CTRL2, self.EN_OVERRIDE | self.TIMEOUT_20MS)

    def standby(self):
        """Enter standby mode."""
        self._write_reg(self.REG_CTRL2, self.STANDBY)

    def status(self):
        """Read STATUS register."""
        return self._read_reg(self.REG_STATUS)
