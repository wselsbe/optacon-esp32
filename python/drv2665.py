class DRV2665:
    """DRV2665 piezo driver IC — I2C register interface."""

    ADDR = 0x59

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
    RESET          = 0x80
    STANDBY        = 0x40
    TIMEOUT_5MS    = 0x00
    TIMEOUT_10MS   = 0x04
    TIMEOUT_15MS   = 0x08
    TIMEOUT_20MS   = 0x0C
    EN_OVERRIDE    = 0x02

    def __init__(self, i2c):
        self.i2c = i2c
        self._buf1 = bytearray(1)
        self._buf2 = bytearray(2)
        # Verify device is present
        status = self.read_reg(self.REG_STATUS)
        if status is None:
            raise OSError(f"DRV2665 not found at 0x{self.ADDR:02X}")

    def read_reg(self, reg):
        self._buf1[0] = reg
        self.i2c.writeto(self.ADDR, self._buf1)
        self.i2c.readfrom_into(self.ADDR, self._buf1)
        return self._buf1[0]

    def write_reg(self, reg, val):
        self._buf2[0] = reg
        self._buf2[1] = val
        self.i2c.writeto(self.ADDR, self._buf2)

    def init_digital(self, gain=GAIN_100):
        """Configure for digital FIFO mode (datasheet 8.3.1)."""
        self.write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)
        self.write_reg(self.REG_CTRL1, self.INPUT_DIGITAL | gain)
        self.write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)

    def init_analog(self, gain=GAIN_100):
        """Configure for analog input mode (datasheet 8.3.1)."""
        self.write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)
        self.write_reg(self.REG_CTRL1, self.INPUT_ANALOG | gain)
        self.write_reg(self.REG_CTRL2, self.EN_OVERRIDE | self.TIMEOUT_20MS)

    def standby(self):
        """Enter standby mode."""
        self.write_reg(self.REG_CTRL2, self.STANDBY)

    def status(self):
        """Read STATUS register."""
        return self.read_reg(self.REG_STATUS)
