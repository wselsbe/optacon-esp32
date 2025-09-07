import time
from machine import I2C, Pin

class DRV2665:
    I2C_ADDR = 0x59
    
    # Register addresses
    REG_STATUS = 0x00
    REG_CTRL_1 = 0x01
    REG_CTRL_2 = 0x02
    REG_FIFO = 0x0B
    
    # Control 1 register bits
    CTRL1_DEV_RST = 0x80
    CTRL1_STANDBY = 0x40
    CTRL1_DIGITAL_IN = 0x00
    CTRL1_ANALOG_IN = 0x04
    
    # Control 2 register bits - gain
    CTRL2_GAIN_100VPP = 0x00
    CTRL2_GAIN_75VPP = 0x01
    CTRL2_GAIN_50VPP = 0x02
    CTRL2_GAIN_25VPP = 0x03
    
    # Control 2 register bits - timeout (bits 2-3)
    CTRL2_TIMEOUT_5MS = 0x00
    CTRL2_TIMEOUT_10MS = 0x04
    CTRL2_TIMEOUT_15MS = 0x08
    CTRL2_TIMEOUT_20MS = 0x0C
    
    # Status register bits
    STATUS_FIFO_EMPTY = 0x01
    STATUS_FIFO_FULL = 0x02
    
    # FIFO status constants
    FIFO_OK = 0
    FIFO_EMPTY = 1
    FIFO_FULL = 2
    
    def __init__(self, i2c, addr=I2C_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.init()
    
    def init(self):
        """Initialize the DRV2665 device"""
        # Reset device
        self.reset()
        time.sleep_ms(2)
        
        # Take out of standby
        ctrl1 = self.read_register(self.REG_CTRL_1)
        ctrl1 &= ~self.CTRL1_STANDBY
        self.write_register(self.REG_CTRL_1, ctrl1)
        time.sleep_ms(2)
    
    def reset(self):
        """Reset the device"""
        self.write_register(self.REG_CTRL_1, self.CTRL1_DEV_RST)
    
    def write_register(self, reg, value):
        """Write a single byte to a register"""
        self.i2c.writeto_mem(self.addr, reg, bytes([value]))
    
    def read_register(self, reg):
        """Read a single byte from a register"""
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]
    
    def write_fifo(self, data):
        """Write data bytes to FIFO, returns number of bytes written"""
        if isinstance(data, list):
            data = bytes(data)
        return self.i2c.writeto_mem(self.addr, self.REG_FIFO, data)
    
    def get_fifo_status(self):
        """Get FIFO status"""
        status = self.read_register(self.REG_STATUS)
        if status & self.STATUS_FIFO_EMPTY:
            return self.FIFO_EMPTY
        elif status & self.STATUS_FIFO_FULL:
            return self.FIFO_FULL
        else:
            return self.FIFO_OK
    
    def enable_digital(self, gain=CTRL2_GAIN_100VPP, timeout=CTRL2_TIMEOUT_10MS):
        """Enable digital input mode
        gain: CTRL2_GAIN_25VPP, CTRL2_GAIN_50VPP, CTRL2_GAIN_75VPP, or CTRL2_GAIN_100VPP
        timeout: CTRL2_TIMEOUT_5MS, CTRL2_TIMEOUT_10MS, CTRL2_TIMEOUT_15MS, or CTRL2_TIMEOUT_20MS
        """
        # Set digital input mode in CTRL1
        ctrl1 = self.read_register(self.REG_CTRL_1)
        ctrl1 &= ~0x07  # Clear input mode bits
        ctrl1 |= self.CTRL1_DIGITAL_IN
        ctrl1 &= ~self.CTRL1_STANDBY  # Ensure not in standby
        self.write_register(self.REG_CTRL_1, ctrl1)
        
        # Set gain and timeout in CTRL2
        ctrl2 = gain | timeout
        self.write_register(self.REG_CTRL_2, ctrl2)
    
    def enable_analog(self, gain=CTRL2_GAIN_100VPP):
        """Enable analog input mode
        gain: CTRL2_GAIN_25VPP, CTRL2_GAIN_50VPP, CTRL2_GAIN_75VPP, or CTRL2_GAIN_100VPP
        """
        # Set analog input mode in CTRL1
        ctrl1 = self.read_register(self.REG_CTRL_1)
        ctrl1 &= ~0x07  # Clear input mode bits
        ctrl1 |= self.CTRL1_ANALOG_IN
        ctrl1 &= ~self.CTRL1_STANDBY  # Ensure not in standby
        self.write_register(self.REG_CTRL_1, ctrl1)
        
        # Set gain in CTRL2 (timeout is don't care in analog mode)
        self.write_register(self.REG_CTRL_2, gain)
    
    def standby(self):
        """Put device in standby mode"""
        ctrl1 = self.read_register(self.REG_CTRL_1)
        ctrl1 |= self.CTRL1_STANDBY
        self.write_register(self.REG_CTRL_1, ctrl1)