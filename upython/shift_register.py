from machine import SPI, Pin
import time

class ShiftRegister:
    def __init__(self, spi, cs_pin):
        """Initialize shift register controller
        
        Args:
            spi: SPI object
            cs_pin: Chip select Pin object
        """
        self.spi = spi
        self.cs = cs_pin
        self.state = bytearray(4)  # 32 bits total
        
        # Initialize CS high (inactive)
        self.cs.value(1)
        
        # Clear all outputs
        self.clear()
    
    def _update(self):
        """Send current state to shift registers"""
        self.cs.value(0)  # CS low to start transfer
        self.spi.write(self.state)
        self.cs.value(1)  # CS high to latch data
    
    def set_pin(self, pin, value):
        """Set individual pin state
        
        Args:
            pin: Pin number (0-19)
            value: 0 or 1
        """
        if pin < 0 or pin > 19:
            raise ValueError(f"Pin {pin} out of range (0-19)")
        
        # Map pin 0-19 to bits 6-25
        bit_pos = pin + 6
        byte_idx = bit_pos // 8
        bit_idx = bit_pos % 8
        
        if value:
            self.state[byte_idx] |= (1 << bit_idx)
        else:
            self.state[byte_idx] &= ~(1 << bit_idx)
        
        self._update()
    
    def set_all(self, value):
        """Set all 20 pins to the same value
        
        Args:
            value: 0 or 1
        """
        if value:
            # Set bits 6-25
            self.state[0] = 0xC0  # bits 6-7
            self.state[1] = 0xFF  # bits 8-15
            self.state[2] = 0xFF  # bits 16-23
            self.state[3] = 0x03  # bits 24-25
        else:
            self.state[0] = 0x00
            self.state[1] = 0x00
            self.state[2] = 0x00
            self.state[3] = 0x00
        
        self._update()
    
    def clear(self):
        """Clear all outputs (set to 0)"""
        for i in range(4):
            self.state[i] = 0x00
        self._update()
    
    def test_pattern(self, delay_ms=100):
        """Run a test pattern across all outputs"""
        # Walking 1 across 20 pins
        for i in range(20):
            self.clear()
            self.set_pin(i, 1)
            time.sleep_ms(delay_ms)
        
        # All on
        self.set_all(1)
        time.sleep_ms(delay_ms * 5)
        
        # All off
        self.clear()