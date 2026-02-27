from machine import Pin


class ShiftRegister:
    """HV509 dual daisy-chained shift register — SPI interface.

    32-bit word layout:
      bits [31:26] = unused (always 0)
      bits [25:6]  = pins 0-19 (pin N -> bit 25-N)
      bits [5:0]   = unused (always 0)
    """

    NUM_PINS = 20
    _COMMON_MASK = 0xFC00003F
    _ALL_PINS_MASK = 0x03FFFFC0

    def __init__(self, spi, cs_pin, pol_a_pin=12, pol_b_pin=13):
        self.spi = spi
        self.cs = cs_pin
        self._state = 0x00000000
        self._tx_buf = bytearray(4)

        # Polarity pins (active-low: LOW = inverted mode = normal operation)
        self._pol_a = Pin(pol_a_pin, Pin.OUT, value=0)
        self._pol_b = Pin(pol_b_pin, Pin.OUT, value=0)
        self._polarity = False

        # Commit initial state (all off)
        self.flush()

    def _pin_bit(self, pin):
        return 1 << (25 - pin)

    def set_pin(self, pin, value):
        if pin < 0 or pin >= self.NUM_PINS:
            raise ValueError("pin must be 0-19")
        bit = self._pin_bit(pin)
        if value:
            self._state |= bit
        else:
            self._state &= ~bit

    def get_pin(self, pin):
        if pin < 0 or pin >= self.NUM_PINS:
            raise ValueError("pin must be 0-19")
        return bool(self._state & self._pin_bit(pin))

    def set_all(self, value):
        if value:
            self._state = self._ALL_PINS_MASK
        else:
            self._state = 0

    def get_all(self):
        return tuple(self.get_pin(i) for i in range(self.NUM_PINS))

    def set_pins(self, values):
        if len(values) != self.NUM_PINS:
            raise ValueError("expected 20 values")
        self._state = 0
        for i, v in enumerate(values):
            if v:
                self._state |= self._pin_bit(i)

    def flush(self):
        """Commit pending state to shift registers via SPI."""
        state = self._state & ~self._COMMON_MASK
        self._tx_buf[0] = (state >> 24) & 0xFF
        self._tx_buf[1] = (state >> 16) & 0xFF
        self._tx_buf[2] = (state >> 8) & 0xFF
        self._tx_buf[3] = state & 0xFF
        self.cs.value(0)
        self.spi.write(self._tx_buf)
        self.cs.value(1)

    def toggle_polarity(self):
        """Toggle HV509 polarity pins."""
        self._polarity = not self._polarity
        val = 1 if self._polarity else 0
        self._pol_a.value(val)
        self._pol_b.value(val)

    def get_polarity(self):
        return self._polarity
