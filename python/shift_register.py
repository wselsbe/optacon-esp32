import pz_drive


class ShiftRegister:
    """HV509 daisy-chained shift register driver.

    Maintains a Python-side 32-bit buffer with pin-to-bit mapping.
    Delegates SPI writes to pz_drive C module for ISR-synchronized latching.

    32-bit word layout:
      bits [31:26] = unused (always 0)
      bits [25:6]  = pins 0-19 (pin N -> bit 25-N)
      bits [5:0]   = unused (always 0)
    """

    NUM_PINS = 20
    _ALL_PINS_MASK = 0x03FFFFC0

    def __init__(self):
        self._state = 0

    @staticmethod
    def _pin_bit(pin):
        if pin < 0 or pin > 19:
            raise ValueError("pin must be 0-19")
        return 1 << (25 - pin)

    def set_pin(self, pin, value, latch=True):
        bit = self._pin_bit(pin)
        if value:
            self._state |= bit
        else:
            self._state &= ~bit
        if latch:
            self.latch()

    def get_pin(self, pin):
        return bool(self._state & self._pin_bit(pin))

    def set_all(self, value, latch=True):
        if value:
            self._state = self._ALL_PINS_MASK
        else:
            self._state = 0
        if latch:
            self.latch()

    def get_all(self):
        return tuple(
            1 if (self._state & (1 << (25 - i))) else 0
            for i in range(self.NUM_PINS)
        )

    def set_pins(self, values, latch=True):
        state = 0
        for i, v in enumerate(values):
            if i >= self.NUM_PINS:
                break
            if v:
                state |= 1 << (25 - i)
        self._state = state
        if latch:
            self.latch()

    def latch(self):
        """Stage current state for ISR-synchronized SPI write.

        If no ISR/task is running, writes immediately.
        Otherwise sets pending flag for latch at next waveform event.
        """
        pz_drive.sr_stage(self._state)

    def _direct_write(self, word32):
        """Immediate SPI write (debug only)."""
        pz_drive.sr_write(word32)
