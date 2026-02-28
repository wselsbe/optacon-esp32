import math
import pz_fifo
import pz_pwm
from machine import I2C, SPI, Pin
from drv2665 import DRV2665
from shift_register import ShiftRegister


MODE_DIGITAL = 'digital'
MODE_ANALOG = 'analog'


class PzActuator:
    """High-level piezo actuator controller.

    Wraps DRV2665 (I2C), HV509 shift registers (SPI), and
    real-time C modules (pz_fifo, pz_pwm).
    """

    GAINS = {
        25: DRV2665.GAIN_25,
        50: DRV2665.GAIN_50,
        75: DRV2665.GAIN_75,
        100: DRV2665.GAIN_100,
    }

    WAVEFORMS = {'sine': 0, 'triangle': 1, 'square': 2}

    def __init__(self):
        # Configure polarity GPIOs before SPI (IOMUX conflict on GPIO 10-13)
        pz_pwm.init_polarity()
        # Own the I2C bus
        self.i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100_000)
        # Own the SPI bus
        self.spi = SPI(1, baudrate=1_000_000, polarity=0, phase=0,
                       sck=Pin(9), mosi=Pin(6), miso=Pin(7))
        # Initialize drivers
        self.drv = DRV2665(self.i2c)
        self.sr = ShiftRegister(self.spi, Pin(10, Pin.OUT))

        self._mode = None
        self._waveform = None
        self._gain = 100

    def set_frequency_digital(self, hz):
        """Configure digital FIFO mode at given frequency.

        Generates one period of signed sine samples for 8 kHz playback.
        """
        if hz < 1 or hz > 4000:
            raise ValueError("hz must be 1-4000")
        n_samples = round(8000 / hz)
        if n_samples < 2:
            n_samples = 2
        # Generate sine: trough at index 0 (phase offset -pi/2)
        self._waveform = bytearray(
            (int(127 * math.sin(-math.pi / 2 + 2 * math.pi * i / n_samples)) & 0xFF)
            for i in range(n_samples)
        )
        self._mode = MODE_DIGITAL

    def set_frequency_analog(self, hz, resolution=8, amplitude=100, fullwave=False,
                             dead_time=0, phase_advance=0, waveform='sine'):
        """Configure analog PWM+DDS mode at given frequency.

        Args:
            hz: 0-400 (0 = DC output)
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage, mapped to internal 0-128)
            fullwave: if True, generate |waveform| and toggle polarity at zero-crossings
            dead_time: ISR ticks (at 32 kHz) to force zero output near each
                       zero-crossing, giving the DRV2665 output time to settle
            phase_advance: ISR ticks to advance polarity toggle, compensating
                           for DRV2665 output lag (~3 ticks at 250 Hz)
            waveform: 'sine', 'triangle', or 'square'
        """
        if hz < 0 or hz > 400:
            raise ValueError("hz must be 0-400")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_pwm.set_frequency(hz, resolution=resolution, amplitude=amp_internal,
                             fullwave=fullwave, dead_time=dead_time,
                             phase_advance=phase_advance,
                             waveform=self.WAVEFORMS[waveform])
        self._mode = MODE_ANALOG

    def start(self, gain=100):
        """Start output in the configured mode."""
        if self._mode is None:
            raise RuntimeError("call set_frequency_digital() or set_frequency_analog() first")
        if gain not in self.GAINS:
            raise ValueError("gain must be 25, 50, 75, or 100")
        self._gain = gain
        gain_bits = self.GAINS[gain]

        if self._mode == MODE_DIGITAL:
            # pz_fifo handles DRV2665 digital config internally
            pz_fifo.start(self.i2c, self._waveform, gain=gain_bits)
        elif self._mode == MODE_ANALOG:
            self.drv.init_analog(gain_bits)
            pz_pwm.start()

    def stop(self):
        """Stop output and put DRV2665 in standby."""
        if pz_fifo.is_running():
            pz_fifo.stop()
        if pz_pwm.is_running():
            pz_pwm.stop()
        self.drv.standby()

    def is_running(self):
        return pz_fifo.is_running() or pz_pwm.is_running()

    # ── Shift register pass-through ─────────────────────────────────────
    def set_pin(self, pin, value, flush=True):
        self.sr.set_pin(pin, value)
        if flush:
            self.sr.flush()

    def get_pin(self, pin):
        return self.sr.get_pin(pin)

    def set_pins(self, values, flush=True):
        self.sr.set_pins(values)
        if flush:
            self.sr.flush()

    def get_all(self):
        return self.sr.get_all()

    def set_all(self, value, flush=True):
        self.sr.set_all(value)
        if flush:
            self.sr.flush()

    def flush(self):
        self.sr.flush()

    def toggle_polarity(self):
        pz_pwm.set_polarity(not pz_pwm.get_polarity())

    def get_polarity(self):
        return pz_pwm.get_polarity()
