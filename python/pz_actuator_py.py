import math
import pz_drive
from drv2665 import DRV2665
from shift_register import ShiftRegister


MODE_DIGITAL = 'digital'
MODE_ANALOG = 'analog'


class PzActuator:
    """High-level piezo actuator controller.

    Wraps DRV2665 (I2C), HV509 shift registers (SPI), and
    real-time C module (pz_drive).
    """

    GAINS = {
        25: DRV2665.GAIN_25,
        50: DRV2665.GAIN_50,
        75: DRV2665.GAIN_75,
        100: DRV2665.GAIN_100,
    }

    WAVEFORMS = {'sine': 0, 'triangle': 1, 'square': 2}

    def __init__(self):
        # pz_drive owns I2C and SPI buses — init happens on first use
        self.drv = DRV2665()
        self.sr = ShiftRegister()

        self._mode = None
        self._waveform = None
        self._gain = 100
        self._fullwave = False

    def set_frequency_digital(self, hz, fullwave=False, waveform='sine'):
        """Configure digital FIFO mode at given frequency.

        Args:
            hz: 1-4000
            fullwave: if True, generate |waveform| half-period buffer,
                      fifo.c toggles polarity at period boundary
            waveform: 'sine', 'triangle', or 'square'
        """
        if hz < 1 or hz > 4000:
            raise ValueError("hz must be 1-4000")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        n_samples = round(8000 / hz)
        if n_samples < 2:
            n_samples = 2

        if fullwave:
            # Half-period |waveform| — fifo.c handles polarity toggle
            if waveform == 'sine':
                self._waveform = bytearray(
                    int(127 * math.sin(math.pi * i / n_samples)) & 0xFF
                    for i in range(n_samples)
                )
            elif waveform == 'triangle':
                self._waveform = bytearray(
                    int(127 * (2 * i / n_samples if i < n_samples // 2
                               else 2 * (n_samples - i) / n_samples)) & 0xFF
                    for i in range(n_samples)
                )
            elif waveform == 'square':
                self._waveform = bytearray(127 for _ in range(n_samples))
        else:
            # Full-period waveform (trough at index 0 for sine)
            if waveform == 'sine':
                self._waveform = bytearray(
                    (int(127 * math.sin(-math.pi / 2 + 2 * math.pi * i / n_samples)) & 0xFF)
                    for i in range(n_samples)
                )
            elif waveform == 'triangle':
                self._waveform = bytearray(
                    (int(127 * (4 * i / n_samples - 1 if i < n_samples // 2
                                else 3 - 4 * i / n_samples)) & 0xFF)
                    for i in range(n_samples)
                )
            elif waveform == 'square':
                half = n_samples // 2
                self._waveform = bytearray(
                    (127 if i < half else ((-128) & 0xFF))
                    for i in range(n_samples)
                )

        self._fullwave = fullwave
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
        pz_drive.pwm_set_frequency(hz, resolution=resolution, amplitude=amp_internal,
                                   fullwave=fullwave, dead_time=dead_time,
                                   phase_advance=phase_advance,
                                   waveform=self.WAVEFORMS[waveform])
        self._fullwave = fullwave
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
            pz_drive.fifo_start(self._waveform, gain=gain_bits,
                                fullwave=self._fullwave)
        elif self._mode == MODE_ANALOG:
            self.drv.init_analog(gain_bits)
            pz_drive.pwm_start()

    def stop(self):
        """Stop output and put DRV2665 in standby."""
        if pz_drive.fifo_is_running():
            pz_drive.fifo_stop()
        if pz_drive.pwm_is_running():
            pz_drive.pwm_stop()
        self.drv.standby()

    def is_running(self):
        return pz_drive.fifo_is_running() or pz_drive.pwm_is_running()

    # ── Shift register pass-through ─────────────────────────────────────
    def set_pin(self, pin, value, latch=True):
        self.sr.set_pin(pin, value, latch=latch)

    def get_pin(self, pin):
        return self.sr.get_pin(pin)

    def set_pins(self, values, latch=True):
        self.sr.set_pins(values, latch=latch)

    def get_all(self):
        return self.sr.get_all()

    def set_all(self, value, latch=True):
        self.sr.set_all(value, latch=latch)

    def latch(self):
        self.sr.latch()

    def toggle_polarity(self):
        pz_drive.pol_set(not pz_drive.pol_get())

    def get_polarity(self):
        return pz_drive.pol_get()
