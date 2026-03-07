import math

import pz_drive
from drv2665 import DRV2665
from shift_register import ShiftRegister

MODE_DIGITAL = "digital"
MODE_ANALOG = "analog"


class PzDrive:
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

    WAVEFORMS = {"sine": 0, "triangle": 1, "square": 2}

    def __init__(self):
        # pz_drive owns I2C and SPI buses — init happens on first use
        self.drv = DRV2665()
        self.shift_register = ShiftRegister()

        self._mode = None
        self._waveform = None
        self._gain = 100
        self._fullwave = False
        self._frequency = 0
        self._waveform_name = "sine"
        self._amplitude = 100
        self._dead_time = 0
        self._phase_advance = 0

    @staticmethod
    def _gen_sine(n):
        """Full-period sine: trough at index 0, peak at n//2."""
        return bytearray(
            int(127 * math.sin(-math.pi / 2 + 2 * math.pi * i / n)) & 0xFF
            for i in range(n)
        )

    @staticmethod
    def _gen_triangle(n):
        """Full-period triangle: trough at index 0, peak at n//2."""
        return bytearray(
            int(127 * (4 * i / n - 1 if i < n // 2 else 3 - 4 * i / n)) & 0xFF
            for i in range(n)
        )

    @staticmethod
    def _gen_square(n):
        """Full-period square: high for first half, low for second."""
        half = n // 2
        return bytearray(
            (127 if i < half else (-128) & 0xFF) for i in range(n)
        )

    def set_frequency_digital(self, hz, fullwave=False, waveform="sine"):
        """Configure digital FIFO mode at given frequency.

        Args:
            hz: 1-1000
            fullwave: if True, play waveform at 2x rate with polarity toggle
            waveform: 'sine', 'triangle', or 'square'
        """
        if hz < 1 or hz > 1000:
            raise ValueError("hz must be 1-1000")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")

        effective_hz = hz * 2 if fullwave else hz
        n_samples = max(2, round(8000 / effective_hz))

        gen = {"sine": self._gen_sine, "triangle": self._gen_triangle,
               "square": self._gen_square}[waveform]
        self._waveform = gen(n_samples)

        self._fullwave = fullwave
        self._frequency = hz
        self._waveform_name = waveform
        self._mode = MODE_DIGITAL

    def set_frequency_analog(
        self,
        hz,
        resolution=8,
        amplitude=100,
        fullwave=False,
        dead_time=0,
        phase_advance=0,
        waveform="sine",
    ):
        """Configure analog PWM+DDS mode at given frequency.

        Args:
            hz: 0-1000 (0 = DC output)
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage, mapped to internal 0-128)
            fullwave: if True, generate |waveform| and toggle polarity at zero-crossings
            dead_time: ISR ticks (at 32 kHz) to force zero output near each
                       zero-crossing, giving the DRV2665 output time to settle
            phase_advance: ISR ticks to advance polarity toggle, compensating
                           for DRV2665 output lag (~3 ticks at 250 Hz)
            waveform: 'sine', 'triangle', or 'square'
        """
        if hz < 0 or hz > 1000:
            raise ValueError("hz must be 0-1000")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_drive.pwm_set_frequency(
            hz,
            resolution=resolution,
            amplitude=amp_internal,
            fullwave=fullwave,
            dead_time=dead_time,
            phase_advance=phase_advance,
            waveform=self.WAVEFORMS[waveform],
        )
        self._fullwave = fullwave
        self._frequency = hz
        self._waveform_name = waveform
        self._amplitude = amplitude
        self._dead_time = dead_time
        self._phase_advance = phase_advance
        self._mode = MODE_ANALOG

    def set_frequency_live(self, hz, amplitude=100, waveform="sine"):
        """Update frequency/amplitude while ISR is running (no restart).

        Use during music playback for smooth note transitions.
        For rests, set amplitude=0 (ISR outputs silence at midpoint).
        """
        if hz < 0 or hz > 1000:
            raise ValueError("hz must be 0-1000")
        if amplitude < 0 or amplitude > 100:
            raise ValueError("amplitude must be 0-100")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_drive.pwm_set_frequency_live(
            hz,
            amplitude=amp_internal,
            waveform=self.WAVEFORMS[waveform],
        )
        self._frequency = hz
        self._amplitude = amplitude
        self._waveform_name = waveform

    def start(self, gain=100):
        """Start output in the configured mode."""
        if self._mode is None:
            raise RuntimeError(
                "call set_frequency_digital() or set_frequency_analog() first"
            )
        if gain not in self.GAINS:
            raise ValueError("gain must be 25, 50, 75, or 100")
        self._gain = gain
        gain_bits = self.GAINS[gain]

        if self._mode == MODE_DIGITAL:
            pz_drive.fifo_start(self._waveform, gain=gain_bits, fullwave=self._fullwave)
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

    def set_pin(self, pin, value, latch=True):
        self.shift_register.set_pin(pin, value, latch)

    def get_pin(self, pin):
        return self.shift_register.get_pin(pin)

    def set_all(self, value, latch=True):
        self.shift_register.set_all(value, latch)

    def get_all(self):
        return self.shift_register.get_all()

    def latch(self):
        self.shift_register.latch()

    def play_wav(self, path, loop=False, gain=None):
        """Play a WAV file through the analog PWM path.

        Args:
            path: filesystem path to WAV file (8/16-bit PCM, mono/stereo, any rate)
            loop: if True, loop until stop() is called
            gain: optional gain override (25, 50, 75, or 100 Vpp)
        """
        if gain is not None:
            if gain not in self.GAINS:
                raise ValueError("gain must be 25, 50, 75, or 100")
            self._gain = gain
        with open(path, "rb") as f:
            header = f.read(44)
            if len(header) < 44 or header[:4] != b"RIFF" or header[8:12] != b"WAVE":
                raise ValueError("not a valid WAV file")

            num_channels = int.from_bytes(header[22:24], "little")
            sample_rate = int.from_bytes(header[24:28], "little")
            bits_per_sample = int.from_bytes(header[34:36], "little")

            raw = f.read()

        # Convert to 8-bit unsigned mono
        if bits_per_sample == 16:
            if num_channels == 2:
                # Stereo 16-bit: take left channel high byte, signed→unsigned
                raw = bytearray(
                    ((raw[i + 1] ^ 0x80) if i + 1 < len(raw) else 128)
                    for i in range(0, len(raw), 4)
                )
            else:
                # Mono 16-bit: high byte + offset to unsigned
                raw = bytearray(
                    ((raw[i + 1] ^ 0x80) if i + 1 < len(raw) else 128)
                    for i in range(0, len(raw), 2)
                )
        elif bits_per_sample == 8:
            if num_channels == 2:
                # Stereo 8-bit: take every other byte (left channel)
                raw = bytearray(raw[i] for i in range(0, len(raw), 2))
            else:
                raw = bytearray(raw)
        else:
            raise ValueError("unsupported bits_per_sample: " + str(bits_per_sample))

        self._mode = MODE_ANALOG
        self.drv.init_analog(self.GAINS[self._gain])
        pz_drive.pwm_play_samples(raw, sample_rate, loop=loop)

    _PWM_SAMPLE_RATE = 32000

    def sweep_analog(
        self,
        start_hz,
        end_hz,
        duration_ms,
        logarithmic=False,
        waveform="sine",
        resolution=8,
        amplitude=100,
        gain=100,
    ):
        """Sweep frequency from start_hz to end_hz over duration_ms.

        After the sweep completes, output holds at end_hz until stop().

        Args:
            start_hz: starting frequency (1-1000)
            end_hz: ending frequency (1-1000)
            duration_ms: sweep duration in milliseconds (1-60000)
            logarithmic: if True, logarithmic sweep; if False, linear
            waveform: 'sine', 'triangle', or 'square'
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage)
            gain: 25, 50, 75, or 100 Vpp
        """
        if start_hz < 1 or start_hz > 1000:
            raise ValueError("start_hz must be 1-1000")
        if end_hz < 1 or end_hz > 1000:
            raise ValueError("end_hz must be 1-1000")
        if duration_ms < 1 or duration_ms > 60000:
            raise ValueError("duration_ms must be 1-60000")
        if start_hz == end_hz:
            raise ValueError("start_hz and end_hz must differ")

        # Configure waveform at starting frequency
        self.set_frequency_analog(
            start_hz,
            resolution=resolution,
            amplitude=amplitude,
            waveform=waveform,
        )

        # Compute sweep parameters
        total_ticks = duration_ms * (self._PWM_SAMPLE_RATE // 1000)
        step_start = (start_hz << 32) // self._PWM_SAMPLE_RATE
        step_end = (end_hz << 32) // self._PWM_SAMPLE_RATE

        if logarithmic:
            # ratio = 1 + ln(end/start) / total_ticks, in 1.31 fixed-point
            # increment = offset from 1.0 = ln(end/start) / total_ticks * 2^31
            log_ratio = math.log(end_hz / start_hz)
            increment = int(log_ratio / total_ticks * (1 << 31))
        else:
            # delta = (step_end - step_start) / total_ticks
            increment = (step_end - step_start) // total_ticks

        pz_drive.pwm_set_sweep(
            step_end, increment, logarithmic=logarithmic
        )

        # Start output
        self.start(gain=gain)

    def get_status(self):
        """Return current state as a dict (for web UI)."""
        status = {
            "running": self.is_running(),
            "mode": self._mode,
            "frequency": self._frequency,
            "gain": self._gain,
            "fullwave": self._fullwave,
            "waveform": self._waveform_name,
            "polarity": pz_drive.pol_get(),
            "pins": list(self.get_all()),
        }
        if self._mode == MODE_ANALOG:
            status["amplitude"] = self._amplitude
            status["dead_time"] = self._dead_time
            status["phase_advance"] = self._phase_advance
        return status

