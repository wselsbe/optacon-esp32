import pz_actuator
import math

# Reset DRV2665 to standby on boot — its state persists across ESP32 reboots
pz_actuator.reset_drv()

def make_sine(freq_hz, sample_rate=8000):
    """Generate one period of a sine wave as a bytearray (trough at index 0)."""
    period = sample_rate // freq_hz
    buf = bytearray(period)
    for i in range(period):
        phase = -math.pi / 2 + (2 * math.pi * i) / period
        buf[i] = int(math.sin(phase) * 127) & 0xFF
    return buf

def demo():
    import time
    pz_actuator.init()
    wave = make_sine(250)
    pz_actuator.set_waveform(wave)
    pz_actuator.set_gain(100)
    pz_actuator.start()

    try:
        while True:
            # Phase 1: cycle through each pin individually
            for i in range(20):
                pz_actuator.set_all(False)
                pz_actuator.set_pin(i, True)
                time.sleep_ms(200)

            # Phase 2: enable one by one (accumulate)
            pz_actuator.set_all(False)
            for i in range(20):
                pz_actuator.set_pin(i, True)
                time.sleep_ms(200)

            # Phase 3: disable one by one
            for i in range(20):
                pz_actuator.set_pin(i, False)
                time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()

# Auto-run disabled for debugging - call demo() manually from REPL
# demo()
