# Put DRV2665 into standby on boot — its state persists across ESP32 reboots
import pz_drive
from pz_actuator_py import PzActuator

pz_drive.i2c_write(0x02, 0x40)


def demo_analog():
    """Demo: analog sine wave via PWM + RC filter."""
    import time

    pa = PzActuator()

    try:
        for freq in [50, 100, 200, 250, 300, 400]:
            print(f"Analog: {freq}Hz")
            pa.set_frequency_analog(freq)
            pa.set_all(1)
            pa.start()
            time.sleep(2)
            pa.stop()

        print("Analog: DC")
        pa.set_frequency_analog(0)
        pa.start()
        time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()


def demo_digital():
    """Demo: digital sine wave via I2C FIFO."""
    import time

    pa = PzActuator()

    try:
        for freq in [50, 100, 200, 250, 300]:
            print(f"Digital: {freq}Hz")
            pa.set_frequency_digital(freq)
            pa.set_all(1)
            pa.start()
            time.sleep(2)
            pa.stop()
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()


def demo():
    """Demo: cycle through pins with analog output."""
    import time

    pa = PzActuator()
    pa.set_frequency_analog(250)
    pa.start()

    try:
        while True:
            for i in range(20):
                pa.set_all(0)
                pa.set_pin(i, 1)
                time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()


# Start web server (blocks — Ctrl+C to access REPL)
import web_server  # noqa: E402

web_server.start()
