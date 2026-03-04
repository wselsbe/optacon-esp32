# Put DRV2665 into standby on boot — its state persists across ESP32 reboots
import _thread

import pz_drive
import web_server
from pz_drive_py import PzActuator

pz_drive.i2c_write(0x02, 0x40)

# Start WebREPL if configured (needs webrepl_cfg.py with PASS='...')
try:
    import webrepl

    webrepl.start()
    print("WebREPL started on port 8266")
except Exception:
    pass


def demo_analog():
    """Demo: analog sine wave via PWM + RC filter."""
    import time

    pa = PzActuator()

    try:
        for freq in [50, 100, 200, 250, 300, 400]:
            print(f"Analog: {freq}Hz")
            pa.set_frequency_analog(freq)
            pa.shift_register.set_all(1)
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
            pa.shift_register.set_all(1)
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
                pa.shift_register.set_all(0)
                pa.shift_register.set_pin(i, 1)
                time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()


# Start web server in background thread (keeps REPL available)
_thread.stack_size(32768)
_thread.start_new_thread(web_server.start, ())
