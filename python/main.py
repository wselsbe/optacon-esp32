from pz_actuator_py import PzActuator


def demo_analog():
    """Demo: analog sine wave via PWM + RC filter."""
    import time
    pa = PzActuator()

    try:
        for freq in [50, 100, 200, 250, 300, 400]:
            print(f"Analog: {freq}Hz")
            pa.set_frequency_analog(freq)
            pa.set_all(True)
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
            pa.set_all(True)
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
                pa.set_all(False)
                pa.set_pin(i, True)
                time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        pa.stop()

# Auto-run disabled for debugging - call demo() manually from REPL
# demo()
