import pz_actuator

# Reset DRV2665 to standby on boot — its state persists across ESP32 reboots
pz_actuator.reset_drv()

def demo_analog():
    """Demo: analog sine wave via PWM + RC filter."""
    import time
    pz_actuator.init()
    pz_actuator.set_gain(100)

    try:
        for freq in [50, 100, 200, 250, 300, 400]:
            print(f"Analog: {freq}Hz")
            pz_actuator.set_frequency_analog(freq)
            pz_actuator.set_all(True)
            time.sleep(2)

        print("Analog: DC")
        pz_actuator.set_frequency_analog(0)
        time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()

def demo_digital():
    """Demo: digital sine wave via I2C FIFO."""
    import time
    pz_actuator.init()
    pz_actuator.set_gain(100)

    try:
        for freq in [50, 100, 200, 250, 300]:
            print(f"Digital: {freq}Hz")
            pz_actuator.set_frequency_digital(freq)
            pz_actuator.set_all(True)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()

def demo():
    """Demo: cycle through pins with analog output."""
    import time
    pz_actuator.init()
    pz_actuator.set_gain(100)
    pz_actuator.set_frequency_analog(250)

    try:
        while True:
            for i in range(20):
                pz_actuator.set_all(False)
                pz_actuator.set_pin(i, True)
                time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()

# Auto-run disabled for debugging - call demo() manually from REPL
# demo()
