import pz_actuator
import time

def demo():
    pz_actuator.init()
    pz_actuator.set_frequency(250)
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
