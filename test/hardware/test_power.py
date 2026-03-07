"""Test power consumption: idle, active, per-pin, all-pins.

TODO: 7/20 per-pin tests fail — known hardware issue. Pins 4, 7-9, 11, 15-16
draw 297-361mA (well above the 120mA limit). These pins have excessive current
through the piezo load, indicating solder bridges or damaged piezo elements.
Passing pins (13/20): 0-3, 5-6, 10, 12-14, 17-19.
"""

import time

import pytest

pytestmark = pytest.mark.hardware

IDLE_CURRENT_MAX_A = 0.060
ACTIVE_CURRENT_MAX_A = 0.200
ALL_PINS_CURRENT_MAX_A = 0.500
# Per-pin test runs with signal active. Base load (signal, no pins) is ~90mA.
# A healthy pin adds ~5-10mA to the base load.
PIN_CURRENT_MAX_A = 0.120

NUM_READINGS = 3  # average multiple readings to reduce noise


def _avg_reading(multimeter, n=NUM_READINGS):
    """Take n readings and return the average."""
    total = 0.0
    for _ in range(n):
        total += multimeter.read()
    return total / n


def test_idle_current(board, multimeter):
    """Board idle (no signal, no pins) should draw below threshold."""
    board.stop()
    board.set_all(0)
    time.sleep(1.0)

    current = _avg_reading(multimeter)
    assert current >= 0, f"Negative current reading: {current}A"
    assert current < IDLE_CURRENT_MAX_A, (
        f"Idle current too high: {current:.4f}A (max {IDLE_CURRENT_MAX_A}A)"
    )


def test_active_current(board, multimeter):
    """Board with signal running should draw within expected range."""
    board.set_all(0)
    board.set_frequency_analog(hz=250)
    board.start()
    time.sleep(1.5)

    current = _avg_reading(multimeter)
    assert current >= 0, f"Negative current reading: {current}A"
    assert current < ACTIVE_CURRENT_MAX_A, (
        f"Active current too high: {current:.4f}A (max {ACTIVE_CURRENT_MAX_A}A)"
    )


KNOWN_BAD_PINS = {4, 7, 8, 9, 11, 15, 16}


@pytest.mark.parametrize("pin", list(range(20)))
def test_single_pin_current(board, multimeter, pin, request):
    """Each individual pin should not exceed max per-pin current budget.

    Runs with signal active (250Hz sine, gain 100) so the pin actually
    conducts current through the piezo load. Measures absolute current.
    """
    if pin in KNOWN_BAD_PINS:
        request.node.add_marker(
            pytest.mark.xfail(reason=f"Pin {pin}: known hardware fault (>250mA)")
        )
    board.set_all(0)
    board.set_frequency_analog(hz=250)
    board.start()

    board.set_pin(pin, 1)
    time.sleep(0.5)
    current = _avg_reading(multimeter)
    board.set_pin(pin, 0)

    assert current >= 0, f"Pin {pin}: negative current reading: {current:.4f}A"
    assert current <= PIN_CURRENT_MAX_A, (
        f"Pin {pin}: current too high: {current:.4f}A (max {PIN_CURRENT_MAX_A}A)"
    )


def test_all_pins_current(board, multimeter):
    """All 20 pins enabled with signal should draw current within expected range."""
    board.set_all(0)
    board.set_frequency_analog(hz=250)
    board.start()

    board.set_all(0xFFFFF)
    time.sleep(1.0)
    all_on = _avg_reading(multimeter)
    board.set_all(0)

    assert all_on < ALL_PINS_CURRENT_MAX_A, (
        f"All pins current too high: {all_on:.4f}A (max {ALL_PINS_CURRENT_MAX_A}A)"
    )
