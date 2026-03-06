"""Test power consumption: idle, active, per-pin, all-pins."""

import asyncio

import pytest

pytestmark = pytest.mark.hardware

IDLE_CURRENT_MAX_A = 0.060
ACTIVE_CURRENT_MAX_A = 0.200
ALL_PINS_CURRENT_MAX_A = 0.500
# Per-pin test runs with signal active. Base load (signal, no pins) is ~90mA.
# A healthy pin adds ~5-10mA to the base load.
PIN_CURRENT_MAX_A = 0.100

NUM_READINGS = 3  # average multiple readings to reduce noise


async def _avg_reading(multimeter, n=NUM_READINGS):
    """Take n readings and return the average."""
    total = 0.0
    for _ in range(n):
        total += await multimeter.read()
    return total / n


@pytest.mark.asyncio
async def test_idle_current(board_ws, multimeter):
    """Board idle (no signal, no pins) should draw below threshold."""
    board = await board_ws()
    try:
        await board.stop()
        await board.set_all(0)
        await asyncio.sleep(1.0)

        current = await _avg_reading(multimeter)
        assert current >= 0, f"Negative current reading: {current}A"
        assert current < IDLE_CURRENT_MAX_A, (
            f"Idle current too high: {current:.4f}A (max {IDLE_CURRENT_MAX_A}A)"
        )
    finally:
        await board.close()


@pytest.mark.asyncio
async def test_active_current(board_ws, multimeter):
    """Board with signal running should draw within expected range."""
    board = await board_ws()
    try:
        await board.set_all(0)
        await board.set_frequency_analog(hz=250)
        await board.start()
        await asyncio.sleep(1.5)

        current = await _avg_reading(multimeter)
        assert current >= 0, f"Negative current reading: {current}A"
        assert current < ACTIVE_CURRENT_MAX_A, (
            f"Active current too high: {current:.4f}A (max {ACTIVE_CURRENT_MAX_A}A)"
        )

        await board.stop()
    finally:
        await board.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("pin", list(range(20)))
async def test_single_pin_current(board_ws, multimeter, pin):
    """Each individual pin should not exceed max per-pin current budget.

    Runs with signal active (250Hz sine, gain 100) so the pin actually
    conducts current through the piezo load. Measures absolute current.
    """
    board = await board_ws()
    try:
        await board.set_all(0)
        await board.set_frequency_analog(hz=250)
        await board.start()

        await board.set_pin(pin, 1)
        await asyncio.sleep(0.5)
        current = await _avg_reading(multimeter)

        await board.set_pin(pin, 0)
        await board.stop()

        assert current >= 0, f"Pin {pin}: negative current reading: {current:.4f}A"
        assert current <= PIN_CURRENT_MAX_A, (
            f"Pin {pin}: current too high: {current:.4f}A (max {PIN_CURRENT_MAX_A}A)"
        )
    finally:
        await board.close()


@pytest.mark.asyncio
async def test_all_pins_current(board_ws, multimeter):
    """All 20 pins enabled with signal should draw current within expected range."""
    board = await board_ws()
    try:
        await board.set_all(0)
        await board.set_frequency_analog(hz=250)
        await board.start()

        await board.set_all(0xFFFFF)
        await asyncio.sleep(1.0)
        all_on = await _avg_reading(multimeter)

        await board.set_all(0)
        await board.stop()

        assert all_on < ALL_PINS_CURRENT_MAX_A, (
            f"All pins current too high: {all_on:.4f}A (max {ALL_PINS_CURRENT_MAX_A}A)"
        )
    finally:
        await board.close()
