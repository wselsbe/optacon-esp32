"""Test power consumption: idle, active, per-pin, all-pins."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

IDLE_CURRENT_MAX_A = 0.200
ACTIVE_CURRENT_MAX_A = 0.500
PIN_CURRENT_MIN_A = 0.001
PIN_CURRENT_MAX_A = 0.050
ALL_PINS_CURRENT_MAX_A = 1.0


@pytest.mark.asyncio
async def test_idle_current(board_ws, multimeter):
    """Board idle (no signal, no pins) should draw below threshold."""
    ws = await board_ws()
    try:
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()
        await asyncio.sleep(1.0)

        current = await multimeter.read()
        assert current >= 0, f"Negative current reading: {current}A"
        assert current < IDLE_CURRENT_MAX_A, (
            f"Idle current too high: {current:.4f}A (max {IDLE_CURRENT_MAX_A}A)"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_active_current(board_ws, multimeter):
    """Board with signal running should draw within expected range."""
    ws = await board_ws()
    try:
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 250,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.5)

        current = await multimeter.read()
        assert current >= 0, f"Negative current reading: {current}A"
        assert current < ACTIVE_CURRENT_MAX_A, (
            f"Active current too high: {current:.4f}A (max {ACTIVE_CURRENT_MAX_A}A)"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("pin", list(range(20)))
async def test_single_pin_current(board_ws, multimeter, pin):
    """Each individual pin should draw measurable current within range."""
    ws = await board_ws()
    try:
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()
        await asyncio.sleep(0.5)
        baseline = await multimeter.read()

        await ws.send(json.dumps({"cmd": "set_pin", "pin": pin, "value": 1}))
        await ws.recv()
        await asyncio.sleep(0.5)
        with_pin = await multimeter.read()

        await ws.send(json.dumps({"cmd": "set_pin", "pin": pin, "value": 0}))
        await ws.recv()

        delta = with_pin - baseline
        assert delta >= PIN_CURRENT_MIN_A, (
            f"Pin {pin}: current delta too low: {delta:.4f}A (min {PIN_CURRENT_MIN_A}A)"
        )
        assert delta <= PIN_CURRENT_MAX_A, (
            f"Pin {pin}: current delta too high: {delta:.4f}A (max {PIN_CURRENT_MAX_A}A)"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_all_pins_current(board_ws, multimeter):
    """All 20 pins enabled should draw current within expected range."""
    ws = await board_ws()
    try:
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()
        await asyncio.sleep(0.5)
        baseline = await multimeter.read()

        await ws.send(json.dumps({"cmd": "set_all", "value": 0xFFFFF}))
        await ws.recv()
        await asyncio.sleep(1.0)
        all_on = await multimeter.read()

        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()

        delta = all_on - baseline
        assert delta > 0, "All pins on should draw more current than baseline"
        assert all_on < ALL_PINS_CURRENT_MAX_A, (
            f"All pins current too high: {all_on:.4f}A (max {ALL_PINS_CURRENT_MAX_A}A)"
        )
    finally:
        await ws.close()
