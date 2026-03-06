"""Test start/stop behavior and negative tests (no signal when stopped)."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

NOISE_FLOOR_V = 0.5
SIGNAL_THRESHOLD_V = 5.0


@pytest.mark.asyncio
async def test_no_signal_before_start(board_ws, oscilloscope, channels):
    """Negative: OUT+ should show no signal before any start command."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await asyncio.sleep(0.5)

        await oscilloscope.configure_channel(ch_out, vdiv="10V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.run()
        await asyncio.sleep(1.0)

        pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert pkpk is None or pkpk < NOISE_FLOOR_V, (
            f"OUT+ should be quiet before start, but PKPK={pkpk}V"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_signal_appears_on_start(board_ws, oscilloscope, channels):
    """Positive: OUT+ should show signal after start."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]

        await oscilloscope.configure_channel(ch_out, vdiv="20V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.configure_trigger(channels["in_plus"], level="1V", slope="POS")
        await oscilloscope.run()

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
        await asyncio.sleep(1.0)

        pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert pkpk is not None and pkpk > SIGNAL_THRESHOLD_V, (
            f"OUT+ should show signal after start, but PKPK={pkpk}V"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_signal_disappears_on_stop(board_ws, oscilloscope, channels):
    """Positive: OUT+ should return to noise floor after stop."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]
        ch_in = channels["in_plus"]

        await oscilloscope.configure_channel(ch_out, vdiv="10V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.run()

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
        await asyncio.sleep(1.0)

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await asyncio.sleep(1.0)

        out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert out_pkpk is None or out_pkpk < NOISE_FLOOR_V, (
            f"OUT+ should be quiet after stop, but PKPK={out_pkpk}V"
        )

        in_pkpk = await oscilloscope.measure_float(ch_in, "PKPK")
        assert in_pkpk is None or in_pkpk < NOISE_FLOOR_V, (
            f"IN+ should be quiet after stop, but PKPK={in_pkpk}V"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_double_stop_safe(board_ws):
    """Negative: calling stop twice should not error."""
    ws = await board_ws()
    try:
        await ws.send(json.dumps({"cmd": "stop"}))
        resp1 = json.loads(await ws.recv())

        await ws.send(json.dumps({"cmd": "stop"}))
        resp2 = json.loads(await ws.recv())

        assert "error" not in resp1, f"First stop errored: {resp1}"
        assert "error" not in resp2, f"Second stop errored: {resp2}"
    finally:
        await ws.close()
