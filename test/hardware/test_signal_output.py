"""Test signal output verification on oscilloscope."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware


@pytest.fixture
def _configure_scope(oscilloscope, channels):
    """Configure oscilloscope channels for signal measurement."""

    async def _setup(freq_hz):
        if freq_hz <= 100:
            timebase = "5MS"
        elif freq_hz <= 300:
            timebase = "2MS"
        else:
            timebase = "1MS"

        ch_in = channels["in_plus"]
        ch_out = channels["out_plus"]

        await oscilloscope.configure_channel(ch_in, vdiv="1V", coupling="D1M", probe=10)
        await oscilloscope.configure_channel(ch_out, vdiv="20V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase(timebase)
        await oscilloscope.configure_trigger(ch_in, level="1V", slope="POS")
        await oscilloscope.run()
        await asyncio.sleep(0.5)

    return _setup


@pytest.mark.asyncio
@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
@pytest.mark.parametrize("freq_hz", [50, 250, 500])
async def test_signal_frequency(
    board_ws, oscilloscope, channels, tolerance, _configure_scope, waveform, freq_hz
):
    """Verify measured frequency matches requested frequency."""
    ws = await board_ws()
    try:
        await _configure_scope(freq_hz)

        await ws.send(
            json.dumps(
                {
                    "cmd": "set_frequency_analog",
                    "hz": freq_hz,
                    "amplitude": 100,
                    "waveform": waveform,
                    "fullwave": False,
                }
            )
        )
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()

        await asyncio.sleep(1.0)

        measured_freq = await oscilloscope.measure_float(channels["in_plus"], "FREQ")
        assert measured_freq is not None, (
            f"Could not measure frequency on {channels['in_plus']}"
        )
        assert abs(measured_freq - freq_hz) / freq_hz <= tolerance, (
            f"Frequency mismatch: expected {freq_hz} Hz, got {measured_freq} Hz "
            f"(tolerance {tolerance * 100}%)"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
async def test_signal_amplitude(
    board_ws, oscilloscope, channels, tolerance, _configure_scope, waveform
):
    """Verify signal has non-trivial amplitude on IN+ and OUT+."""
    ws = await board_ws()
    try:
        await _configure_scope(250)

        await ws.send(
            json.dumps(
                {
                    "cmd": "set_frequency_analog",
                    "hz": 250,
                    "amplitude": 100,
                    "waveform": waveform,
                    "fullwave": False,
                }
            )
        )
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()

        await asyncio.sleep(1.0)

        in_pkpk = await oscilloscope.measure_float(channels["in_plus"], "PKPK")
        assert in_pkpk is not None and in_pkpk > 0.1, f"IN+ PKPK too low: {in_pkpk}V"

        out_pkpk = await oscilloscope.measure_float(channels["out_plus"], "PKPK")
        assert out_pkpk is not None and out_pkpk > 5.0, f"OUT+ PKPK too low: {out_pkpk}V"

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()
