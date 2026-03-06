"""Test frequency sweep verification."""

import asyncio
import json

import aiohttp
import pytest

pytestmark = pytest.mark.hardware


@pytest.mark.asyncio
async def test_sweep_frequency_increases(board_url, board_ws, oscilloscope, channels):
    """During a sweep from 50-500 Hz, measured frequency should increase over time."""
    ws = await board_ws()
    try:
        ch_in = channels["in_plus"]

        await oscilloscope.configure_channel(ch_in, vdiv="1V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("5MS")
        await oscilloscope.configure_trigger(ch_in, level="1V", slope="POS")
        await oscilloscope.run()

        # Set initial frequency and start
        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 50,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(0.5)

        # Start sweep via exec API (sweep_analog is not a WS command)
        async with aiohttp.ClientSession() as session:
            await session.post(
                f"{board_url}/api/exec",
                json={"code": "pa.sweep_analog(50, 500, 5000, waveform='sine', gain=100)"},
            )

        # Sample frequency at intervals during the sweep
        frequencies = []
        for _ in range(5):
            await asyncio.sleep(0.8)
            freq = await oscilloscope.measure_float(ch_in, "FREQ")
            if freq is not None:
                frequencies.append(freq)

        # Stop after sweep
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()

        assert len(frequencies) >= 3, f"Not enough frequency samples: {frequencies}"

        # Verify frequency trend is increasing
        increasing_count = sum(
            1 for i in range(len(frequencies) - 1) if frequencies[i + 1] > frequencies[i]
        )
        assert increasing_count >= len(frequencies) // 2, (
            f"Frequency should generally increase during sweep. Samples: {frequencies}"
        )

        assert frequencies[0] < 200, f"First frequency too high for sweep start: {frequencies[0]}"
        assert frequencies[-1] > 200, f"Last frequency too low for sweep end: {frequencies[-1]}"
    finally:
        await ws.close()
