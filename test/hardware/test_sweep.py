"""Test frequency sweep verification."""

import asyncio

import aiohttp
import pytest

pytestmark = pytest.mark.hardware


@pytest.mark.asyncio
async def test_sweep_frequency_increases(board_url, board_ws, oscilloscope, channels, configure_scope):
    """During a sweep from 50-500 Hz, measured frequency should increase over time."""
    board = await board_ws()
    try:
        ch_in = channels["in_plus"]

        await configure_scope(250)

        # Set initial frequency and start
        await board.set_frequency_analog(hz=50)
        await board.start()
        await asyncio.sleep(0.5)

        # Start sweep via exec API (fire-and-forget — sweep blocks for 5s)
        async def _fire_sweep():
            async with aiohttp.ClientSession() as session:
                await session.post(
                    f"{board_url}/api/exec",
                    json={"code": "pa.sweep_analog(50, 500, 5000, waveform='sine', gain=100)"},
                    timeout=aiohttp.ClientTimeout(total=15),
                )

        sweep_task = asyncio.create_task(_fire_sweep())

        # Sample frequency at intervals during the sweep
        await asyncio.sleep(0.3)  # let sweep start
        frequencies = []
        for _ in range(5):
            await asyncio.sleep(0.8)
            freq = await oscilloscope.measure_float(ch_in, "FREQ")
            if freq is not None:
                frequencies.append(freq)

        # Wait for sweep to finish, then stop
        await sweep_task
        await board.stop()

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
        await board.close()
