"""Test frequency sweep verification."""

import threading
import time

import requests
import pytest

pytestmark = pytest.mark.hardware


def test_sweep_frequency_increases(board_url, board, oscilloscope, channels, configure_scope):
    """During a sweep from 50-500 Hz, measured frequency should increase over time."""
    ch_in = channels["in_plus"]

    configure_scope(250)

    # Set initial frequency and start
    board.set_frequency_analog(hz=50)
    board.start()
    time.sleep(0.5)

    # Start sweep via exec API (fire-and-forget — sweep blocks for 5s)
    def _fire_sweep():
        requests.post(
            f"{board_url}/api/exec",
            json={"code": "pa.sweep_analog(50, 500, 5000, waveform='sine', gain=100)"},
            timeout=15,
        )

    sweep_thread = threading.Thread(target=_fire_sweep)
    sweep_thread.start()

    # Sample frequency at intervals during the sweep
    time.sleep(0.3)  # let sweep start
    frequencies = []
    for _ in range(5):
        time.sleep(0.8)
        freq = oscilloscope.measure_float(ch_in, "FREQ")
        if freq is not None:
            frequencies.append(freq)

    # Wait for sweep to finish, then stop
    sweep_thread.join(timeout=15)
    board.stop()

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
