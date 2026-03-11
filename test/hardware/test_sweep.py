"""Test frequency sweep verification.

TODO: Sweep test fails — sweep_analog() blocks the microdot async event loop
for its full 5-second duration, preventing the scope from triggering (no signal
output during the blocking call). The /api/exec endpoint has access to `pa`
(confirmed: failure changed from stale-50Hz to empty-frequencies after fix),
but the blocking call starves the event loop. Options to fix:
1. Add a dedicated WS command for sweep that runs in a background thread
2. Use the REPL client (mpremote) to start the sweep, bypassing the web server
3. Make sweep_analog non-blocking on the board side
"""

import threading
import time

import pytest
import requests

pytestmark = pytest.mark.hardware


def test_sweep_frequency_increases(
    board_url, board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """During a sweep from 50-500 Hz, measured frequency should increase over time."""
    ch_in = channels["in_plus"]
    ch_out = channels["out_plus"]

    # Start signal first so scope can trigger
    board.set_frequency_analog(hz=50)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_in, vdiv="1V")
    configure_channel(ch_out, vdiv="20V")
    configure_timebase(250)
    configure_trigger(ch_in)
    start_acquisition()
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
    time.sleep(0.5)  # let sweep start
    frequencies = []
    for _ in range(5):
        time.sleep(0.8)
        freq = oscilloscope.measure_float(ch_in, "FREQ")
        if freq is not None:
            frequencies.append(freq)

    # Wait for sweep to finish
    sweep_thread.join(timeout=15)

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
