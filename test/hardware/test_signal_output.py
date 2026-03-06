"""Test signal output verification on oscilloscope."""

import time

import pytest

pytestmark = pytest.mark.hardware


@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
@pytest.mark.parametrize("freq_hz", [50, 250, 500])
def test_signal_frequency(board, oscilloscope, channels, tolerance, configure_scope, waveform, freq_hz):
    """Verify measured frequency matches requested frequency."""
    configure_scope(freq_hz)
    oscilloscope.configure_channel(channels["out_plus"], vdiv="20V", coupling="D1M", probe=10)

    board.set_frequency_analog(hz=freq_hz, waveform=waveform)
    board.start()
    time.sleep(1.0)

    measured_freq = oscilloscope.measure_float(channels["in_plus"], "FREQ")
    assert measured_freq is not None, (
        f"Could not measure frequency on {channels['in_plus']}"
    )
    assert abs(measured_freq - freq_hz) / freq_hz <= tolerance, (
        f"Frequency mismatch: expected {freq_hz} Hz, got {measured_freq} Hz "
        f"(tolerance {tolerance * 100}%)"
    )

    board.stop()


@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
def test_signal_amplitude(board, oscilloscope, channels, configure_scope, waveform):
    """Verify signal has non-trivial amplitude on IN+ and OUT+."""
    configure_scope(250)
    oscilloscope.configure_channel(channels["out_plus"], vdiv="20V", coupling="D1M", probe=10)

    board.set_frequency_analog(hz=250, waveform=waveform)
    board.start()
    time.sleep(1.0)

    in_pkpk = oscilloscope.measure_float(channels["in_plus"], "PKPK")
    assert in_pkpk is not None and in_pkpk > 0.1, f"IN+ PKPK too low: {in_pkpk}V"

    out_pkpk = oscilloscope.measure_float(channels["out_plus"], "PKPK")
    assert out_pkpk is not None and out_pkpk > 5.0, f"OUT+ PKPK too low: {out_pkpk}V"

    board.stop()
