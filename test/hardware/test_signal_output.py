"""Test signal output verification on oscilloscope.

TODO: Triangle frequency measurements are flaky — scope miscounts on the
low-amplitude IN+ signal (~2.5Vpp) due to gentle zero-crossing slopes.
Options: measure frequency on OUT+ instead (better SNR at ~100Vpp),
apply bandwidth limiting on IN+, or use period-based measurement.
"""

import time

import pytest

pytestmark = pytest.mark.hardware


@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
@pytest.mark.parametrize("freq_hz", [50, 250, 500])
def test_signal_frequency(
    board, oscilloscope, channels, tolerance, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
    waveform, freq_hz,
):
    """Verify measured frequency matches requested frequency."""
    ch_out = channels["out_plus"]
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=freq_hz, waveform=waveform)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_in, vdiv="1V")
    configure_channel(ch_out, vdiv="20V")
    configure_timebase(freq_hz)
    configure_trigger(ch_in)
    start_acquisition()
    time.sleep(1.0)

    measured_freq = oscilloscope.measure_float(ch_in, "FREQ")
    assert measured_freq is not None, (
        f"Could not measure frequency on {ch_in}"
    )
    assert abs(measured_freq - freq_hz) / freq_hz <= tolerance, (
        f"Frequency mismatch: expected {freq_hz} Hz, got {measured_freq} Hz "
        f"(tolerance {tolerance * 100}%)"
    )


@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
def test_signal_amplitude(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
    waveform,
):
    """Verify signal has non-trivial amplitude on IN+ and OUT+."""
    ch_out = channels["out_plus"]
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=250, waveform=waveform)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_in, vdiv="1V")
    configure_channel(ch_out, vdiv="20V")
    configure_timebase(250)
    configure_trigger(ch_in)
    start_acquisition()
    time.sleep(1.0)

    in_pkpk = oscilloscope.measure_float(ch_in, "PKPK")
    assert in_pkpk is not None and in_pkpk > 0.1, f"IN+ PKPK too low: {in_pkpk}V"

    out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert out_pkpk is not None and out_pkpk > 5.0, f"OUT+ PKPK too low: {out_pkpk}V"
