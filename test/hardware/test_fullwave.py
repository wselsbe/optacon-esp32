"""Test fullwave mode and polarity toggling."""

import time

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250


def test_fullwave_polarity_toggles(board, oscilloscope, channels, tolerance):
    """In fullwave mode, polarity channel should toggle at signal frequency."""
    ch_pol = channels["polarity"]

    board.set_frequency_analog(hz=FREQ_HZ, fullwave=True)
    board.start()
    time.sleep(0.5)

    oscilloscope.configure_channel(ch_pol, vdiv="2V", coupling="D1M", probe=10)
    oscilloscope.configure_timebase("2MS")
    oscilloscope.configure_trigger(ch_pol, level="2V", slope="POS")
    oscilloscope.run()
    time.sleep(1.0)

    pol_freq = oscilloscope.measure_float(ch_pol, "FREQ")
    assert pol_freq is not None, "Could not measure polarity frequency"
    assert abs(pol_freq - FREQ_HZ) / FREQ_HZ <= tolerance, (
        f"Polarity frequency mismatch: expected {FREQ_HZ} Hz, got {pol_freq} Hz"
    )

    pol_pkpk = oscilloscope.measure_float(ch_pol, "PKPK")
    assert pol_pkpk is not None and pol_pkpk > 2.0, (
        f"Polarity PKPK too low for digital signal: {pol_pkpk}V"
    )

    board.stop()


def test_non_fullwave_polarity_static(board, oscilloscope, channels):
    """In non-fullwave mode, polarity channel should be static (no toggling)."""
    ch_pol = channels["polarity"]

    board.set_frequency_analog(hz=FREQ_HZ, fullwave=False)
    board.start()
    time.sleep(0.5)

    oscilloscope.configure_channel(ch_pol, vdiv="2V", coupling="D1M", probe=10)
    oscilloscope.configure_timebase("2MS")
    oscilloscope.configure_trigger(ch_pol, level="2V", slope="POS")
    oscilloscope.run()
    time.sleep(1.0)

    pol_pkpk = oscilloscope.measure_float(ch_pol, "PKPK")
    assert pol_pkpk is None or pol_pkpk < 1.5, (
        f"Polarity should be static in non-fullwave mode, but PKPK={pol_pkpk}V"
    )

    board.stop()


def test_fullwave_doubles_frequency(board, oscilloscope, channels, tolerance):
    """In fullwave mode, IN+ frequency should be 2x the set frequency (|sin|)."""
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=FREQ_HZ, fullwave=True)
    board.start()
    time.sleep(0.5)

    oscilloscope.configure_channel(ch_in, vdiv="1V", coupling="D1M", probe=10)
    oscilloscope.configure_timebase("2MS")
    oscilloscope.configure_trigger(ch_in, level="0.5V", slope="POS")
    oscilloscope.run()
    time.sleep(1.0)

    in_freq = oscilloscope.measure_float(ch_in, "FREQ")
    expected_freq = FREQ_HZ * 2
    assert in_freq is not None, "Could not measure IN+ frequency"
    assert abs(in_freq - expected_freq) / expected_freq <= tolerance, (
        f"IN+ frequency in fullwave: expected {expected_freq} Hz, got {in_freq} Hz"
    )

    board.stop()
