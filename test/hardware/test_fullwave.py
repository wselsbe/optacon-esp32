"""Test fullwave mode and polarity toggling.

TODO: test_fullwave_doubles_frequency is flaky — scope frequency counter
miscounts on rectified sine (|sin|) due to ringing at zero-crossing cusps.
Reads ~850 Hz instead of expected 500 Hz. May need different measurement
approach (e.g. measure polarity freq and assert 2x, or use DC coupling).
"""

import time

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250


def test_fullwave_polarity_toggles(
    board, oscilloscope, channels, tolerance, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """In fullwave mode, polarity channel should toggle at signal frequency."""
    ch_pol = channels["polarity"]

    board.set_frequency_analog(hz=FREQ_HZ, fullwave=True)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_pol, vdiv="2V", coupling="D1M")
    configure_timebase(FREQ_HZ)
    configure_trigger(ch_pol, level="2V", coupling="D1M")
    start_acquisition()
    time.sleep(1.0)

    pol_freq = oscilloscope.measure_float(ch_pol, "FREQ")
    assert abs(pol_freq - FREQ_HZ) / FREQ_HZ <= tolerance, (
        f"Polarity frequency mismatch: expected {FREQ_HZ} Hz, got {pol_freq} Hz"
    )

    pol_pkpk = oscilloscope.measure_float(ch_pol, "PKPK")
    assert pol_pkpk > 2.0, (
        f"Polarity PKPK too low for digital signal: {pol_pkpk}V"
    )


def test_non_fullwave_polarity_static(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """In non-fullwave mode, polarity channel should be static (no toggling)."""
    ch_pol = channels["polarity"]

    board.set_frequency_analog(hz=FREQ_HZ, fullwave=False)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_pol, vdiv="2V", coupling="D1M")
    configure_timebase(FREQ_HZ)
    configure_trigger(ch_pol, level="2V", coupling="D1M")
    start_acquisition()
    time.sleep(1.0)

    pol_pkpk = oscilloscope.measure_float(ch_pol, "PKPK")
    assert pol_pkpk is None or pol_pkpk < 1.5, (
        f"Polarity should be static in non-fullwave mode, but PKPK={pol_pkpk}V"
    )


def test_fullwave_doubles_frequency(
    board, oscilloscope, channels, tolerance, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """In fullwave mode, IN+ frequency should be 2x the set frequency (|sin|)."""
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=FREQ_HZ, fullwave=True)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_in, vdiv="1V")
    configure_timebase(FREQ_HZ)
    configure_trigger(ch_in)
    start_acquisition()
    time.sleep(1.0)

    in_freq = oscilloscope.measure_float(ch_in, "FREQ")
    expected_freq = FREQ_HZ * 2
    assert abs(in_freq - expected_freq) / expected_freq <= tolerance, (
        f"IN+ frequency in fullwave: expected {expected_freq} Hz, got {in_freq} Hz"
    )
