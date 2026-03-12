"""Test gain level amplitude scaling on OUT+."""

import time

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250
GAIN_LEVELS = [25, 50, 75, 100]
# V/div must fit the full output swing at each gain level
GAIN_VDIV = {25: "5V", 50: "10V", 75: "20V", 100: "20V"}


def test_gain_ordering(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """Higher gain settings should produce higher OUT+ amplitude."""
    measurements = {}
    ch_out = channels["out_plus"]
    ch_in = channels["in_plus"]

    for gain in GAIN_LEVELS:
        board.set_frequency_analog(hz=FREQ_HZ)
        board.start(gain=gain)
        time.sleep(0.5)

        configure_channel(ch_in, vdiv="1V")
        configure_channel(ch_out, vdiv=GAIN_VDIV[gain])
        configure_timebase(FREQ_HZ)
        configure_trigger(ch_in)
        start_acquisition()
        time.sleep(1.5)

        out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
        measurements[gain] = out_pkpk

    # Verify monotonically increasing amplitude with gain
    for i in range(len(GAIN_LEVELS) - 1):
        low = GAIN_LEVELS[i]
        high = GAIN_LEVELS[i + 1]
        assert measurements[high] > measurements[low], (
            f"Gain {high} ({measurements[high]:.2f}Vpp) should be > "
            f"gain {low} ({measurements[low]:.2f}Vpp). "
            f"All measurements: {measurements}"
        )


@pytest.mark.parametrize("gain", GAIN_LEVELS)
def test_gain_produces_signal(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
    gain,
):
    """Each gain setting should produce a measurable signal on OUT+."""
    ch_out = channels["out_plus"]
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=FREQ_HZ)
    board.start(gain=gain)
    time.sleep(0.5)

    configure_channel(ch_in, vdiv="1V")
    configure_channel(ch_out, vdiv=GAIN_VDIV[gain])
    configure_timebase(FREQ_HZ)
    configure_trigger(ch_in)
    start_acquisition()
    time.sleep(1.5)

    out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert out_pkpk > 1.5, (
        f"Gain {gain}: OUT+ PKPK too low ({out_pkpk}V), expected signal"
    )
