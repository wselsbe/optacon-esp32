"""Test gain level amplitude scaling on OUT+."""

import time

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250
GAIN_LEVELS = [25, 50, 75, 100]
# amplitude=55 drives IN+ to ~2.5Vpp single-ended, which slightly overdrives
# the DRV2665 1.8Vpp differential input. However, there is unwanted signal
# coupling on IN- (~0.8Vpp from boost converter noise) that reduces the
# effective differential swing. This needs to be addressed in a future
# hardware revision (better IN- decoupling cap or layout).
AMPLITUDE = 55
# OUT+ has a ~30V DC bias from the boost converter — AC coupling isolates
# the signal component for accurate PKPK measurement.
COUPLING = "A1M"
# V/div must fit the full output swing at each gain level
GAIN_VDIV = {25: "5V", 50: "10V", 75: "10V", 100: "10V"}


def test_gain_ordering(board_url, oscilloscope, channels, configure_scope):
    """Higher gain settings should produce higher OUT+ amplitude."""
    from test.hardware.board_client import WSBoardClient

    measurements = {}
    ws_url = board_url.replace("http://", "ws://") + "/ws"

    for gain in GAIN_LEVELS:
        client = WSBoardClient(ws_url)
        client.connect()
        try:
            ch_out = channels["out_plus"]

            client.set_frequency_analog(hz=FREQ_HZ, amplitude=AMPLITUDE)
            client.start(gain=gain)
            time.sleep(0.5)

            configure_scope(
                FREQ_HZ, ch=ch_out, vdiv=GAIN_VDIV[gain], coupling=COUPLING
            )
            time.sleep(1.5)

            out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
            if out_pkpk is None:
                time.sleep(1.0)
                out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
            assert out_pkpk is not None, f"Could not measure OUT+ PKPK at gain={gain}"
            measurements[gain] = out_pkpk
        finally:
            client.stop()
            client.close()

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
def test_gain_produces_signal(board, oscilloscope, channels, configure_scope, gain):
    """Each gain setting should produce a measurable signal on OUT+."""
    ch_out = channels["out_plus"]

    board.set_frequency_analog(hz=FREQ_HZ, amplitude=AMPLITUDE)
    board.start(gain=gain)
    time.sleep(0.5)

    configure_scope(FREQ_HZ, ch=ch_out, vdiv=GAIN_VDIV[gain], coupling=COUPLING)
    time.sleep(1.5)

    out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    if out_pkpk is None:
        time.sleep(1.0)
        out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert out_pkpk is not None and out_pkpk > 1.5, (
        f"Gain {gain}: OUT+ PKPK too low ({out_pkpk}V), expected signal"
    )
