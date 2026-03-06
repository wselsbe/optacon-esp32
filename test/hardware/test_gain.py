"""Test gain level amplitude scaling on OUT+."""

import time

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250
GAIN_LEVELS = [25, 50, 75, 100]


def test_gain_ordering(board_url, oscilloscope, channels, configure_scope):
    """Higher gain settings should produce higher OUT+ amplitude."""
    from test.hardware.board_client import BoardClient

    measurements = {}
    ws_url = board_url.replace("http://", "ws://") + "/ws"

    for gain in GAIN_LEVELS:
        client = BoardClient(ws_url)
        client.connect()
        try:
            ch_out = channels["out_plus"]

            configure_scope(FREQ_HZ, ch=ch_out)
            client.set_frequency_analog(hz=FREQ_HZ)
            client.start(gain=gain)
            time.sleep(2.0)

            out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
            if out_pkpk is None:
                time.sleep(1.0)
                out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
            assert out_pkpk is not None, f"Could not measure OUT+ PKPK at gain={gain}"
            measurements[gain] = out_pkpk

            client.stop()
        finally:
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

    configure_scope(FREQ_HZ, ch=ch_out)
    board.set_frequency_analog(hz=FREQ_HZ)
    board.start(gain=gain)
    time.sleep(2.0)

    out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    if out_pkpk is None:
        time.sleep(1.0)
        out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert out_pkpk is not None and out_pkpk > 1.5, (
        f"Gain {gain}: OUT+ PKPK too low ({out_pkpk}V), expected signal"
    )

    board.stop()
