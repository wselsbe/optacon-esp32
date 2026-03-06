"""Test gain level amplitude scaling on OUT+."""

import asyncio

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250
GAIN_LEVELS = [25, 50, 75, 100]


@pytest.mark.asyncio
async def test_gain_ordering(board_ws, oscilloscope, channels, configure_scope):
    """Higher gain settings should produce higher OUT+ amplitude."""
    measurements = {}

    for gain in GAIN_LEVELS:
        board = await board_ws()
        try:
            ch_out = channels["out_plus"]

            await configure_scope(FREQ_HZ, ch=ch_out)
            await board.set_frequency_analog(hz=FREQ_HZ)
            await board.start(gain=gain)
            await asyncio.sleep(2.0)

            out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
            if out_pkpk is None:
                await asyncio.sleep(1.0)
                out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
            assert out_pkpk is not None, f"Could not measure OUT+ PKPK at gain={gain}"
            measurements[gain] = out_pkpk

            await board.stop()
        finally:
            await board.close()

    # Verify monotonically increasing amplitude with gain
    for i in range(len(GAIN_LEVELS) - 1):
        low = GAIN_LEVELS[i]
        high = GAIN_LEVELS[i + 1]
        assert measurements[high] > measurements[low], (
            f"Gain {high} ({measurements[high]:.2f}Vpp) should be > "
            f"gain {low} ({measurements[low]:.2f}Vpp). "
            f"All measurements: {measurements}"
        )


@pytest.mark.asyncio
@pytest.mark.parametrize("gain", GAIN_LEVELS)
async def test_gain_produces_signal(board_ws, oscilloscope, channels, configure_scope, gain):
    """Each gain setting should produce a measurable signal on OUT+."""
    board = await board_ws()
    try:
        ch_out = channels["out_plus"]

        await configure_scope(FREQ_HZ, ch=ch_out)
        await board.set_frequency_analog(hz=FREQ_HZ)
        await board.start(gain=gain)
        await asyncio.sleep(2.0)

        out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        if out_pkpk is None:
            await asyncio.sleep(1.0)
            out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert out_pkpk is not None and out_pkpk > 1.5, (
            f"Gain {gain}: OUT+ PKPK too low ({out_pkpk}V), expected signal"
        )

        await board.stop()
    finally:
        await board.close()
