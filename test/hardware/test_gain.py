"""Test gain level amplitude scaling on OUT+."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250
GAIN_LEVELS = [25, 50, 75, 100]


@pytest.mark.asyncio
@pytest.mark.parametrize("gain", GAIN_LEVELS)
async def test_gain_amplitude(board_ws, oscilloscope, channels, tolerance, gain):
    """OUT+ PKPK should be proportional to gain setting."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]

        vdiv = f"{max(gain // 4, 5)}V"
        await oscilloscope.configure_channel(ch_out, vdiv=vdiv, coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.configure_trigger(channels["in_plus"], level="1V", slope="POS")
        await oscilloscope.run()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": FREQ_HZ,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": gain}))
        await ws.recv()
        await asyncio.sleep(1.5)

        out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert out_pkpk is not None, f"Could not measure OUT+ PKPK at gain={gain}"

        expected_vpp = float(gain)
        assert abs(out_pkpk - expected_vpp) / expected_vpp <= tolerance, (
            f"Gain {gain}: expected ~{expected_vpp}Vpp, got {out_pkpk}Vpp "
            f"(tolerance {tolerance*100}%)"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()
