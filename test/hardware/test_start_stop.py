"""Test start/stop behavior and negative tests (no signal when stopped)."""

import time

import pytest

pytestmark = pytest.mark.hardware

NOISE_FLOOR_V = 3.0  # DRV2665 quiescent output ripple measures ~1.6-2.4V pk-pk on scope
SIGNAL_THRESHOLD_V = 5.0


def test_no_signal_before_start(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """Negative: OUT+ should show no signal before any start command."""
    ch_out = channels["out_plus"]

    board.stop()
    time.sleep(0.5)

    configure_channel(ch_out, vdiv="10V")
    configure_timebase(250)
    configure_trigger(ch_out)
    start_acquisition()
    time.sleep(1.0)

    pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert pkpk is None or pkpk < NOISE_FLOOR_V, (
        f"OUT+ should be quiet before start, but PKPK={pkpk}V"
    )


def test_signal_appears_on_start(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """Positive: OUT+ should show signal after start."""
    ch_out = channels["out_plus"]
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=250)
    board.start()
    time.sleep(0.5)

    configure_channel(ch_out, vdiv="20V")
    configure_channel(ch_in, vdiv="1V")
    configure_timebase(250)
    configure_trigger(ch_in)
    start_acquisition()
    time.sleep(1.0)

    pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert pkpk > SIGNAL_THRESHOLD_V, (
        f"OUT+ should show signal after start, but PKPK={pkpk}V"
    )


def test_signal_disappears_on_stop(
    board, oscilloscope, channels, clear_measurements,
    configure_channel, configure_timebase, configure_trigger, start_acquisition,
):
    """Positive: OUT+ should return to noise floor after stop."""
    ch_out = channels["out_plus"]
    ch_in = channels["in_plus"]

    board.set_frequency_analog(hz=250)
    board.start()
    time.sleep(1.0)

    board.stop()
    time.sleep(1.0)

    configure_channel(ch_out, vdiv="20V")
    configure_channel(ch_in, vdiv="1V")
    configure_timebase(250)
    configure_trigger(ch_out)
    start_acquisition()
    time.sleep(1.0)

    out_pkpk = oscilloscope.measure_float(ch_out, "PKPK")
    assert out_pkpk is None or out_pkpk < NOISE_FLOOR_V, (
        f"OUT+ should be quiet after stop, but PKPK={out_pkpk}V"
    )

    in_pkpk = oscilloscope.measure_float(ch_in, "PKPK")
    assert in_pkpk is None or in_pkpk < NOISE_FLOOR_V, (
        f"IN+ should be quiet after stop, but PKPK={in_pkpk}V"
    )


def test_double_stop_safe(board):
    """Negative: calling stop twice should not error."""
    resp1 = board.stop()
    resp2 = board.stop()

    assert "error" not in resp1, f"First stop errored: {resp1}"
    assert "error" not in resp2, f"Second stop errored: {resp2}"
