"""Mock of the pz_drive C module for testing under CPython."""

from unittest.mock import MagicMock

# PWM / Analog DDS
pwm_set_frequency = MagicMock()
pwm_start = MagicMock()
pwm_stop = MagicMock()
pwm_is_running = MagicMock(return_value=False)
pwm_set_frequency_live = MagicMock()
pwm_set_sweep = MagicMock()
pwm_is_sweep_done = MagicMock(return_value=True)
pwm_play_samples = MagicMock()
pwm_is_sample_done = MagicMock(return_value=True)

# FIFO / Digital
fifo_start = MagicMock()
fifo_stop = MagicMock()
fifo_is_running = MagicMock(return_value=False)

# Shift Register
sr_stage = MagicMock()
sr_write = MagicMock()

# I2C

def _i2c_read_default(reg):
    """Return register-appropriate values for i2c_read.

    Register 0x00 (STATUS): returns 2 (FIFO_EMPTY bit set) matching real HW default.
    All other registers: returns 0.

    Note: when tests need to override for all registers (e.g. ``i2c_read.return_value = -1``),
    they must also clear ``side_effect`` (``i2c_read.side_effect = None``) so that
    ``return_value`` takes effect.  The ``_reset()`` helper restores the default side_effect.
    """
    if reg == 0x00:
        return 2  # STATUS: FIFO_EMPTY
    return 0

i2c_read = MagicMock(side_effect=_i2c_read_default)
i2c_write = MagicMock()

# Polarity
pol_init = MagicMock()
pol_get = MagicMock(return_value=False)
pol_set = MagicMock()


def _reset():
    """Reset all mocks and restore defaults."""
    pwm_set_frequency.reset_mock()
    pwm_start.reset_mock()
    pwm_stop.reset_mock()
    pwm_is_running.reset_mock(return_value=True)
    pwm_is_running.return_value = False
    pwm_set_frequency_live.reset_mock()
    pwm_set_sweep.reset_mock()
    pwm_is_sweep_done.reset_mock(return_value=True)
    pwm_is_sweep_done.return_value = True
    pwm_play_samples.reset_mock()
    pwm_is_sample_done.reset_mock(return_value=True)
    pwm_is_sample_done.return_value = True

    fifo_start.reset_mock()
    fifo_stop.reset_mock()
    fifo_is_running.reset_mock(return_value=True)
    fifo_is_running.return_value = False

    sr_stage.reset_mock()
    sr_write.reset_mock()

    i2c_read.reset_mock(return_value=True, side_effect=True)
    i2c_read.return_value = 0
    i2c_read.side_effect = _i2c_read_default
    i2c_write.reset_mock()

    pol_init.reset_mock()
    pol_get.reset_mock(return_value=True)
    pol_get.return_value = False
    pol_set.reset_mock()
