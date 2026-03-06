"""Integration tests for PzActuator class."""

import pytest

import pz_drive
from pz_drive_py import PzActuator


class TestSetFrequencyAnalog:
    def test_basic_call(self):
        pa = PzActuator()
        pa.set_frequency_analog(250)
        pz_drive.pwm_set_frequency.assert_called_once()
        call_kwargs = pz_drive.pwm_set_frequency.call_args
        assert call_kwargs.args[0] == 250
        assert pa._mode == "analog"

    def test_negative_hz_raises(self):
        pa = PzActuator()
        with pytest.raises(ValueError):
            pa.set_frequency_analog(-1)

    def test_above_1000_raises(self):
        pa = PzActuator()
        with pytest.raises(ValueError):
            pa.set_frequency_analog(1001)

    def test_invalid_waveform_raises(self):
        pa = PzActuator()
        with pytest.raises(ValueError, match="waveform"):
            pa.set_frequency_analog(250, waveform="invalid")

    def test_amplitude_100_maps_to_128(self):
        pa = PzActuator()
        pa.set_frequency_analog(250, amplitude=100)
        kwargs = pz_drive.pwm_set_frequency.call_args.kwargs
        assert kwargs["amplitude"] == 128

    def test_amplitude_50_maps_to_64(self):
        pa = PzActuator()
        pa.set_frequency_analog(250, amplitude=50)
        kwargs = pz_drive.pwm_set_frequency.call_args.kwargs
        assert kwargs["amplitude"] == 64

    def test_amplitude_0_maps_to_0(self):
        pa = PzActuator()
        pa.set_frequency_analog(250, amplitude=0)
        kwargs = pz_drive.pwm_set_frequency.call_args.kwargs
        assert kwargs["amplitude"] == 0


class TestSetFrequencyDigital:
    def test_basic_call(self):
        pa = PzActuator()
        pa.set_frequency_digital(100)
        assert pa._mode == "digital"
        assert pa._waveform is not None
        assert isinstance(pa._waveform, bytearray)

    def test_stores_frequency(self):
        pa = PzActuator()
        pa.set_frequency_digital(200)
        assert pa._frequency == 200


class TestSetFrequencyLive:
    def test_calls_pwm_set_frequency_live(self):
        pa = PzActuator()
        pa.set_frequency_live(300)
        pz_drive.pwm_set_frequency_live.assert_called_once()
        args = pz_drive.pwm_set_frequency_live.call_args
        assert args.args[0] == 300


class TestStart:
    def test_no_mode_raises(self):
        pa = PzActuator()
        with pytest.raises(RuntimeError, match="set_frequency"):
            pa.start()

    def test_analog_mode(self):
        pa = PzActuator()
        pa.set_frequency_analog(250)
        pa.start(gain=100)
        pz_drive.pwm_start.assert_called_once()

    def test_digital_mode(self):
        pa = PzActuator()
        pa.set_frequency_digital(100)
        pa.start(gain=100)
        pz_drive.fifo_start.assert_called_once()
        call_kwargs = pz_drive.fifo_start.call_args.kwargs
        assert call_kwargs["gain"] == 3  # GAIN_100

    def test_invalid_gain_raises(self):
        pa = PzActuator()
        pa.set_frequency_analog(250)
        with pytest.raises(ValueError, match="gain"):
            pa.start(gain=99)


class TestStop:
    def test_stop_calls_standby(self):
        pa = PzActuator()
        pa.stop()
        # i2c_write called for standby
        pz_drive.i2c_write.assert_called()

    def test_stop_stops_fifo_if_running(self):
        pz_drive.fifo_is_running.return_value = True
        pa = PzActuator()
        pa.stop()
        pz_drive.fifo_stop.assert_called_once()

    def test_stop_stops_pwm_if_running(self):
        pz_drive.pwm_is_running.return_value = True
        pa = PzActuator()
        pa.stop()
        pz_drive.pwm_stop.assert_called_once()


class TestIsRunning:
    def test_false_by_default(self):
        pa = PzActuator()
        assert pa.is_running() is False

    def test_true_when_pwm_running(self):
        pz_drive.pwm_is_running.return_value = True
        pa = PzActuator()
        assert pa.is_running() is True

    def test_true_when_fifo_running(self):
        pz_drive.fifo_is_running.return_value = True
        pa = PzActuator()
        assert pa.is_running() is True


class TestGetStatus:
    def test_returns_expected_keys(self):
        pa = PzActuator()
        pa.set_frequency_analog(250)
        status = pa.get_status()
        assert "running" in status
        assert "mode" in status
        assert "frequency" in status
        assert "gain" in status
        assert "fullwave" in status
        assert "waveform" in status
        assert "polarity" in status
        assert "pins" in status
        # Analog mode adds extra keys
        assert "amplitude" in status
        assert "dead_time" in status
        assert "phase_advance" in status

    def test_frequency_value(self):
        pa = PzActuator()
        pa.set_frequency_analog(350)
        assert pa.get_status()["frequency"] == 350


class TestSweepAnalog:
    def test_sweep_calls_pwm_set_sweep_and_start(self):
        pa = PzActuator()
        pa.sweep_analog(100, 500, 1000)
        pz_drive.pwm_set_sweep.assert_called_once()
        pz_drive.pwm_start.assert_called_once()

    def test_sweep_same_frequency_raises(self):
        pa = PzActuator()
        with pytest.raises(ValueError, match="must differ"):
            pa.sweep_analog(250, 250, 1000)
