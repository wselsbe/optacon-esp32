"""Integration tests for PzDrive class."""

import os
import struct

import pytest
import pz_drive
from pz_drive_py import _WAV_MAX_SIZE, PzDrive


def _make_wav(sample_rate=22050, bits_per_sample=8, num_channels=1, num_samples=100):
    """Build a minimal valid WAV file in memory."""
    bytes_per_sample = bits_per_sample // 8
    data_size = num_samples * num_channels * bytes_per_sample
    header = bytearray(44)
    header[0:4] = b"RIFF"
    struct.pack_into("<I", header, 4, 36 + data_size)
    header[8:12] = b"WAVE"
    header[12:16] = b"fmt "
    struct.pack_into("<I", header, 16, 16)  # fmt chunk size
    struct.pack_into("<H", header, 20, 1)   # PCM
    struct.pack_into("<H", header, 22, num_channels)
    struct.pack_into("<I", header, 24, sample_rate)
    struct.pack_into("<I", header, 28, sample_rate * num_channels * bytes_per_sample)
    struct.pack_into("<H", header, 32, num_channels * bytes_per_sample)
    struct.pack_into("<H", header, 34, bits_per_sample)
    header[36:40] = b"data"
    struct.pack_into("<I", header, 40, data_size)
    return bytes(header) + b"\x80" * data_size


class TestSetFrequencyAnalog:
    def test_basic_call(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        pz_drive.pwm_set_frequency.assert_called_once()
        call_kwargs = pz_drive.pwm_set_frequency.call_args
        assert call_kwargs.args[0] == 250
        assert pa._mode == "analog"

    def test_negative_hz_raises(self):
        pa = PzDrive()
        with pytest.raises(ValueError):
            pa.set_frequency_analog(-1)

    def test_above_1000_raises(self):
        pa = PzDrive()
        with pytest.raises(ValueError):
            pa.set_frequency_analog(1001)

    def test_invalid_waveform_raises(self):
        pa = PzDrive()
        with pytest.raises(ValueError, match="waveform"):
            pa.set_frequency_analog(250, waveform="invalid")

    def test_amplitude_100_maps_to_128(self):
        pa = PzDrive()
        pa.set_frequency_analog(250, amplitude=100)
        kwargs = pz_drive.pwm_set_frequency.call_args.kwargs
        assert kwargs["amplitude"] == 128

    def test_amplitude_50_maps_to_64(self):
        pa = PzDrive()
        pa.set_frequency_analog(250, amplitude=50)
        kwargs = pz_drive.pwm_set_frequency.call_args.kwargs
        assert kwargs["amplitude"] == 64

    def test_amplitude_0_maps_to_0(self):
        pa = PzDrive()
        pa.set_frequency_analog(250, amplitude=0)
        kwargs = pz_drive.pwm_set_frequency.call_args.kwargs
        assert kwargs["amplitude"] == 0


class TestSetFrequencyDigital:
    def test_basic_call(self):
        pa = PzDrive()
        pa.set_frequency_digital(100)
        assert pa._mode == "digital"
        assert pa._waveform is not None
        assert isinstance(pa._waveform, bytearray)

    def test_stores_frequency(self):
        pa = PzDrive()
        pa.set_frequency_digital(200)
        assert pa._frequency == 200


class TestSetFrequencyLive:
    def test_calls_pwm_set_frequency_live(self):
        pa = PzDrive()
        pa.set_frequency_live(300)
        pz_drive.pwm_set_frequency_live.assert_called_once()
        args = pz_drive.pwm_set_frequency_live.call_args
        assert args.args[0] == 300

    def test_amplitude_boundary_zero(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        pa.start()
        pa.set_frequency_live(250, amplitude=0)

    def test_amplitude_boundary_max(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        pa.start()
        pa.set_frequency_live(250, amplitude=100)

    def test_amplitude_above_max_raises(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        pa.start()
        with pytest.raises(ValueError):
            pa.set_frequency_live(250, amplitude=101)

    def test_amplitude_negative_raises(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        pa.start()
        with pytest.raises(ValueError):
            pa.set_frequency_live(250, amplitude=-1)


class TestSetFrequencyAnalogAmplitudeBounds:
    @pytest.mark.xfail(reason="set_frequency_analog does not validate amplitude bounds")
    def test_amplitude_above_100_raises(self):
        pa = PzDrive()
        with pytest.raises(ValueError):
            pa.set_frequency_analog(250, amplitude=101)

    @pytest.mark.xfail(reason="set_frequency_analog does not validate amplitude bounds")
    def test_amplitude_negative_raises(self):
        pa = PzDrive()
        with pytest.raises(ValueError):
            pa.set_frequency_analog(250, amplitude=-1)


class TestStart:
    def test_no_mode_raises(self):
        pa = PzDrive()
        with pytest.raises(RuntimeError, match="set_frequency"):
            pa.start()

    def test_analog_mode(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        pa.start(gain=100)
        pz_drive.pwm_start.assert_called_once()

    def test_digital_mode(self):
        pa = PzDrive()
        pa.set_frequency_digital(100)
        pa.start(gain=100)
        pz_drive.fifo_start.assert_called_once()
        call_kwargs = pz_drive.fifo_start.call_args.kwargs
        assert call_kwargs["gain"] == 3  # GAIN_100

    def test_invalid_gain_raises(self):
        pa = PzDrive()
        pa.set_frequency_analog(250)
        with pytest.raises(ValueError, match="gain"):
            pa.start(gain=99)


class TestStop:
    def test_stop_calls_standby(self):
        pa = PzDrive()
        pa.stop()
        # i2c_write called for standby
        pz_drive.i2c_write.assert_called()

    def test_stop_stops_fifo_if_running(self):
        pz_drive.fifo_is_running.return_value = True
        pa = PzDrive()
        pa.stop()
        pz_drive.fifo_stop.assert_called_once()

    def test_stop_stops_pwm_if_running(self):
        pz_drive.pwm_is_running.return_value = True
        pa = PzDrive()
        pa.stop()
        pz_drive.pwm_stop.assert_called_once()


class TestIsRunning:
    def test_false_by_default(self):
        pa = PzDrive()
        assert pa.is_running() is False

    def test_true_when_pwm_running(self):
        pz_drive.pwm_is_running.return_value = True
        pa = PzDrive()
        assert pa.is_running() is True

    def test_true_when_fifo_running(self):
        pz_drive.fifo_is_running.return_value = True
        pa = PzDrive()
        assert pa.is_running() is True


class TestGetStatus:
    def test_returns_expected_keys(self):
        pa = PzDrive()
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
        pa = PzDrive()
        pa.set_frequency_analog(350)
        assert pa.get_status()["frequency"] == 350


class TestSweepAnalog:
    def test_sweep_calls_pwm_set_sweep_and_start(self):
        pa = PzDrive()
        pa.sweep_analog(100, 500, 1000)
        pz_drive.pwm_set_sweep.assert_called_once()
        pz_drive.pwm_start.assert_called_once()

    def test_sweep_same_frequency_raises(self):
        pa = PzDrive()
        with pytest.raises(ValueError, match="must differ"):
            pa.sweep_analog(250, 250, 1000)


class TestPlayWav:
    def _write_tmp(self, data, tmp_path):
        p = os.path.join(str(tmp_path), "test.wav")
        with open(p, "wb") as f:
            f.write(data)
        return p

    def test_valid_wav_plays(self, tmp_path):
        pa = PzDrive()
        path = self._write_tmp(_make_wav(), tmp_path)
        pa.play_wav(path)
        pz_drive.pwm_play_samples.assert_called_once()

    def test_file_too_large_raises(self, tmp_path):
        pa = PzDrive()
        data = _make_wav(num_samples=_WAV_MAX_SIZE + 1)
        path = self._write_tmp(data, tmp_path)
        with pytest.raises(ValueError, match="too large"):
            pa.play_wav(path)

    def test_sample_rate_zero_raises(self, tmp_path):
        pa = PzDrive()
        path = self._write_tmp(_make_wav(sample_rate=0), tmp_path)
        with pytest.raises(ValueError, match="sample rate"):
            pa.play_wav(path)

    def test_invalid_header_raises(self, tmp_path):
        pa = PzDrive()
        path = self._write_tmp(b"not a wav file at all!!", tmp_path)
        with pytest.raises(ValueError, match="not a valid WAV"):
            pa.play_wav(path)

    def test_unsupported_bits_per_sample_raises(self, tmp_path):
        pa = PzDrive()
        path = self._write_tmp(_make_wav(bits_per_sample=32), tmp_path)
        with pytest.raises(ValueError, match="unsupported bits_per_sample"):
            pa.play_wav(path)
