"""Integration tests for DRV2665 class."""

import pytest
import pz_drive
from drv2665 import DRV2665


class TestInit:
    def test_init_reads_status_register(self):
        DRV2665()
        pz_drive.i2c_read.assert_called_with(0x00)

    def test_init_raises_when_device_not_found(self):
        pz_drive.i2c_read.side_effect = None
        pz_drive.i2c_read.return_value = -1
        with pytest.raises(RuntimeError, match="not responding"):
            DRV2665()

    def test_init_raises_on_i2c_error(self):
        pz_drive.i2c_read.side_effect = OSError("NACK")
        with pytest.raises(OSError):
            DRV2665()


class TestInitAnalog:
    def test_init_analog_gain_100(self):
        drv = DRV2665()
        pz_drive.i2c_write.reset_mock()
        drv.init_analog(DRV2665.GAIN_100)
        calls = pz_drive.i2c_write.call_args_list
        assert len(calls) == 3
        # CTRL2 = TIMEOUT_20MS (0x0C)
        assert calls[0].args == (0x02, 0x0C)
        # CTRL1 = INPUT_ANALOG | GAIN_100 = 0x04 | 0x03 = 0x07
        assert calls[1].args == (0x01, 0x07)
        # CTRL2 = EN_OVERRIDE | TIMEOUT_20MS = 0x02 | 0x0C = 0x0E
        assert calls[2].args == (0x02, 0x0E)


class TestInitDigital:
    def test_init_digital_gain_100(self):
        drv = DRV2665()
        pz_drive.i2c_write.reset_mock()
        drv.init_digital(DRV2665.GAIN_100)
        calls = pz_drive.i2c_write.call_args_list
        assert len(calls) == 3
        # CTRL2 = TIMEOUT_20MS (0x0C)
        assert calls[0].args == (0x02, 0x0C)
        # CTRL1 = INPUT_DIGITAL | GAIN_100 = 0x00 | 0x03 = 0x03
        assert calls[1].args == (0x01, 0x03)
        # CTRL2 = TIMEOUT_20MS (0x0C)
        assert calls[2].args == (0x02, 0x0C)


class TestStandby:
    def test_standby_writes_correct_register(self):
        drv = DRV2665()
        pz_drive.i2c_write.reset_mock()
        drv.standby()
        pz_drive.i2c_write.assert_called_once_with(0x02, 0x40)


class TestStatus:
    def test_status_reads_reg_0(self):
        drv = DRV2665()
        pz_drive.i2c_read.reset_mock()
        pz_drive.i2c_read.side_effect = None
        pz_drive.i2c_read.return_value = 0x02
        assert drv.status() == 0x02
        pz_drive.i2c_read.assert_called_with(0x00)


class TestGainConstants:
    def test_gain_values(self):
        assert DRV2665.GAIN_25 == 0
        assert DRV2665.GAIN_50 == 1
        assert DRV2665.GAIN_75 == 2
        assert DRV2665.GAIN_100 == 3
