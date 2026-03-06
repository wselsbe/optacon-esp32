"""Integration tests for ShiftRegister class."""

import pz_drive
from shift_register import ShiftRegister


class TestPinBit:
    def test_pin_0_returns_bit_25(self):
        assert ShiftRegister._pin_bit(0) == 1 << 25

    def test_pin_19_returns_bit_6(self):
        assert ShiftRegister._pin_bit(19) == 1 << 6

    def test_pin_negative_raises(self):
        import pytest

        with pytest.raises(ValueError):
            ShiftRegister._pin_bit(-1)

    def test_pin_20_raises(self):
        import pytest

        with pytest.raises(ValueError):
            ShiftRegister._pin_bit(20)


class TestSetPin:
    def test_set_pin_sets_correct_bit_and_stages(self):
        sr = ShiftRegister()
        sr.set_pin(4, 1)
        expected_bit = 1 << (25 - 4)
        assert sr._state & expected_bit
        pz_drive.sr_stage.assert_called_with(expected_bit)

    def test_set_pin_no_latch(self):
        sr = ShiftRegister()
        sr.set_pin(4, 1, latch=False)
        assert sr._state & (1 << 21)
        pz_drive.sr_stage.assert_not_called()

    def test_get_pin_returns_correct_state(self):
        sr = ShiftRegister()
        sr.set_pin(4, 1, latch=False)
        assert sr.get_pin(4) is True
        assert sr.get_pin(3) is False

    def test_set_pin_clear(self):
        sr = ShiftRegister()
        sr.set_pin(4, 1, latch=False)
        sr.set_pin(4, 0, latch=False)
        assert sr.get_pin(4) is False


class TestSetAll:
    def test_set_all_true(self):
        sr = ShiftRegister()
        sr.set_all(True)
        assert sr._state == 0x03FFFFC0
        pz_drive.sr_stage.assert_called_with(0x03FFFFC0)

    def test_set_all_false(self):
        sr = ShiftRegister()
        sr.set_all(True, latch=False)
        sr.set_all(False)
        assert sr._state == 0
        pz_drive.sr_stage.assert_called_with(0)


class TestGetAll:
    def test_get_all_all_set(self):
        sr = ShiftRegister()
        sr.set_all(True, latch=False)
        result = sr.get_all()
        assert len(result) == 20
        assert all(v == 1 for v in result)

    def test_get_all_none_set(self):
        sr = ShiftRegister()
        result = sr.get_all()
        assert len(result) == 20
        assert all(v == 0 for v in result)

    def test_get_all_partial(self):
        sr = ShiftRegister()
        sr.set_pin(0, 1, latch=False)
        sr.set_pin(19, 1, latch=False)
        result = sr.get_all()
        assert result[0] == 1
        assert result[19] == 1
        assert result[10] == 0


class TestSetPins:
    def test_set_pins_constructs_correct_word(self):
        sr = ShiftRegister()
        values = [1, 0, 1] + [0] * 17
        sr.set_pins(values)
        assert sr.get_pin(0) is True
        assert sr.get_pin(1) is False
        assert sr.get_pin(2) is True
        expected = (1 << 25) | (1 << 23)
        pz_drive.sr_stage.assert_called_with(expected)


class TestLatch:
    def test_latch_calls_sr_stage(self):
        sr = ShiftRegister()
        sr.set_pin(5, 1, latch=False)
        pz_drive.sr_stage.assert_not_called()
        sr.latch()
        pz_drive.sr_stage.assert_called_once_with(sr._state)
