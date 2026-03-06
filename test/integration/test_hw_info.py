"""Tests for hw_info frozen module."""

import sys

import esp32
import pytest


def _make_blob_writer(data_dict):
    """Return a get_blob side_effect that copies values into the buffer."""
    def get_blob(key, buf):
        if key not in data_dict:
            raise OSError("key not found")
        value = data_dict[key].encode()
        buf[:len(value)] = value
    return get_blob


@pytest.fixture()
def hw_info():
    """Import hw_info with a fresh NVS mock each time."""
    # Remove cached module so module-level esp32.NVS() re-executes
    sys.modules.pop("hw_info", None)
    import hw_info as mod
    return mod


class TestGet:
    def test_returns_default_when_key_missing(self, hw_info):
        """get() returns default when NVS raises OSError."""
        esp32.NVS.return_value.get_blob.side_effect = OSError("key not found")
        hw_info._nvs = esp32.NVS("hw_info")

        assert hw_info.get("serial_number") is None
        assert hw_info.get("serial_number", "fallback") == "fallback"

    def test_returns_value_from_nvs(self, hw_info):
        """get() returns decoded string when key exists in NVS."""
        esp32.NVS.return_value.get_blob.side_effect = _make_blob_writer(
            {"serial_number": "OPT-001"}
        )
        hw_info._nvs = esp32.NVS("hw_info")

        assert hw_info.get("serial_number") == "OPT-001"

    def test_returns_default_when_nvs_is_none(self, hw_info):
        """get() returns default when _nvs is None (no partition)."""
        hw_info._nvs = None

        assert hw_info.get("serial_number") is None
        assert hw_info.get("serial_number", "default") == "default"


class TestGetAll:
    def test_returns_dict_of_known_keys(self, hw_info):
        """get_all() returns only keys that have values in NVS."""
        esp32.NVS.return_value.get_blob.side_effect = _make_blob_writer(
            {"serial_number": "OPT-002", "hw_revision": "rev3"}
        )
        hw_info._nvs = esp32.NVS("hw_info")

        result = hw_info.get_all()
        assert result == {"serial_number": "OPT-002", "hw_revision": "rev3"}

    def test_omits_missing_keys(self, hw_info):
        """get_all() omits keys that are not present in NVS."""
        esp32.NVS.return_value.get_blob.side_effect = _make_blob_writer(
            {"serial_number": "OPT-003"}
        )
        hw_info._nvs = esp32.NVS("hw_info")

        result = hw_info.get_all()
        assert result == {"serial_number": "OPT-003"}
        assert "hw_revision" not in result


class TestGetHeaders:
    def test_returns_formatted_headers(self, hw_info):
        """get_headers() returns X-Device-* headers including firmware version."""
        esp32.NVS.return_value.get_blob.side_effect = _make_blob_writer(
            {"serial_number": "OPT-004", "hw_revision": "rev1"}
        )
        hw_info._nvs = esp32.NVS("hw_info")

        headers = hw_info.get_headers()
        assert headers["X-Device-Serial-Number"] == "OPT-004"
        assert headers["X-Device-Hw-Revision"] == "rev1"
        assert headers["X-Device-Firmware-Version"] == "0.1.0"

    def test_includes_firmware_version_even_without_nvs_keys(self, hw_info):
        """get_headers() always includes firmware version."""
        hw_info._nvs = None

        headers = hw_info.get_headers()
        assert headers == {"X-Device-Firmware-Version": "0.1.0"}
