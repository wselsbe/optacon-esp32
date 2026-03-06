"""Integration tests for the wifi module."""

import importlib
import json
import sys

import pytest


@pytest.fixture(autouse=True)
def reset_wifi_module():
    """Reload wifi module to reset globals between tests."""
    if "wifi" in sys.modules:
        del sys.modules["wifi"]
    yield
    if "wifi" in sys.modules:
        del sys.modules["wifi"]


@pytest.fixture()
def wifi_dir(tmp_path, monkeypatch):
    """Change to tmp_path so wifi_config.json is isolated."""
    monkeypatch.chdir(tmp_path)
    return tmp_path


class TestConnect:
    def test_sta_with_valid_config(self, wifi_dir):
        import micropython_builtins

        cfg = {"ssid": "TestNet", "password": "secret"}
        (wifi_dir / "wifi_config.json").write_text(json.dumps(cfg))

        # Ensure WLAN mock returns connected
        wlan = micropython_builtins._wlan_instance
        wlan.isconnected.return_value = True
        wlan.ifconfig.return_value = ("10.0.0.42", "255.255.255.0", "10.0.0.1", "8.8.8.8")

        import wifi

        result = wifi.connect()
        assert result == ("10.0.0.42", "sta", "TestNet")
        assert wifi.ip == "10.0.0.42"
        assert wifi.mode == "sta"

    def test_no_config_falls_back_to_ap(self, wifi_dir):
        import micropython_builtins

        wlan = micropython_builtins._wlan_instance
        wlan.ifconfig.return_value = ("192.168.4.1", "255.255.255.0", "192.168.4.1", "0.0.0.0")

        import wifi

        result = wifi.connect()
        assert result[1] == "ap"
        assert wifi.mode == "ap"

    def test_sta_timeout_falls_back_to_ap(self, wifi_dir):
        import micropython_builtins

        cfg = {"ssid": "TestNet", "password": "secret"}
        (wifi_dir / "wifi_config.json").write_text(json.dumps(cfg))

        wlan = micropython_builtins._wlan_instance
        wlan.isconnected.return_value = False
        wlan.ifconfig.return_value = ("192.168.4.1", "255.255.255.0", "192.168.4.1", "0.0.0.0")

        import wifi

        result = wifi.connect()
        assert result[1] == "ap"


class TestGetStatus:
    def test_returns_expected_dict(self, wifi_dir):
        import wifi

        wifi.ip = "1.2.3.4"
        wifi.mode = "sta"
        wifi.ssid = "MyNet"
        status = wifi.get_status()
        assert status["mode"] == "sta"
        assert status["ssid"] == "MyNet"
        assert status["ip"] == "1.2.3.4"
        assert "hostname" in status


class TestSaveConfig:
    def test_writes_json(self, wifi_dir):
        import wifi

        wifi.save_config("NewNet", "pass123")
        data = json.loads((wifi_dir / "wifi_config.json").read_text())
        assert data["ssid"] == "NewNet"
        assert data["password"] == "pass123"


class TestReconnect:
    def test_reconnect_calls_connect(self, wifi_dir):
        import micropython_builtins

        wlan = micropython_builtins._wlan_instance
        wlan.ifconfig.return_value = ("192.168.4.1", "255.255.255.0", "192.168.4.1", "0.0.0.0")

        import wifi

        result = wifi.reconnect()
        # Should return a valid tuple from connect()
        assert len(result) == 3
