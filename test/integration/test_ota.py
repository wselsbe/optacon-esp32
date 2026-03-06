"""Integration tests for the ota module."""

import importlib
import json
import sys

import esp32
import pytest


@pytest.fixture(autouse=True)
def reset_ota_module():
    """Reload ota module between tests."""
    if "ota" in sys.modules:
        del sys.modules["ota"]
    yield
    if "ota" in sys.modules:
        del sys.modules["ota"]


@pytest.fixture()
def ota_dir(tmp_path, monkeypatch):
    """Isolate filesystem access."""
    monkeypatch.chdir(tmp_path)
    return tmp_path


class TestVerGt:
    def test_newer_version(self):
        import ota

        assert ota._ver_gt("0.2.0", "0.1.0") is True

    def test_same_version(self):
        import ota

        assert ota._ver_gt("0.1.0", "0.1.0") is False

    def test_major_version_bump(self):
        import ota

        assert ota._ver_gt("1.0.0", "0.9.9") is True

    def test_older_version(self):
        import ota

        assert ota._ver_gt("0.1.0", "0.2.0") is False


class TestLoadConfig:
    def test_defaults_when_no_file(self, ota_dir, monkeypatch):
        import ota

        monkeypatch.setattr(ota, "_CONFIG_FILE", str(ota_dir / "ota_config.json"))
        cfg = ota.load_config()
        assert cfg["firmware_version"] == "0.1.0"
        assert cfg["auto_check"] is True

    def test_reads_file_when_present(self, ota_dir, monkeypatch):
        import ota

        config_path = str(ota_dir / "ota_config.json")
        monkeypatch.setattr(ota, "_CONFIG_FILE", config_path)
        data = {"firmware_version": "1.2.3", "update_url": "http://example.com"}
        with open(config_path, "w") as f:
            json.dump(data, f)
        cfg = ota.load_config()
        assert cfg["firmware_version"] == "1.2.3"
        assert cfg["update_url"] == "http://example.com"


class TestSaveConfig:
    def test_writes_json(self, ota_dir, monkeypatch):
        import ota

        config_path = str(ota_dir / "ota_config.json")
        monkeypatch.setattr(ota, "_CONFIG_FILE", config_path)
        cfg = {"firmware_version": "2.0.0", "auto_check": False}
        ota.save_config(cfg)
        with open(config_path) as f:
            saved = json.load(f)
        assert saved["firmware_version"] == "2.0.0"
        assert saved["auto_check"] is False


class TestGetStatus:
    def test_returns_expected_structure(self, ota_dir, monkeypatch):
        import ota

        config_path = str(ota_dir / "ota_config.json")
        monkeypatch.setattr(ota, "_CONFIG_FILE", config_path)
        status = ota.get_status()
        assert "firmware_version" in status
        assert "files_version" in status
        assert "update_url" in status
        assert "auto_check" in status
        assert "diagnostics_url" in status


class TestGetLog:
    def test_returns_empty_when_no_file(self, ota_dir):
        import ota

        assert ota.get_log("boot") == ""

    def test_returns_content_when_file_exists(self, ota_dir, monkeypatch):
        import ota

        log_path = str(ota_dir / "boot.log")
        with open(log_path, "w") as f:
            f.write("boot ok\n")
        # Patch the paths dict lookup by patching the actual file path
        # get_log maps "boot" -> "/boot.log", so we must write to that path
        # Instead, use monkeypatch to override the open for the expected path
        # Simpler: test with "check" log which uses _CHECK_LOG
        check_path = str(ota_dir / "ota_check.log")
        monkeypatch.setattr(ota, "_CHECK_LOG", check_path)
        with open(check_path, "w") as f:
            f.write("check completed\n")
        assert ota.get_log("check") == "check completed\n"


class TestClearUpdateFlag:
    def test_clears_nvs_flag(self):
        import ota

        # Simulate pending flag
        esp32.NVS.get_i32.return_value = 1

        ota.clear_update_flag()

        esp32.NVS.set_i32.assert_called_with("file_upd", 0)
        esp32.NVS.commit.assert_called()

    def test_no_pending_flag(self):
        import ota

        esp32.NVS.get_i32.return_value = 0

        ota.clear_update_flag()

        # Should NOT call set_i32 since no pending update
        esp32.NVS.set_i32.assert_not_called()
