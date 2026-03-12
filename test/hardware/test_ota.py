"""OTA hardware tests — real board + mock server.

Tests verify the full OTA flow: check for updates, firmware update,
file update, rollback on failure, and log event sequences.

Tests are ordered from least to most disruptive. pytest runs them
in file order by default.

NOTE: Firmware version string won't change after update because the
binary is the same build (boot_cfg.FIRMWARE_VERSION is frozen).
Tests verify the OTA mechanism (download, verify, flash, reboot),
not the version bump.
"""

import hashlib
import os
import shutil
import time

import pytest

from test.hardware.ota_helpers import (
    assert_log_absent,
    assert_log_sequence,
    wait_for_board,
)

pytestmark = pytest.mark.hardware

_FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures", "ota")
_FIRMWARE_BUILD_PATH = os.path.expanduser(
    "~/micropython/ports/esp32/build-ESP32_GENERIC_S3/micropython.bin"
)


def _require_firmware_build():
    """Skip test if firmware build not found."""
    if not os.path.exists(_FIRMWARE_BUILD_PATH):
        pytest.skip(f"Firmware build not found at {_FIRMWARE_BUILD_PATH}")


@pytest.fixture(autouse=True)
def _reset_ota_server(request):
    """Reset mock server state before each OTA test.

    Only runs if the test uses the ota_server fixture.
    Defined locally so it doesn't affect other hardware tests.
    """
    if "ota_server" not in request.fixturenames:
        return
    mock, _url = request.getfixturevalue("ota_server")
    mock.reset()


def _setup_firmware_fixture(mock, version="0.2.0"):
    """Copy real firmware build to mock server fixtures directory."""
    _require_firmware_build()
    fw_dir = os.path.join(mock.fixtures_dir, "firmware")
    os.makedirs(fw_dir, exist_ok=True)
    shutil.copy2(_FIRMWARE_BUILD_PATH, os.path.join(fw_dir, f"{version}.bin"))


def _setup_file_fixtures(mock, version="0.2.0"):
    """Copy E2E file fixtures to mock server fixtures directory."""
    e2e_files = os.path.join(os.path.dirname(__file__), "..", "e2e", "fixtures", "files", version)
    if not os.path.isdir(e2e_files):
        pytest.skip(f"E2E file fixtures not found at {e2e_files}")
    dest = os.path.join(mock.fixtures_dir, "files", version)
    os.makedirs(dest, exist_ok=True)
    for name in os.listdir(e2e_files):
        shutil.copy2(os.path.join(e2e_files, name), os.path.join(dest, name))


# --- Happy path tests ---


class TestOTAHappyPath:
    """Happy path OTA tests — check, firmware update, file update."""

    def test_ota_check_detects_update(self, board_http, ota_server, ota_baseline):
        """Verify the board can check for updates and detect availability."""
        mock, server_url = ota_server

        _setup_firmware_fixture(mock)
        _setup_file_fixtures(mock)

        result = board_http.ota_check()
        assert result.get("firmware_available") is True or result.get("files_available") is True, (
            f"No update detected: {result}"
        )

        check_log = board_http.get_device_log("check")
        assert_log_sequence(check_log, ["Check OK", "detected"], log_name="check")

    def test_ota_firmware_update_happy_path(self, board_http, board_url, ota_server, ota_baseline):
        """Firmware update: download, verify SHA, flash, reboot."""
        mock, server_url = ota_server

        _setup_firmware_fixture(mock)

        # Get manifest from check
        check = board_http.ota_check()
        manifest = check.get("manifest", {})
        assert manifest, f"No manifest in check result: {check}"

        # Trigger firmware update
        result = board_http.ota_update_firmware(manifest, "0.2.0")
        assert result.get("status") == "ok", f"Firmware update failed: {result}"

        # Wait for board to reboot and come back
        time.sleep(3)
        wait_for_board(board_url, timeout=60)

        # Verify update log sequence
        update_log = board_http.get_device_log("update")
        assert_log_sequence(
            update_log,
            ["Downloading firmware", "SHA-256 verified", "Set boot partition", "Firmware update complete"],
            log_name="update",
        )

        # Verify clean boot (no rollback)
        boot_log = board_http.get_device_log("boot")
        assert_log_absent(boot_log, ["Rollback", "rolling back"], log_name="boot")

    def test_ota_file_update_happy_path(self, board_http, board_url, ota_server, ota_baseline):
        """File update: download, verify, rename, soft-reset."""
        mock, server_url = ota_server

        _setup_firmware_fixture(mock)
        _setup_file_fixtures(mock)

        # Get manifest from check (files must be available)
        check = board_http.ota_check()
        manifest = check.get("manifest", {})

        # Trigger file update
        result = board_http.ota_update_files(manifest, "0.2.0")
        assert result.get("status") == "ok", f"File update failed: {result}"

        # Wait for soft-reset
        time.sleep(3)
        wait_for_board(board_url, timeout=30)

        # Verify files_version updated
        config = board_http.get_ota_config()
        assert config.get("files_version") == "0.2.0", (
            f"files_version not updated: {config}"
        )

        # Verify update log sequence
        update_log = board_http.get_device_log("update")
        assert_log_sequence(
            update_log,
            ["Downloading", "All files downloaded", "Installed", "File update committed"],
            log_name="update",
        )

        # Verify clean boot
        boot_log = board_http.get_device_log("boot")
        assert_log_absent(boot_log, ["Rollback", "rolling back"], log_name="boot")


# --- Failure tests ---


class TestOTAFailure:
    """OTA failure tests — download errors, no reboot expected."""

    def test_ota_file_download_failure(self, board_http, board_url, ota_server, ota_baseline):
        """Server returns 500 — file update aborts, no reboot."""
        mock, server_url = ota_server

        _setup_firmware_fixture(mock)
        _setup_file_fixtures(mock)

        # Get manifest while server is healthy
        check = board_http.ota_check()
        manifest = check.get("manifest", {})

        # Now break the server for file downloads
        mock.error_mode = "500"

        # Attempt file update — should fail
        with pytest.raises(Exception):
            # Board API returns HTTP 500 when update_files() fails
            board_http.ota_update_files(manifest, "0.2.0")

        # Board should still be responsive (no reboot)
        status = board_http.get_device_status()
        assert status is not None, "Board unresponsive after failed update"

        # Verify update log
        update_log = board_http.get_device_log("update")
        assert_log_sequence(
            update_log,
            ["Download FAILED", "aborted"],
            log_name="update",
        )


# --- Rollback tests ---


class TestOTARollback:
    """OTA rollback tests — most disruptive, run last."""

    def test_ota_firmware_rollback_bad_binary(
        self, board_http, board_url, ota_server, ota_baseline
    ):
        """Flash garbage binary — bootloader rejects, reverts to previous partition."""
        mock, server_url = ota_server

        # Build manifest pointing to bad firmware with correct SHA
        bad_fw_path = os.path.join(_FIXTURES_DIR, "bad_firmware.bin")
        with open(bad_fw_path, "rb") as f:
            bad_data = f.read()
        bad_sha = hashlib.sha256(bad_data).hexdigest()

        # Copy bad firmware to mock server fixtures
        fw_dir = os.path.join(mock.fixtures_dir, "firmware")
        os.makedirs(fw_dir, exist_ok=True)
        shutil.copy2(bad_fw_path, os.path.join(fw_dir, "0.3.0.bin"))

        # Override manifest to serve bad firmware
        mock.versions_override = {
            "latest_firmware": "0.3.0",
            "latest_files": "0.0.0",
            "firmware": {
                "0.3.0": {
                    "url": "firmware/0.3.0.bin",
                    "size": len(bad_data),
                    "sha256": bad_sha,
                }
            },
            "files": {},
        }

        # Check and update
        check = board_http.ota_check()
        manifest = check.get("manifest", {})
        result = board_http.ota_update_firmware(manifest, "0.3.0")
        assert result.get("status") == "ok", f"Firmware update call failed: {result}"

        # Board reboots, bootloader rejects, reverts — may take two boot cycles
        time.sleep(5)
        wait_for_board(board_url, timeout=90)

        # Board should be alive and functional
        status = board_http.get_device_status()
        assert status is not None, "Board unresponsive after firmware rollback"
        assert status.get("firmware_version") == ota_baseline["firmware_version"], (
            f"Firmware version changed unexpectedly: {status}"
        )

    def test_ota_file_rollback_broken_file(
        self, board_http, board_url, ota_server, ota_baseline
    ):
        """Install broken web_server.py — board crashes, mpremote reset, boot.py rolls back."""
        import subprocess

        mock, server_url = ota_server

        # Copy broken file to mock server fixtures
        broken_src = os.path.join(_FIXTURES_DIR, "broken_web_server.py")
        files_dir = os.path.join(mock.fixtures_dir, "files", "0.3.0")
        os.makedirs(files_dir, exist_ok=True)
        shutil.copy2(broken_src, os.path.join(files_dir, "web_server.py"))

        # Build manifest with broken file and correct SHA
        with open(broken_src, "rb") as f:
            broken_sha = hashlib.sha256(f.read()).hexdigest()

        mock.versions_override = {
            "latest_firmware": "0.0.0",
            "latest_files": "0.3.0",
            "firmware": {},
            "files": {
                "0.3.0": {
                    "changes": [
                        {
                            "path": "web_server.py",
                            "url": "files/0.3.0/web_server.py",
                            "sha256": broken_sha,
                        }
                    ]
                }
            },
        }

        # Record baseline files_version
        config_before = board_http.get_ota_config()
        files_version_before = config_before.get("files_version")

        # Check and update files
        check = board_http.ota_check()
        manifest = check.get("manifest", {})
        result = board_http.ota_update_files(manifest, "0.3.0")
        assert result.get("status") == "ok", f"File update call failed: {result}"

        # Board soft-resets, crashes on import web_server — wait a bit
        time.sleep(5)

        # Board should be unreachable (crashed before starting web server)
        board_unreachable = False
        try:
            import urllib.request

            urllib.request.urlopen(board_url + "/api/device/status", timeout=5)
        except Exception:
            board_unreachable = True
        assert board_unreachable, "Board should be unreachable after crash"

        # Trigger second reboot via mpremote reset
        try:
            subprocess.run(["mpremote", "reset"], timeout=10, capture_output=True)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            # Fallback: if mpremote fails, try hard reset via PSU would go here
            pass

        # Wait for board to come back — boot.py should roll back
        time.sleep(3)
        wait_for_board(board_url, timeout=60)

        # Verify files_version reverted
        config_after = board_http.get_ota_config()
        assert config_after.get("files_version") == files_version_before, (
            f"files_version not reverted: expected {files_version_before}, "
            f"got {config_after.get('files_version')}"
        )

        # Verify boot log shows rollback sequence
        boot_log = board_http.get_device_log("boot")
        assert_log_sequence(
            boot_log,
            ["File update pending flag SET", "Rollback:", "Rollback complete"],
            log_name="boot",
        )
