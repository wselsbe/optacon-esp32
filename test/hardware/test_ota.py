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
import subprocess
import time
import urllib.request

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
_REAL_WEB_SERVER = os.path.join(
    os.path.dirname(__file__), "..", "..", "python", "web_server.py"
)


def _require_firmware_build():
    """Skip test if firmware build not found."""
    if not os.path.exists(_FIRMWARE_BUILD_PATH):
        pytest.skip(f"Firmware build not found at {_FIRMWARE_BUILD_PATH}")


def _sha256_file(path):
    """Compute SHA-256 hex digest of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(8192)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def _bump_version(version_str):
    """Bump a semver string's major version to guarantee it's higher.

    E.g. '0.5.0' -> '1.5.0', '99.0.0' -> '100.0.0'.
    """
    parts = version_str.split(".")
    parts[0] = str(int(parts[0]) + 1)
    return ".".join(parts)


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


def _get_next_versions(board_http):
    """Read board's current config and return versions guaranteed to be higher."""
    config = board_http.get_ota_config()
    fw_ver = _bump_version(config.get("firmware_version", "0.0.0"))
    files_ver = _bump_version(config.get("files_version", "0.0.0"))
    return fw_ver, files_ver


def _setup_firmware_fixture(mock, version):
    """Copy real firmware build to mock server fixtures directory.

    Returns (file_size, sha256_hex) for building versions_override.
    """
    _require_firmware_build()
    fw_dir = os.path.join(mock.fixtures_dir, "firmware")
    os.makedirs(fw_dir, exist_ok=True)
    dest = os.path.join(fw_dir, f"{version}.bin")
    shutil.copy2(_FIRMWARE_BUILD_PATH, dest)
    return os.path.getsize(dest), _sha256_file(dest)


def _setup_file_fixture_real(mock, version):
    """Copy real web_server.py to mock server fixtures directory.

    Uses the actual web_server.py so the board works after file update.
    Returns list of change dicts for versions_override.
    """
    files_dir = os.path.join(mock.fixtures_dir, "files", version)
    os.makedirs(files_dir, exist_ok=True)
    dest = os.path.join(files_dir, "web_server.py")
    shutil.copy2(_REAL_WEB_SERVER, dest)
    return [
        {
            "path": "web_server.py",
            "url": f"files/{version}/web_server.py",
            "sha256": _sha256_file(dest),
        }
    ]


def _build_versions_override(
    fw_version=None, fw_size=None, fw_sha=None,
    files_version=None, file_changes=None,
):
    """Build a versions_override dict for the mock server."""
    override = {
        "latest_firmware": fw_version or "0.0.0",
        "latest_files": files_version or "0.0.0",
        "firmware": {},
        "files": {},
    }
    if fw_version and fw_size and fw_sha:
        override["firmware"][fw_version] = {
            "url": f"firmware/{fw_version}.bin",
            "size": fw_size,
            "sha256": fw_sha,
        }
    if files_version and file_changes:
        override["files"][files_version] = {
            "changes": file_changes,
        }
    return override


def _trigger_soft_reset():
    """Trigger board soft-reset via mpremote.

    May fail silently if serial port is busy (e.g. MCP server holding it).
    """
    try:
        subprocess.run(["mpremote", "reset"], timeout=10, capture_output=True)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass


def _board_is_reachable(board_url, timeout=5):
    """Check if the board responds to HTTP."""
    try:
        urllib.request.urlopen(board_url + "/api/device/status", timeout=timeout)
        return True
    except Exception:
        return False


# --- Happy path tests ---


class TestOTAHappyPath:
    """Happy path OTA tests — check, firmware update, file update."""

    def test_ota_check_detects_update(self, board_http, ota_server, ota_baseline):
        """Verify the board can check for updates and detect availability."""
        mock, server_url = ota_server
        fw_ver, files_ver = _get_next_versions(board_http)

        fw_size, fw_sha = _setup_firmware_fixture(mock, fw_ver)
        file_changes = _setup_file_fixture_real(mock, files_ver)

        mock.versions_override = _build_versions_override(
            fw_version=fw_ver, fw_size=fw_size, fw_sha=fw_sha,
            files_version=files_ver, file_changes=file_changes,
        )

        result = board_http.ota_check()
        assert result.get("firmware_available") is True or result.get("files_available") is True, (
            f"No update detected: {result}"
        )

        check_log = board_http.get_device_log("check")
        assert_log_sequence(check_log, ["Check OK", "detected"], log_name="check")

    def test_ota_firmware_update_happy_path(self, board_http, board_url, ota_server, ota_baseline):
        """Firmware update: download, verify SHA, flash, reboot."""
        mock, server_url = ota_server
        fw_ver, _ = _get_next_versions(board_http)

        fw_size, fw_sha = _setup_firmware_fixture(mock, fw_ver)
        mock.versions_override = _build_versions_override(
            fw_version=fw_ver, fw_size=fw_size, fw_sha=fw_sha,
        )

        # Get manifest from check
        check = board_http.ota_check()
        assert check.get("firmware_available") is True, f"Firmware not available: {check}"
        manifest = check.get("manifest", {})

        # Trigger firmware update — this downloads ~1.4MB, flashes, then reboots.
        # The HTTP response may arrive before or after reboot starts.
        try:
            result = board_http.ota_update_firmware(manifest, fw_ver)
            assert result.get("status") == "ok", f"Firmware update failed: {result}"
        except Exception:
            # Board may have rebooted before sending response — that's OK
            pass

        # Wait for board to reboot and come back
        time.sleep(5)
        wait_for_board(board_url, timeout=90)

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
        """File update: download, verify, rename, reboot, verify version."""
        mock, server_url = ota_server
        _, files_ver = _get_next_versions(board_http)

        file_changes = _setup_file_fixture_real(mock, files_ver)
        mock.versions_override = _build_versions_override(
            files_version=files_ver, file_changes=file_changes,
        )

        # Get manifest from check (files must be available)
        check = board_http.ota_check()
        assert check.get("files_available") is True, f"Files not available: {check}"
        manifest = check.get("manifest", {})

        # Trigger file update — endpoint calls _schedule_reset() after success.
        # Board reboots ~1s after responding, but response may not arrive.
        try:
            result = board_http.ota_update_files(manifest, files_ver)
            assert result.get("status") == "ok", f"File update failed: {result}"
        except Exception:
            # Board may have rebooted before sending response
            pass

        # Wait for board to reboot and come back
        time.sleep(5)
        wait_for_board(board_url, timeout=30)

        # Verify files_version updated
        config = board_http.get_ota_config()
        assert config.get("files_version") == files_ver, (
            f"files_version not updated: {config}"
        )

        # Verify update log sequence
        update_log = board_http.get_device_log("update")
        assert_log_sequence(
            update_log,
            ["Downloading", "All files downloaded", "Installed", "File update committed"],
            log_name="update",
        )

        # Verify clean boot (boot.py cleared NVS flag, no rollback)
        boot_log = board_http.get_device_log("boot")
        assert_log_absent(boot_log, ["Rollback", "rolling back"], log_name="boot")


# --- Failure tests ---


class TestOTAFailure:
    """OTA failure tests — download errors, no reboot expected."""

    def test_ota_file_download_failure(self, board_http, board_url, ota_server, ota_baseline):
        """Server returns 500 — file update aborts, no reboot."""
        mock, server_url = ota_server
        _, files_ver = _get_next_versions(board_http)

        file_changes = _setup_file_fixture_real(mock, files_ver)
        mock.versions_override = _build_versions_override(
            files_version=files_ver, file_changes=file_changes,
        )

        # Get manifest while server is healthy
        check = board_http.ota_check()
        assert check.get("files_available") is True, f"Files not available: {check}"
        manifest = check.get("manifest", {})

        # Now break the server for file downloads
        mock.error_mode = "500"

        # Attempt file update — should fail
        with pytest.raises(Exception):
            # Board API returns HTTP 500 when update_files() fails internally
            board_http.ota_update_files(manifest, files_ver)

        # Board should still be responsive (no reboot)
        status = board_http.get_device_status()
        assert status is not None, "Board unresponsive after failed update"

        # Verify update log shows failure
        update_log = board_http.get_device_log("update")
        assert_log_sequence(
            update_log,
            ["Download FAILED", "aborted"],
            log_name="update",
        )

    def test_ota_firmware_validation_rejects_bad_binary(
        self, board_http, board_url, ota_server, ota_baseline
    ):
        """Upload garbage binary — ESP32 OTA validation rejects it, no reboot.

        ESP32's esp_ota_end() validates the binary header before marking
        the partition as bootable. A 4KB garbage binary fails validation
        with ESP_ERR_OTA_VALIDATE_FAILED, so the board never reboots.
        """
        mock, server_url = ota_server
        fw_ver, _ = _get_next_versions(board_http)

        # Build manifest pointing to bad firmware with correct SHA
        bad_fw_path = os.path.join(_FIXTURES_DIR, "bad_firmware.bin")
        bad_size = os.path.getsize(bad_fw_path)
        bad_sha = _sha256_file(bad_fw_path)

        # Copy bad firmware to mock server fixtures
        fw_dir = os.path.join(mock.fixtures_dir, "firmware")
        os.makedirs(fw_dir, exist_ok=True)
        shutil.copy2(bad_fw_path, os.path.join(fw_dir, f"{fw_ver}.bin"))

        mock.versions_override = _build_versions_override(
            fw_version=fw_ver, fw_size=bad_size, fw_sha=bad_sha,
        )

        # Check for update
        check = board_http.ota_check()
        assert check.get("firmware_available") is True, (
            f"Firmware not available for validation test: {check}"
        )
        manifest = check.get("manifest", {})

        # Attempt firmware update — should fail with HTTP 500
        # because esp_ota_end() rejects the invalid binary
        with pytest.raises(Exception) as exc_info:
            board_http.ota_update_firmware(manifest, fw_ver)
        assert "500" in str(exc_info.value), f"Expected HTTP 500, got: {exc_info.value}"

        # Board should still be responsive (no reboot occurred)
        status = board_http.get_device_status()
        assert status is not None, "Board unresponsive after rejected firmware"

        # Verify update log shows validation failure
        update_log = board_http.get_device_log("update")
        assert_log_sequence(
            update_log,
            ["Downloading firmware", "SHA-256 verified", "Firmware update FAILED"],
            log_name="update",
        )


# --- Rollback tests ---


class TestOTARollback:
    """OTA rollback tests — most disruptive, run last."""

    def test_ota_file_rollback_broken_file(
        self, board_http, board_url, ota_server, ota_baseline
    ):
        """Install broken web_server.py — board reboots, crashes on import,
        second boot triggers boot.py rollback via NVS flag."""
        mock, server_url = ota_server
        _, files_ver = _get_next_versions(board_http)

        # Copy broken file to mock server fixtures
        broken_src = os.path.join(_FIXTURES_DIR, "broken_web_server.py")
        files_dir = os.path.join(mock.fixtures_dir, "files", files_ver)
        os.makedirs(files_dir, exist_ok=True)
        dest = os.path.join(files_dir, "web_server.py")
        shutil.copy2(broken_src, dest)
        broken_sha = _sha256_file(dest)

        mock.versions_override = _build_versions_override(
            files_version=files_ver,
            file_changes=[
                {
                    "path": "web_server.py",
                    "url": f"files/{files_ver}/web_server.py",
                    "sha256": broken_sha,
                }
            ],
        )

        # Record baseline files_version
        config_before = board_http.get_ota_config()
        files_version_before = config_before.get("files_version")

        # Check and update files
        check = board_http.ota_check()
        assert check.get("files_available") is True, (
            f"Files not available for rollback test: {check}"
        )
        manifest = check.get("manifest", {})

        # Trigger file update — endpoint reboots after success
        try:
            result = board_http.ota_update_files(manifest, files_ver)
            assert result.get("status") == "ok", f"File update call failed: {result}"
        except Exception:
            # Board may reboot before responding
            pass

        # Board reboots, crashes on `import web_server` (syntax error).
        # boot.py on NEXT boot detects NVS file_upd flag and rolls back.
        # The crash→reboot cycle may happen automatically via watchdog,
        # or we may need to trigger it manually.
        time.sleep(10)

        if not _board_is_reachable(board_url):
            # Board crashed and didn't auto-recover — trigger manual reset
            # TODO: investigate watchdog-based auto-recovery
            _trigger_soft_reset()
            time.sleep(3)

        wait_for_board(board_url, timeout=60)

        # Verify files_version reverted
        config_after = board_http.get_ota_config()
        assert config_after.get("files_version") == files_version_before, (
            f"files_version not reverted: expected {files_version_before}, "
            f"got {config_after.get('files_version')}"
        )

        # Verify boot log shows rollback sequence.
        # boot.py uses a two-phase flag: first boot bumps file_upd 1→2 and
        # allows main.py to run. If main.py crashes (broken web_server.py),
        # second boot sees file_upd=2 and rolls back.
        # The rollback happens on the second boot, which becomes boot.log.prev
        # after the final (successful) boot overwrites boot.log.
        boot_prev = board_http.get_device_log("boot.prev")
        assert_log_sequence(
            boot_prev,
            ["File update pending flag SET", "rolling back", "Rollback complete"],
            log_name="boot.prev",
        )
