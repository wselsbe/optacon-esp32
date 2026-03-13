# OTA Hardware Tests Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add hardware tests that verify OTA firmware updates, file updates, rollback on failure, and log event sequences on real hardware.

**Architecture:** A session-scoped mock OTA server (reusing `OTAMockServer`) runs on the Pi, reachable by the board over WiFi. A `BoardHTTPClient` class wraps the board's HTTP API. Tests are sequential in a single file, ordered from happy path to most disruptive. An autouse fixture resets mock state between tests.

**Tech Stack:** pytest, aiohttp (mock server), urllib.request (board HTTP client), mpremote (serial reset)

**Spec:** `docs/superpowers/specs/2026-03-12-ota-hardware-tests-design.md`

**Important:** The log endpoint is `GET /api/device/log?file={name}` (not `/api/ota/log/{name}` as the spec says). The plan uses the correct endpoint.

---

## File Structure

| File | Responsibility |
|------|---------------|
| `test/hardware/board_http.py` (create) | `BoardHTTPClient` class — wraps board HTTP API for OTA operations |
| `test/hardware/ota_helpers.py` (create) | `wait_for_board()`, `get_local_ip()`, log assertion helpers |
| `test/hardware/conftest.py` (modify) | Add `ota_server`, `board_http`, `ota_baseline` fixtures; guard existing autouse fixtures |
| `test/hardware/test_ota.py` (create) | All 6 OTA tests + local `_reset_ota_server` autouse fixture |
| `test/hardware/fixtures/ota/bad_firmware.bin` (create) | 4 KB garbage binary |
| `test/hardware/fixtures/ota/broken_web_server.py` (create) | Python file with syntax error |

## Critical: Existing Autouse Fixture Conflict

The existing `conftest.py` has two `autouse=True` fixtures that depend on `oscilloscope` and `board`:
- `_reset_scope_channels(oscilloscope)` — fires for every test in `test/hardware/`
- `_screenshot_after_test(request, oscilloscope, board)` — same

OTA tests don't use these fixtures. If left as-is, pytest will try to instantiate oscilloscope/board connections for OTA tests (slow, possibly failing).

**Fix:** Guard both fixtures to only run for tests that actually request the `board` fixture (signal tests), not OTA tests. Check `request.fixturenames` to see if `board` is in the test's dependency chain.

---

## Chunk 1: Infrastructure

### Task 1: Create static fixture files

**Files:**
- Create: `test/hardware/fixtures/ota/bad_firmware.bin`
- Create: `test/hardware/fixtures/ota/broken_web_server.py`

- [ ] **Step 1: Create fixtures directory and bad firmware binary**

```bash
mkdir -p test/hardware/fixtures/ota
python3 -c "import os; open('test/hardware/fixtures/ota/bad_firmware.bin', 'wb').write(os.urandom(4096))"
```

- [ ] **Step 2: Create broken web_server.py**

Create `test/hardware/fixtures/ota/broken_web_server.py`:
```python
# Intentionally broken file — causes SyntaxError on import.
# Used by OTA hardware tests to trigger file rollback via boot.py.
def !!!syntax_error
```

- [ ] **Step 3: Verify files exist**

```bash
ls -la test/hardware/fixtures/ota/
```

Expected: `bad_firmware.bin` (4096 bytes) and `broken_web_server.py`.

- [ ] **Step 4: Commit**

```bash
git add test/hardware/fixtures/ota/
git commit -m "test: add OTA hardware test fixture files"
```

---

### Task 2: Create `BoardHTTPClient`

**Files:**
- Create: `test/hardware/board_http.py`

The client wraps `urllib.request` to call the board's HTTP API. All methods return parsed JSON. Timeout is generous (30s) since the board is a microcontroller.

- [ ] **Step 1: Write `BoardHTTPClient`**

Create `test/hardware/board_http.py`:

```python
"""HTTP client for board OTA and device API endpoints."""

import json
import logging
import urllib.request

_log = logging.getLogger("board.http")


class BoardHTTPClient:
    """Wraps the board's HTTP API for OTA operations."""

    def __init__(self, base_url: str, timeout: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        url = self.base_url + path
        data = None
        headers = {}
        if body is not None:
            data = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read())

    def get_device_status(self) -> dict:
        """GET /api/device/status"""
        result = self._request("GET", "/api/device/status")
        _log.info("device status: %s", result)
        return result

    def get_ota_config(self) -> dict:
        """GET /api/ota/config"""
        result = self._request("GET", "/api/ota/config")
        _log.info("OTA config: %s", result)
        return result

    def put_ota_config(self, cfg: dict) -> dict:
        """PUT /api/ota/config"""
        result = self._request("PUT", "/api/ota/config", body=cfg)
        _log.info("OTA config updated: %s", result)
        return result

    def ota_check(self) -> dict:
        """POST /api/ota/check"""
        result = self._request("POST", "/api/ota/check")
        _log.info("OTA check: %s", result)
        return result

    def ota_update_firmware(self, manifest: dict, version: str) -> dict:
        """POST /api/ota/update/firmware"""
        result = self._request(
            "POST", "/api/ota/update/firmware",
            body={"manifest": manifest, "version": version},
        )
        _log.info("OTA firmware update: %s", result)
        return result

    def ota_update_files(self, manifest: dict, version: str) -> dict:
        """POST /api/ota/update/files"""
        result = self._request(
            "POST", "/api/ota/update/files",
            body={"manifest": manifest, "version": version},
        )
        _log.info("OTA file update: %s", result)
        return result

    def get_device_log(self, name: str) -> str:
        """GET /api/device/log?file={name}

        name: 'boot', 'boot.prev', 'check', 'update'
        """
        result = self._request("GET", f"/api/device/log?file={name}")
        log_content = result.get("log", "")
        _log.info("device log '%s': %d chars", name, len(log_content))
        return log_content
```

- [ ] **Step 2: Verify syntax**

```bash
python3 -c "import test.hardware.board_http"
```

Expected: no errors.

- [ ] **Step 3: Commit**

```bash
git add test/hardware/board_http.py
git commit -m "test: add BoardHTTPClient for OTA hardware tests"
```

---

### Task 3: Create OTA helper functions

**Files:**
- Create: `test/hardware/ota_helpers.py`

Contains `wait_for_board()`, `get_local_ip()`, and `assert_log_sequence()`.

- [ ] **Step 1: Write helpers**

Create `test/hardware/ota_helpers.py`:

```python
"""OTA hardware test helpers: board polling, IP discovery, log assertions."""

import logging
import re
import socket
import time
import urllib.request

_log = logging.getLogger("ota.helpers")


def get_local_ip() -> str:
    """Get this machine's LAN IP via hostname resolution."""
    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)
    if ip.startswith("127."):
        # Fallback: connect to a public DNS to discover LAN IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        finally:
            s.close()
    _log.info("local IP: %s", ip)
    return ip


def wait_for_board(base_url: str, timeout: float = 60.0) -> bool:
    """Poll board's /api/device/status until responsive or timeout.

    Returns True when board responds with HTTP 200.
    Raises TimeoutError if timeout is exceeded.
    """
    url = base_url.rstrip("/") + "/api/device/status"
    deadline = time.monotonic() + timeout
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        try:
            req = urllib.request.urlopen(url, timeout=5)
            if req.status == 200:
                _log.info("board responsive after %d attempts", attempt)
                return True
        except Exception:
            pass
        time.sleep(1)
    raise TimeoutError(f"Board at {base_url} did not respond within {timeout}s")


def assert_log_sequence(log_content: str, patterns: list[str], log_name: str = "log"):
    """Assert that patterns appear in order within log content.

    Each pattern is a substring (not regex). Searches for each pattern
    starting from where the previous pattern was found.

    Raises AssertionError with details on failure.
    """
    pos = 0
    for i, pattern in enumerate(patterns):
        idx = log_content.find(pattern, pos)
        if idx == -1:
            # Show context around where we were searching
            context_start = max(0, pos - 100)
            context = log_content[context_start:pos + 200]
            raise AssertionError(
                f"Log sequence broken at step {i + 1}/{len(patterns)}: "
                f"pattern '{pattern}' not found in {log_name} after position {pos}.\n"
                f"Remaining log context: ...{context}..."
            )
        pos = idx + len(pattern)


def assert_log_absent(log_content: str, patterns: list[str], log_name: str = "log"):
    """Assert that none of the patterns appear in the log."""
    for pattern in patterns:
        if pattern in log_content:
            raise AssertionError(
                f"Unexpected pattern '{pattern}' found in {log_name}"
            )
```

- [ ] **Step 2: Verify syntax**

```bash
python3 -c "import test.hardware.ota_helpers"
```

Expected: no errors.

- [ ] **Step 3: Commit**

```bash
git add test/hardware/ota_helpers.py
git commit -m "test: add OTA helper functions (wait_for_board, log assertions)"
```

---

### Task 4: Modify conftest.py — guard autouse fixtures + add OTA fixtures

**Files:**
- Modify: `test/hardware/conftest.py`

Two changes: (1) guard existing autouse fixtures to skip for OTA tests, (2) add OTA fixtures.

- [ ] **Step 1: Guard `_reset_scope_channels` to skip for non-scope tests**

In `test/hardware/conftest.py`, change the `_reset_scope_channels` fixture to check if the test uses `board`:

```python
@pytest.fixture(autouse=True)
def _reset_scope_channels(request, oscilloscope):
    """Converge scope to default channels before each test.

    Only runs for tests that use the 'board' fixture (signal tests).
    Skipped for OTA tests that don't need the oscilloscope.
    """
    if "board" not in request.fixturenames:
        return
    for ch in _ALL_SCOPE_CHANNELS:
        if ch in _DEFAULT_CHANNELS:
            vdiv = _DEFAULT_CHANNELS[ch]
            oscilloscope._write_if_changed(f"{ch}:TRA", f"{ch}:TRA ON")
            oscilloscope._write_if_changed(f"{ch}:VDIV", f"{ch}:VDIV {vdiv}")
        else:
            oscilloscope._write_if_changed(f"{ch}:TRA", f"{ch}:TRA OFF")
```

- [ ] **Step 2: Guard `_screenshot_after_test` similarly**

```python
@pytest.fixture(autouse=True)
def _screenshot_after_test(request, oscilloscope, board):
    """Take a scope screenshot after each test, before signal is stopped.

    Only runs for tests that use the 'board' fixture.
    """
    if "board" not in request.fixturenames:
        return
    yield
    # ... rest of existing screenshot logic unchanged ...
```

**Wait — this won't work.** The fixtures have `oscilloscope` and `board` as parameters, so pytest will try to resolve them regardless of the guard. The fix is to make the fixture not depend on them directly:

```python
@pytest.fixture(autouse=True)
def _reset_scope_channels(request):
    """Converge scope to default channels before each test.

    Only runs for tests that use the 'board' fixture (signal tests).
    """
    if "board" not in request.fixturenames:
        return
    oscilloscope = request.getfixturevalue("oscilloscope")
    for ch in _ALL_SCOPE_CHANNELS:
        if ch in _DEFAULT_CHANNELS:
            vdiv = _DEFAULT_CHANNELS[ch]
            oscilloscope._write_if_changed(f"{ch}:TRA", f"{ch}:TRA ON")
            oscilloscope._write_if_changed(f"{ch}:VDIV", f"{ch}:VDIV {vdiv}")
        else:
            oscilloscope._write_if_changed(f"{ch}:TRA", f"{ch}:TRA OFF")


@pytest.fixture(autouse=True)
def _screenshot_after_test(request):
    """Take a scope screenshot after each test, before signal is stopped.

    Only runs for tests that use the 'board' fixture.
    """
    if "board" not in request.fixturenames:
        yield
        return
    oscilloscope = request.getfixturevalue("oscilloscope")
    yield
    # ... existing screenshot logic using oscilloscope variable ...
    parts = request.node.nodeid.split("::")
    file_stem = os.path.splitext(os.path.basename(parts[0]))[0]
    test_name = parts[1] if len(parts) > 1 else "unknown"
    safe_name = test_name.replace("/", "_").replace("\\", "_")
    out_dir = os.path.join(_SCREENSHOT_DIR, file_stem)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{safe_name}.png")
    try:
        import io
        import time
        from PIL import Image
        OVERLAY_DURATION = 4.5
        remaining = OVERLAY_DURATION - (time.monotonic() - oscilloscope._overlay_triggered)
        if remaining > 0:
            time.sleep(remaining)
        bmp_data = oscilloscope.screenshot()
        img = Image.open(io.BytesIO(bmp_data))
        img.save(out_path, "PNG")
        request.node._screenshot_path = out_path
    except Exception as e:
        import warnings
        warnings.warn(f"Screenshot failed: {e}", stacklevel=2)
```

- [ ] **Step 3: Add OTA imports to conftest.py**

At the top of `test/hardware/conftest.py`, add after existing imports:

```python
import asyncio
import threading

from test.e2e.ota_server import OTAMockServer
from test.hardware.board_http import BoardHTTPClient
from test.hardware.ota_helpers import get_local_ip
```

- [ ] **Step 4: Add `ota_server` fixture**

Uses a temporary directory for fixtures to avoid polluting the E2E fixtures dir.

```python
@pytest.fixture(scope="session")
def ota_server(board_url, tmp_path_factory):
    """Start mock OTA server on LAN, yield (mock, url).

    The server binds to 0.0.0.0 so the board can reach it.
    Uses a temp directory for fixture files to avoid polluting E2E fixtures.
    """
    from aiohttp import web

    fixtures_dir = str(tmp_path_factory.mktemp("ota_fixtures"))
    mock = OTAMockServer(fixtures_dir=fixtures_dir)
    loop = asyncio.new_event_loop()
    runner = web.AppRunner(mock.app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, "0.0.0.0", 0)
    loop.run_until_complete(site.start())
    port = site._server.sockets[0].getsockname()[1]
    ip = get_local_ip()
    url = f"http://{ip}:{port}"

    # Run event loop in background thread
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()

    yield mock, url

    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=5)
    loop.run_until_complete(runner.cleanup())
    loop.close()
```

- [ ] **Step 5: Add `board_http` and `ota_baseline` fixtures**

```python
@pytest.fixture(scope="session")
def board_http(board_url):
    """HTTP client for board API."""
    return BoardHTTPClient(board_url)


@pytest.fixture(scope="session")
def ota_baseline(board_http, ota_server):
    """Capture board's initial OTA state for rollback assertions.

    Also configures update_url to point to mock server and restores
    the original config on teardown.
    """
    mock, server_url = ota_server
    config = board_http.get_ota_config()
    original_url = config.get("update_url", "")
    original_files_version = config.get("files_version", "unknown")
    status = board_http.get_device_status()
    original_firmware_version = status.get("firmware_version", "unknown")

    # Point board at mock server
    board_http.put_ota_config({"update_url": server_url})

    baseline = {
        "firmware_version": original_firmware_version,
        "files_version": original_files_version,
        "update_url": original_url,
    }

    yield baseline

    # Restore original config
    board_http.put_ota_config({"update_url": original_url})
```

- [ ] **Step 6: Verify conftest loads**

```bash
python3 -c "import test.hardware.conftest"
```

Expected: no import errors (fixtures won't activate without pytest).

- [ ] **Step 7: Commit**

```bash
git add test/hardware/conftest.py
git commit -m "test: guard autouse fixtures for OTA compat, add OTA fixtures"
```

---

## Chunk 2: Tests

### Task 5: Write OTA test file — happy path tests (1-3)

**Files:**
- Create: `test/hardware/test_ota.py`

Tests are defined in execution order in the file. All tests use the `hardware` marker.

**Important context for the implementer:**
- The board's firmware version string won't change after firmware update (same build — `boot_cfg.FIRMWARE_VERSION` is frozen). Assert on logs and board responsiveness, not version string.
- `GET /api/device/log?file={name}` returns `{"log": "..."}` — use `board_http.get_device_log(name)`.
- `POST /api/ota/check` returns the full check result including `manifest` — reuse it for update calls.
- After firmware update, board reboots via `machine.reset()` (1s timer). After file update, response returns but no auto-reboot — wait for soft-reset.

- [ ] **Step 1: Write test file with happy path tests**

Create `test/hardware/test_ota.py`:

```python
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
    Defined locally in test_ota.py so it doesn't affect other hardware tests.
    """
    if "ota_server" not in request.fixturenames:
        return
    mock, _url = request.getfixturevalue("ota_server")
    mock.reset()


class TestOTAHappyPath:
    """Happy path OTA tests — check, firmware update, file update."""

    def test_ota_check_detects_update(self, board_http, ota_server, ota_baseline):
        """Verify the board can check for updates and detect availability."""
        _require_firmware_build()
        mock, server_url = ota_server

        # Copy real firmware to mock server fixtures
        fw_dir = os.path.join(mock.fixtures_dir, "firmware")
        os.makedirs(fw_dir, exist_ok=True)
        shutil.copy2(_FIRMWARE_BUILD_PATH, os.path.join(fw_dir, "0.2.0.bin"))

        result = board_http.ota_check()
        assert result.get("firmware_available") is True or result.get("files_available") is True, (
            f"No update detected: {result}"
        )

        check_log = board_http.get_device_log("check")
        assert_log_sequence(check_log, ["Check OK", "detected"], log_name="check")

    def test_ota_firmware_update_happy_path(self, board_http, board_url, ota_server, ota_baseline):
        """Firmware update: download, verify SHA, flash, reboot."""
        _require_firmware_build()
        mock, server_url = ota_server

        # Ensure firmware fixture exists
        fw_dir = os.path.join(mock.fixtures_dir, "firmware")
        os.makedirs(fw_dir, exist_ok=True)
        shutil.copy2(_FIRMWARE_BUILD_PATH, os.path.join(fw_dir, "0.2.0.bin"))

        # Get manifest from check
        check = board_http.ota_check()
        manifest = check.get("manifest", {})
        assert manifest, f"No manifest in check result: {check}"

        # Trigger firmware update
        result = board_http.ota_update_firmware(manifest, "0.2.0")
        assert result.get("status") == "ok", f"Firmware update failed: {result}"

        # Wait for board to reboot and come back
        time.sleep(3)  # give board time to start rebooting
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
```

- [ ] **Step 2: Verify syntax**

```bash
python3 -c "import ast; ast.parse(open('test/hardware/test_ota.py').read()); print('OK')"
```

- [ ] **Step 3: Commit**

```bash
git add test/hardware/test_ota.py
git commit -m "test: add OTA happy path hardware tests (check, firmware, files)"
```

---

### Task 6: Add failure and rollback tests (4-6)

**Files:**
- Modify: `test/hardware/test_ota.py`

Append failure and rollback test classes to the existing file.

**Important context:**
- Test 4 (file download failure): mock returns HTTP 500, board's `update_files()` returns False, web server returns HTTP 500 to the test client. No reboot happens.
- Test 5 (firmware rollback): uses `bad_firmware.bin` with correct SHA in manifest. Board flashes garbage, reboots, bootloader rejects, reverts. Allow 90s for recovery.
- Test 6 (file rollback): uses `broken_web_server.py`. After file update succeeds and board soft-resets, it crashes on import. Need `mpremote reset` to trigger second boot where `boot.py` rolls back.

- [ ] **Step 1: Add failure and rollback tests**

Append to `test/hardware/test_ota.py`:

```python
class TestOTAFailure:
    """OTA failure tests — download errors, no reboot expected."""

    def test_ota_file_download_failure(self, board_http, board_url, ota_server, ota_baseline):
        """Server returns 500 — file update aborts, no reboot."""
        mock, server_url = ota_server
        mock.error_mode = "500"

        # Check still needs to work to get manifest — temporarily disable error
        mock.error_mode = None
        check = board_http.ota_check()
        manifest = check.get("manifest", {})
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
            # Fallback: if mpremote fails, try hard reset
            # (PSU power cycle would go here if needed)
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
```

- [ ] **Step 2: Verify syntax**

```bash
python3 -c "import ast; ast.parse(open('test/hardware/test_ota.py').read()); print('OK')"
```

- [ ] **Step 3: Commit**

```bash
git add test/hardware/test_ota.py
git commit -m "test: add OTA failure and rollback hardware tests"
```

---

### Task 7: Run the tests on real hardware

- [ ] **Step 1: Verify all files exist and import cleanly**

```bash
python3 -c "
import test.hardware.board_http
import test.hardware.ota_helpers
print('All imports OK')
"
```

- [ ] **Step 2: Run OTA hardware tests**

```bash
pytest test/hardware/test_ota.py -v --tb=short 2>&1 | tail -50
```

Expected: all 6 tests pass. If any fail, debug based on output.

Note: These tests interact with real hardware and take several minutes due to reboots.

- [ ] **Step 3: Review the HTML report**

Check `test/hardware/report.html` for log output and screenshots.

- [ ] **Step 4: Final commit if any fixes were needed**

```bash
git add -A test/hardware/
git commit -m "test: fix OTA hardware tests after real-hardware run"
```

---

## Notes

- The `_reset_ota_server` autouse fixture is defined locally in `test_ota.py` (not in conftest.py) so it only affects OTA tests. It uses `request.getfixturevalue()` to lazily resolve the mock — no-op for non-OTA tests.
- The mock server uses a temp directory (`tmp_path_factory`) for fixture files, avoiding pollution of E2E fixtures.
- Existing autouse fixtures (`_reset_scope_channels`, `_screenshot_after_test`) are guarded with `request.fixturenames` checks and lazy `request.getfixturevalue()` so they don't trigger oscilloscope/board connections for OTA tests.
- Tests that need the firmware build call `_require_firmware_build()` which skips with a clear message if the binary doesn't exist.
- `mpremote reset` requires the board to be connected via USB serial. If the test runner doesn't have serial access, test 6 will need a different reset mechanism (PSU power cycle via the `power_supply` fixture).
- The spec references `GET /api/ota/log/{name}` but the actual endpoint is `GET /api/device/log?file={name}`. The plan uses the correct endpoint. The spec should be updated separately.
