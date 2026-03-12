"""E2E test fixtures -- real web server with mock deps."""

import asyncio
import contextlib
import os
import socket
from unittest.mock import MagicMock

# Patch microdot send_file to resolve /web/ paths to repo root BEFORE importing web_server
import microdot
import microdot.microdot as _microdot_mod
import pytest
import pytest_asyncio

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCREENSHOT_DIR = os.path.join(_HERE, "screenshots")

_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_original_send_file = microdot.send_file


def _patched_send_file(filename, *args, **kwargs):
    if filename.startswith("/web/"):
        filename = os.path.join(_repo_root, filename.lstrip("/"))
    return _original_send_file(filename, *args, **kwargs)


microdot.send_file = _patched_send_file
_microdot_mod.send_file = _patched_send_file

import web_server  # noqa: E402

from test.e2e.ota_server import start_server  # noqa: E402


class MockDeps:
    """Mock dependency container for web_server."""

    def __init__(self):
        self.pa = MagicMock()
        self.pa._mode = "analog"
        self.pa._gain = 100
        self.pa._fullwave = False
        self.pa._dead_time = 0
        self.pa._phase_advance = 0
        self.pa._frequency = 250
        self.pa._amplitude = 100
        self.pa._waveform_name = "sine"
        self.pa.is_running.return_value = False
        self.pa.get_status.return_value = {
            "running": False,
            "mode": "analog",
            "frequency": 250,
            "gain": 100,
            "fullwave": False,
            "waveform": "sine",
            "polarity": False,
            "pins": [0] * 20,
            "amplitude": 100,
            "dead_time": 0,
            "phase_advance": 0,
        }
        self.pa.shift_register = MagicMock()

        self.wifi = MagicMock()
        self.wifi.ip = "127.0.0.1"
        self.wifi.mode = "sta"
        self.wifi.get_status.return_value = {
            "mode": "sta",
            "ssid": "test",
            "ip": "127.0.0.1",
            "hostname": "test.local",
        }

        self.ota = MagicMock()
        self.ota.get_status.return_value = {
            "firmware_version": "0.1.0",
            "files_version": "0.1.0",
            "update_url": "",
            "auto_check": True,
            "diagnostics_url": "",
        }
        self.ota.load_config.return_value = {
            "update_url": "",
            "firmware_version": "0.1.0",
            "files_version": "0.1.0",
            "auto_check": True,
        }
        self.ota.check_for_updates.return_value = {
            "firmware_available": False,
            "files_available": False,
        }
        self.ota.get_log.return_value = ""
        self.ota.send_diagnostics.return_value = True
        self.ota.update_firmware.return_value = True
        self.ota.update_files.return_value = True
        self.ota.upload_file = MagicMock()
        self.ota.upload_firmware = MagicMock(return_value=True)

    def reset(self):
        """Reset all mock call history."""
        for attr in ("pa", "wifi", "ota"):
            getattr(self, attr).reset_mock()


@pytest.fixture
def mock_deps():
    return MockDeps()


@pytest_asyncio.fixture
async def test_app(mock_deps):
    """Start test web server on random port, yield (url, deps)."""
    app = web_server.create_app(mock_deps)

    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()

    server_task = asyncio.create_task(
        app.start_server(host="127.0.0.1", port=port, debug=False)
    )
    await asyncio.sleep(0.3)

    yield f"http://127.0.0.1:{port}", mock_deps

    app.shutdown()
    server_task.cancel()
    with contextlib.suppress(asyncio.CancelledError):
        await server_task


@pytest_asyncio.fixture
async def ota_mock():
    """Start mock OTA server, yield (mock, url)."""
    mock, runner, url = await start_server()
    yield mock, url
    await runner.cleanup()


@pytest.fixture
def page_url(mock_deps):
    """Start test web server in a background thread for Playwright sync tests."""
    import threading

    app = web_server.create_app(mock_deps)

    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()

    loop = asyncio.new_event_loop()

    def _run():
        asyncio.set_event_loop(loop)
        loop.run_until_complete(
            app.start_server(host="127.0.0.1", port=port, debug=False)
        )

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    import time
    time.sleep(0.4)

    yield f"http://127.0.0.1:{port}"

    app.shutdown()
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    loop.close()


@pytest.fixture
def _pw_screenshot(request, page):
    """Take a Playwright screenshot after each UI test.

    Not autouse — applied via pytest_collection_modifyitems to tests using page.
    """
    yield
    parts = request.node.nodeid.split("::")
    file_stem = os.path.splitext(os.path.basename(parts[0]))[0]
    test_name = parts[1] if len(parts) > 1 else "unknown"
    safe_name = test_name.replace("/", "_").replace("\\", "_")
    out_dir = os.path.join(_SCREENSHOT_DIR, file_stem)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{safe_name}.png")
    try:
        page.screenshot(path=out_path, full_page=True)
        request.node._screenshot_path = out_path
    except Exception:
        pass


def pytest_collection_modifyitems(items):
    """Auto-apply _pw_screenshot fixture to tests that use the page fixture."""
    for item in items:
        if "page" in item.fixturenames:
            item.fixturenames.append("_pw_screenshot")


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    if report.when == "teardown":
        screenshot_path = getattr(item, "_screenshot_path", None)
        if screenshot_path and os.path.exists(screenshot_path):
            from pytest_html import extras

            report_dir = os.path.dirname(item.config.option.htmlpath or "")
            rel_path = os.path.relpath(screenshot_path, report_dir)
            extra = getattr(report, "extras", [])
            extra.append(extras.html(
                f'<div class="image"><img src="{rel_path}" '
                f'style="max-width:800px" alt="screenshot"></div>'
            ))
            report.extras = extra
