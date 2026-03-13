"""Hardware E2E test fixtures — real board + real instruments.

Scope message suppression: "Enable test is off!" hidden by enabling pass/fail
(PFEN ON) at connect. "Fine adjustment" overlay (triggered by VDIV/TDIV SCPI
commands) auto-dismisses after ~3s — screenshot fixture waits before capture.
"""

import asyncio
import json
import os
import subprocess
import threading
import time

import pytest

from test.e2e.ota_server import OTAMockServer
from test.hardware.board_client import REPLBoardClient, WSBoardClient
from test.hardware.board_http import BoardHTTPClient
from test.hardware.instruments import Multimeter, Oscilloscope, PowerSupply
from test.hardware.ota_helpers import get_local_ip

_HERE = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_HERE, "config.json")


def _parse_address(addr: str) -> tuple[str, int]:
    """Parse 'host:port' string into (host, port) tuple."""
    host, port = addr.rsplit(":", 1)
    return host, int(port)


@pytest.fixture(scope="session")
def config():
    """Load hardware test configuration from config.json."""
    if not os.path.exists(_CONFIG_PATH):
        pytest.fail(
            f"Hardware test config not found: {_CONFIG_PATH}\n"
            f"Copy config.example.json to config.json and update with your lab settings."
        )
    with open(_CONFIG_PATH) as f:
        return json.load(f)


@pytest.fixture(scope="session")
def tolerance(config):
    return config.get("tolerance", 0.20)


@pytest.fixture(scope="session")
def channels(config):
    return config["oscilloscope_channels"]


@pytest.fixture(scope="session")
def psu_channel(config):
    return config["power_supply_channel"]


@pytest.fixture(scope="session")
def oscilloscope(config):
    host, port = _parse_address(config["sds"])
    scope = Oscilloscope(host, port)
    scope.connect()
    # Enable pass/fail test to suppress "Enable test is off!" popup in screenshots
    scope._conn.write("PFEN ON")
    yield scope
    scope.disconnect()


@pytest.fixture(scope="session")
def power_supply(config, psu_channel):
    host, port = _parse_address(config["spd"])
    psu = PowerSupply(host, port)
    psu.connect()
    voltage = config.get("power_supply_voltage", 5.0)
    current_limit = config.get("power_supply_current_limit", 0.5)
    was_on = psu.is_output_on(psu_channel)
    psu.ensure_output_on(psu_channel, voltage, current_limit)
    if not was_on:
        import time

        time.sleep(3)  # let board boot after power-on
    yield psu
    psu.disconnect()


@pytest.fixture(scope="session")
def multimeter(config):
    host, port = _parse_address(config["sdm"])
    dmm = Multimeter(host, port)
    dmm.connect()
    dmm.configure_dc_current("0.6")
    yield dmm
    dmm.set_local()
    dmm.disconnect()


def _discover_board(hostname: str) -> str:
    import socket

    mdns = f"{hostname}.local"
    try:
        ip = socket.gethostbyname(mdns)
        return ip
    except socket.gaierror:
        pass

    try:
        result = subprocess.run(
            ["mpremote", "exec", "import wifi; print(wifi.ip)"],
            capture_output=True,
            text=True,
            timeout=15,
        )
        ip = result.stdout.strip()
        if ip and not ip.startswith("Traceback"):
            return ip
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    pytest.fail(
        f"Cannot discover board. mDNS '{mdns}' failed and mpremote fallback failed.\n"
        "Ensure the board is powered on and connected to WiFi."
    )


@pytest.fixture(scope="session")
def board_url(config, power_supply):
    import urllib.request

    hostname = config["board_hostname"]
    ip = _discover_board(hostname)
    url = f"http://{ip}"

    for attempt in range(5):
        try:
            req = urllib.request.urlopen(f"{url}/api/device/status", timeout=5)
            if req.status == 200:
                return url
        except Exception:
            if attempt < 4:
                import time

                time.sleep(2)

    pytest.fail(f"Board at {url} is not responding to /api/device/status")


def pytest_configure(config):
    """Auto-enable HTML report for hardware tests."""
    if not config.option.htmlpath:
        config.option.htmlpath = os.path.join(_HERE, "report.html")


def pytest_addoption(parser):
    parser.addoption(
        "--board-client",
        default="ws",
        choices=["ws", "repl"],
        help="Board client type: ws (WebSocket API) or repl (mpremote serial)",
    )
    parser.addoption(
        "--board-port",
        default=None,
        help="Serial port for REPL client (e.g. /dev/ttyACM0)",
    )


@pytest.fixture
def board(board_url, request):
    client_type = request.config.getoption("--board-client")
    if client_type == "repl":
        port = request.config.getoption("--board-port")
        client = REPLBoardClient(port)
    else:
        ws_url = board_url.replace("http://", "ws://") + "/ws"
        client = WSBoardClient(ws_url)
    client.connect()
    yield client
    try:
        client.stop()
    finally:
        client.close()


_SCREENSHOT_DIR = os.path.join(_HERE, "screenshots")


_ALL_SCOPE_CHANNELS = ["C1", "C2", "C3", "C4"]


_DEFAULT_CHANNELS = {"C2": "20V", "C4": "1V"}  # OUT+ and IN+


@pytest.fixture(autouse=True)
def _reset_scope_channels(request):
    """Converge scope to default channels before each test.

    Uses cached state to skip redundant SCPI writes — if channels already
    match defaults from the previous test, no commands are sent.
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

    Only runs for tests that use the 'board' fixture (signal tests).
    """
    if "board" not in request.fixturenames:
        yield
        return
    oscilloscope = request.getfixturevalue("oscilloscope")
    yield
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

        # Wait for "Fine adjustment" overlay triggered by VDIV/TDIV commands.
        # Only sleeps the remaining time since the last triggering command.
        OVERLAY_DURATION = 4.5
        remaining = OVERLAY_DURATION - (time.monotonic() - oscilloscope._overlay_triggered)
        if remaining > 0:
            time.sleep(remaining)
        bmp_data = oscilloscope.screenshot()
        img = Image.open(io.BytesIO(bmp_data))
        img.save(out_path, "PNG")
        # Store path for pytest-html hook to pick up
        request.node._screenshot_path = out_path
    except Exception as e:
        import warnings

        warnings.warn(f"Screenshot failed: {e}", stacklevel=2)


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
                f'style="max-width:800px" alt="scope screenshot"></div>'
            ))
            report.extras = extra


@pytest.fixture
def configure_channel(oscilloscope):
    """Configure a single oscilloscope channel.

    AC coupling by default: OUT+ has ~30V DC bias, IN+ has ~1.6V DC bias.
    Use coupling="D1M" for polarity channel (digital signal).
    """

    def _setup(ch, vdiv, coupling="A1M", probe=10):
        oscilloscope.configure_channel(ch, vdiv=vdiv, coupling=coupling, probe=probe)

    return _setup


@pytest.fixture
def configure_timebase(oscilloscope):
    """Set oscilloscope timebase from frequency."""

    def _setup(freq_hz):
        if freq_hz <= 60:
            timebase = "10MS"
        elif freq_hz <= 200:
            timebase = "5MS"
        elif freq_hz <= 500:
            timebase = "2MS"
        else:
            timebase = "1MS"
        oscilloscope.configure_timebase(timebase)

    return _setup


@pytest.fixture
def configure_trigger(oscilloscope):
    """Set trigger source and level. Level auto-picked from coupling if not given."""

    def _setup(ch, level=None, slope="POS", coupling="A1M", trigger_coupling="DC"):
        if level is None:
            level = "0V" if coupling == "A1M" else "0.5V"
        oscilloscope.configure_trigger(
            ch, level=level, slope=slope, coupling=trigger_coupling,
        )

    return _setup


@pytest.fixture
def start_acquisition(oscilloscope):
    """Wait for pending operations and start acquisition."""

    def _setup():
        oscilloscope.wait_ready()
        oscilloscope.run()
        # Accumulate enough waveform cycles for stable measurements
        time.sleep(1.0)

    return _setup


@pytest.fixture
def clear_measurements(oscilloscope):
    """Clear all scope measurements after the test."""
    yield
    oscilloscope._conn.write("PACL")


# --- OTA test fixtures ---


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
