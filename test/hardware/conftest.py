"""Hardware E2E test fixtures — real board + real instruments."""

import json
import os
import subprocess

import pytest

from test.hardware.board_client import BoardClient
from test.hardware.instruments import Multimeter, Oscilloscope, PowerSupply

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
    yield scope
    scope.disconnect()


@pytest.fixture(scope="session")
def power_supply(config):
    host, port = _parse_address(config["spd"])
    psu = PowerSupply(host, port)
    psu.connect()
    yield psu
    psu.disconnect()


@pytest.fixture(scope="session")
def multimeter(config):
    host, port = _parse_address(config["sdm"])
    dmm = Multimeter(host, port)
    dmm.connect()
    dmm.configure_dc_current("0.6")
    yield dmm
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
def board_url(config):
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


@pytest.fixture
def board(board_url):
    ws_url = board_url.replace("http://", "ws://") + "/ws"
    client = BoardClient(ws_url)
    client.connect()
    yield client
    client.close()


@pytest.fixture
def configure_scope(oscilloscope, channels):
    """Configure oscilloscope for signal measurement at a given frequency."""

    def _setup(freq_hz, ch=None, vdiv="20V"):
        if freq_hz <= 100:
            timebase = "5MS"
        elif freq_hz <= 300:
            timebase = "2MS"
        else:
            timebase = "1MS"

        target_ch = ch or channels["in_plus"]
        oscilloscope.configure_channel(target_ch, vdiv=vdiv, coupling="D1M", probe=10)
        oscilloscope.configure_timebase(timebase)
        oscilloscope.configure_trigger(channels["in_plus"], level="1V", slope="POS")
        oscilloscope.run()

    return _setup
