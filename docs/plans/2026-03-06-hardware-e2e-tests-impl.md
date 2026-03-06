# Hardware E2E Tests Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Automated pytest tests that exercise the ESP32 board via REST/WS API and verify electrical behavior using oscilloscope, power supply, and multimeter.

**Architecture:** Tests live in `test/hardware/`, communicate with instruments via raw TCP/SCPI using a copied `SCPIConnection` class, and talk to the board via `aiohttp`/`websockets`. Config is loaded from a gitignored `config.json`. Board is discovered via mDNS hostname with mpremote fallback.

**Tech Stack:** pytest, pytest-asyncio, aiohttp, websockets, raw TCP sockets (SCPI)

---

### Task 1: SCPIConnection class

**Files:**
- Create: `test/hardware/instruments.py`

**Step 1: Create the SCPIConnection class**

Copy and adapt from `~/projects/siglent-sdm-mcp/siglent_sdm_mcp/scpi_connection.py`. The class provides async TCP communication with SCPI instruments. Make it generic (the SDM version drains a welcome banner — keep that behavior but make it optional since the SDS and SPD don't send banners).

```python
"""Lightweight SCPI instrument clients for hardware E2E tests."""

import asyncio


class SCPIConnection:
    """Async TCP connection to a SCPI instrument."""

    def __init__(self, host: str, port: int, timeout: float = 5.0, drain_banner: bool = False):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._drain_banner = drain_banner
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._lock = asyncio.Lock()

    async def connect(self):
        self._reader, self._writer = await asyncio.wait_for(
            asyncio.open_connection(self.host, self.port),
            timeout=self.timeout,
        )
        if self._drain_banner:
            await self._read_banner()

    async def _read_banner(self):
        """Read and discard welcome banner (SDM sends one on connect)."""
        buf = b""
        try:
            while b">>" not in buf:
                chunk = await asyncio.wait_for(self._reader.read(256), timeout=2.0)
                if not chunk:
                    break
                buf += chunk
        except asyncio.TimeoutError:
            pass

    async def disconnect(self):
        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None

    async def _ensure_connected(self):
        if self._writer is None or self._writer.is_closing():
            await self.connect()

    async def query(self, command: str) -> str:
        """Send a command and return the response."""
        async with self._lock:
            await self._ensure_connected()
            self._writer.write(f"{command}\n".encode("ascii"))
            await self._writer.drain()
            try:
                response = await asyncio.wait_for(
                    self._reader.readline(),
                    timeout=self.timeout,
                )
                return response.decode("ascii").strip().lstrip("\x00")
            except asyncio.TimeoutError:
                await self.disconnect()
                raise

    async def write(self, command: str):
        """Send a command with no response expected."""
        async with self._lock:
            await self._ensure_connected()
            self._writer.write(f"{command}\n".encode("ascii"))
            await self._writer.drain()
            await asyncio.sleep(0.05)
```

**Step 2: Verify it connects to any instrument**

Quick manual check — not a unit test, just verification:

```bash
cd /home/pi/projects/optacon-firmware/.claude/worktrees/feat-api-restructure
python -c "
import asyncio
from test.hardware.instruments import SCPIConnection
async def main():
    c = SCPIConnection('192.168.30.236', 5025)
    print(await c.query('*IDN?'))
    await c.disconnect()
asyncio.run(main())
"
```

Expected: prints the SDS1104X-E identification string.

**Step 3: Commit**

```bash
git add test/hardware/instruments.py
git commit -m "feat(test): add SCPIConnection class for hardware E2E tests"
```

---

### Task 2: Instrument wrapper classes

**Files:**
- Modify: `test/hardware/instruments.py`

**Step 1: Add oscilloscope wrapper (Oscilloscope class)**

Add below `SCPIConnection` in `instruments.py`:

```python
class Oscilloscope:
    """Siglent SDS oscilloscope — measurement and configuration."""

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)

    async def connect(self):
        await self._conn.connect()

    async def disconnect(self):
        await self._conn.disconnect()

    async def identify(self) -> str:
        return await self._conn.query("*IDN?")

    async def configure_channel(self, channel: str, vdiv: str, coupling: str = "D1M",
                                 trace: bool = True, probe: int = 10):
        """Configure a channel. vdiv e.g. '500mV', '1V', '10V'."""
        await self._conn.write(f"{channel}:VDIV {vdiv}")
        await self._conn.write(f"{channel}:CPL {coupling}")
        await self._conn.write(f"{channel}:TRA {'ON' if trace else 'OFF'}")
        await self._conn.write(f"{channel}:ATTN {probe}")

    async def configure_timebase(self, timebase: str):
        """Set time/div. E.g. '2MS', '500US', '1S'."""
        await self._conn.write(f"TDIV {timebase}")

    async def configure_trigger(self, source: str, level: str, slope: str = "POS"):
        """Configure edge trigger. level e.g. '1.5V'."""
        await self._conn.write(f"{source}:TRLV {level}")
        await self._conn.write(f"{source}:TRSL {slope}")
        await self._conn.write("TRSE EDGE,SR," + source + ",HT,OFF")

    async def run(self):
        await self._conn.write("ARM")

    async def stop(self):
        await self._conn.write("STOP")

    async def auto_setup(self):
        await self._conn.write("ASET")

    async def measure(self, channel: str, parameter: str) -> str:
        """Take a measurement. Returns raw string value.

        Parameters: PKPK, FREQ, RMS, MEAN, AMPL, MAX, MIN, DUTY, RISE, FALL, etc.
        Returns '****' if measurement cannot be made.
        """
        result = await self._conn.query(f"{channel}:PAVA? {parameter}")
        # Response format: "C4:PAVA FREQ,5.024000E+02Hz" or "C4:PAVA FREQ,****"
        parts = result.split(",")
        if len(parts) >= 2:
            # Strip unit suffix (Hz, V, s, etc.)
            val = parts[1].rstrip("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ%")
            return val
        return result

    async def measure_float(self, channel: str, parameter: str) -> float | None:
        """Take a measurement and return as float, or None if '****'."""
        val = await self.measure(channel, parameter)
        if "****" in val or not val:
            return None
        return float(val)
```

**Step 2: Add power supply wrapper (PowerSupply class)**

```python
class PowerSupply:
    """Siglent SPD power supply — output control and measurement."""

    def __init__(self, host: str, port: int = 5025):
        self._conn = SCPIConnection(host, port)

    async def connect(self):
        await self._conn.connect()

    async def disconnect(self):
        await self._conn.disconnect()

    async def identify(self) -> str:
        return await self._conn.query("*IDN?")

    async def set_output(self, channel: str, state: bool):
        """Turn channel output on/off. channel: 'CH1', 'CH2'."""
        await self._conn.write(f"OUTPut {channel},{'ON' if state else 'OFF'}")

    async def set_voltage(self, channel: str, voltage: float):
        await self._conn.write(f"{channel}:VOLTage {voltage:.3f}")

    async def set_current(self, channel: str, current: float):
        await self._conn.write(f"{channel}:CURRent {current:.3f}")

    async def measure_voltage(self, channel: str) -> float:
        result = await self._conn.query(f"MEASure:VOLTage? {channel}")
        return float(result)

    async def measure_current(self, channel: str) -> float:
        result = await self._conn.query(f"MEASure:CURRent? {channel}")
        return float(result)

    async def measure_power(self, channel: str) -> float:
        result = await self._conn.query(f"MEASure:POWEr? {channel}")
        return float(result)
```

**Step 3: Add multimeter wrapper (Multimeter class)**

```python
class Multimeter:
    """Siglent SDM multimeter — precision current measurement."""

    def __init__(self, host: str, port: int = 5024):
        self._conn = SCPIConnection(host, port, drain_banner=True)

    async def connect(self):
        await self._conn.connect()

    async def disconnect(self):
        await self._conn.disconnect()

    async def identify(self) -> str:
        return await self._conn.query("*IDN?")

    async def configure_dc_current(self, range: str = "6"):
        """Configure for DC current measurement. range: '6' for 6A."""
        await self._conn.write(f"CONFigure:CURRent:DC {range}")

    async def configure_dc_voltage(self, range: str = "AUTO"):
        """Configure for DC voltage measurement."""
        if range.upper() == "AUTO":
            await self._conn.write("CONFigure:VOLTage:DC")
            await self._conn.write("VOLTage:DC:RANGe:AUTO ON")
        else:
            await self._conn.write(f"CONFigure:VOLTage:DC {range}")

    async def read(self) -> float:
        """Trigger a measurement and return the reading."""
        result = await self._conn.query("READ?")
        return float(result)

    async def measure_dc_current(self, range: str = "6") -> float:
        """One-shot DC current measurement."""
        result = await self._conn.query(f"MEASure:CURRent:DC? {range}")
        return float(result)
```

**Step 4: Verify all three connect**

```bash
python -c "
import asyncio
from test.hardware.instruments import Oscilloscope, PowerSupply, Multimeter
async def main():
    scope = Oscilloscope('192.168.30.236')
    await scope.connect()
    print('Scope:', await scope.identify())
    await scope.disconnect()

    psu = PowerSupply('192.168.30.134')
    await psu.connect()
    print('PSU:', await psu.identify())
    await psu.disconnect()

    dmm = Multimeter('192.168.30.17')
    await dmm.connect()
    print('DMM:', await dmm.identify())
    await dmm.disconnect()
asyncio.run(main())
"
```

Expected: all three print identification strings.

**Step 5: Commit**

```bash
git add test/hardware/instruments.py
git commit -m "feat(test): add oscilloscope, power supply, and multimeter wrappers"
```

---

### Task 3: Config file and pytest configuration

**Files:**
- Create: `test/hardware/config.example.json`
- Create: `test/hardware/__init__.py` (empty)
- Modify: `.gitignore` (add `test/hardware/config.json`)
- Create: `pyproject.toml` (pytest config)

**Step 1: Create config example**

```json
{
  "sds": "192.168.30.236:5025",
  "spd": "192.168.30.134:5025",
  "sdm": "192.168.30.17:5024",
  "board_hostname": "esp-optacon",
  "oscilloscope_channels": {
    "out_plus": "C2",
    "polarity": "C3",
    "in_plus": "C4"
  },
  "power_supply_channel": "CH1",
  "tolerance": 0.20
}
```

**Step 2: Create actual config.json**

Same content as above — this is the real config for the local lab.

**Step 3: Create empty `__init__.py`**

```python
```

**Step 4: Add to `.gitignore`**

Append this line:
```
test/hardware/config.json
```

**Step 5: Create `pyproject.toml` with pytest config**

```toml
[tool.pytest.ini_options]
testpaths = ["test/integration", "test/e2e"]
asyncio_mode = "auto"
markers = [
    "hardware: tests requiring physical lab instruments (not run in CI)",
]
```

This excludes `test/hardware/` from default `pytest test/` runs. Hardware tests must be targeted explicitly: `pytest test/hardware/ -v`.

**Step 6: Verify default tests still work**

```bash
pytest test/ --co -q | tail -5
```

Expected: lists integration + e2e tests, no hardware tests.

**Step 7: Commit**

```bash
git add test/hardware/config.example.json test/hardware/__init__.py .gitignore pyproject.toml
git commit -m "feat(test): add hardware test config and pytest configuration"
```

---

### Task 4: Conftest fixtures

**Files:**
- Create: `test/hardware/conftest.py`

**Step 1: Write conftest with all fixtures**

```python
"""Hardware E2E test fixtures — real board + real instruments."""

import asyncio
import json
import os
import subprocess

import aiohttp
import pytest
import pytest_asyncio

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
    """Measurement tolerance (fractional, e.g. 0.20 = 20%)."""
    return config.get("tolerance", 0.20)


@pytest.fixture(scope="session")
def channels(config):
    """Oscilloscope channel mapping."""
    return config["oscilloscope_channels"]


@pytest.fixture(scope="session")
def psu_channel(config):
    """Power supply channel name (e.g. 'CH1')."""
    return config["power_supply_channel"]


@pytest_asyncio.fixture(scope="session")
async def oscilloscope(config):
    """Connected oscilloscope instance for the test session."""
    host, port = _parse_address(config["sds"])
    scope = Oscilloscope(host, port)
    await scope.connect()
    yield scope
    await scope.disconnect()


@pytest_asyncio.fixture(scope="session")
async def power_supply(config):
    """Connected power supply instance for the test session."""
    host, port = _parse_address(config["spd"])
    psu = PowerSupply(host, port)
    await psu.connect()
    yield psu
    await psu.disconnect()


@pytest_asyncio.fixture(scope="session")
async def multimeter(config):
    """Connected multimeter instance, configured for DC current 6A range."""
    host, port = _parse_address(config["sdm"])
    dmm = Multimeter(host, port)
    await dmm.connect()
    await dmm.configure_dc_current("6")
    yield dmm
    await dmm.disconnect()


def _discover_board(hostname: str) -> str:
    """Discover board IP: try mDNS hostname first, fallback to mpremote."""
    import socket

    mdns = f"{hostname}.local"
    try:
        ip = socket.gethostbyname(mdns)
        return ip
    except socket.gaierror:
        pass

    # Fallback: mpremote
    try:
        result = subprocess.run(
            ["mpremote", "exec", "import wifi; print(wifi.ip)"],
            capture_output=True, text=True, timeout=15,
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
    """Resolved board HTTP base URL, verified reachable."""
    import urllib.request

    hostname = config["board_hostname"]
    ip = _discover_board(hostname)
    url = f"http://{ip}"

    # Verify web server is up (retry a few times — board may be booting)
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


@pytest_asyncio.fixture
async def board_ws(board_url):
    """Async context manager for WebSocket connection to board."""
    import websockets

    ws_url = board_url.replace("http://", "ws://") + "/ws"

    async def _connect():
        ws = await websockets.connect(ws_url)
        # Read initial status message
        await ws.recv()
        return ws

    return _connect


@pytest_asyncio.fixture
async def board_api(board_url):
    """aiohttp session with board base URL."""
    async with aiohttp.ClientSession(base_url=board_url) as session:
        yield session
```

**Step 2: Verify fixtures load**

```bash
pytest test/hardware/ --co -q 2>&1 | head -5
```

Expected: no collection errors (no tests yet, but fixtures should load).

**Step 3: Commit**

```bash
git add test/hardware/conftest.py
git commit -m "feat(test): add hardware test fixtures (instruments, board discovery, config)"
```

---

### Task 5: Signal output tests

**Files:**
- Create: `test/hardware/test_signal_output.py`

**Step 1: Write signal output tests**

```python
"""Test signal output verification on oscilloscope."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware


@pytest.fixture
def _configure_scope(oscilloscope, channels):
    """Configure oscilloscope channels for signal measurement."""
    async def _setup(freq_hz):
        # Set appropriate timebase for frequency
        if freq_hz <= 100:
            timebase = "5MS"
        elif freq_hz <= 300:
            timebase = "2MS"
        else:
            timebase = "1MS"

        ch_in = channels["in_plus"]
        ch_out = channels["out_plus"]

        await oscilloscope.configure_channel(ch_in, vdiv="1V", coupling="D1M", probe=10)
        await oscilloscope.configure_channel(ch_out, vdiv="20V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase(timebase)
        await oscilloscope.configure_trigger(ch_in, level="1V", slope="POS")
        await oscilloscope.run()
        await asyncio.sleep(0.5)

    return _setup


@pytest.mark.asyncio
@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
@pytest.mark.parametrize("freq_hz", [50, 250, 500])
async def test_signal_frequency(board_ws, oscilloscope, channels, tolerance,
                                 _configure_scope, waveform, freq_hz):
    """Verify measured frequency matches requested frequency."""
    ws = await board_ws()
    try:
        await _configure_scope(freq_hz)

        # Set frequency and start
        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": freq_hz,
            "amplitude": 100,
            "waveform": waveform,
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()

        # Allow signal to stabilize
        await asyncio.sleep(1.0)

        # Measure frequency on IN+
        measured_freq = await oscilloscope.measure_float(channels["in_plus"], "FREQ")
        assert measured_freq is not None, f"Could not measure frequency on {channels['in_plus']}"
        assert abs(measured_freq - freq_hz) / freq_hz <= tolerance, (
            f"Frequency mismatch: expected {freq_hz} Hz, got {measured_freq} Hz "
            f"(tolerance {tolerance*100}%)"
        )

        # Stop
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("waveform", ["sine", "triangle", "square"])
async def test_signal_amplitude(board_ws, oscilloscope, channels, tolerance,
                                 _configure_scope, waveform):
    """Verify signal has non-trivial amplitude on IN+ and OUT+."""
    ws = await board_ws()
    try:
        await _configure_scope(250)

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 250,
            "amplitude": 100,
            "waveform": waveform,
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()

        await asyncio.sleep(1.0)

        # IN+ should have measurable amplitude
        in_pkpk = await oscilloscope.measure_float(channels["in_plus"], "PKPK")
        assert in_pkpk is not None and in_pkpk > 0.1, (
            f"IN+ PKPK too low: {in_pkpk}V"
        )

        # OUT+ should have amplified signal
        out_pkpk = await oscilloscope.measure_float(channels["out_plus"], "PKPK")
        assert out_pkpk is not None and out_pkpk > 5.0, (
            f"OUT+ PKPK too low: {out_pkpk}V"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()
```

**Step 2: Run (expect pass if board + instruments connected)**

```bash
pytest test/hardware/test_signal_output.py -v --timeout=60
```

**Step 3: Commit**

```bash
git add test/hardware/test_signal_output.py
git commit -m "feat(test): add signal output hardware tests (frequency + amplitude)"
```

---

### Task 6: Fullwave and polarity tests

**Files:**
- Create: `test/hardware/test_fullwave.py`

**Step 1: Write fullwave/polarity tests**

```python
"""Test fullwave mode and polarity toggling."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250


@pytest.mark.asyncio
async def test_fullwave_polarity_toggles(board_ws, oscilloscope, channels, tolerance):
    """In fullwave mode, polarity channel should toggle at signal frequency."""
    ws = await board_ws()
    try:
        ch_pol = channels["polarity"]

        await oscilloscope.configure_channel(ch_pol, vdiv="2V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.configure_trigger(ch_pol, level="2V", slope="POS")
        await oscilloscope.run()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": FREQ_HZ,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": True,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.0)

        # Polarity should be toggling — measure frequency
        pol_freq = await oscilloscope.measure_float(ch_pol, "FREQ")
        assert pol_freq is not None, "Could not measure polarity frequency"
        assert abs(pol_freq - FREQ_HZ) / FREQ_HZ <= tolerance, (
            f"Polarity frequency mismatch: expected {FREQ_HZ} Hz, got {pol_freq} Hz"
        )

        # Polarity should be a digital signal — PKPK close to 3.3V
        pol_pkpk = await oscilloscope.measure_float(ch_pol, "PKPK")
        assert pol_pkpk is not None and pol_pkpk > 2.0, (
            f"Polarity PKPK too low for digital signal: {pol_pkpk}V"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_non_fullwave_polarity_static(board_ws, oscilloscope, channels):
    """In non-fullwave mode, polarity channel should be static (no toggling)."""
    ws = await board_ws()
    try:
        ch_pol = channels["polarity"]

        await oscilloscope.configure_channel(ch_pol, vdiv="2V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.run()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": FREQ_HZ,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.0)

        # Polarity should NOT toggle — PKPK should be near zero (static)
        pol_pkpk = await oscilloscope.measure_float(ch_pol, "PKPK")
        assert pol_pkpk is not None and pol_pkpk < 1.0, (
            f"Polarity should be static in non-fullwave mode, but PKPK={pol_pkpk}V"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_fullwave_doubles_frequency(board_ws, oscilloscope, channels, tolerance):
    """In fullwave mode, IN+ frequency should be 2x the set frequency (|sin|)."""
    ws = await board_ws()
    try:
        ch_in = channels["in_plus"]

        await oscilloscope.configure_channel(ch_in, vdiv="1V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.configure_trigger(ch_in, level="1.5V", slope="POS")
        await oscilloscope.run()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": FREQ_HZ,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": True,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.0)

        # IN+ should show 2x frequency in fullwave
        in_freq = await oscilloscope.measure_float(ch_in, "FREQ")
        expected_freq = FREQ_HZ * 2
        assert in_freq is not None, "Could not measure IN+ frequency"
        assert abs(in_freq - expected_freq) / expected_freq <= tolerance, (
            f"IN+ frequency in fullwave: expected {expected_freq} Hz, got {in_freq} Hz"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()
```

**Step 2: Run**

```bash
pytest test/hardware/test_fullwave.py -v --timeout=60
```

**Step 3: Commit**

```bash
git add test/hardware/test_fullwave.py
git commit -m "feat(test): add fullwave/polarity hardware tests"
```

---

### Task 7: Gain level tests

**Files:**
- Create: `test/hardware/test_gain.py`

**Step 1: Write gain level tests**

```python
"""Test gain level amplitude scaling on OUT+."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

FREQ_HZ = 250
GAIN_LEVELS = [25, 50, 75, 100]


@pytest.mark.asyncio
@pytest.mark.parametrize("gain", GAIN_LEVELS)
async def test_gain_amplitude(board_ws, oscilloscope, channels, tolerance, gain):
    """OUT+ PKPK should be proportional to gain setting."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]

        # Scale vdiv based on expected amplitude
        vdiv = f"{max(gain // 4, 5)}V"
        await oscilloscope.configure_channel(ch_out, vdiv=vdiv, coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.configure_trigger(channels["in_plus"], level="1V", slope="POS")
        await oscilloscope.run()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": FREQ_HZ,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": gain}))
        await ws.recv()
        await asyncio.sleep(1.5)

        out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert out_pkpk is not None, f"Could not measure OUT+ PKPK at gain={gain}"

        # Expected PKPK roughly equals the gain setting in volts
        expected_vpp = float(gain)
        assert abs(out_pkpk - expected_vpp) / expected_vpp <= tolerance, (
            f"Gain {gain}: expected ~{expected_vpp}Vpp, got {out_pkpk}Vpp "
            f"(tolerance {tolerance*100}%)"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()
```

**Step 2: Run**

```bash
pytest test/hardware/test_gain.py -v --timeout=120
```

**Step 3: Commit**

```bash
git add test/hardware/test_gain.py
git commit -m "feat(test): add gain level amplitude hardware tests"
```

---

### Task 8: Power consumption tests

**Files:**
- Create: `test/hardware/test_power.py`

**Step 1: Write power consumption tests**

```python
"""Test power consumption: idle, active, per-pin, all-pins."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

# Thresholds to be tuned from baseline measurements.
# Starting with generous ranges — tighten after calibration.
IDLE_CURRENT_MAX_A = 0.200  # 200mA max idle
ACTIVE_CURRENT_MAX_A = 0.500  # 500mA max with signal running
PIN_CURRENT_MIN_A = 0.001  # 1mA min per pin (above baseline)
PIN_CURRENT_MAX_A = 0.050  # 50mA max per pin (above baseline)
ALL_PINS_CURRENT_MAX_A = 1.0  # 1A max with all pins on


@pytest.mark.asyncio
async def test_idle_current(board_ws, multimeter):
    """Board idle (no signal, no pins) should draw below threshold."""
    ws = await board_ws()
    try:
        # Ensure stopped, all pins off
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()
        await asyncio.sleep(1.0)

        current = await multimeter.read()
        assert current >= 0, f"Negative current reading: {current}A"
        assert current < IDLE_CURRENT_MAX_A, (
            f"Idle current too high: {current:.4f}A (max {IDLE_CURRENT_MAX_A}A)"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_active_current(board_ws, multimeter, tolerance):
    """Board with signal running should draw within expected range."""
    ws = await board_ws()
    try:
        # Ensure pins off
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 250,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.5)

        current = await multimeter.read()
        assert current >= 0, f"Negative current reading: {current}A"
        assert current < ACTIVE_CURRENT_MAX_A, (
            f"Active current too high: {current:.4f}A (max {ACTIVE_CURRENT_MAX_A}A)"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("pin", list(range(20)))
async def test_single_pin_current(board_ws, multimeter, tolerance, pin):
    """Each individual pin should draw measurable current within range."""
    ws = await board_ws()
    try:
        # Baseline: all pins off, no signal
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()
        await asyncio.sleep(0.5)
        baseline = await multimeter.read()

        # Enable single pin
        await ws.send(json.dumps({"cmd": "set_pin", "pin": pin, "value": 1}))
        await ws.recv()
        await asyncio.sleep(0.5)
        with_pin = await multimeter.read()

        # Disable pin
        await ws.send(json.dumps({"cmd": "set_pin", "pin": pin, "value": 0}))
        await ws.recv()

        delta = with_pin - baseline
        assert delta >= PIN_CURRENT_MIN_A, (
            f"Pin {pin}: current delta too low: {delta:.4f}A (min {PIN_CURRENT_MIN_A}A)"
        )
        assert delta <= PIN_CURRENT_MAX_A, (
            f"Pin {pin}: current delta too high: {delta:.4f}A (max {PIN_CURRENT_MAX_A}A)"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_all_pins_current(board_ws, multimeter, tolerance):
    """All 20 pins enabled should draw current within expected range."""
    ws = await board_ws()
    try:
        # Baseline: all pins off
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()
        await asyncio.sleep(0.5)
        baseline = await multimeter.read()

        # All pins on (bits 0-19 = 0xFFFFF)
        await ws.send(json.dumps({"cmd": "set_all", "value": 0xFFFFF}))
        await ws.recv()
        await asyncio.sleep(1.0)
        all_on = await multimeter.read()

        # All pins off
        await ws.send(json.dumps({"cmd": "set_all", "value": 0}))
        await ws.recv()

        delta = all_on - baseline
        assert delta > 0, f"All pins on should draw more current than baseline"
        assert all_on < ALL_PINS_CURRENT_MAX_A, (
            f"All pins current too high: {all_on:.4f}A (max {ALL_PINS_CURRENT_MAX_A}A)"
        )
    finally:
        await ws.close()
```

**Step 2: Run**

```bash
pytest test/hardware/test_power.py -v --timeout=300
```

Note: per-pin tests run 20 iterations, may take a few minutes.

**Step 3: Commit**

```bash
git add test/hardware/test_power.py
git commit -m "feat(test): add power consumption hardware tests (idle, active, per-pin, all-pins)"
```

---

### Task 9: Start/stop and negative tests

**Files:**
- Create: `test/hardware/test_start_stop.py`

**Step 1: Write start/stop + negative tests**

```python
"""Test start/stop behavior and negative tests (no signal when stopped)."""

import asyncio
import json

import pytest

pytestmark = pytest.mark.hardware

NOISE_FLOOR_V = 0.5  # PKPK below this = no signal
SIGNAL_THRESHOLD_V = 5.0  # PKPK above this = signal present


@pytest.mark.asyncio
async def test_no_signal_before_start(board_ws, oscilloscope, channels):
    """Negative: OUT+ should show no signal before any start command."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]

        # Ensure stopped
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await asyncio.sleep(0.5)

        await oscilloscope.configure_channel(ch_out, vdiv="10V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.run()
        await asyncio.sleep(1.0)

        pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert pkpk is None or pkpk < NOISE_FLOOR_V, (
            f"OUT+ should be quiet before start, but PKPK={pkpk}V"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_signal_appears_on_start(board_ws, oscilloscope, channels):
    """Positive: OUT+ should show signal after start."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]

        await oscilloscope.configure_channel(ch_out, vdiv="20V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.configure_trigger(channels["in_plus"], level="1V", slope="POS")
        await oscilloscope.run()

        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 250,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.0)

        pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert pkpk is not None and pkpk > SIGNAL_THRESHOLD_V, (
            f"OUT+ should show signal after start, but PKPK={pkpk}V"
        )

        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_signal_disappears_on_stop(board_ws, oscilloscope, channels):
    """Positive: OUT+ should return to noise floor after stop."""
    ws = await board_ws()
    try:
        ch_out = channels["out_plus"]
        ch_in = channels["in_plus"]

        await oscilloscope.configure_channel(ch_out, vdiv="10V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("2MS")
        await oscilloscope.run()

        # Start signal
        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 250,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(1.0)

        # Stop signal
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()
        await asyncio.sleep(1.0)

        # OUT+ should be quiet
        out_pkpk = await oscilloscope.measure_float(ch_out, "PKPK")
        assert out_pkpk is None or out_pkpk < NOISE_FLOOR_V, (
            f"OUT+ should be quiet after stop, but PKPK={out_pkpk}V"
        )

        # IN+ should also be quiet
        in_pkpk = await oscilloscope.measure_float(ch_in, "PKPK")
        assert in_pkpk is None or in_pkpk < NOISE_FLOOR_V, (
            f"IN+ should be quiet after stop, but PKPK={in_pkpk}V"
        )
    finally:
        await ws.close()


@pytest.mark.asyncio
async def test_double_stop_safe(board_ws, oscilloscope, channels):
    """Negative: calling stop twice should not error."""
    ws = await board_ws()
    try:
        await ws.send(json.dumps({"cmd": "stop"}))
        resp1 = json.loads(await ws.recv())

        await ws.send(json.dumps({"cmd": "stop"}))
        resp2 = json.loads(await ws.recv())

        assert "error" not in resp1, f"First stop errored: {resp1}"
        assert "error" not in resp2, f"Second stop errored: {resp2}"
    finally:
        await ws.close()
```

**Step 2: Run**

```bash
pytest test/hardware/test_start_stop.py -v --timeout=60
```

**Step 3: Commit**

```bash
git add test/hardware/test_start_stop.py
git commit -m "feat(test): add start/stop and negative hardware tests"
```

---

### Task 10: Frequency sweep tests

**Files:**
- Create: `test/hardware/test_sweep.py`

**Step 1: Write sweep tests**

```python
"""Test frequency sweep verification."""

import asyncio
import json

import aiohttp
import pytest

pytestmark = pytest.mark.hardware


@pytest.mark.asyncio
async def test_sweep_frequency_increases(board_url, board_ws, oscilloscope, channels, tolerance):
    """During a sweep from 50-500 Hz, measured frequency should increase over time."""
    ws = await board_ws()
    try:
        ch_in = channels["in_plus"]

        await oscilloscope.configure_channel(ch_in, vdiv="1V", coupling="D1M", probe=10)
        await oscilloscope.configure_timebase("5MS")
        await oscilloscope.configure_trigger(ch_in, level="1V", slope="POS")
        await oscilloscope.run()

        # Set initial frequency and start
        await ws.send(json.dumps({
            "cmd": "set_frequency_analog",
            "hz": 50,
            "amplitude": 100,
            "waveform": "sine",
            "fullwave": False,
        }))
        await ws.recv()
        await ws.send(json.dumps({"cmd": "start", "gain": 100}))
        await ws.recv()
        await asyncio.sleep(0.5)

        # Start sweep via direct MicroPython exec (sweep_analog is not a WS command)
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{board_url}/api/exec",
                json={"code": "pa.sweep_analog(50, 500, 5000, waveform='sine', gain=100)"},
            ) as resp:
                # Don't await response — sweep is blocking on the board
                pass

        # Sample frequency at intervals during the sweep
        frequencies = []
        for i in range(5):
            await asyncio.sleep(0.8)
            freq = await oscilloscope.measure_float(ch_in, "FREQ")
            if freq is not None:
                frequencies.append(freq)

        # Stop after sweep
        await ws.send(json.dumps({"cmd": "stop"}))
        await ws.recv()

        assert len(frequencies) >= 3, f"Not enough frequency samples: {frequencies}"

        # Verify frequency trend is increasing
        increasing_count = sum(
            1 for i in range(len(frequencies) - 1) if frequencies[i + 1] > frequencies[i]
        )
        assert increasing_count >= len(frequencies) // 2, (
            f"Frequency should generally increase during sweep. Samples: {frequencies}"
        )

        # First sample should be near start, last near end
        assert frequencies[0] < 200, f"First frequency too high for sweep start: {frequencies[0]}"
        assert frequencies[-1] > 200, f"Last frequency too low for sweep end: {frequencies[-1]}"
    finally:
        await ws.close()
```

**Step 2: Run**

```bash
pytest test/hardware/test_sweep.py -v --timeout=120
```

**Step 3: Commit**

```bash
git add test/hardware/test_sweep.py
git commit -m "feat(test): add frequency sweep hardware test"
```

---

### Task 11: Final integration — run all hardware tests

**Step 1: Create config.json from example (if not already done in Task 3)**

```bash
cp test/hardware/config.example.json test/hardware/config.json
```

**Step 2: Run the full hardware test suite**

```bash
pytest test/hardware/ -v --timeout=300
```

**Step 3: Verify default test runs still exclude hardware**

```bash
pytest test/ -v --timeout=60
```

Expected: only integration + e2e tests run, no hardware tests.

**Step 4: Fix any failures, tune thresholds**

Power thresholds and signal tolerances may need adjustment based on actual measurements. Update constants in the test files as needed.

**Step 5: Commit any threshold tuning**

```bash
git add test/hardware/
git commit -m "fix(test): tune hardware test thresholds from baseline measurements"
```

---

Plan complete and saved to `docs/plans/2026-03-06-hardware-e2e-tests-impl.md`. Two execution options:

**1. Subagent-Driven (this session)** — I dispatch fresh subagent per task, review between tasks, fast iteration

**2. Parallel Session (separate)** — Open new session with executing-plans, batch execution with checkpoints

**Which approach?**