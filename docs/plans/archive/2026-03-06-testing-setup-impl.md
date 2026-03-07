# Testing Setup Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a comprehensive test suite with integration tests (CPython + mocked C modules), E2E tests (API/WebSocket + Playwright), OTA mock server, hardware identity partition, and provisioning script.

**Architecture:** Mock boundary at C module level (`pz_drive`, `sam`, `board_utils`) plus MicroPython builtins. `web_server.py` refactored for dependency injection so E2E tests run the real server code. Hardware identity stored in a separate NVS partition, read at runtime, sent as HTTP headers during OTA.

**Tech Stack:** pytest, pytest-asyncio, pytest-playwright, microdot (CPython-compatible), aiohttp, websockets

**Design doc:** `docs/plans/2026-03-06-testing-setup-design.md`

---

## Phase 1: Foundation (all tasks independent — run in parallel)

### Task 1: Mock C Modules

Create mock implementations of all C modules and MicroPython builtins.

**Files:**
- Create: `test/mocks/__init__.py`
- Create: `test/mocks/pz_drive.py`
- Create: `test/mocks/sam.py`
- Create: `test/mocks/board_utils.py`
- Create: `test/mocks/micropython_builtins.py`

**Step 1: Create `test/mocks/__init__.py`**

```python
"""Mock C modules for testing under CPython."""
```

**Step 2: Create `test/mocks/pz_drive.py`**

Every function from the real `pz_drive` C module, backed by MagicMock with sensible defaults:

```python
"""Mock pz_drive C module."""
from unittest.mock import MagicMock

# PWM / Analog DDS
pwm_set_frequency = MagicMock()
pwm_start = MagicMock()
pwm_stop = MagicMock()
pwm_is_running = MagicMock(return_value=False)
pwm_set_frequency_live = MagicMock()
pwm_set_sweep = MagicMock()
pwm_is_sweep_done = MagicMock(return_value=True)
pwm_play_samples = MagicMock()
pwm_is_sample_done = MagicMock(return_value=True)

# FIFO / Digital
fifo_start = MagicMock()
fifo_stop = MagicMock()
fifo_is_running = MagicMock(return_value=False)

# Shift Register
sr_stage = MagicMock()
sr_write = MagicMock()

# I2C
i2c_read = MagicMock(return_value=0)
i2c_write = MagicMock()

# Polarity
pol_init = MagicMock()
pol_get = MagicMock(return_value=False)
pol_set = MagicMock()


def _reset():
    """Reset all mocks — called between tests."""
    for obj in list(globals().values()):
        if isinstance(obj, MagicMock):
            obj.reset_mock()
    # Restore defaults
    pwm_is_running.return_value = False
    fifo_is_running.return_value = False
    i2c_read.return_value = 0
    pol_get.return_value = False
    pwm_is_sweep_done.return_value = True
    pwm_is_sample_done.return_value = True
```

**Step 3: Create `test/mocks/sam.py`**

```python
"""Mock sam C module."""
from unittest.mock import MagicMock

say = MagicMock()
render = MagicMock(return_value=bytearray(100))


def _reset():
    for obj in list(globals().values()):
        if isinstance(obj, MagicMock):
            obj.reset_mock()
    render.return_value = bytearray(100)
```

**Step 4: Create `test/mocks/board_utils.py`**

```python
"""Mock board_utils C module."""
from unittest.mock import MagicMock

enter_bootloader = MagicMock()


def _reset():
    enter_bootloader.reset_mock()
```

**Step 5: Create `test/mocks/micropython_builtins.py`**

This installs fake MicroPython-specific modules into `sys.modules` so application code can `import machine`, `import esp32`, etc.

```python
"""Install MicroPython builtin stubs into sys.modules."""
import sys
import time
import types
from unittest.mock import MagicMock

# Patch time.sleep_ms and time.ticks_ms (MicroPython-specific)
if not hasattr(time, "sleep_ms"):
    time.sleep_ms = lambda ms: time.sleep(ms / 1000)
if not hasattr(time, "ticks_ms"):
    time.ticks_ms = lambda: int(time.time() * 1000)
if not hasattr(time, "ticks_diff"):
    time.ticks_diff = lambda a, b: a - b


def _make_module(name, attrs=None):
    """Create a fake module with MagicMock attributes."""
    mod = types.ModuleType(name)
    if attrs:
        for k, v in attrs.items():
            setattr(mod, k, v)
    return mod


# machine module
_machine = _make_module("machine", {
    "Pin": MagicMock(),
    "I2C": MagicMock(),
    "SPI": MagicMock(),
    "Timer": MagicMock(),
    "reset": MagicMock(),
    "freq": MagicMock(return_value=240000000),
})
sys.modules["machine"] = _machine

# esp32 module
_nvs_mock = MagicMock()
_nvs_mock.return_value = _nvs_mock  # NVS("namespace") returns itself
_partition_mock = MagicMock()
_partition_mock.RUNNING = 0
_partition_mock.return_value = _partition_mock
_partition_mock.get_next_update.return_value = _partition_mock
_partition_mock.info.return_value = (0, 0, 0, 1572864, "ota_1", False)

_esp32 = _make_module("esp32", {
    "NVS": _nvs_mock,
    "Partition": _partition_mock,
})
sys.modules["esp32"] = _esp32

# network module
_wlan_mock = MagicMock()
_wlan_mock.return_value = _wlan_mock
_wlan_mock.isconnected.return_value = True
_wlan_mock.ifconfig.return_value = ("192.168.1.100", "255.255.255.0", "192.168.1.1", "8.8.8.8")

_network = _make_module("network", {
    "WLAN": _wlan_mock,
    "STA_IF": 0,
    "AP_IF": 1,
    "hostname": MagicMock(),
})
sys.modules["network"] = _network

# ntptime module
_ntptime = _make_module("ntptime", {
    "settime": MagicMock(),
})
sys.modules["ntptime"] = _ntptime

# neopixel module
sys.modules["neopixel"] = _make_module("neopixel", {
    "NeoPixel": MagicMock(),
})

# _thread module (MicroPython's threading)
if "_thread" not in sys.modules:
    sys.modules["_thread"] = _make_module("_thread", {
        "start_new_thread": MagicMock(),
    })

# boot_cfg module (frozen)
sys.modules["boot_cfg"] = _make_module("boot_cfg", {
    "FIRMWARE_VERSION": "0.1.0",
    "_LOG": "/boot.log",
    "_LOG_PREV": "/boot.log.prev",
    "_log": MagicMock(),
})

# webrepl module (optional, not always present)
sys.modules["webrepl"] = _make_module("webrepl", {
    "start": MagicMock(),
})
sys.modules["webrepl_cfg"] = _make_module("webrepl_cfg", {
    "PASS": "",
})


def _reset_all():
    """Reset all MicroPython builtin mocks."""
    _nvs_mock.reset_mock()
    _partition_mock.reset_mock()
    _partition_mock.RUNNING = 0
    _partition_mock.return_value = _partition_mock
    _partition_mock.get_next_update.return_value = _partition_mock
    _partition_mock.info.return_value = (0, 0, 0, 1572864, "ota_1", False)
    _wlan_mock.reset_mock()
    _wlan_mock.return_value = _wlan_mock
    _wlan_mock.isconnected.return_value = True
    _wlan_mock.ifconfig.return_value = ("192.168.1.100", "255.255.255.0", "192.168.1.1", "8.8.8.8")
```

**Step 6: Verify mocks import cleanly**

Run: `cd /home/pi/projects/optacon-firmware && python -c "import test.mocks.pz_drive; import test.mocks.sam; import test.mocks.board_utils; import test.mocks.micropython_builtins; print('OK')"`
Expected: `OK`

---

### Task 2: Test Configuration and Fixtures

**Files:**
- Create: `test/__init__.py`
- Create: `test/integration/__init__.py`
- Create: `test/e2e/__init__.py`
- Create: `test/conftest.py`
- Create: `test/requirements.txt`
- Create: `test/e2e/fixtures/` (directory)

**Step 1: Create package init files**

```python
# test/__init__.py, test/integration/__init__.py, test/e2e/__init__.py
# (all empty)
```

**Step 2: Create `test/conftest.py`**

This is the critical file — it sets up `sys.path` and `sys.modules` so all application code imports resolve correctly.

```python
"""Root conftest — path setup and mock injection for all tests."""
import os
import sys

# Insert mock modules FIRST on sys.path so `import pz_drive` resolves to mock
_test_dir = os.path.dirname(__file__)
_mocks_dir = os.path.join(_test_dir, "mocks")
_repo_root = os.path.dirname(_test_dir)

# Mocks must be on path before any application code is imported
sys.path.insert(0, _mocks_dir)

# Install MicroPython builtins into sys.modules BEFORE any app imports
import micropython_builtins  # noqa: E402, F401

# Now add application code paths
sys.path.insert(1, os.path.join(_repo_root, "python", "frozen"))
sys.path.insert(2, os.path.join(_repo_root, "python"))

import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _reset_mocks():
    """Reset all mocks before each test."""
    import pz_drive
    import sam
    import board_utils
    import micropython_builtins

    pz_drive._reset()
    sam._reset()
    board_utils._reset()
    micropython_builtins._reset_all()
    yield
```

**Step 3: Create `test/requirements.txt`**

```
pytest>=8.0
pytest-asyncio>=0.23
pytest-playwright>=0.5
microdot>=2.0
aiohttp>=3.9
websockets>=12.0
```

**Step 4: Create fixture directories**

Run: `mkdir -p test/e2e/fixtures/files test/unit`

**Step 5: Create test fixture files**

Create `test/e2e/fixtures/test_firmware.bin` — a small dummy binary:

```python
# Generate via: python -c "open('test/e2e/fixtures/test_firmware.bin','wb').write(b'\\xe9' + b'\\x00'*4095)"
```

Create `test/e2e/fixtures/versions.json`:

```json
{
  "latest_firmware": "0.2.0",
  "latest_files": "0.2.0",
  "firmware": {
    "0.2.0": {
      "url": "firmware/0.2.0.bin",
      "size": 4096,
      "sha256": ""
    }
  },
  "files": {
    "0.2.0": {
      "changes": [
        {
          "path": "web_server.py",
          "url": "files/0.2.0/web_server.py",
          "sha256": ""
        }
      ]
    }
  }
}
```

The SHA-256 values will be computed and filled in by the OTA mock server at startup.

**Step 6: Install dependencies and verify pytest discovers tests**

Run: `cd /home/pi/projects/optacon-firmware && pip install -r test/requirements.txt`
Run: `pytest test/ --collect-only`
Expected: no errors, no tests collected yet (that's fine)

**Step 7: Commit foundation**

```bash
git add test/
git commit -m "test: add test infrastructure with mock C modules and fixtures"
```

---

### Task 3: Hardware Info Module

**Files:**
- Create: `python/frozen/hw_info.py`
- Test: `test/integration/test_hw_info.py`

**Step 1: Write test**

```python
"""Tests for hw_info frozen module."""
import pytest


def test_get_returns_default_when_key_missing():
    import hw_info
    assert hw_info.get("nonexistent") is None
    assert hw_info.get("nonexistent", "fallback") == "fallback"


def test_get_returns_value_from_nvs(monkeypatch):
    import esp32
    import hw_info

    # Simulate NVS returning a blob
    def fake_get_blob(key, buf):
        data = b"OPT-2026-0042"
        buf[:len(data)] = data

    esp32.NVS.return_value.get_blob = fake_get_blob
    # Force re-init of hw_info's _nvs handle
    hw_info._nvs = esp32.NVS("hw_info")

    assert hw_info.get("serial_number") == "OPT-2026-0042"


def test_get_all_returns_dict(monkeypatch):
    import esp32
    import hw_info

    values = {"serial_number": b"SN001", "hw_revision": b"1.2"}

    def fake_get_blob(key, buf):
        if key in values:
            data = values[key]
            buf[:len(data)] = data
        else:
            raise OSError("key not found")

    esp32.NVS.return_value.get_blob = fake_get_blob
    hw_info._nvs = esp32.NVS("hw_info")

    result = hw_info.get_all()
    assert result == {"serial_number": "SN001", "hw_revision": "1.2"}


def test_get_headers():
    import esp32
    import hw_info

    values = {"serial_number": b"SN001", "hw_revision": b"1.2"}

    def fake_get_blob(key, buf):
        if key in values:
            data = values[key]
            buf[:len(data)] = data
        else:
            raise OSError("key not found")

    esp32.NVS.return_value.get_blob = fake_get_blob
    hw_info._nvs = esp32.NVS("hw_info")

    headers = hw_info.get_headers()
    assert headers["X-Device-Serial-Number"] == "SN001"
    assert headers["X-Device-Hw-Revision"] == "1.2"
    assert "X-Device-Firmware-Version" in headers
```

**Step 2: Run test to verify it fails**

Run: `pytest test/integration/test_hw_info.py -v`
Expected: FAIL — `hw_info` module doesn't exist yet

**Step 3: Create `python/frozen/hw_info.py`**

```python
"""Read-only device identity from hw_info NVS partition.

Written once per board via scripts/provision.py. Never modified by firmware
or OTA updates. All values are sent as X-Device-* HTTP headers during OTA.
"""

try:
    import esp32
    _nvs = esp32.NVS("hw_info")
except Exception:
    _nvs = None

_KEYS = ("serial_number", "hw_revision")


def get(key, default=None):
    """Read a string value from the hw_info partition."""
    if _nvs is None:
        return default
    try:
        buf = bytearray(64)
        _nvs.get_blob(key, buf)
        return buf.rstrip(b"\x00").decode()
    except OSError:
        return default


def get_all():
    """Return dict of all known device identity keys."""
    return {k: get(k) for k in _KEYS if get(k) is not None}


def get_headers():
    """Return dict of X-Device-* HTTP headers for OTA requests."""
    import boot_cfg

    headers = {}
    for k, v in get_all().items():
        # serial_number -> X-Device-Serial-Number
        header_name = "X-Device-" + k.replace("_", "-").title()
        headers[header_name] = v
    headers["X-Device-Firmware-Version"] = boot_cfg.FIRMWARE_VERSION
    return headers
```

**Step 4: Run test to verify it passes**

Run: `pytest test/integration/test_hw_info.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add python/frozen/hw_info.py test/integration/test_hw_info.py
git commit -m "feat: add hw_info module for device identity from NVS partition"
```

---

### Task 4: Provisioning Script

**Files:**
- Create: `scripts/provision.py`
- Create: `config/partitions-4MiB-ota.csv`

**Step 1: Locate current partition table**

The partition CSV is referenced in `config/sdkconfig.ota` as `partitions-4MiB-ota.csv`. Check if it exists in the MicroPython build tree or needs creating. The file must be found or created at the path ESP-IDF expects (typically relative to the project root or the sdkconfig location).

Run: `find ~/micropython -name "partitions-4MiB*" 2>/dev/null; find /home/pi/projects/optacon-firmware -name "partitions*" 2>/dev/null`

If not found, create `config/partitions-4MiB-ota.csv` based on ESP32-S3 4MB OTA layout:

```csv
# ESP-IDF Partition Table
# Name,    Type, SubType, Offset,   Size,   Flags
nvs,       data, nvs,     0x9000,   0x6000,
phy_init,  data, phy,     0xf000,   0x1000,
otadata,   data, ota,     0x10000,  0x2000,
hw_info,   data, nvs,     0x12000,  0x1000,
ota_0,     app,  ota_0,   0x20000,  0x180000,
ota_1,     app,  ota_1,   0x1A0000, 0x180000,
vfs,       data, fat,     0x320000, 0xD8000,
```

Note: The exact offsets depend on the existing layout. Verify against the current partition table. The key addition is the `hw_info` row (4KB NVS partition at 0x12000, between otadata and ota_0). Adjust offsets if needed to avoid overlap.

**Step 2: Create `scripts/provision.py`**

```python
#!/usr/bin/env python3
"""One-time hardware provisioning — write device identity to hw_info NVS partition.

Usage:
    python scripts/provision.py --port /dev/ttyACM0 \\
        --serial-number "OPT-2026-0042" \\
        --hw-revision "1.2"

    # Arbitrary extra keys:
    python scripts/provision.py --port /dev/ttyACM0 \\
        --serial-number "OPT-2026-0042" \\
        --hw-revision "1.2" \\
        --set batch_date=2026-03-06

Requires: esptool (pip install esptool)
"""
import argparse
import csv
import hashlib
import io
import struct
import subprocess
import sys
import tempfile


# NVS partition binary format constants
NVS_PAGE_SIZE = 4096
NVS_ENTRY_SIZE = 32
NVS_ENTRIES_PER_PAGE = 126  # (4096 - 32) / 32
NVS_NAMESPACE_ENTRY = 0x01
NVS_BLOB_DATA_ENTRY = 0x48  # type=blob_data


def _create_nvs_binary(namespace, kv_pairs, partition_size=0x1000):
    """Create a minimal NVS partition binary.

    Uses ESP-IDF's nvs_partition_gen.py if available, otherwise builds
    a CSV and calls the tool.
    """
    # Build CSV for nvs_partition_gen.py
    csv_content = "key,type,encoding,value\n"
    csv_content += f"{namespace},namespace,,\n"
    for key, value in kv_pairs.items():
        csv_content += f"{key},data,string,{value}\n"

    # Try using nvs_partition_gen.py from ESP-IDF
    idf_path = None
    for path in [
        os.path.expanduser("~/esp/v5.5.1/components/nvs_flash/nvs_partition_generator"),
        os.environ.get("IDF_PATH", "") + "/components/nvs_flash/nvs_partition_generator",
    ]:
        if os.path.exists(os.path.join(path, "nvs_partition_gen.py")):
            idf_path = path
            break

    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write(csv_content)
        csv_path = f.name

    out_path = csv_path.replace(".csv", ".bin")

    if idf_path:
        cmd = [
            sys.executable, os.path.join(idf_path, "nvs_partition_gen.py"),
            "generate", csv_path, out_path, hex(partition_size),
        ]
    else:
        # Try via python -m esp_idf_nvs_partition_gen (pip-installed)
        cmd = [
            sys.executable, "-m", "esp_idf_nvs_partition_gen",
            "generate", csv_path, out_path, hex(partition_size),
        ]

    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        with open(out_path, "rb") as f:
            return f.read()
    finally:
        import os
        os.unlink(csv_path)
        try:
            os.unlink(out_path)
        except FileNotFoundError:
            pass


def _find_partition_offset(partitions_csv, name):
    """Find the offset of a named partition in the CSV."""
    with open(partitions_csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if parts[0] == name:
                return int(parts[3], 0)
    return None


def main():
    import os

    parser = argparse.ArgumentParser(description="Provision device hardware identity")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0, COM7)")
    parser.add_argument("--serial-number", required=True, help="Device serial number")
    parser.add_argument("--hw-revision", required=True, help="Hardware revision (e.g. 1.2)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="Additional key=value pairs")
    parser.add_argument("--partitions", default=None,
                        help="Path to partition CSV (auto-detected if not specified)")
    parser.add_argument("--dry-run", action="store_true", help="Generate binary but don't flash")

    args = parser.parse_args()

    # Build key-value pairs
    kv = {
        "serial_number": args.serial_number,
        "hw_revision": args.hw_revision,
    }
    for extra in args.set:
        if "=" not in extra:
            print(f"Error: --set value must be KEY=VALUE, got: {extra}", file=sys.stderr)
            sys.exit(1)
        k, v = extra.split("=", 1)
        kv[k] = v

    # Find partition table
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if args.partitions:
        partitions_csv = args.partitions
    else:
        partitions_csv = os.path.join(repo_root, "config", "partitions-4MiB-ota.csv")

    if not os.path.exists(partitions_csv):
        print(f"Error: partition table not found: {partitions_csv}", file=sys.stderr)
        sys.exit(1)

    offset = _find_partition_offset(partitions_csv, "hw_info")
    if offset is None:
        print("Error: 'hw_info' partition not found in partition table", file=sys.stderr)
        sys.exit(1)

    print(f"Provisioning device:")
    for k, v in kv.items():
        print(f"  {k}: {v}")
    print(f"Partition offset: {hex(offset)}")

    # Generate NVS binary
    nvs_bin = _create_nvs_binary("hw_info", kv)
    print(f"NVS binary size: {len(nvs_bin)} bytes")

    if args.dry_run:
        out_path = os.path.join(repo_root, "hw_info.bin")
        with open(out_path, "wb") as f:
            f.write(nvs_bin)
        print(f"Dry run — binary saved to {out_path}")
        return

    # Flash to device
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(nvs_bin)
        bin_path = f.name

    try:
        cmd = [
            sys.executable, "-m", "esptool",
            "--port", args.port,
            "--baud", "460800",
            "write_flash", hex(offset), bin_path,
        ]
        print(f"Flashing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        print("Provisioning complete!")
    finally:
        os.unlink(bin_path)


if __name__ == "__main__":
    main()
```

**Step 3: Commit**

```bash
git add config/partitions-4MiB-ota.csv scripts/provision.py
git commit -m "feat: add hw_info partition and provisioning script"
```

---

## Phase 2: Source Refactors (sequential — web_server depends on working mocks)

### Task 5: Add X-Device Headers to OTA

**Files:**
- Modify: `python/ota.py` (lines 90-91, 114-117, and function `_http_get`)
- Test: `test/integration/test_ota.py` (header tests only — full OTA tests in Task 12)

**Step 1: Write test for device headers**

```python
"""Tests for OTA device header injection."""
import importlib
import json
import os
import sys

import pytest


@pytest.fixture
def ota_module(tmp_path, monkeypatch):
    """Import ota module with temp filesystem."""
    monkeypatch.chdir(tmp_path)
    # ota.py uses open() for config files — tmp_path makes these safe
    import ota
    importlib.reload(ota)
    return ota


def test_device_headers_included_in_http_request(ota_module, monkeypatch):
    """Verify X-Device-* headers are sent with OTA HTTP requests."""
    import hw_info
    import esp32

    # Mock hw_info to return test values
    monkeypatch.setattr(hw_info, "get_headers", lambda: {
        "X-Device-Serial-Number": "TEST-001",
        "X-Device-Hw-Revision": "1.0",
        "X-Device-Firmware-Version": "0.1.0",
    })

    # Capture headers passed to _http_get
    captured_headers = {}
    original_http_get = ota_module._http_get

    def mock_http_get(url, headers=None):
        captured_headers.update(headers or {})
        raise ConnectionError("mock")  # prevent actual connection

    monkeypatch.setattr(ota_module, "_http_get", mock_http_get)

    # Configure update URL so check_for_updates makes a request
    cfg_path = os.path.join(str(monkeypatch.chdir), "ota_config.json") if hasattr(monkeypatch, 'chdir') else "ota_config.json"
    with open("ota_config.json", "w") as f:
        json.dump({"update_url": "http://example.com/updates", "firmware_version": "0.1.0"}, f)

    ota_module.check_for_updates()

    assert "X-Device-Serial-Number" in captured_headers
    assert captured_headers["X-Device-Serial-Number"] == "TEST-001"
    assert captured_headers["X-Device-Hw-Revision"] == "1.0"
```

**Step 2: Run test, verify it fails**

Run: `pytest test/integration/test_ota.py::test_device_headers_included_in_http_request -v`

**Step 3: Modify `python/ota.py`**

Add device headers to `_http_get` and `_http_request`. Near the top of the file, add a helper:

```python
def _device_headers():
    """Get X-Device-* headers for OTA requests."""
    try:
        import hw_info
        return hw_info.get_headers()
    except Exception:
        return {}
```

Then modify `_http_request` (line ~90) to merge device headers:

```python
def _http_request(method, url, body=None, headers=None):
    """Minimal HTTP request returning (status, headers_dict, socket)."""
    import socket

    try:
        import ssl
    except ImportError:
        ssl = None

    host, port, path, use_ssl = _parse_url(url)

    # Merge device identity headers
    all_headers = _device_headers()
    if headers:
        all_headers.update(headers)

    addr = socket.getaddrinfo(host, port)[0][-1]
    s = socket.socket()
    try:
        s.settimeout(30)
        s.connect(addr)
        if use_ssl and ssl:
            s = ssl.wrap_socket(s, server_hostname=host)

        req = f"{method} {path} HTTP/1.0\r\nHost: {host}\r\n"
        for k, v in all_headers.items():
            req += f"{k}: {v}\r\n"
        if body is not None:
            req += f"Content-Length: {len(body)}\r\n"
        req += "\r\n"
        s.write(req.encode())
        if body is not None:
            s.write(body.encode() if isinstance(body, str) else body)

        line = s.readline().decode()
        status = int(line.split()[1])

        resp_headers = {}
        while True:
            line = s.readline().decode().strip()
            if not line:
                break
            if ":" in line:
                k, v = line.split(":", 1)
                resp_headers[k.strip().lower()] = v.strip()

        return status, resp_headers, s
    except Exception:
        s.close()
        raise
```

The key change: replaced the local `if headers:` block with a merge of `_device_headers()` + caller headers.

**Step 4: Run test, verify it passes**

Run: `pytest test/integration/test_ota.py -v`

**Step 5: Commit**

```bash
git add python/ota.py test/integration/test_ota.py
git commit -m "feat: send X-Device-* headers with OTA HTTP requests"
```

---

### Task 6: Refactor web_server.py for Dependency Injection

**Files:**
- Modify: `python/web_server.py`
- Test: `test/e2e/test_api.py` (basic smoke test — full API tests in Task 14)

**Step 1: Write smoke test**

```python
"""Smoke test — verify web_server.py imports and creates an app under CPython."""
import pytest


def test_create_app_with_mock_deps():
    """web_server.create_app() should return a Microdot app with mock deps."""
    from unittest.mock import MagicMock
    import web_server

    deps = MagicMock()
    deps.pa.get_status.return_value = {"running": False, "mode": None, "frequency": 0,
                                        "gain": 100, "fullwave": False, "waveform": "sine",
                                        "polarity": False, "pins": [0]*20}
    deps.wifi.get_status.return_value = {"mode": "ap", "ssid": "test", "ip": "192.168.4.1",
                                          "hostname": "test.local"}

    app = web_server.create_app(deps)
    assert app is not None
```

**Step 2: Refactor `python/web_server.py`**

Transform from module-level globals to dependency-injected `create_app()`:

```python
import asyncio
import json

from microdot import Microdot, send_file
from microdot.websocket import with_websocket


def _schedule_reset():
    """Schedule reboot via timer so HTTP response can flush first."""
    from machine import Timer, reset
    Timer(0).init(period=1000, mode=Timer.ONE_SHOT, callback=lambda t: reset())


class _Deps:
    """Container for injectable dependencies."""
    def __init__(self):
        from pz_drive_py import PzActuator
        import ota as _ota
        import wifi as _wifi
        self.pa = PzActuator()
        self.ota = _ota
        self.wifi = _wifi


def create_app(deps=None):
    """Create and configure the Microdot app.

    Args:
        deps: dependency container with .pa, .ota, .wifi attributes.
              If None, creates real hardware dependencies.
    """
    if deps is None:
        deps = _Deps()

    app = Microdot()

    def _get_status():
        status = deps.pa.get_status()
        status.update(deps.wifi.get_status())
        return status

    def _handle_command(msg):
        data = json.loads(msg)
        cmd = data.get("cmd")
        pa = deps.pa
        was_running = pa.is_running()

        if cmd == "set_frequency_analog":
            hz = data.get("hz", 250)
            amp = data.get("amplitude", 100)
            wf = data.get("waveform", "sine")
            fw = data.get("fullwave", False)
            dt = data.get("dead_time", 0)
            adv = data.get("phase_advance", 0)
            if (was_running and pa._mode == "analog"
                    and fw == pa._fullwave and dt == pa._dead_time
                    and adv == pa._phase_advance):
                pa.set_frequency_live(hz, amplitude=amp, waveform=wf)
            else:
                if was_running:
                    pa.stop()
                pa.set_frequency_analog(
                    hz, amplitude=amp, fullwave=fw,
                    dead_time=dt, phase_advance=adv, waveform=wf,
                )
                if was_running:
                    pa.start(gain=pa._gain)
        elif cmd == "set_frequency_digital":
            if was_running:
                pa.stop()
            pa.set_frequency_digital(
                data.get("hz", 100),
                fullwave=data.get("fullwave", False),
                waveform=data.get("waveform", "sine"),
            )
            if was_running:
                pa.start(gain=pa._gain)
        elif cmd == "start":
            if was_running:
                pa.stop()
            pa.start(gain=data.get("gain", 100))
        elif cmd == "stop":
            pa.stop()
        elif cmd == "set_pin":
            pa.shift_register.set_pin(data["pin"], data["value"])
        elif cmd == "set_all":
            pa.shift_register.set_all(data["value"])
        elif cmd == "set_polarity":
            import pz_drive
            pz_drive.pol_set(bool(data.get("value", False)))
        elif cmd == "get_status":
            pass
        elif cmd == "wifi_config":
            deps.wifi.save_config(data["ssid"], data.get("password", ""))
            deps.wifi.reconnect()
            return {"msg": "WiFi reconnected", "ip": deps.wifi.ip}
        elif cmd == "say":
            import sam
            text = data.get("text", "")
            if not text:
                return {"error": "no text provided"}
            sam.say(
                text,
                speed=data.get("speed", 72),
                pitch=data.get("pitch", 64),
                mouth=data.get("mouth", 128),
                throat=data.get("throat", 128),
            )
            return {"msg": "speech complete"}
        elif cmd == "play_music":
            import music
            notes_str = data.get("notes", "")
            if not notes_str:
                return {"error": "no notes provided"}
            song = []
            for token in notes_str.split():
                parts = token.split(":")
                note = parts[0]
                try:
                    beats = float(parts[1]) if len(parts) > 1 else 1
                except ValueError:
                    return {"error": "invalid beat value: " + parts[1]}
                song.append((note if note != "R" else "R", beats))
            music.play(
                song,
                bpm=data.get("bpm", 120),
                waveform=data.get("waveform", "sine"),
                gain=data.get("gain", 75),
            )
            return {"msg": "music complete"}
        elif cmd == "play_song":
            import music
            name = data.get("name", "")
            if name not in music.SONGS:
                return {"error": "unknown song: " + name}
            music.play_song(
                name,
                waveform=data.get("waveform", "sine"),
            )
            return {"msg": "music complete"}
        elif cmd == "exec":
            code = data.get("code", "")
            output = []

            def _print(*args, **kw):
                sep = kw.get("sep", " ")
                end = kw.get("end", "\n")
                output.append(sep.join(str(a) for a in args) + end)

            g = {"print": _print}
            try:
                try:
                    result = eval(code, g)
                    if result is not None:
                        output.append(repr(result) + "\n")
                except SyntaxError:
                    exec(code, g)
            except Exception as e:
                import io
                import sys
                buf = io.StringIO()
                # CPython uses traceback; MicroPython uses sys.print_exception
                try:
                    sys.print_exception(e, buf)
                except AttributeError:
                    import traceback
                    traceback.print_exception(type(e), e, e.__traceback__, file=buf)
                output.append(buf.getvalue())
            return {"output": "".join(output)}
        else:
            return {"error": f"unknown command: {cmd}"}

        return None

    @app.route("/")
    async def index(request):
        return send_file("/web/index.html")

    @app.route("/web/<path:path>")
    async def static(request, path):
        if ".." in path:
            return "Not found", 404
        return send_file("/web/" + path)

    @app.route("/wifi")
    async def wifi_page(request):
        return send_file("/web/wifi.html")

    @app.route("/wifi/status")
    async def wifi_status(request):
        return json.dumps(deps.wifi.get_status()), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/status")
    async def ota_status(request):
        return json.dumps(deps.ota.get_status()), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/config", methods=["GET", "PUT"])
    async def ota_config(request):
        if request.method == "GET":
            return json.dumps(deps.ota.load_config()), 200, {"Content-Type": "application/json"}
        data = json.loads(request.body.decode())
        cfg = deps.ota.load_config()
        for key in ("update_url", "diagnostics_url", "auto_check"):
            if key in data:
                cfg[key] = data[key]
        deps.ota.save_config(cfg)
        return json.dumps(cfg), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/check", methods=["POST"])
    async def ota_check(request):
        result = deps.ota.check_for_updates()
        if result is None:
            return json.dumps({"error": "Check failed"}), 500, {"Content-Type": "application/json"}
        return json.dumps(result), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/update/firmware", methods=["POST"])
    async def ota_update_firmware(request):
        data = json.loads(request.body.decode())
        version = data.get("version")
        manifest = data.get("manifest")
        if not version or not manifest:
            return json.dumps({"error": "version and manifest required"}), 400, {
                "Content-Type": "application/json"}
        ok = deps.ota.update_firmware(manifest, version)
        if ok:
            _schedule_reset()
            return (json.dumps({"status": "ok", "message": "Firmware updated. Rebooting..."}),
                    200, {"Content-Type": "application/json"})
        return json.dumps({"error": "Firmware update failed"}), 500, {"Content-Type": "application/json"}

    @app.route("/api/ota/update/files", methods=["POST"])
    async def ota_update_files(request):
        data = json.loads(request.body.decode())
        version = data.get("version")
        manifest = data.get("manifest")
        if not version or not manifest:
            return json.dumps({"error": "version and manifest required"}), 400, {
                "Content-Type": "application/json"}
        ok = deps.ota.update_files(manifest, version)
        if ok:
            return (json.dumps({"status": "ok", "message": "Files updated. Rebooting..."}),
                    200, {"Content-Type": "application/json"})
        return json.dumps({"error": "File update failed"}), 500, {"Content-Type": "application/json"}

    @app.route("/api/ota/upload", methods=["POST"])
    async def ota_upload(request):
        filename = request.args.get("filename", "")
        if filename:
            if ".." in filename:
                return json.dumps({"error": "Invalid filename"}), 400, {"Content-Type": "application/json"}
            path = "/" + filename.lstrip("/")
            deps.ota.upload_file(path, request.body)
            return json.dumps({"status": "ok", "path": path}), 200, {"Content-Type": "application/json"}
        else:
            ok = deps.ota.upload_firmware(request.body, len(request.body))
            if ok:
                _schedule_reset()
                return (json.dumps({"status": "ok", "message": "Firmware uploaded. Rebooting..."}),
                        200, {"Content-Type": "application/json"})
            return json.dumps({"error": "Upload failed"}), 500, {"Content-Type": "application/json"}

    @app.route("/api/ota/log")
    async def ota_log(request):
        name = request.args.get("file", "boot")
        content = deps.ota.get_log(name)
        return json.dumps({"log": content}), 200, {"Content-Type": "application/json"}

    @app.route("/api/ota/diagnostics", methods=["POST"])
    async def ota_diagnostics(request):
        ok = deps.ota.send_diagnostics()
        if ok:
            return json.dumps({"status": "ok"}), 200, {"Content-Type": "application/json"}
        return json.dumps({"error": "Failed to send diagnostics"}), 500, {
            "Content-Type": "application/json"}

    @app.route("/api/music/songs")
    async def music_songs(request):
        import music
        songs = []
        for name, (data, bpm, gain) in music.SONGS.items():
            notes = " ".join(f"{t[0]}:{t[1]}" for t in data)
            songs.append({"name": name, "notes": notes, "bpm": bpm, "gain": gain})
        return json.dumps({"songs": songs}), 200, {"Content-Type": "application/json"}

    @app.route("/update")
    async def update_page(request):
        return send_file("/web/update.html")

    @app.route("/ws")
    @with_websocket
    async def websocket(request, ws):
        try:
            await ws.send(json.dumps(_get_status()))
            while True:
                msg = await ws.receive()
                try:
                    err = _handle_command(msg)
                    if err:
                        await ws.send(json.dumps(err))
                    else:
                        await ws.send(json.dumps(_get_status()))
                except Exception as e:
                    await ws.send(json.dumps({"error": str(e)}))
        except Exception as e:
            print("WebSocket error:", type(e).__name__, e)

    return app


def start():
    """Connect WiFi and start the web server (runs forever)."""
    import ota
    import wifi

    wifi.connect()
    print("Starting web server on http://" + wifi.ip + ":80")

    if wifi.mode == "sta":
        try:
            cfg = ota.load_config()
            if cfg.get("auto_check", True):
                result = ota.check_for_updates()
                if result and (result["firmware_available"] or result["files_available"]):
                    parts = []
                    if result["firmware_available"]:
                        parts.append("firmware " + result["latest_firmware"])
                    if result["files_available"]:
                        parts.append("files " + result["latest_files"])
                    msg = "OTA auto-check: update available (" + ", ".join(parts) + ")"
                    print("[BOOT]", msg)
                    try:
                        with open("/boot.log", "a") as f:
                            f.write(msg + "\n")
                    except Exception:
                        pass
        except Exception as e:
            print("[BOOT] OTA auto-check failed:", e)

    app = create_app()
    asyncio.run(app.start_server(host="0.0.0.0", port=80))
```

Key changes:
- All routes moved inside `create_app()` as closures over `deps`
- `_handle_command` and `_get_status` are now inner functions using `deps`
- `start()` now calls `create_app()` with default deps
- `exec` command handles both CPython and MicroPython exception formatting
- Module-level `app` and `pa` globals removed

**Step 3: Run smoke test**

Run: `pytest test/e2e/test_api.py::test_create_app_with_mock_deps -v`
Expected: PASS

**Step 4: Commit**

```bash
git add python/web_server.py test/e2e/test_api.py
git commit -m "refactor: dependency injection in web_server.py for testability"
```

---

## Phase 3: Integration Tests (all tasks independent — run in parallel)

### Task 7: Integration Tests — ShiftRegister

**Files:**
- Create: `test/integration/test_shift_register.py`

Write tests covering:
- `_pin_bit()`: pin 0 → bit 25, pin 19 → bit 6, out-of-range raises ValueError
- `set_pin()`: sets correct bit, calls `pz_drive.sr_stage`
- `get_pin()`: reads correct bit state
- `set_all(True)`: sets mask `0x03FFFFC0`, calls `sr_stage`
- `set_all(False)`: clears to 0
- `get_all()`: returns 20-element tuple
- `set_pins()`: from list, correct word construction
- `latch=False`: does NOT call `sr_stage`
- `latch()`: calls `pz_drive.sr_stage` with current state

Run: `pytest test/integration/test_shift_register.py -v`

**Commit:**
```bash
git add test/integration/test_shift_register.py
git commit -m "test: add shift register integration tests"
```

---

### Task 8: Integration Tests — DRV2665

**Files:**
- Create: `test/integration/test_drv2665.py`

Write tests covering:
- `__init__()`: reads STATUS register, raises RuntimeError on -1
- `init_analog(gain)`: writes correct register sequence (CTRL2, CTRL1, CTRL2)
- `init_digital(gain)`: writes correct register sequence
- `standby()`: writes STANDBY bit to CTRL2
- `status()`: returns i2c_read result
- Gain constants: GAIN_25=0x00, GAIN_50=0x01, GAIN_75=0x02, GAIN_100=0x03
- I2C error: `i2c_read.side_effect = OSError("NACK")` → RuntimeError on init

Run: `pytest test/integration/test_drv2665.py -v`

**Commit:**
```bash
git add test/integration/test_drv2665.py
git commit -m "test: add DRV2665 register driver integration tests"
```

---

### Task 9: Integration Tests — PzActuator

**Files:**
- Create: `test/integration/test_pz_drive_py.py`

Write tests covering:
- `set_frequency_analog()`: calls `pz_drive.pwm_set_frequency` with correct args, sets mode/state
- `set_frequency_analog()` validation: hz<0 or >1000 → ValueError, bad waveform → ValueError
- `set_frequency_digital()`: generates waveform buffer, sets mode to digital
- `set_frequency_live()`: calls `pwm_set_frequency_live`, updates internal state
- `start()` without set_frequency → RuntimeError
- `start()` analog: calls `drv.init_analog` + `pwm_start`
- `start()` digital: calls `fifo_start` with waveform buffer
- `start()` bad gain → ValueError
- `stop()`: calls fifo_stop/pwm_stop as needed, then standby
- `is_running()`: checks both fifo and pwm
- `sweep_analog()`: validation, calls pwm_set_sweep + start
- `get_status()`: returns correct dict with all fields
- Amplitude mapping: 100% → 128 internal, 50% → 64, 0% → 0
- Waveform generation: `_gen_sine`, `_gen_triangle`, `_gen_square` return correct-length bytearrays

Run: `pytest test/integration/test_pz_drive_py.py -v`

**Commit:**
```bash
git add test/integration/test_pz_drive_py.py
git commit -m "test: add PzActuator integration tests"
```

---

### Task 10: Integration Tests — Music

**Files:**
- Create: `test/integration/test_music.py`

Write tests covering:
- `note_freq()`: known notes (A4=440, C4≈261.63, etc.), unknown note → ValueError
- Enharmonic aliases: C#4 == Db4, F#4 == Gb4
- `NOTES` dict: all octaves 2-6 present, all 12 note names
- `SONGS` dict: all 7 songs exist, each is (list, int, int)
- `play()`: calls `pa.set_frequency_analog`, `pa.start`, `pa.set_frequency_live` for notes, `pa.stop` at end
- `play()` with rest: sets amplitude=0 for "R" notes
- `play_song()`: valid name works, unknown name → ValueError
- `play_song()` override kwargs: bpm and gain can be overridden
- Note tuple formats: (note, beats), (note, beats, amp), (note, beats, amp, sweep)

Note: `music.play()` calls `time.sleep_ms()` — the mock in `micropython_builtins.py` patches this. For speed, monkeypatch `time.sleep_ms` to a no-op in these tests.

Run: `pytest test/integration/test_music.py -v`

**Commit:**
```bash
git add test/integration/test_music.py
git commit -m "test: add music player integration tests"
```

---

### Task 11: Integration Tests — WiFi

**Files:**
- Create: `test/integration/test_wifi.py`

Write tests covering:
- `connect()` STA success: reads wifi_config.json, returns IP, sets mode="sta"
- `connect()` STA failure: WLAN.isconnected returns False → falls back to AP
- `connect()` no config: OSError on open → falls back to AP
- `get_status()`: returns correct dict
- `save_config()`: writes JSON file with ssid+password
- `reconnect()`: disconnects, reconnects
- Hostname: set to "esp-optacon"

Use `tmp_path` fixture and `monkeypatch.chdir(tmp_path)` for filesystem isolation. Mock `network.WLAN` behavior via the builtins mock.

Run: `pytest test/integration/test_wifi.py -v`

**Commit:**
```bash
git add test/integration/test_wifi.py
git commit -m "test: add WiFi integration tests"
```

---

### Task 12: Integration Tests — OTA (full)

**Files:**
- Create or extend: `test/integration/test_ota.py`

Write tests covering:
- `_ver_gt()`: "0.2.0" > "0.1.0", "0.1.0" not > "0.1.0", "1.0.0" > "0.9.9"
- `load_config()`: returns defaults when no file, reads file when present
- `save_config()`: writes JSON
- `check_for_updates()`: no URL → None, HTTP error → None, newer version available → True
- `update_files()`: downloads, verifies SHA, atomic rename, NVS watchdog
- `clear_update_flag()`: clears NVS, removes .bak files
- `get_status()`: returns firmware_version, files_version, etc.
- `get_log()`: reads log files, returns "" on missing
- Error cases: corrupt SHA → False, HTTP 500 → False, cleanup of .new files on failure

Use `tmp_path` for filesystem isolation. Mock `socket` for HTTP tests (or test the higher-level functions that use `_http_get`).

Run: `pytest test/integration/test_ota.py -v`

**Commit:**
```bash
git add test/integration/test_ota.py
git commit -m "test: add OTA update integration tests"
```

---

## Phase 4: End-to-End Tests

### Task 13: OTA Mock Server

**Files:**
- Create: `test/e2e/ota_server.py`

Build a standalone aiohttp server that serves OTA update artifacts:

```python
"""Mock OTA update server for testing.

Usable as:
- pytest fixture: `ota_server` fixture in conftest.py
- standalone: python -m test.e2e.ota_server --port 8080
"""
import argparse
import hashlib
import json
import os

from aiohttp import web


class OTAMockServer:
    """Configurable mock OTA server."""

    def __init__(self, fixtures_dir=None):
        self.fixtures_dir = fixtures_dir or os.path.join(os.path.dirname(__file__), "fixtures")
        self.app = web.Application()
        self.diagnostics_log = []
        self.request_log = []
        # Configurable responses
        self.versions_override = None
        self.error_mode = None  # None, "500", "corrupt", "timeout"

        self._setup_routes()

    def _setup_routes(self):
        self.app.router.add_get("/versions.json", self._handle_versions)
        self.app.router.add_get("/firmware/{version}.bin", self._handle_firmware)
        self.app.router.add_get("/files/{version}/{filename}", self._handle_file)
        self.app.router.add_post("/diagnostics", self._handle_diagnostics)

    async def _handle_versions(self, request):
        self.request_log.append(("GET", "/versions.json", dict(request.headers)))
        if self.error_mode == "500":
            return web.Response(status=500)
        if self.versions_override:
            return web.json_response(self.versions_override)
        # Load and compute SHA-256 for test firmware
        manifest_path = os.path.join(self.fixtures_dir, "versions.json")
        with open(manifest_path) as f:
            manifest = json.load(f)
        # Fill in SHA-256 for firmware
        for ver, info in manifest.get("firmware", {}).items():
            bin_path = os.path.join(self.fixtures_dir, "test_firmware.bin")
            if os.path.exists(bin_path):
                with open(bin_path, "rb") as bf:
                    info["sha256"] = hashlib.sha256(bf.read()).hexdigest()
                    bf.seek(0)
                    info["size"] = len(bf.read())
        return web.json_response(manifest)

    async def _handle_firmware(self, request):
        version = request.match_info["version"]
        self.request_log.append(("GET", f"/firmware/{version}.bin", dict(request.headers)))
        if self.error_mode == "500":
            return web.Response(status=500)
        bin_path = os.path.join(self.fixtures_dir, "test_firmware.bin")
        if not os.path.exists(bin_path):
            return web.Response(status=404)
        with open(bin_path, "rb") as f:
            data = f.read()
        if self.error_mode == "corrupt":
            data = data[:len(data)//2]  # truncate
        return web.Response(body=data, content_type="application/octet-stream")

    async def _handle_file(self, request):
        version = request.match_info["version"]
        filename = request.match_info["filename"]
        self.request_log.append(("GET", f"/files/{version}/{filename}", dict(request.headers)))
        if self.error_mode == "500":
            return web.Response(status=500)
        file_path = os.path.join(self.fixtures_dir, "files", filename)
        if not os.path.exists(file_path):
            return web.Response(status=404, text="File not found")
        with open(file_path, "rb") as f:
            data = f.read()
        return web.Response(body=data, content_type="application/octet-stream")

    async def _handle_diagnostics(self, request):
        self.request_log.append(("POST", "/diagnostics", dict(request.headers)))
        if self.error_mode == "500":
            return web.Response(status=500)
        body = await request.json()
        self.diagnostics_log.append(body)
        return web.json_response({"status": "ok"})

    def reset(self):
        """Reset all state."""
        self.diagnostics_log.clear()
        self.request_log.clear()
        self.versions_override = None
        self.error_mode = None


async def start_server(host="127.0.0.1", port=0):
    """Start server, return (server, url)."""
    mock = OTAMockServer()
    runner = web.AppRunner(mock.app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    actual_port = site._server.sockets[0].getsockname()[1]
    return mock, runner, f"http://{host}:{actual_port}"


def main():
    parser = argparse.ArgumentParser(description="Mock OTA update server")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    mock = OTAMockServer()
    print(f"OTA mock server starting on http://{args.host}:{args.port}")
    web.run_app(mock.app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
```

**Test it starts:**

Run: `python -c "import asyncio; from test.e2e.ota_server import start_server; m,r,u = asyncio.run(start_server()); print('OK:', u); asyncio.run(r.cleanup())"`

**Commit:**
```bash
git add test/e2e/ota_server.py
git commit -m "test: add mock OTA update server"
```

---

### Task 14: E2E API and WebSocket Tests

**Files:**
- Create: `test/e2e/conftest.py`
- Extend: `test/e2e/test_api.py`

**Step 1: Create `test/e2e/conftest.py`**

```python
"""E2E test fixtures — real web server with mock deps."""
import asyncio
from unittest.mock import MagicMock

import pytest
import pytest_asyncio

# These are available because test/conftest.py sets up sys.path
import web_server
from test.e2e.ota_server import OTAMockServer, start_server


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
            "running": False, "mode": "analog", "frequency": 250,
            "gain": 100, "fullwave": False, "waveform": "sine",
            "polarity": False, "pins": [0] * 20,
            "amplitude": 100, "dead_time": 0, "phase_advance": 0,
        }
        self.pa.shift_register = MagicMock()

        self.wifi = MagicMock()
        self.wifi.ip = "127.0.0.1"
        self.wifi.mode = "sta"
        self.wifi.get_status.return_value = {
            "mode": "sta", "ssid": "test", "ip": "127.0.0.1",
            "hostname": "test.local",
        }

        self.ota = MagicMock()
        self.ota.get_status.return_value = {
            "firmware_version": "0.1.0", "files_version": "0.1.0",
            "update_url": "", "auto_check": True, "diagnostics_url": "",
        }
        self.ota.load_config.return_value = {
            "update_url": "", "firmware_version": "0.1.0",
            "files_version": "0.1.0", "auto_check": True,
        }
        self.ota.check_for_updates.return_value = {
            "firmware_available": False, "files_available": False,
        }
        self.ota.get_log.return_value = ""
        self.ota.send_diagnostics.return_value = True
        self.ota.update_firmware.return_value = True
        self.ota.update_files.return_value = True

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

    # Find a free port
    import socket
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()

    server_task = asyncio.create_task(
        app.start_server(host="127.0.0.1", port=port, debug=False)
    )
    await asyncio.sleep(0.2)  # let server start

    yield f"http://127.0.0.1:{port}", mock_deps

    app.shutdown()
    server_task.cancel()
    try:
        await server_task
    except asyncio.CancelledError:
        pass


@pytest_asyncio.fixture
async def ota_mock():
    """Start mock OTA server, yield (mock, url)."""
    mock, runner, url = await start_server()
    yield mock, url
    await runner.cleanup()
```

**Step 2: Write full API tests in `test/e2e/test_api.py`**

Add tests for all HTTP routes and WebSocket commands. Key tests:

```python
"""E2E tests for HTTP API and WebSocket commands."""
import json

import aiohttp
import pytest
import pytest_asyncio


# --- HTTP Route Tests ---

@pytest.mark.asyncio
async def test_wifi_status(test_app):
    url, deps = test_app
    async with aiohttp.ClientSession() as s:
        async with s.get(f"{url}/wifi/status") as r:
            assert r.status == 200
            data = await r.json()
            assert data["mode"] == "sta"
            assert data["ip"] == "127.0.0.1"


@pytest.mark.asyncio
async def test_ota_status(test_app):
    url, deps = test_app
    async with aiohttp.ClientSession() as s:
        async with s.get(f"{url}/api/ota/status") as r:
            assert r.status == 200
            data = await r.json()
            assert data["firmware_version"] == "0.1.0"


@pytest.mark.asyncio
async def test_music_songs(test_app):
    url, deps = test_app
    async with aiohttp.ClientSession() as s:
        async with s.get(f"{url}/api/music/songs") as r:
            assert r.status == 200
            data = await r.json()
            assert "songs" in data
            assert len(data["songs"]) > 0
            song = data["songs"][0]
            assert "name" in song
            assert "notes" in song
            assert "bpm" in song
            assert "gain" in song


# --- WebSocket Command Tests ---

@pytest.mark.asyncio
async def test_ws_initial_status(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            msg = await ws.receive_json()
            assert "running" in msg
            assert "pins" in msg
            assert "frequency" in msg


@pytest.mark.asyncio
async def test_ws_start_stop(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()  # initial status
            await ws.send_json({"cmd": "start", "gain": 75})
            msg = await ws.receive_json()
            deps.pa.start.assert_called_with(gain=75)


@pytest.mark.asyncio
async def test_ws_set_frequency_analog(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()
            await ws.send_json({"cmd": "set_frequency_analog", "hz": 300, "waveform": "triangle"})
            await ws.receive_json()
            deps.pa.set_frequency_analog.assert_called()


@pytest.mark.asyncio
async def test_ws_say(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()
            await ws.send_json({"cmd": "say", "text": "hello", "speed": 72, "pitch": 64})
            msg = await ws.receive_json()
            assert msg.get("msg") == "speech complete"


@pytest.mark.asyncio
async def test_ws_play_music(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()
            await ws.send_json({"cmd": "play_music", "notes": "C4:4 E4:4", "bpm": 120})
            msg = await ws.receive_json()
            assert msg.get("msg") == "music complete"


@pytest.mark.asyncio
async def test_ws_set_pin(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()
            await ws.send_json({"cmd": "set_pin", "pin": 5, "value": 1})
            await ws.receive_json()
            deps.pa.shift_register.set_pin.assert_called_with(5, 1)


@pytest.mark.asyncio
async def test_ws_unknown_command(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()
            await ws.send_json({"cmd": "nonexistent"})
            msg = await ws.receive_json()
            assert "error" in msg


@pytest.mark.asyncio
async def test_ws_exec(test_app):
    url, deps = test_app
    ws_url = url.replace("http://", "ws://") + "/ws"
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(ws_url) as ws:
            await ws.receive_json()
            await ws.send_json({"cmd": "exec", "code": "2+2"})
            msg = await ws.receive_json()
            assert "4" in msg.get("output", "")
```

Add tests for all remaining routes (GET/PUT ota/config, POST ota/check, POST ota/update/firmware, POST ota/update/files, POST ota/upload, GET ota/log, POST ota/diagnostics, set_all, set_polarity, play_song, wifi_config, set_frequency_digital, stop, get_status).

Run: `pytest test/e2e/test_api.py -v`

**Commit:**
```bash
git add test/e2e/conftest.py test/e2e/test_api.py
git commit -m "test: add E2E API and WebSocket tests"
```

---

### Task 15: Playwright UI Tests

**Files:**
- Create: `test/e2e/test_ui.py`

**Prerequisites:**
- `playwright install chromium` (one-time)
- Test server must serve static files — may need to mock `send_file` or use a static file directory

**Step 1: Set up Playwright fixture in `test/e2e/conftest.py`**

Add to the existing conftest:

```python
@pytest.fixture
def page_url(test_app):
    """Return base URL for Playwright tests."""
    url, _ = test_app
    return url
```

Note: Playwright needs the server to actually serve `index.html`. Since `send_file("/web/index.html")` won't work on CPython (no `/web/` directory), we need to configure the test server to serve files from the repo's `web/` directory. Add a static file override to `MockDeps` or monkey-patch `send_file`.

**Step 2: Write Playwright tests**

Test all JavaScript functionality:

```python
"""Playwright UI tests — all JavaScript behavior in index.html."""
import json
import re

import pytest
from playwright.sync_api import Page, expect


# --- Tab Switching ---

def test_tabs_exist(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#tab-btn-signal")).to_be_visible()
    expect(page.locator("#tab-btn-music")).to_be_visible()
    expect(page.locator("#tab-btn-speech")).to_be_visible()


def test_signal_tab_active_by_default(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#tab-btn-signal")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-signal")).to_be_visible()
    expect(page.locator("#tab-music")).to_be_hidden()
    expect(page.locator("#tab-speech")).to_be_hidden()


def test_switch_to_music_tab(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-music")
    expect(page.locator("#tab-btn-music")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-music")).to_be_visible()
    expect(page.locator("#tab-signal")).to_be_hidden()


def test_switch_to_speech_tab(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-speech")
    expect(page.locator("#tab-btn-speech")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-speech")).to_be_visible()


# --- Waveform Selector ---

def test_waveform_buttons_exist(page: Page, page_url):
    page.goto(page_url)
    for wf in ["dc", "sine", "triangle", "square"]:
        expect(page.locator(f"#wf-{wf}")).to_be_visible()


def test_sine_selected_by_default(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#wf-sine")).to_have_class(re.compile(r"sel"))


def test_waveform_per_tab_memory(page: Page, page_url):
    page.goto(page_url)
    # Signal: select triangle
    page.click("#wf-triangle")
    expect(page.locator("#wf-triangle")).to_have_class(re.compile(r"sel"))
    # Switch to music, select square
    page.click("#tab-btn-music")
    page.click("#wf-square")
    expect(page.locator("#wf-square")).to_have_class(re.compile(r"sel"))
    # Switch back to signal — should remember triangle
    page.click("#tab-btn-signal")
    expect(page.locator("#wf-triangle")).to_have_class(re.compile(r"sel"))
    # Switch back to music — should remember square
    page.click("#tab-btn-music")
    expect(page.locator("#wf-square")).to_have_class(re.compile(r"sel"))


def test_dc_disabled_on_music_tab(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-music")
    dc_btn = page.locator("#wf-dc")
    expect(dc_btn).to_have_css("pointer-events", "none")


def test_speech_tab_waveform_disabled(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-speech")
    # All waveform buttons should have pointer-events:none
    for wf in ["dc", "sine", "triangle", "square"]:
        expect(page.locator(f"#wf-{wf}")).to_have_css("pointer-events", "none")
    # Square should be selected
    expect(page.locator("#wf-square")).to_have_class(re.compile(r"sel"))


# --- Signal Tab ---

def test_frequency_slider_and_input_sync(page: Page, page_url):
    page.goto(page_url)
    page.fill("#freq", "300")
    page.press("#freq", "Tab")
    expect(page.locator("#freq-slider")).to_have_value("300")


def test_amplitude_slider_updates_label(page: Page, page_url):
    page.goto(page_url)
    page.evaluate("document.getElementById('amplitude').value=75; document.getElementById('amplitude').dispatchEvent(new Event('input'))")
    expect(page.locator("#amp-val")).to_have_text("75%")


# --- DRV2665 Card ---

def test_gain_buttons_exist(page: Page, page_url):
    page.goto(page_url)
    for g in [25, 50, 75, 100]:
        expect(page.locator(f"#gain-{g}")).to_be_visible()


def test_gain_100_selected_by_default(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#gain-100")).to_have_class(re.compile(r"sel"))


def test_gain_button_click_selects(page: Page, page_url):
    page.goto(page_url)
    page.click("#gain-50")
    expect(page.locator("#gain-50")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#gain-100")).not_to_have_class(re.compile(r"sel"))


def test_start_button_label_per_tab(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#start-stop")).to_have_text("START")
    page.click("#tab-btn-speech")
    expect(page.locator("#start-stop")).to_have_text("SPEAK")
    page.click("#tab-btn-music")
    expect(page.locator("#start-stop")).to_have_text("PLAY")


# --- Pin Grid ---

def test_pin_grid_has_20_buttons(page: Page, page_url):
    page.goto(page_url)
    pins = page.locator(".pin-btn")
    expect(pins).to_have_count(20)


def test_set_all_clear_all_buttons(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#set-all")).to_be_visible()
    expect(page.locator("#clear-all")).to_be_visible()


# --- Speech Tab ---

def test_speech_text_input(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-speech")
    page.fill("#tts-text", "hello world")
    expect(page.locator("#tts-text")).to_have_value("hello world")


def test_speech_speed_pitch_sliders(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-speech")
    expect(page.locator("#tts-speed")).to_be_visible()
    expect(page.locator("#ttsPitch")).to_be_visible()


# --- Music Tab ---

def test_music_notes_textarea(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-music")
    page.fill("#music-notes", "C4:4 E4:4 G4:2")
    expect(page.locator("#music-notes")).to_have_value("C4:4 E4:4 G4:2")


def test_music_bpm_input(page: Page, page_url):
    page.goto(page_url)
    page.click("#tab-btn-music")
    page.fill("#music-bpm", "200")
    expect(page.locator("#music-bpm")).to_have_value("200")


# --- Terminal ---

def test_terminal_toggle(page: Page, page_url):
    page.goto(page_url)
    term = page.locator("#term")
    expect(term).not_to_have_class(re.compile(r"open"))
    page.click("#term-toggle")
    expect(term).to_have_class(re.compile(r"open"))


# --- Header ---

def test_header_elements(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator(".brand")).to_have_text("OPTACON")
    expect(page.locator("#badge")).to_be_visible()
    expect(page.locator("#wifi-link")).to_be_visible()


# --- Footer ---

def test_footer_links(page: Page, page_url):
    page.goto(page_url)
    expect(page.locator("#footer")).to_be_visible()
```

Note: Some tests that depend on WebSocket messages (song loading, pin state updates, start/stop button state changes when running) require the WebSocket to be connected. The test server fixture handles this automatically. Tests that check WebSocket-driven UI updates should wait for the initial status message.

The Playwright test fixture needs to handle `send_file()` for serving static files. The e2e conftest.py should configure the test server to serve files from the repo's `web/` directory.

Run: `pytest test/e2e/test_ui.py -v --headed` (first time, to see the browser)
Run: `pytest test/e2e/test_ui.py -v` (headless for CI)

**Commit:**
```bash
git add test/e2e/test_ui.py
git commit -m "test: add Playwright UI tests for all JS functionality"
```

---

## Phase 5: Documentation and Final Verification

### Task 16: Update Documentation

**Files:**
- Modify: `CLAUDE.md` — add test commands to Development Commands section
- Modify: `README.md` — add Testing section

Add to CLAUDE.md Development Commands:

```markdown
### Run tests:
```bash
# All tests
pytest test/

# Integration only
pytest test/integration/

# E2E API tests
pytest test/e2e/test_api.py

# Playwright UI tests
pytest test/e2e/test_ui.py

# OTA mock server (standalone)
python -m test.e2e.ota_server --port 8080
```

### Provision hardware identity:
```bash
python scripts/provision.py --port /dev/ttyACM0 \
    --serial-number "OPT-2026-0042" \
    --hw-revision "1.2"
```
```

Add `hw_info.py` to project structure, mention test/ directory.

**Commit:**
```bash
git add CLAUDE.md README.md
git commit -m "docs: add testing and provisioning documentation"
```

---

### Task 17: Run All Tests and Verify

**Step 1:** Run full test suite:

```bash
pytest test/ -v --tb=short
```

**Step 2:** Fix any failures.

**Step 3:** Run Playwright tests:

```bash
pytest test/e2e/test_ui.py -v
```

**Step 4:** Verify OTA mock server starts standalone:

```bash
timeout 3 python -m test.e2e.ota_server --port 8080 || true
```

**Step 5:** Final commit if any fixes were needed.

---

## Parallelization Guide

Tasks that can run in parallel (same phase, no shared files):

| Phase | Parallel Tasks |
|-------|---------------|
| 1 | Tasks 1, 2, 3, 4 |
| 2 | Tasks 5, 6 (independent files) |
| 3 | Tasks 7, 8, 9, 10, 11, 12 |
| 4 | Tasks 13, 14, 15 (13 first, then 14+15 parallel) |
| 5 | Tasks 16, 17 (sequential) |
