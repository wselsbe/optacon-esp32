# Testing Setup Design

## Overview

Create a comprehensive testing setup for the optacon-firmware project with integration tests (CPython + mocked C modules), end-to-end tests (API/WebSocket + Playwright browser tests), OTA update testing with a local mock server, and a hardware identity partition for device provisioning.

## Test Directory Structure

```
test/
  conftest.py              # pytest config, sys.path setup, shared fixtures
  mocks/
    __init__.py
    pz_drive.py            # mock pz_drive C module (MagicMock-backed)
    sam.py                 # mock sam C module
    board_utils.py         # mock board_utils C module
    micropython_builtins.py # machine, esp32, network, ntptime, neopixel, etc.
  integration/
    test_pz_drive_py.py    # PzActuator state machine, mode switching, errors
    test_drv2665.py        # register driver logic
    test_shift_register.py # pin mapping, 32-bit word construction
    test_music.py          # note parsing, song playback sequences
    test_ota.py            # update checking, firmware flash, file rollback
    test_wifi.py           # connection logic, config persistence
  e2e/
    conftest.py            # e2e-specific fixtures (server startup, Playwright)
    test_api.py            # HTTP routes + WebSocket commands
    test_ui.py             # Playwright browser tests (all JS functionality)
    ota_server.py          # standalone mock OTA update server
    fixtures/
      test_firmware.bin    # dummy firmware binary
      versions.json        # test manifest
      files/               # test file update payloads
  unit/                    # reserved for future C unit tests
```

## Mock Architecture

### Mock Boundary

The mock boundary sits at the C module level. Three C modules are mocked:

- `pz_drive` — PWM/DDS, FIFO, shift register, I2C, polarity
- `sam` — text-to-speech (say, render)
- `board_utils` — enter_bootloader

Plus MicroPython builtins: `machine`, `esp32`, `network`, `ntptime`, `neopixel`, `_thread`.

### Mock Implementation

Each mock module exposes the same function signatures as the real C module, backed by `unittest.mock.MagicMock`. This gives:

- **Call assertion**: `assert_called_with`, `assert_called_once`, `call_args_list`
- **Return control**: `return_value`, wraps
- **Error injection**: `side_effect = OSError("NACK")`
- **Call counting**: `call_count`, `called`

A shared `_reset()` function on each mock module clears all mocks between tests, called by a pytest autouse fixture.

```python
# Example: test/mocks/pz_drive.py
from unittest.mock import MagicMock

pwm_set_frequency = MagicMock()
pwm_start = MagicMock()
pwm_stop = MagicMock()
pwm_is_running = MagicMock(return_value=False)
# ... all other functions ...

def _reset():
    for obj in list(globals().values()):
        if isinstance(obj, MagicMock):
            obj.reset_mock()
```

### sys.path Injection

`test/conftest.py` inserts `test/mocks/` at the front of `sys.path` and pre-loads MicroPython builtins into `sys.modules` before any application code is imported. This ensures `import pz_drive` resolves to the mock.

## web_server.py Refactor (Dependency Injection)

Currently `web_server.py` imports hardware dependencies at module level. Refactor to a `create_app(deps)` pattern:

```python
def create_app(deps=None):
    if deps is None:
        deps = _default_deps()  # real hardware
    app = Microdot()
    _register_routes(app, deps)
    return app

def start():
    app = create_app()
    app.run(port=80)
```

The `deps` namespace holds: `pa` (PzActuator), `music`, `sam`, `wifi`, `ota`. Tests pass mock objects; production code passes real ones. External behavior is unchanged.

Microdot supports CPython, so the real web_server.py code runs in tests without a MicroPython interpreter.

## Integration Tests

Run the full Python stack under CPython with mocked C modules. Test targets:

- **pz_drive_py.py**: Mode switching (analog/digital), frequency/gain validation, start/stop state machine, pin state tracking, sweep behavior, play_wav, error handling for invalid parameters
- **drv2665.py**: Register read/write sequences, gain mapping, init_analog/init_digital register values, standby, I2C error handling
- **shift_register.py**: Pin-to-bit mapping (pin N → bit 25-N), 32-bit word construction, bulk set/get, latch calls
- **music.py**: Note parsing (note:beats format), song lookup, BPM timing, amplitude/dynamics, rest handling, sweep/glissando
- **ota.py**: Version comparison, manifest parsing, update flow, rollback logic, error handling (network failures, corrupt downloads, hash mismatches)
- **wifi.py**: STA/AP fallback, config save/load, reconnect logic

## End-to-End Tests

### API Tests (test_api.py)

Use `aiohttp` or `requests` + `websockets` as clients against the real `web_server.py` (with mocked hardware deps) running on a random local port.

Test every HTTP route:
- `GET /`, `GET /wifi`, `GET /update` — page serving
- `GET /wifi/status` — WiFi status JSON
- `GET /api/music/songs` — song list with notes/BPM/gain
- `POST /api/ota/check`, `GET /api/ota/status`, `GET|PUT /api/ota/config` — OTA endpoints
- `POST /api/ota/update/firmware`, `POST /api/ota/update/files` — update triggers
- `POST /api/ota/upload` — file/firmware upload
- `GET /api/ota/log`, `POST /api/ota/diagnostics` — logging

Test every WebSocket command:
- `set_frequency_analog`, `set_frequency_digital` — frequency config
- `start`, `stop` — playback control
- `set_pin`, `set_all` — shift register
- `set_polarity` — polarity toggle
- `say` — TTS
- `play_music`, `play_song` — music playback
- `exec` — code execution
- `wifi_config` — WiFi reconfiguration

Assert both HTTP responses/WebSocket messages AND mock call sequences.

### Playwright Tests (test_ui.py)

Use `pytest-playwright` to test all JavaScript functionality in the web UI:

- **Tabs**: switching between Signal/Music/Speech, active tab highlighting
- **Waveform selector**: per-tab waveform memory, DC disabled on Music tab, disabled appearance on Speech tab with Square pre-selected
- **Signal tab**: frequency input, amplitude slider, polarity toggle, fullwave checkbox, dead_time/phase_advance inputs
- **Music tab**: song button loading (notes + BPM + gain update), notes textarea editing, BPM input
- **Speech tab**: text input, speed/pitch sliders, Enter key triggering START
- **DRV2665 card**: START/STOP button label changes per tab (START/PLAY/SPEAK), gain button selection and highlighting
- **Pin grid**: individual pin toggling, set_all, visual state updates
- **WebSocket**: connection status display, message send/receive
- **Footer**: version info display
- **Responsive behavior**: layout at different viewport sizes

### OTA Mock Server (ota_server.py)

Standalone `aiohttp` server serving:
- `GET /versions.json` — configurable version manifest
- `GET /firmware/<version>.bin` — test firmware binary with valid SHA-256
- `GET /files/<version>/manifest.json` — file update manifest
- `GET /files/<version>/<filename>` — individual update files
- `POST /diagnostics` — accepts and logs diagnostic data

Configurable behavior: version numbers, file contents, error injection (HTTP 500, corrupt data, slow responses, connection drops).

Usable as:
- **pytest fixture** — started/stopped automatically per test session
- **standalone script** — `python -m test.e2e.ota_server --port 8080` for testing against real hardware

## Hardware Info Partition

### Partition Layout

Add a 4KB read-only NVS partition to `partitions-4MiB-ota.csv`:

```
hw_info,  data, nvs, , 0x1000
```

Placed outside OTA app slots. Not touched by firmware flashes or OTA updates.

### Provisioning Script (scripts/provision.py)

One-time per-board provisioning tool:

```bash
python scripts/provision.py --port /dev/ttyACM0 \
    --serial-number "OPT-2026-0042" \
    --hw-revision "1.2" \
    --custom-key value
```

Uses ESP-IDF's `nvs_partition_gen.py` to create a partition binary, then `esptool` to flash it to the `hw_info` offset. Supports arbitrary key/value pairs for future extensibility.

### Runtime Reader (python/frozen/hw_info.py)

Frozen module that reads the `hw_info` NVS partition via `esp32.NVS`:

```python
import esp32

_nvs = esp32.NVS("hw_info")

def get(key, default=None):
    try:
        buf = bytearray(64)
        _nvs.get_blob(key, buf)
        return buf.rstrip(b'\x00').decode()
    except OSError:
        return default

def get_all():
    """Return dict of all known device identity keys."""
    keys = ["serial_number", "hw_revision"]
    return {k: get(k) for k in keys if get(k) is not None}
```

### OTA HTTP Headers

`ota.py` includes all hw_info values as `X-Device-*` headers on every OTA request:

```
X-Device-Serial-Number: OPT-2026-0042
X-Device-Hw-Revision: 1.2
X-Device-Firmware-Version: 0.1.0
```

Key names are transformed: underscores to hyphens, title-cased, prefixed with `X-Device-`.

## Source Code Changes Required

| File | Change |
|------|--------|
| `python/web_server.py` | Refactor to `create_app(deps)` for dependency injection |
| `python/ota.py` | Add `X-Device-*` headers from `hw_info` to all OTA HTTP requests |
| `python/frozen/hw_info.py` | **New** — read-only NVS accessor for device identity |
| `partitions-4MiB-ota.csv` | Add `hw_info` NVS partition (4KB) |
| `scripts/provision.py` | **New** — one-time hardware provisioning tool |

## Test Dependencies

```
# test/requirements.txt
pytest
pytest-asyncio
pytest-playwright
aiohttp
websockets
```

## Running Tests

```bash
# All tests
pytest test/

# Integration only
pytest test/integration/

# E2E only
pytest test/e2e/

# Playwright UI tests only
pytest test/e2e/test_ui.py

# OTA server standalone
python -m test.e2e.ota_server --port 8080
```
