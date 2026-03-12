# Hardware E2E Tests Design

## Goal

Automated pytest tests that exercise the ESP32 board via its REST/WebSocket API and verify electrical behavior using lab instruments (oscilloscope, power supply, multimeter). These tests run against physical hardware only — excluded from CI.

## Lab Setup

| Instrument | Model | Address | Role |
|---|---|---|---|
| Oscilloscope | SDS1104X-E | 192.168.30.236:5025 | Verify analog signals on OUT+, IN+, polarity |
| Power Supply | SPD3303X | 192.168.30.134:5025 | Board power (CH1), quick current reads |
| Multimeter | SDM3045X | 192.168.30.17:5024 | Precision current measurement (6A range, in series with PSU) |

### Oscilloscope Channel Mapping

| Channel | Signal | Description |
|---|---|---|
| C2 | OUT+ | DRV2665 amplified output |
| C3 | Polarity | GPIO 12/13 toggle (digital) |
| C4 | IN+ | Pre-amplifier PWM filtered signal |
| C1 | — | Unused / ignore |

### Board Connection

The ESP32 board runs a web server (Microdot) accessible over WiFi. Hostname: `esp-optacon`.

Discovery order:
1. Try `http://esp-optacon.local/api/device/status` (mDNS)
2. Fallback: query IP via `mpremote exec "import wifi; print(wifi.ip)"`
3. After resolving IP, hit `/api/device/status` to confirm web server is running. If mpremote fallback was used, the board may need a poke (wait/retry) to ensure the web server is up.
4. Fail with clear error if neither method works.

## Instrument Communication

All three Siglent instruments speak SCPI over TCP. The test code uses a lightweight `SCPIConnection` class (copied from the existing `siglent-sdm-mcp` project pattern) — raw TCP sockets, no pyvisa dependency.

Thin wrapper classes per instrument expose only the operations needed for tests:

- **SDS1104XE**: configure_channel, measure (FREQ, PKPK, RMS, MEAN), configure_acquisition, screenshot
- **SPD3303X**: set_output (on/off), set_voltage, set_current, measure_voltage, measure_current, measure_power
- **SDM3045X**: configure (DC current, 6A range), read, measure

## Configuration

`test/hardware/config.json` (gitignored) with `config.example.json` committed as template:

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

Tests fail with a clear error message if `config.json` is missing.

## Test Scenarios

### 1. Signal Output Verification

For each waveform (sine, triangle, square) at representative frequencies (50, 250, 500 Hz):
- Set frequency via WS `set_frequency_analog`, then `start`
- Measure FREQ and PKPK on C4 (IN+) and C2 (OUT+)
- Assert frequency matches within tolerance
- Assert amplitude is non-zero and reasonable for the given gain

### 2. Fullwave / Polarity

Run with `fullwave=True` and `fullwave=False`:
- **Fullwave**: C3 (polarity) should toggle — measure FREQ on C3, expect signal frequency
- **Non-fullwave**: C3 should be static — measure PKPK on C3, expect near-zero
- Verify C4 frequency doubles in fullwave mode (|sin| has 2x fundamental)

### 3. Gain Levels

At fixed frequency (250 Hz sine), cycle through gain 25, 50, 75, 100:
- Measure C2 (OUT+) PKPK
- Assert proportional to gain setting: expected ~25Vpp, ~50Vpp, ~75Vpp, ~100Vpp
- Tolerance: configurable (default 20%)

### 4. Idle Power

Board powered, no signal running, no pins active:
- Read current from multimeter (SDM, DC current, 6A range)
- Assert below expected idle threshold

### 5. Active Power

Signal running at gain=100, 250 Hz:
- Read current from multimeter
- Assert within expected active current range

### 6. Pin Power Consumption

Individual pin current characterization:
- For each pin 0-19: enable only that pin (`set_pin`), measure current via multimeter, assert within expected per-pin range
- All pins on: `set_all(0xFFFFF)`, measure total current, assert within expected range
- Baseline: all pins off, measure idle current for delta calculation

### 7. Start / Stop

- **Positive**: Start signal, verify PKPK > threshold on C2 (OUT+)
- **Positive**: Stop signal, verify PKPK drops to noise floor on C2
- **Negative**: Before any start command, verify C2 PKPK is at noise floor
- **Negative**: After stop, verify C4 (IN+) also returns to noise floor

### 8. Frequency Sweep

Run `sweep_analog(50, 500, 5000)` (50-500 Hz over 5 seconds):
- Sample C4 FREQ multiple times during the sweep
- Assert frequency increases over time from ~50 Hz toward ~500 Hz

## File Structure

```
test/hardware/
  config.example.json   # committed template
  config.json           # gitignored, actual IPs
  conftest.py           # fixtures: config, instruments, board connection
  instruments.py        # SCPIConnection + SDS/SPD/SDM wrapper classes
  test_signal_output.py # scenario 1: frequency, amplitude, waveform
  test_fullwave.py      # scenario 2: polarity toggling
  test_gain.py          # scenario 3: gain level amplitude scaling
  test_power.py         # scenarios 4, 5, 6: idle, active, per-pin power
  test_start_stop.py    # scenario 7: start/stop + negative tests
  test_sweep.py         # scenario 8: frequency sweep
```

## Fixtures (conftest.py)

- `config` — loads `config.json`, fails with helpful error if missing
- `oscilloscope` — SDS wrapper, auto-disconnects after session
- `power_supply` — SPD wrapper, auto-disconnects after session
- `multimeter` — SDM wrapper, configured for DC current 6A range, auto-disconnects after session
- `board_url` — resolved board URL (mDNS hostname, mpremote fallback), verified reachable
- `board_ws` — WebSocket connection helper for real-time commands

## Pytest Integration

- Marker: `@pytest.mark.hardware` on all tests
- Location: `test/hardware/` excluded from default test runs
- `pyproject.toml`: configure `testpaths` to exclude `test/hardware/` from default `pytest test/`
- Run hardware tests explicitly: `pytest test/hardware/ -v`

## Expected Values & Tolerances

- Tolerance is configurable via `config.json` (default: 20%)
- Theoretical expected values derived from hardware specs (DRV2665 gain settings, signal chain)
- Tests may fail initially — fine-tuning amplitude and tolerance is expected
- Per-pin current thresholds to be established from baseline measurements

## Dependencies

- `aiohttp` (already in test requirements — for REST API calls)
- `websockets` (already in test requirements — for WS commands)
- No new external dependencies; instrument communication is raw TCP sockets
