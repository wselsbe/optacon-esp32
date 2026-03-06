# Hardware E2E Test Report

**Date:** 2026-03-07
**Board:** Optacon ESP32-S3 (OTA firmware, WiFi connected)
**Instruments:** Siglent SDS1104X-E oscilloscope, SPD3303X PSU, SDM3045X multimeter

## Summary

48 tests total: **31 passed, 10 failed, 7 xfail (known hardware faults)**

| Module | Pass | Fail | xfail | Notes |
|--------|------|------|-------|-------|
| start_stop | 4/4 | 0 | — | All passing |
| fullwave | 3/3 | 0 | — | All passing |
| signal_frequency | 3/9 | 6 | — | Scope can't measure at 50/500 Hz |
| signal_amplitude | 2/3 | 1 | — | Triangle waveform fails |
| gain | 2/5 | 3 | — | OUT+ PKPK returns None intermittently |
| power (idle/active/all) | 3/3 | 0 | — | All passing |
| power (per-pin) | 13/20 | 0 | 7 | Known hardware faults marked xfail |
| sweep | 0/1 | 1 | — | sweep_analog blocks event loop |

## Passing Tests

- **start_stop**: Signal appears on start, disappears on stop, no signal before start, double-stop safe.
- **fullwave**: Polarity toggles at signal frequency, static in non-fullwave, IN+ frequency doubles (|sin|).
- **signal_frequency**: 50-triangle, 250-sine, 250-square pass reliably.
- **signal_amplitude**: sine and square produce measurable amplitude on IN+ and OUT+.
- **gain**: gain=25 and gain=75 produce measurable signal.
- **power**: Idle <60mA, active <200mA, all-pins <500mA.
- **power (per-pin)**: Pins 0-3, 5-6, 10, 12-14, 17-19 pass at 120mA limit.

## Failing Tests — Analysis

### Signal Frequency (6/9 fail)

Pattern by waveform:
- **sine**: passes at 250Hz, fails at 50Hz and 500Hz
- **triangle**: passes at 50Hz, fails at 250Hz and 500Hz
- **square**: passes at 250Hz, fails at 50Hz and 500Hz

The scope returns `****` (can't measure) for failing combinations. Root cause is likely
timebase/trigger settling — the `configure_scope` fixture selects timebase by frequency
(5MS for <=100Hz, 2MS for <=300Hz, 1MS for >300Hz) but the scope may need more settling
time or different timebase choices for reliable measurement. The waveform-dependent pattern
suggests the scope's frequency counter algorithm works differently for each shape.

### Signal Amplitude — Triangle (1/3 fail)

IN+ PKPK returns None for triangle waveform. Triangle may have lower amplitude than
sine/square at the same settings, falling below the scope's auto-measurement threshold.

### Gain (3/5 fail)

`test_gain_ordering` fails because it can't measure OUT+ PKPK at gain=25 (first iteration).
`test_gain_produces_signal` fails for gain=50 and gain=100 — returns None. These sometimes
pass in isolation, suggesting a scope state issue from previous test iterations. The test
creates fresh BoardClient connections per gain level, which may contribute.

### Power Per-Pin (7/20 xfail)

All failures are genuine hardware faults marked as `xfail` — drawing 297-361mA (well above 120mA limit):

| Pin | Current | Pin | Current |
|-----|---------|-----|---------|
| 4   | 361mA   | 11  | 310mA   |
| 7   | 341mA   | 15  | 336mA   |
| 8   | 317mA   | 16  | 297mA   |
| 9   | 339mA   |     |         |

These pins have excessive current draw through the piezo load, indicating a genuine
hardware issue (likely solder bridges or damaged piezo elements).

**Passing pins (13/20):** 0-3, 5-6, 10, 12-14, 17-19

### Sweep (1/1 fail)

`sweep_analog()` blocks the microdot async event loop for its full 5-second duration.
The `/api/exec` endpoint has access to `pa` (confirmed — failure changed from stale-50Hz
to empty-frequencies after adding `pa` to exec globals), but the blocking call starves
the event loop so no signal is output during the sweep. Fix options: dedicated WS command
with background thread, REPL client bypass, or non-blocking sweep on board side.

## Test Infrastructure

- **Instruments:** Session-scoped TCP connections (single socket per instrument per run)
- **Board:** Function-scoped WebSocket client with fixture teardown (stop + close)
- **BoardClient:** Asserts on returned status for stop/start/set_frequency_analog
- **Scope config:** `configure_scope` fixture handles timebase, V/div, trigger, probe
- **Multimeter:** 600mA range for per-pin resolution, 3-reading average
