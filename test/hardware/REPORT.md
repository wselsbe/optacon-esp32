# Hardware E2E Test Report

**Date:** 2026-03-07
**Board:** Optacon ESP32-S3 (OTA firmware, WiFi connected)
**Instruments:** Siglent SDS1104X-E oscilloscope, SPD3303X PSU, SDM3045X multimeter

## Summary

48 tests total: **19 passed, 29 failed**

| Module | Pass | Fail | Notes |
|--------|------|------|-------|
| start_stop | 4/4 | 0 | All passing |
| fullwave | 3/3 | 0 | All passing |
| signal_frequency | 3/9 | 6 | Scope can't measure at 50/500 Hz |
| signal_amplitude | 2/3 | 1 | Triangle waveform fails |
| gain | 2/5 | 3 | OUT+ PKPK returns None intermittently |
| power (idle/active/all) | 3/3 | 0 | All passing |
| power (per-pin) | 5/20 | 15 | Known hardware fault |
| sweep | 0/1 | 1 | Sweep never executes via /api/exec |

## Passing Tests

- **start_stop**: Signal appears on start, disappears on stop, no signal before start, double-stop safe.
- **fullwave**: Polarity toggles at signal frequency, static in non-fullwave, IN+ frequency doubles (|sin|).
- **signal_frequency**: 50-triangle, 250-sine, 250-square pass reliably.
- **signal_amplitude**: sine and square produce measurable amplitude on IN+ and OUT+.
- **gain**: gain=25 and gain=75 produce measurable signal.
- **power**: Idle <60mA, active <200mA, all-pins <500mA.
- **power (per-pin)**: Pins 5, 10, 12, 17, 19 pass at 120mA limit.

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

### Power Per-Pin (15/20 fail)

Two categories of failure at the 120mA limit:

**Marginal (100-120mA):** Pins 0-3, 6, 13-14 — just above base load (~90mA) plus
measurement noise. These are borderline and may pass with a slightly higher limit.

**Hardware fault (270-365mA):** Pins 4, 7-9, 11, 15-16 — significantly over limit.
These pins have excessive current draw through the piezo load, indicating a genuine
hardware issue (likely solder bridges or damaged piezo elements).

**Passing pins:** 5, 10, 12, 17, 19

### Sweep (1/1 fail)

Frequency stays at 50.1Hz throughout the 5-second sweep. The `/api/exec` endpoint fires
`pa.sweep_analog()` in a background thread, but the measured frequency never changes.
The exec endpoint may not have access to the running PzActuator instance, or the sweep
blocks the web server's async event loop.

## Test Infrastructure

- **Instruments:** Session-scoped TCP connections (single socket per instrument per run)
- **Board:** Function-scoped WebSocket client with fixture teardown (stop + close)
- **BoardClient:** Asserts on returned status for stop/start/set_frequency_analog
- **Scope config:** `configure_scope` fixture handles timebase, V/div, trigger, probe
- **Multimeter:** 600mA range for per-pin resolution, 3-reading average
