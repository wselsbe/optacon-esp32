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

### Signal Frequency (6/9 fail → likely improved by scope fixes, pending re-run)

Pattern by waveform:
- **sine**: passes at 250Hz, fails at 50Hz and 500Hz
- **triangle**: passes at 50Hz, fails at 250Hz and 500Hz
- **square**: passes at 250Hz, fails at 50Hz and 500Hz

The scope returns `****` (can't measure) for failing combinations. The ATTN/VDIV ordering
bug and ARM single-trigger bug (both now fixed) likely caused many of these failures —
the scope was configured with wrong V/div scales and measuring stale data. Re-run needed
to determine which failures persist after the scope fixes.

### Signal Amplitude — Triangle (1/3 fail)

IN+ PKPK returns None for triangle waveform. Triangle may have lower amplitude than
sine/square at the same settings, falling below the scope's auto-measurement threshold.

### Gain (3/5 fail → fixed, pending re-run)

Root cause identified via manual MCP/REPL investigation:

1. **ATTN/VDIV ordering bug** — `instruments.py` sent `VDIV` before `ATTN`. On Siglent
   scopes, changing ATTN rescales the existing VDIV (e.g., VDIV 10V then ATTN 10 → 100V/div).
   Fix: send ATTN first, invalidate VDIV cache when ATTN changes.

2. **ARM overrides TRMD AUTO** — `run()` sent `ARM` which forces single-trigger mode.
   Subsequent measurements read stale captured data. Fix: use `TRMD AUTO` instead of `ARM`.

3. **Fixed 10V/div too coarse for low gains** — OUT+ amplitude scales with gain:
   gain=25 → ~3.5Vpp, gain=50 → ~10Vpp, gain=75/100 → clipped at higher V/div.
   Fix: per-gain VDIV map (500mV/2V/2V/5V), default OUT+ lowered to 2V/div.

These fixes also likely resolve some signal_frequency failures (same ATTN/VDIV and trigger
mode bugs affected all scope measurements).

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
