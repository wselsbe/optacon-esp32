# API Documentation — Design

Date: 2026-03-04

## Goal

Add a hand-written HTML API reference page served from the ESP32, documenting the MicroPython hardware API with descriptions and code examples.

## Scope

**Modules documented:**
1. **PzActuator** (`pz_drive_py`) — Main high-level API: signal generation, frequency control, WAV playback, sweeps
2. **music** — Sheet music player: `note_freq()`, `play()`, `play_song()`, song registry
3. **DRV2665** (`drv2665`) — Piezo driver IC I2C interface: mode configuration, gain, standby
4. **ShiftRegister** (`shift_register`) — HV509 pin control: individual and bulk pin operations

**Not documented:** `wifi.py`, `web_server.py` (internal infrastructure, not user-facing hardware API).

## Architecture

### Single static HTML page

`web/docs.html` — hand-written, self-contained HTML page styled consistently with the existing UI (same CSS variables, card layout, monospace fonts). No build step, no dependencies.

### Content per module

Each module gets a card-style section containing:
- Module description and import example
- Method signatures with parameter types and ranges
- 1-2 code examples per key method (drawn from `main.py` demos and `music.py` patterns)
- Constants/class attributes where relevant (gain values, register addresses)

### Examples source

Code examples drawn from:
- `main.py`: `demo_analog()`, `demo_digital()`, `demo()` — basic usage patterns
- `music.py`: module docstring examples, `play_song()` usage — advanced patterns
- `shift_register.py`: pin control patterns

### Serving

The existing static route `@app.route("/web/<path:path>")` already serves any file under `/web/`. No new route needed — `docs.html` is accessible at `/web/docs.html` automatically.

### UI link

In the terminal card's toggle bar, add a small "API Docs" link (styled like the WiFi link) that opens `/web/docs.html` in a new tab via `target="_blank"`.

### CI

Add a step to the `lint-python` job that verifies `web/docs.html` exists. Lightweight existence check only — no generation.

### Flash/upload

`web/docs.html` is a filesystem file (not frozen). Added to `mpremote cp` in the flash skill alongside `index.html` and `wifi.html`.

## Page structure

```
[Header: "Optacon API Reference" + back link to "/"]

[Card: PzActuator]
  - Description
  - Constructor
  - set_frequency_analog() — with parameter table + example
  - set_frequency_digital() — with parameter table + example
  - set_frequency_live() — with example
  - start() / stop() / is_running()
  - play_wav() — with example
  - sweep_analog() — with example
  - get_status()
  - shift_register (attribute) — link to ShiftRegister section

[Card: music]
  - Description
  - note_freq()
  - play() — with note format table + example
  - play_song() — with song list + example
  - Available songs table

[Card: DRV2665]
  - Description
  - Gain constants
  - init_digital() / init_analog()
  - standby()
  - status()

[Card: ShiftRegister]
  - Description
  - set_pin() / get_pin()
  - set_all() / get_all()
  - set_pins()
  - latch()
```
