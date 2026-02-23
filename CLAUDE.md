# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Development Commands

### Local build (Windows, requires make + gcc + ESP-IDF):
```bash
# One-time setup: source ESP-IDF environment
source ~/esp/v5.5.1/export.sh  # Git Bash
# or: %USERPROFILE%\esp\v5.5.1\export.bat  # CMD

export MICROPYTHON_DIR=C:/Projects/Optacon/micropython
./scripts/build.sh
```

### Docker build (no local toolchain needed):
```bash
build.cmd
# or manually:
MSYS_NO_PATHCONV=1 docker compose run --rm dev bash /workspace/scripts/build.sh
```

### Flash to ESP32-S3 (from Windows, default COM7):
```bash
flash.cmd [COM_PORT]
# or: ./scripts/flash.sh COM7
```

### Connect to REPL:
```bash
mpremote connect COM7
```

## Architecture Overview

MicroPython firmware for ESP32-S3 driving piezo actuators via DRV2665 + HV509 shift registers. The C driver is compiled as a MicroPython user module (`pz_actuator`), with Python scripts frozen into the firmware.

### Project Structure

- `modules/pz_actuator/` — C user module (DRV2665 I2C, shift register SPI, waveform generation, FreeRTOS background task)
- `python/` — MicroPython scripts frozen into firmware (`main.py`, `manifest.py`)
- `scripts/` — Docker build and flash helpers (`build.sh`, `flash.sh`)
- `mcp_micropython.py` — MCP server for serial interaction with the board from Claude Code

### MicroPython API (`pz_actuator` module)

- `init()` — Initialize DRV2665 + HV509 hardware
- `start()` / `stop()` — Start/stop background waveform task
- `set_frequency(hz)` / `get_frequency()` — Waveform frequency (50–4000 Hz, default 250)
- `set_pin(pin, value, flush=True)` / `get_pin(pin)` — Single actuator channel (0–19)
- `set_pins(list, flush=True)` / `get_all()` / `set_all(value, flush=True)` — Bulk pin control
- `flush()` — Commit pending shift register changes
- `toggle_polarity()` / `get_polarity()` — HV509 polarity control
- `set_gain(gain)` — DRV2665 gain (25/50/75/100 Vpp)
- `is_running()` — Check if background task is active

### Hardware Pins

| Function        | GPIO |
|-----------------|------|
| I2C SDA         | 47   |
| I2C SCL         | 21   |
| SPI MOSI        | 6    |
| SPI MISO        | 7    |
| SPI SCK         | 9    |
| SPI CS          | 10   |
| Polarity toggle | 34   |

### Key Design Patterns

- **Background task**: FreeRTOS task keeps DRV2665's 100-byte FIFO filled. Frequency changes and polarity toggles are signaled via atomic flags so MicroPython never blocks on hardware I/O.
- **Shift register**: Two HV509 daisy-chained. 32-bit SPI word with pins 0–19 mapped to bits 25–6.
- **MCP server**: `mcp_micropython.py` uses stateless per-call serial connections via `mpremote` to avoid port locking.

### Important Notes

- `ESP_LOGE`/`ESP_LOGI` go to UART0, NOT USB-CDC REPL — use `mp_printf(&mp_plat_print, ...)` for REPL-visible output
- `i2c_master_transmit()` timeout is in milliseconds, not FreeRTOS ticks
- DRV2665: do NOT send reset (0x80 to CTRL2) during init — device NACKs mid-transaction
- MicroPython v1.27.0, ESP-IDF v5.5.1, ESP32-S3 with 4MB flash
