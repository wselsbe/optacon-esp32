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

- `init()` — Initialize DRV2665 + HV509 + PWM hardware
- `stop()` — Stop all output (digital FIFO task + analog PWM)
- `set_frequency_analog(hz)` — Analog sine wave via PWM + RC filter (50–400 Hz), 0 = DC fully on
- `set_frequency_digital(hz)` — Digital sine wave via I2C FIFO (50–4000 Hz)
- `set_pwm_resolution(bits)` — PWM resolution: 8 (312.5 kHz) or 10 (78.1 kHz)
- `set_waveform(buf)` — Custom waveform for digital path (advanced)
- `start()` — Start digital FIFO task (requires prior `set_waveform()`)
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
| PWM out         | 5    |

### Key Design Patterns

- **Dual signal paths**: Analog (PWM + RC filter → DRV2665 IN+) and digital (I2C FIFO → DRV2665 internal DAC). Analog path uses DDS phase accumulator at 32 kHz for sine generation. Digital path uses FreeRTOS background task to keep the 100-byte FIFO filled.
- **Shift register**: Two HV509 daisy-chained. 32-bit SPI word with pins 0–19 mapped to bits 25–6.
- **MCP server**: `mcp_micropython.py` uses stateless per-call serial connections via `mpremote` to avoid port locking.

### Important Notes

- `ESP_LOGE`/`ESP_LOGI` go to UART0, NOT USB-CDC REPL — use `mp_printf(&mp_plat_print, ...)` for REPL-visible output
- `i2c_master_transmit()` timeout is in milliseconds, not FreeRTOS ticks
- DRV2665: do NOT send reset (0x80 to CTRL2) during init — device NACKs mid-transaction
- MicroPython v1.27.0, ESP-IDF v5.5.1, ESP32-S3 with 4MB flash
