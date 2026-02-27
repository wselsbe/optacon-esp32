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

MicroPython firmware for ESP32-S3 driving piezo actuators via DRV2665 + HV509 shift registers. Python controls all hardware (I2C, SPI); minimal C modules handle only real-time tasks (FIFO fill, analog DDS).

### Project Structure

- `modules/drv_fifo/` — C module: DRV2665 FIFO fill background task (esp_timer + FreeRTOS)
- `modules/pz_pwm/` — C module: analog DDS sine generation via LEDC PWM at 32 kHz
- `modules/board_utils/` — C module: enter_bootloader() utility
- `python/drv2665.py` — Python DRV2665 I2C register driver
- `python/shift_register.py` — Python HV509 SPI shift register driver
- `python/pz_actuator_py.py` — Python high-level PzActuator orchestrator
- `python/main.py` — Boot script and demo functions
- `scripts/` — Docker build and flash helpers (`build.sh`, `flash.sh`)
- `mcp_micropython.py` — MCP server for serial interaction with the board from Claude Code

### Python API (`PzActuator` class in `pz_actuator_py`)

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
pa.set_frequency_analog(hz, resolution=8, amplitude=100)  # analog PWM mode (0-400 Hz)
pa.set_frequency_digital(hz)  # digital FIFO mode (1-4000 Hz)
pa.start(gain=100)  # gain: 25/50/75/100 Vpp
pa.stop()
pa.is_running()
pa.set_pin(pin, value) / pa.get_pin(pin)  # single actuator channel (0-19)
pa.set_all(value) / pa.get_all()  # bulk pin control
pa.toggle_polarity() / pa.get_polarity()
```

### C Module APIs

- `drv_fifo.start(i2c, waveform)` / `stop()` / `is_running()` — FIFO fill task
- `drv_fifo.read_reg(reg)` / `write_reg(reg, val)` — bypass I2C probe for bus sharing
- `pz_pwm.set_frequency(hz, resolution=8, amplitude=128)` / `start()` / `stop()` / `is_running()`
- `board_utils.enter_bootloader()` — switch to USB download mode

### Hardware Pins

| Function        | GPIO |
|-----------------|------|
| I2C SDA         | 47   |
| I2C SCL         | 21   |
| SPI MOSI        | 6    |
| SPI MISO        | 7    |
| SPI SCK         | 9    |
| SPI CS          | 10   |
| Polarity A      | 12   |
| Polarity B      | 13   |
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
