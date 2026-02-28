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

MicroPython firmware for ESP32-S3 driving piezo actuators via DRV2665 + HV509 shift registers. Python controls high-level logic; the `pz_drive` C module handles all real-time hardware (PWM ISR, FIFO task, SPI, I2C).

### Project Structure

- `modules/pz_drive/` — Unified C module: DRV2665 I2C, HV509 SPI, analog DDS ISR, digital FIFO task
  - `pz_drive.c` — Module registration + Python bindings
  - `pz_drive.h` — Shared internal header
  - `pwm.c` — DDS ISR (32 kHz), sine/triangle/square, fullwave, dead_time, phase_advance
  - `fifo.c` — FreeRTOS FIFO fill task (8 kHz), digital fullwave
  - `hv509.c` — SPI bus + CS/LE, polarity GPIOs (12/13), 32-bit write, stage/latch
  - `drv2665.c` — I2C bus init, register read/write, FIFO helpers
- `modules/board_utils/` — C module: enter_bootloader() utility
- `python/drv2665.py` — Python DRV2665 I2C register driver (delegates to pz_drive)
- `python/shift_register.py` — Python HV509 shift register driver (delegates to pz_drive)
- `python/pz_actuator_py.py` — Python high-level PzActuator orchestrator
- `python/main.py` — Boot script and demo functions
- `scripts/` — Docker build and flash helpers (`build.sh`, `flash.sh`)
- `mcp_micropython.py` — MCP server for serial interaction with the board from Claude Code

### Python API (`PzActuator` class in `pz_actuator_py`)

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
pa.set_frequency_analog(hz, resolution=8, amplitude=100, fullwave=False,
                        dead_time=0, phase_advance=0, waveform='sine')  # 0-400 Hz
pa.set_frequency_digital(hz, fullwave=False, waveform='sine')  # 1-4000 Hz
pa.start(gain=100)  # gain: 25/50/75/100 Vpp
pa.stop()
pa.is_running()
pa.set_pin(pin, value, latch=True) / pa.get_pin(pin)  # single actuator channel (0-19)
pa.set_all(value, latch=True) / pa.get_all()  # bulk pin control
pa.latch()  # commit pending shift register changes
pa.get_polarity()  # read-only; polarity toggling is internal to ISR/task
```

### C Module APIs

**pz_drive** (unified hardware control):
```python
import pz_drive
# PWM / Analog DDS
pz_drive.pwm_set_frequency(hz, resolution=8, amplitude=128,
                           fullwave=False, dead_time=0, phase_advance=0,
                           waveform=0)  # 0=sine, 1=triangle, 2=square
pz_drive.pwm_start() / pz_drive.pwm_stop() / pz_drive.pwm_is_running()
# FIFO / Digital
pz_drive.fifo_start(waveform_buf, gain=3, fullwave=False)
pz_drive.fifo_stop() / pz_drive.fifo_is_running()
# Shift Register
pz_drive.sr_stage(word32)  # stage + latch pending (immediate if ISR stopped)
pz_drive.sr_write(word32)  # immediate SPI write (debug)
# I2C
pz_drive.i2c_read(reg) / pz_drive.i2c_write(reg, val)
# Polarity
pz_drive.pol_init() / pz_drive.pol_get()  # pol_toggle is internal only
```

**board_utils**:
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

- **Dual signal paths**: Analog (PWM + RC filter → DRV2665 IN+) and digital (I2C FIFO → DRV2665 internal DAC). Analog path uses DDS phase accumulator at 32 kHz for sine/triangle/square generation. Digital path uses FreeRTOS background task to keep the 100-byte FIFO filled.
- **Fullwave mode**: Generates |waveform| and toggles polarity at zero-crossings for ~200V pk-pk output. Dead time and phase advance compensate for DRV2665 output settling.
- **Shift register latch**: SPI writes are synchronized to waveform events (zero-crossing in fullwave, cycle start in non-fullwave) via stage/latch mechanism. If no ISR/task is running, writes are immediate.
- **Shift register**: Two HV509 daisy-chained. 32-bit SPI word with pins 0–19 mapped to bits 25–6.
- **I2C bus sharing**: pz_drive owns the I2C bus persistently. ESP-IDF's I2C master driver is thread-safe (mutex per transaction). FIFO task and Python register access share the same handle safely.
- **MCP server**: `mcp_micropython.py` uses stateless per-call serial connections via `mpremote` to avoid port locking.

### Important Notes

- `ESP_LOGE`/`ESP_LOGI` go to UART0, NOT USB-CDC REPL — use `mp_printf(&mp_plat_print, ...)` for REPL-visible output
- `i2c_master_transmit()` timeout is in milliseconds, not FreeRTOS ticks
- DRV2665: do NOT send reset (0x80 to CTRL2) during init — device NACKs mid-transaction
- GPIO 12/13 (polarity) MUST be configured BEFORE SPI bus init (IOMUX conflict on GPIO 10-13)
- MicroPython v1.27.0, ESP-IDF v5.5.1, ESP32-S3 with 4MB flash
