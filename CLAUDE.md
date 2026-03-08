# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

### Setup after clone
```bash
# Install git pre-commit hook (auto-fixes Python with ruff, checks C with clang-format)
ln -sf ../../scripts/pre-commit.sh .git/hooks/pre-commit
```

### Building and flashing
Use your /flash skill to build the firmware and flash to hardware

### Connect to REPL:
```bash
mpremote connect COM7
```

### Run tests:
```bash
# All tests (142 tests: integration + E2E API + Playwright UI)
pytest test/

# Integration only
pytest test/integration/

# E2E API tests
pytest test/e2e/test_api.py

# Playwright UI tests
pytest test/e2e/test_ui.py

# OTA mock server (standalone, for testing against real hardware)
python -m test.e2e.ota_server --port 8080
```

### Provision hardware identity:
```bash
python scripts/provision.py --port /dev/ttyACM0 \
    --serial-number "OPT-2026-0042" \
    --hw-revision "1.2"
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
- `modules/sam/` — SAM speech synthesizer C module (text-to-speech, 22 kHz 8-bit PCM)
- `modules/board_utils/` — C module: enter_bootloader() utility
- `python/frozen/` — Frozen Python modules (compiled into firmware image)
  - `boot_cfg.py` — OTA/boot configuration constants
  - `boot.py` — Boot sequence: OTA rollback safety, watchdog, HW init
  - `pz_drive_py.py` — High-level PzDrive orchestrator
  - `drv2665.py` — DRV2665 I2C register driver (delegates to pz_drive)
  - `shift_register.py` — HV509 shift register driver (delegates to pz_drive)
  - `hw_info.py` — Read-only device identity from hw_info NVS partition
  - `main.py` — Application entry point and demo functions
- `python/_boot.py` — Custom VFS mount (overlaid onto vendor _boot.py by build.sh)
- `python/` — Filesystem Python modules (uploaded to board, not frozen)
  - `web_server.py` — Async HTTP/WebSocket server (microdot), `create_app(deps)` for DI/testability
- `config/partitions-4MiB-ota.csv` — Custom partition table with hw_info NVS partition
- `scripts/provision.py` — One-time hardware identity provisioning tool
- `test/` — Test suite (CPython, pytest)
  - `mocks/` — Mock C modules (pz_drive, sam, board_utils) + MicroPython builtins
  - `integration/` — Integration tests for all Python modules
  - `e2e/` — E2E tests: API/WebSocket (aiohttp) + Playwright UI tests + OTA mock server
  - `music.py` — Sheet music player: note sequences, built-in songs, play via PzDrive
  - `wifi.py` — WiFi STA management, config persistence
  - `ota.py` — OTA firmware update client
- `web/` — Web UI files (served from board filesystem)
  - `index.html` — Main control panel with tabbed Signal/Music/Speech interface
  - `docs.html` — API reference documentation
  - `wifi.html` — WiFi configuration page
  - `update.html` — OTA firmware update page
- `scripts/` — Docker build and flash helpers (`build.sh`, `flash.sh`)
- `mcp_micropython.py` — MCP server for serial interaction with the board from Claude Code

### Python API (`PzDrive` class in `pz_drive_py`)

```python
from pz_drive_py import PzDrive
pa = PzDrive()
pa.set_frequency_analog(hz, resolution=8, amplitude=100, fullwave=False,
                        dead_time=0, phase_advance=0, waveform='sine')  # 0-1000 Hz
pa.set_frequency_digital(hz, fullwave=False, waveform='sine')  # 1-500 Hz (FIFO rate limited)
pa.set_frequency_live(hz, amplitude=100, waveform='sine')  # update while running (no restart)
pa.start(gain=100)  # gain: 25/50/75/100 Vpp
pa.stop()
pa.is_running()
pa.set_pin(pin, value, latch=True) / pa.get_pin(pin)  # single actuator channel (0-19)
pa.set_all(value, latch=True) / pa.get_all()  # bulk pin control
pa.latch()  # commit pending shift register changes
pa.get_polarity()  # read-only; polarity toggling is internal to ISR/task
pa.sweep_analog(start_hz, end_hz, duration_ms,
                logarithmic=False, waveform='sine',
                resolution=8, amplitude=100, gain=100)  # frequency sweep
```

### C Module APIs

**pz_drive** (unified hardware control):
```python
import pz_drive
# PWM / Analog DDS
pz_drive.pwm_set_frequency(hz, resolution=8, amplitude=128,
                           fullwave=False, dead_time=0, phase_advance=0,
                           waveform=0)  # 0=sine, 1=triangle, 2=square; hz 0-1000
pz_drive.pwm_start() / pz_drive.pwm_stop() / pz_drive.pwm_is_running()
pz_drive.pwm_set_frequency_live(hz, amplitude=128, waveform=0)  # update while running
pz_drive.pwm_set_sweep(target_step, increment, logarithmic=False)
pz_drive.pwm_is_sweep_done()
pz_drive.pwm_play_samples(buf, sample_rate, loop=False)  # raw sample playback
pz_drive.pwm_is_sample_done()
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

**sam** (text-to-speech via SAM formant synthesizer):
```python
import sam
sam.say("hello world")                          # blocking TTS playback via pz_drive ISR
sam.say("test", speed=72, pitch=64, gain=100)   # with voice parameters
buf = sam.render("hello")                       # returns 8-bit PCM bytearray at 22050 Hz
buf = sam.render("test", speed=72, pitch=64, mouth=128, throat=128)
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
- **ISR core affinity**: GPTimer ISR is pinned to core 1 via a temporary FreeRTOS task that calls both `gptimer_register_even

### Important Notes

- `ESP_LOGE`/`ESP_LOGI` go to UART0, NOT USB-CDC REPL — use `mp_printf(&mp_plat_print, ...)` for REPL-visible output
- `i2c_master_transmit()` timeout is in milliseconds, not FreeRTOS ticks
- DRV2665: do NOT send reset (0x80 to CTRL2) during init — device NACKs mid-transaction
- GPIO 12/13 (polarity) MUST be configured BEFORE SPI bus init (IOMUX conflict on GPIO 10-13)
- MicroPython v1.27.0, ESP-IDF v5.5.1, ESP32-S3 with 4MB flash
