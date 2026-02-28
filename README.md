# Optacon Firmware

MicroPython firmware for ESP32-S3 driving piezo actuators via DRV2665 + HV509 shift registers.

## Hardware

- **ESP32-S3-WROOM-1** -- main microcontroller
- **DRV2665** -- TI piezo haptic driver (I2C, address 0x59)
- **2x HV509** -- high-voltage shift registers, daisy-chained (SPI)
- **20 piezo actuators** -- connected to HV509 outputs

### Pin Assignments

| Function   | GPIO | Notes                                 |
|------------|------|---------------------------------------|
| I2C SDA    | 47   | DRV2665 data                          |
| I2C SCL    | 21   | DRV2665 clock, 100 kHz                |
| SPI MOSI   | 6    | Shift register data                   |
| SPI MISO   | 7    | Shift register data (unused)          |
| SPI SCK    | 9    | Shift register clock, 1 MHz           |
| SPI CS/LE  | 10   | Latch enable (manual GPIO, not SPI CS)|
| Polarity A | 12   | HV509 POL_bar (active-low)            |
| Polarity B | 13   | HV509 POL_bar (active-low)            |
| PWM out    | 5    | LEDC -> RC filter -> DRV2665 IN+      |

## Quick Start

Connect to the board's MicroPython REPL (`mpremote connect COM7`), then:

```python
from pz_actuator_py import PzActuator

pa = PzActuator()

# Output a 250 Hz sine wave (analog mode, via PWM + RC filter)
pa.set_frequency_analog(250)
pa.set_all(1)       # enable all 20 actuator channels
pa.start()          # start output at 100 Vpp

# ... actuators are vibrating ...

pa.stop()
```

To drive a single pin:

```python
pa.set_frequency_analog(250)
pa.set_all(0)        # all channels off
pa.set_pin(4, 1)     # enable channel 4
pa.start()
```

## Python API

### `PzActuator` Class

The high-level controller. Import from `pz_actuator_py`:

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
```

#### Mode Configuration

**`set_frequency_analog(hz, resolution=8, amplitude=100, fullwave=False, dead_time=0, phase_advance=0, waveform='sine')`**

Configure analog output mode. The ESP32 generates a waveform via DDS (direct digital synthesis) on a 32 kHz GPTimer ISR, outputs it as PWM on GPIO 5, which passes through an RC low-pass filter into the DRV2665 analog input.

| Parameter       | Type | Default  | Description                                     |
|-----------------|------|----------|-------------------------------------------------|
| `hz`            | int  | required | Frequency: 0 (DC) or 50-400 Hz                 |
| `resolution`    | int  | 8        | PWM bit depth: 8 or 10                          |
| `amplitude`     | int  | 100      | Output amplitude, 0-100 (percentage)            |
| `fullwave`      | bool | False    | Rectified output with polarity toggle (~200 Vpp)|
| `dead_time`     | int  | 0        | ISR ticks to blank output near zero-crossings   |
| `phase_advance` | int  | 0        | ISR ticks to advance polarity toggle timing     |
| `waveform`      | str  | 'sine'   | 'sine', 'triangle', or 'square'                 |

**`set_frequency_digital(hz, fullwave=False, waveform='sine')`**

Configure digital output mode. Python generates a waveform buffer, and a FreeRTOS background task streams it to the DRV2665's internal DAC via I2C FIFO at 8 kHz.

| Parameter  | Type | Default  | Description                                     |
|------------|------|----------|-------------------------------------------------|
| `hz`       | int  | required | Frequency: 1-4000 Hz                            |
| `fullwave` | bool | False    | Half-period buffer with polarity toggle          |
| `waveform` | str  | 'sine'   | 'sine', 'triangle', or 'square'                 |

#### Playback

**`start(gain=100)`** -- Start output in the configured mode. `gain` sets output voltage: 25, 50, 75, or 100 Vpp.

**`stop()`** -- Stop output and put DRV2665 in standby.

**`is_running()`** -- Returns `True` if output is active.

#### Actuator Channels (Shift Register)

The 20 actuator channels are controlled via two daisy-chained HV509 high-voltage shift registers. When output is running, pin changes are synchronized to waveform events (zero-crossings in fullwave mode, cycle start otherwise) to avoid glitches.

**`set_pin(pin, value, latch=True)`** -- Set a single channel (0-19) on or off.

**`get_pin(pin)`** -- Read current state of a channel.

**`set_all(value, latch=True)`** -- Set all 20 channels to the same state.

**`get_all()`** -- Returns tuple of 20 values (0 or 1).

**`set_pins(values, latch=True)`** -- Set all channels from a 20-element iterable.

**`latch()`** -- Commit pending changes. When `latch=True` (default) on set methods, this happens automatically.

#### Polarity

**`toggle_polarity()`** -- Toggle the HV509 polarity pins (GPIO 12/13).

**`get_polarity()`** -- Returns current polarity state.

### `DRV2665` Class

Low-level DRV2665 register interface. Import from `drv2665`:

```python
from drv2665 import DRV2665
drv = DRV2665()
```

| Method                     | Description                                    |
|----------------------------|------------------------------------------------|
| `init_analog(gain)`        | Configure analog input mode + EN_OVERRIDE      |
| `init_digital(gain)`       | Configure digital FIFO mode                    |
| `standby()`                | Enter standby (CTRL2 bit 6)                    |
| `status()`                 | Read STATUS register (FIFO_FULL/FIFO_EMPTY)    |

Gain constants: `DRV2665.GAIN_25` (0), `GAIN_50` (1), `GAIN_75` (2), `GAIN_100` (3).

### `ShiftRegister` Class

Low-level HV509 shift register interface. Import from `shift_register`:

```python
from shift_register import ShiftRegister
sr = ShiftRegister()
```

Same pin control methods as `PzActuator` (`set_pin`, `get_pin`, `set_all`, `get_all`, `set_pins`, `latch`). The 32-bit SPI word maps pin N to bit `25 - N`, with bits [31:26] and [5:0] unused.

## C Module: `pz_drive`

The `pz_drive` C module handles all real-time hardware interaction. It is used internally by the Python classes above, but can also be called directly:

```python
import pz_drive
```

### Analog (PWM/DDS)

```python
pz_drive.pwm_set_frequency(hz, resolution=8, amplitude=128,
                           fullwave=False, dead_time=0, phase_advance=0,
                           waveform=0)   # 0=sine, 1=triangle, 2=square
pz_drive.pwm_start()
pz_drive.pwm_stop()
pz_drive.pwm_is_running()               # -> bool
```

The DDS runs at 32 kHz via GPTimer ISR. A 32-bit phase accumulator indexes a 256-entry waveform LUT. The output drives LEDC PWM on GPIO 5. In fullwave mode, the ISR generates |waveform| and toggles polarity at zero-crossings. `amplitude` is 0-128 (internal scale; the Python API maps 0-100% to this range).

### Digital (FIFO)

```python
pz_drive.fifo_start(waveform_buf, gain=3, fullwave=False)
pz_drive.fifo_stop()
pz_drive.fifo_is_running()              # -> bool
```

`waveform_buf` is a `bytearray` of signed 8-bit samples representing one period. A FreeRTOS task (pinned to core 1) streams samples to the DRV2665's 100-byte FIFO at 8 kHz using `esp_timer` for precise timing. At each period boundary, the task latches pending shift register data and optionally toggles polarity.

### Shift Register

```python
pz_drive.sr_stage(word32)    # stage data + latch (immediate if no ISR running)
pz_drive.sr_write(word32)    # immediate write (debug)
```

`sr_stage` clocks data into the shift register via SPI, then either latches immediately (if no ISR/task is running) or sets a pending flag so the ISR/task latches at the next waveform event.

### I2C (DRV2665)

```python
pz_drive.i2c_read(reg)       # -> int (0-255, or -1 on error)
pz_drive.i2c_write(reg, val)
```

### Polarity

```python
pz_drive.pol_init()           # configure GPIO 12/13 as outputs
pz_drive.pol_set(val)         # set both polarity pins
pz_drive.pol_get()            # -> bool
```

## C Module: `board_utils`

```python
import board_utils
board_utils.enter_bootloader()  # switch to USB download mode (never returns)
```

## Architecture

### Signal Paths

There are two independent signal paths from the ESP32 to the DRV2665:

**Analog path** (PWM -> RC filter -> DRV2665 IN+): A GPTimer ISR at 32 kHz runs a DDS phase accumulator that generates waveform samples. Each sample sets the LEDC PWM duty cycle on GPIO 5. An external RC low-pass filter (R=390R, C=100nF, fc~4 kHz) smooths the PWM into an analog voltage that feeds the DRV2665 analog input. The DRV2665's internal boost converter amplifies this to the configured gain (25-100 Vpp).

**Digital path** (I2C FIFO -> DRV2665 internal DAC): Python pre-computes one period of the waveform as a `bytearray`. A FreeRTOS background task streams these samples to the DRV2665's 100-byte FIFO register over I2C. The DRV2665's internal DAC converts samples to analog at 8 kHz. An `esp_timer` fires every 8 ms, waking the task to refill the FIFO with the estimated number of consumed samples.

### Fullwave Mode

For higher output voltage (~200 Vpp), fullwave mode generates the absolute value of the waveform (`|sin(t)|`) and toggles the HV509 polarity pins at each zero-crossing. This effectively doubles the output voltage by using both halves of the waveform constructively. The `dead_time` and `phase_advance` parameters compensate for the DRV2665 output settling time during polarity transitions.

### Shift Register Synchronization

Pin changes are staged via `sr_stage()` which clocks data into the shift register immediately (SPI is safe from the main thread), but defers the latch (LE rising edge on GPIO 10) to the next waveform event. This prevents glitches from mid-cycle actuator switching. If no ISR or FIFO task is running, the latch happens immediately.

### Project Structure

```
modules/pz_drive/         C module: all real-time hardware control
  pz_drive.c              module registration + Python bindings
  pz_drive.h              shared internal header
  pwm.c                   analog DDS ISR (32 kHz GPTimer)
  fifo.c                  digital FIFO task (8 kHz esp_timer)
  hv509.c                 SPI shift register + polarity GPIOs
  drv2665.c               I2C bus init, register access, FIFO helpers
modules/board_utils/      C module: enter_bootloader()
python/
  pz_actuator_py.py       high-level PzActuator orchestrator
  drv2665.py              DRV2665 register driver
  shift_register.py       HV509 shift register driver
  main.py                 boot script + demo functions
scripts/                  build and flash helpers
```

## Building

### Docker (no local toolchain needed)

```bash
docker compose build               # first time only
docker compose run --rm dev bash /workspace/scripts/build.sh
```

### Local (Windows, requires ESP-IDF + GNU Make)

```bash
source ~/esp/v5.5.1/export.sh      # or export.bat on CMD
export MICROPYTHON_DIR=C:/Projects/Optacon/micropython
./scripts/build.sh
```

### Flashing

```bash
./scripts/flash.sh COM7             # adjust COM port as needed
```

To enter bootloader mode from the REPL:

```python
import board_utils
board_utils.enter_bootloader()
```
