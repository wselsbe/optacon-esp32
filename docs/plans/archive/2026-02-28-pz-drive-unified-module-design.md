# pz_drive Unified Module Design

**Goal:** Consolidate all real-time hardware control (PWM, FIFO, shift register, polarity, I2C) into a single C module `pz_drive`. Add waveform types and fullwave to the digital path. Synchronize shift register latching to waveform zero-crossings.

**Replaces:** `pz_pwm` (C), `pz_fifo` (C), `shift_register.py` (Python SPI layer)

## C Module: `pz_drive`

### File Layout

```
modules/pz_drive/
  pz_drive.c         module registration + Python bindings
  pz_drive.h         shared internal header
  pwm.c              DDS ISR (32 kHz), sine/triangle/square, fullwave, dead_time, phase_advance
  fifo.c             FreeRTOS FIFO fill task (8 kHz), digital fullwave
  hv509.c            SPI bus + CS/LE, polarity GPIOs (12/13), 32-bit write, stage/latch
  drv2665.c          I2C bus init, register read/write
  micropython.cmake
```

### Shared Header (`pz_drive.h`)

Internal functions called across files:

```c
// hv509.c — SPI shift register + polarity GPIOs
void hv509_init(void);                    // SPI bus + CS/LE + polarity GPIO setup
void hv509_sr_stage(uint32_t word32);     // set pending buffer + flag (immediate if ISR stopped)
void hv509_sr_write(uint32_t word32);     // immediate SPI write (debug)
void hv509_sr_latch_if_pending(void);     // called from ISR/task at the right moment
void hv509_pol_init(void);
void hv509_pol_set(bool val);
bool hv509_pol_get(void);
void hv509_pol_toggle(void);

// pwm.c
bool pwm_is_running(void);

// fifo.c
bool fifo_is_running(void);
```

### Python API (`import pz_drive`)

```python
# PWM / Analog DDS
pz_drive.pwm_set_frequency(hz, resolution=8, amplitude=128,
                           fullwave=False, dead_time=0, phase_advance=0,
                           waveform=0)  # 0=sine, 1=triangle, 2=square
pz_drive.pwm_start()
pz_drive.pwm_stop()
pz_drive.pwm_is_running()

# FIFO / Digital
pz_drive.fifo_start(waveform_buf, gain=3, fullwave=False)
pz_drive.fifo_stop()
pz_drive.fifo_is_running()

# Shift Register (raw 32-bit)
pz_drive.sr_stage(word32)      # stage + latch pending (immediate if ISR stopped)
pz_drive.sr_write(word32)      # immediate SPI write (debug)

# I2C (raw DRV2665 register access)
pz_drive.i2c_read(reg)
pz_drive.i2c_write(reg, val)

# Polarity
pz_drive.pol_init()            # part of hv509 — GPIO 12/13
pz_drive.pol_set(val)
pz_drive.pol_get()
```

## Python Classes

### DRV2665 (`drv2665.py`)

Delegates I2C to `pz_drive.i2c_read()` / `pz_drive.i2c_write()`. No longer owns an `I2C` object.

Note: Python code should include a warning comment that heavy I2C access (tight loops of `_read_reg`/`_write_reg`) during active FIFO playback may cause audio glitches.

```python
class DRV2665:
    def init_analog(self, gain=GAIN_100)
    def init_digital(self, gain=GAIN_100)
    def standby(self)
    def status(self)
    def _read_reg(self, reg)       # pz_drive.i2c_read(reg)
    def _write_reg(self, reg, val) # pz_drive.i2c_write(reg, val)
```

### ShiftRegister (`shift_register.py`)

Maintains Python-side 32-bit buffer with pin-to-bit mapping (pin N -> bit 25-N). Delegates SPI to `pz_drive`.

```python
class ShiftRegister:
    def set_pin(self, pin, value, latch=True)
    def get_pin(self, pin)
    def set_all(self, value, latch=True)
    def get_all(self)
    def set_pins(self, values, latch=True)
    def latch(self)                  # pz_drive.sr_stage(self._state)
    def _direct_write(self, word32)  # pz_drive.sr_write(word32) -- debug
```

When `latch=True`, the method calls `self.latch()` after updating the buffer. `sr_stage()` auto-detects whether the ISR/task is running: if yes, sets pending flag for ISR-timed latch; if no, does immediate SPI write.

### PzActuator (`pz_actuator_py.py`)

Orchestrator, mostly unchanged API. No longer passes `I2C`/`SPI` objects — `pz_drive` owns the buses.

## Latch Timing

| Mode                   | Latch fires at                              |
|------------------------|---------------------------------------------|
| Analog, no fullwave    | Cycle start (phase_acc wraps past 0)        |
| Analog, fullwave       | Zero-crossing (polarity toggle moment)      |
| Digital, no fullwave   | write_index == 0 (period start)             |
| Digital, fullwave      | write_index == 0 (polarity toggle + start)  |
| ISR/task not running   | Immediate SPI write                         |

## I2C Bus Sharing

`pz_drive` owns the I2C bus and DRV2665 device handle persistently. Both the FIFO task and Python `DRV2665._read_reg()` / `_write_reg()` use the same handle. ESP-IDF's I2C master driver is thread-safe (mutex per transaction), and the DRV2665 FIFO holds 100 samples (12.5 ms at 8 kHz), so occasional Python register reads (~100 us each) won't cause underruns. Heavy burst I2C access from Python during FIFO playback should be avoided but is not a hard constraint.

## board_utils

`board_utils.enter_bootloader()` remains a separate module (`modules/board_utils/`). It is independent from `pz_drive`.

## Digital Fullwave

For digital fullwave, Python generates a half-period `|waveform|` buffer. The `fifo.c` task tracks `write_index` and at each period boundary (index wraps to 0):
1. Toggles polarity via `hv509_pol_toggle()`
2. Latches shift register via `hv509_sr_latch_if_pending()`

## Naming Changes

- `flush()` renamed to `latch()` everywhere (Python ShiftRegister method, C latch mechanism)
- Module import: `import pz_drive` (replaces `import pz_pwm` and `import pz_fifo`)
