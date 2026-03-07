# Python I2C Refactor Design

Move DRV2665 configuration and shift register control from C to Python. Keep only real-time tasks (FIFO fill, analog DDS) in C.

## Motivation

The current C module handles everything: I2C bus creation, DRV2665 register config, SPI shift register control, FIFO fill task, and PWM DDS. This makes the C code large and hard to iterate on. Most of these operations don't need real-time performance and are better suited to Python, where they're easier to modify and debug interactively.

## Architecture

```
Python layer (drv2665.py, shift_register.py, pz_actuator.py)
    |
    +-- machine.I2C(0, sda=47, scl=21, freq=100000)  <- Python owns
    +-- machine.SPI(1, ...)                            <- Python owns
    |
    +-- pz_fifo C module
    |   +-- start(i2c, waveform) -> extracts bus_handle, spawns FIFO task
    |   +-- stop() -> stops task, removes device
    |   +-- is_running() -> bool
    |
    +-- pz_pwm C module
        +-- set_frequency(hz, resolution=8, amplitude=128) -> configures DDS + LEDC
        +-- start() -> starts 32kHz timer ISR
        +-- stop() -> stops timer
        +-- is_running() -> bool
```

### What moves to Python

- DRV2665 init: exit standby, set mode/gain, EN_OVERRIDE
- DRV2665 register reads/writes (status, debug, config)
- Shift register: SPI writes, pin state tracking
- Start/stop orchestration, mode selection, gain mapping
- Waveform generation (sine table via `math.sin`)

### What stays in C

- **pz_fifo**: FreeRTOS background task that fills the DRV2665 100-byte FIFO at 8kHz sample rate. Extracts `bus_handle` from MicroPython I2C object, adds device at 0x59, feeds samples in a loop.
- **pz_pwm**: 32kHz DDS timer ISR with LEDC duty cycle updates. Python cannot update PWM duty at 32kHz (VM overhead limits to ~hundreds of updates/sec).

## I2C Driver Choice

MicroPython defaults to the legacy I2C driver (`MICROPY_HW_ESP_NEW_I2C_DRIVER=0`), which uses the old `i2c_port_t` + command-link API. Our C code uses the new `i2c_master.h` API. These are incompatible on the same port.

**Decision**: Enable the new driver (`MICROPY_HW_ESP_NEW_I2C_DRIVER=1`). MicroPython v1.27.0 fully supports it. On IDF 5.5+, MicroPython keeps a persistent `bus_handle` + `dev_handle` in `machine_hw_i2c_obj_t`. Our C code can extract the `bus_handle` and add its own device handle for the FIFO task.

Set via cmake (no MicroPython source changes needed):
```cmake
target_compile_definitions(usermod_pz_actuator INTERFACE
    MICROPY_HW_ESP_NEW_I2C_DRIVER=1
)
```

This works because `mpconfigport.h` uses `#ifndef MICROPY_HW_ESP_NEW_I2C_DRIVER`.

## Concurrency Model

### I2C bus sharing (Python + FIFO task)

The ESP-IDF new I2C driver has a built-in binary semaphore (`bus_lock_mux`) per bus. When two tasks call `i2c_master_transmit()` on the same bus:
- The first caller acquires the lock
- The second caller blocks on the semaphore
- When the first finishes, the second proceeds

No additional mutex needed. The FIFO task runs at `configMAX_PRIORITIES - 2`. If Python is mid-transaction, the task blocks for ~200-400us (one I2C transaction at 100kHz). With 12.5ms of FIFO buffer (100 samples at 8kHz), this is well within margin.

### SPI (shift register)

Only Python accesses SPI. The FIFO task does not touch SPI. No concurrency concern. Future trough-sync (C task writing shift register at waveform troughs) would need a mutex around SPI.

## C Module APIs

### pz_fifo

```python
import pz_fifo

# Start FIFO fill task
# i2c: machine.I2C object (new driver required)
# waveform: bytearray of signed 8-bit samples (one period at 8kHz)
pz_fifo.start(i2c, waveform)

# Stop task, remove device from bus
pz_fifo.stop()

# Check if running
pz_fifo.is_running()  # -> bool
```

Internally:
1. Cast MP object to `machine_hw_i2c_obj_t*`, extract `bus_handle`
2. `i2c_master_bus_add_device(bus_handle, ...)` at 0x59
3. Spawn FreeRTOS task (hybrid bulk + byte-by-byte fill strategy)
4. On stop: set `running = false`, wait for task exit, `i2c_master_bus_rm_device()`

### pz_pwm

```python
import pz_pwm

# Configure DDS + LEDC
# hz: 50-400 (0 = DC)
# resolution: 8 or 10 bit
# amplitude: 0-128 (internal scale, default 128 = full)
pz_pwm.set_frequency(hz, resolution=8, amplitude=128)

pz_pwm.start()
pz_pwm.stop()
pz_pwm.is_running()  # -> bool
```

## Python Driver Layer

### python/drv2665.py

```python
class DRV2665:
    ADDR = 0x59
    REG_STATUS = 0x00
    REG_CTRL1  = 0x01
    REG_CTRL2  = 0x02

    GAIN_25  = 0b00
    GAIN_50  = 0b01
    GAIN_75  = 0b10
    GAIN_100 = 0b11

    def __init__(self, i2c):
        self.i2c = i2c
        self._verify_device()  # read STATUS, raise OSError if no ACK

    def read_reg(self, reg): ...
    def write_reg(self, reg, val): ...
    def init_analog(self, gain=GAIN_100): ...
    def init_digital(self, gain=GAIN_100): ...
    def standby(self): ...
    def status(self): ...
```

### python/shift_register.py

```python
class ShiftRegister:
    def __init__(self, spi, cs_pin):
        self.spi = spi
        self.cs = cs_pin
        self._state = 0x00000000  # 32-bit shadow

    def set_pin(self, pin, value): ...
    def get_pin(self, pin): ...
    def flush(self): ...
    def set_all(self, value): ...
    def get_all(self): ...
```

### python/pz_actuator.py

```python
class PzActuator:
    GAINS = {25: DRV2665.GAIN_25, 50: DRV2665.GAIN_50,
             75: DRV2665.GAIN_75, 100: DRV2665.GAIN_100}

    def __init__(self):
        self.i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100_000)
        self.spi = SPI(1, baudrate=1_000_000, ...)
        self.drv = DRV2665(self.i2c)
        self.sr = ShiftRegister(self.spi, Pin(10, Pin.OUT))
        self._mode = None

    def set_frequency_digital(self, hz):
        n = round(8000 / hz)
        self._waveform = bytearray(int(127 * sin(2*pi*i/n)) for i in range(n))
        self._mode = 'digital'

    def set_frequency_analog(self, hz, resolution=8, amplitude=100):
        amp_internal = (amplitude * 128 + 50) // 100
        pz_pwm.set_frequency(hz, resolution, amp_internal)
        self._mode = 'analog'

    def start(self, gain=100):
        gain_bits = self.GAINS[gain]
        if self._mode == 'digital':
            self.drv.init_digital(gain_bits)
            pz_fifo.start(self.i2c, self._waveform)
        elif self._mode == 'analog':
            self.drv.init_analog(gain_bits)
            pz_pwm.start()

    def stop(self):
        if pz_fifo.is_running(): pz_fifo.stop()
        if pz_pwm.is_running(): pz_pwm.stop()
        self.drv.standby()

    # Shift register pass-through
    def set_pin(self, pin, value, flush=True): ...
    def set_pins(self, pins, flush=True): ...
    def flush(self): ...
```

## Error Handling

- **Python I2C errors**: `OSError` on NACK/timeout, propagated naturally
- **FIFO task errors**: `i2c_master_transmit` failures logged, task continues (best-effort)
- **Device not found**: `DRV2665.__init__()` raises `OSError` if STATUS read fails
- **Bus add failure**: `pz_fifo.start()` raises `OSError` if device can't be added

## Testing Plan

1. **Build**: Enable new I2C driver, verify firmware builds and boots
2. **I2C**: `machine.I2C` works, `i2c.scan()` finds DRV2665 at 0x59
3. **DRV2665 Python driver**: Register reads/writes, analog init, digital init
4. **FIFO task**: `pz_fifo.start()` produces sine on oscilloscope
5. **Bus sharing**: Python register reads during active FIFO playback
6. **SPI shift register**: Pin toggling via `machine.SPI`
7. **Full integration**: `PzActuator` class end-to-end

## Files Changed

### New files
- `python/drv2665.py` — DRV2665 register driver
- `python/shift_register.py` — HV509 shift register driver
- `python/pz_actuator.py` — High-level orchestrator
- `modules/pz_fifo/` — Minimal C module for FIFO task
- `modules/pz_pwm/` — Minimal C module for analog DDS

### Modified files
- `modules/micropython.cmake` — Include new modules, remove old
- `python/manifest.py` — Freeze new Python files

### Removed files
- `modules/pz_actuator/drv2665.c` — Config moved to Python
- `modules/pz_actuator/drv2665.h` — Config moved to Python
- `modules/pz_actuator/shift_register.c` — Moved to Python
- `modules/pz_actuator/shift_register.h` — Moved to Python
- `modules/pz_actuator/pz_actuator.c` — Replaced by Python orchestrator
- `modules/pz_actuator/sine.c` — Waveform generation moved to Python
