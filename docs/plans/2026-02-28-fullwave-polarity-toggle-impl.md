# Full-Wave Polarity Toggle Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Enable ~200V pk-pk actuator drive by generating a full-wave rectified sine and toggling HV509 polarity at zero-crossings.

**Architecture:** The pz_pwm C module gains polarity GPIO ownership (12/13) and a fullwave mode. In fullwave mode, the 32 kHz ISR mirrors the negative half of the sine LUT and toggles polarity GPIOs at zero-crossings. Python's ShiftRegister loses polarity ownership; PzActuator delegates polarity to pz_pwm.

**Tech Stack:** ESP-IDF GPIO driver, MicroPython C module API, existing pz_pwm DDS ISR

---

### Task 1: Add polarity GPIO ownership to pz_pwm C module

Move GPIO 12/13 (polarity) ownership from Python ShiftRegister to the pz_pwm C module. Configure them as outputs with initial LOW state. Expose `set_polarity()`/`get_polarity()` to Python.

**Files:**
- Modify: `modules/pz_pwm/pz_pwm.c`

**Step 1: Add GPIO include and polarity defines**

Add after the existing `#include "esp_log.h"` line:

```c
#include "driver/gpio.h"

// ...existing defines...

#define POL_A_GPIO  12
#define POL_B_GPIO  13
```

**Step 2: Add polarity state variable**

Add to the "Module state" section, after `s_freq_configured`:

```c
static bool s_polarity = false;  // false=LOW (inverted mode), true=HIGH
static bool s_pol_gpio_initialized = false;
```

**Step 3: Add polarity GPIO init function**

Add before `ensure_hw_init()`:

```c
static void ensure_pol_gpio_init(void) {
    if (s_pol_gpio_initialized) return;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POL_A_GPIO) | (1ULL << POL_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(POL_A_GPIO, 0);
    gpio_set_level(POL_B_GPIO, 0);
    s_polarity = false;
    s_pol_gpio_initialized = true;
}
```

**Step 4: Add init_polarity() Python binding**

This must be called before SPI init (due to IOMUX conflict on GPIO 10-13). Add after `pz_pwm_is_running`:

```c
// pz_pwm.init_polarity()
static mp_obj_t pz_pwm_init_polarity(void) {
    ensure_pol_gpio_init();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_init_polarity_obj, pz_pwm_init_polarity);
```

**Step 5: Add set_polarity() and get_polarity() Python bindings**

Add after `init_polarity`:

```c
// pz_pwm.set_polarity(value)
static mp_obj_t pz_pwm_set_polarity(mp_obj_t value_obj) {
    ensure_pol_gpio_init();
    bool val = mp_obj_is_true(value_obj);
    gpio_set_level(POL_A_GPIO, val ? 1 : 0);
    gpio_set_level(POL_B_GPIO, val ? 1 : 0);
    s_polarity = val;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_pwm_set_polarity_obj, pz_pwm_set_polarity);

// pz_pwm.get_polarity()
static mp_obj_t pz_pwm_get_polarity(void) {
    return mp_obj_new_bool(s_polarity);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_get_polarity_obj, pz_pwm_get_polarity);
```

**Step 6: Register new functions in module globals table**

Add to `pz_pwm_globals_table[]`:

```c
    {MP_ROM_QSTR(MP_QSTR_init_polarity), MP_ROM_PTR(&pz_pwm_init_polarity_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_polarity), MP_ROM_PTR(&pz_pwm_set_polarity_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_polarity), MP_ROM_PTR(&pz_pwm_get_polarity_obj)},
```

**Step 7: Update module header comment**

Update the Python API comment at top of file to include:
```
//   pz_pwm.init_polarity()            — configure GPIO 12/13 (call before SPI init)
//   pz_pwm.set_polarity(value)        — set polarity GPIO state (True/False)
//   pz_pwm.get_polarity()             — get current polarity state
```

**Step 8: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`
Expected: Clean build, no errors.

**Step 9: Commit**

```
feat(pz_pwm): add polarity GPIO ownership (12/13)

Move polarity pin control from Python ShiftRegister to C module.
GPIO 12/13 configured as outputs via ESP-IDF gpio driver.
New API: init_polarity(), set_polarity(), get_polarity().
```

---

### Task 2: Add fullwave mode to pz_pwm ISR

Add the `fullwave` kwarg to `set_frequency()` and modify the ISR to generate |sin| with polarity toggling.

**Files:**
- Modify: `modules/pz_pwm/pz_pwm.c`

**Step 1: Add fullwave state variables**

Add to "Module state" section, after `s_pol_gpio_initialized`:

```c
// Fullwave mode state
static volatile bool s_fullwave = false;
static volatile uint8_t s_prev_half = 0;  // 0=first half (0..pi), 1=second half (pi..2pi)
```

**Step 2: Modify ISR for fullwave mode**

Replace the ISR body (inside `timer_isr_callback`) with:

```c
    s_phase_acc += s_phase_step;

    // Top 8 bits of phase accumulator index the 256-entry LUT
    uint8_t index = (uint8_t)(s_phase_acc >> 24);

    // Scale sine by amplitude: center at 128, scale deviation, re-center
    int32_t raw = (int32_t)sine_lut[index] - 128; // -128 to +127
    int32_t scaled = 128 + ((raw * (int32_t)s_amplitude) >> 7);
    uint32_t duty = (uint32_t)scaled;

    if (s_fullwave) {
        // Full-wave rectification: mirror negative half around 128
        uint8_t half = (uint8_t)(s_phase_acc >> 31);
        if (half) {
            duty = 256 - duty;  // mirror: values below 128 become above 128
        }

        // Toggle polarity at zero-crossing (when half-cycle changes)
        if (half != s_prev_half) {
            s_prev_half = half;
            uint32_t pol_val = half ? 1 : 0;
            gpio_set_level(POL_A_GPIO, pol_val);
            gpio_set_level(POL_B_GPIO, pol_val);
        }
    }

    // Scale 8-bit value to current resolution
    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6); // 0-255 → 0-1023
    }

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);

    return false; // no need to yield
```

**Step 3: Add fullwave kwarg to set_frequency()**

Update the `allowed_args[]` array and enum in `pz_pwm_set_frequency()`:

```c
    enum { ARG_hz, ARG_resolution, ARG_amplitude, ARG_fullwave };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_hz, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_resolution, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8}},
        {MP_QSTR_amplitude, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_fullwave, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
```

Add after the existing validation code (after `if (amplitude > 128) amplitude = 128;`):

```c
    bool fullwave = args[ARG_fullwave].u_bool;
```

Add before `s_freq_configured = true;`:

```c
    s_fullwave = fullwave;
```

Update the print statements to show fullwave mode:

```c
    if (hz == 0) {
        mp_printf(&mp_plat_print, "pz_pwm: DC mode, %d-bit, amplitude=%d\n", resolution, amplitude);
    } else {
        mp_printf(&mp_plat_print, "pz_pwm: %d Hz, %d-bit, amplitude=%d%s\n", hz, resolution,
                  amplitude, fullwave ? " [fullwave]" : "");
    }
```

**Step 4: Reset polarity on start() and stop()**

In `pwm_start_internal()`, add after `s_phase_acc = 0;`:

```c
    if (s_fullwave) {
        ensure_pol_gpio_init();
        s_prev_half = 0;
        gpio_set_level(POL_A_GPIO, 0);
        gpio_set_level(POL_B_GPIO, 0);
        s_polarity = false;
    }
```

In `pwm_stop_internal()`, add after `s_running = false;`:

```c
    if (s_fullwave) {
        gpio_set_level(POL_A_GPIO, 0);
        gpio_set_level(POL_B_GPIO, 0);
        s_polarity = false;
    }
```

**Step 5: Update module header comment**

Update the Python API line for `set_frequency`:
```
//   pz_pwm.set_frequency(hz, resolution=8, amplitude=128, fullwave=False)
```

**Step 6: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`
Expected: Clean build, no errors.

**Step 7: Commit**

```
feat(pz_pwm): add fullwave mode with ISR polarity toggling

In fullwave mode, the ISR generates |sin(t)| by mirroring the negative
half of the sine LUT, and toggles GPIO 12/13 at each zero-crossing.
This enables ~200V pk-pk drive using a single DRV2665 output.
```

---

### Task 3: Remove polarity from Python ShiftRegister

Remove polarity GPIO ownership from `shift_register.py` since it now lives in `pz_pwm`.

**Files:**
- Modify: `python/shift_register.py`

**Step 1: Remove polarity from constructor**

Replace the constructor with:

```python
    def __init__(self, spi, cs_pin):
        self.spi = spi
        self.cs = cs_pin
        self._state = 0x00000000
        self._tx_buf = bytearray(4)

        # Commit initial state (all off)
        self.flush()
```

**Step 2: Remove toggle_polarity() and get_polarity() methods**

Delete these two methods entirely (lines 76-84 of current file).

**Step 3: Commit**

```
refactor(shift_register): remove polarity GPIO ownership

Polarity pins (GPIO 12/13) now owned by pz_pwm C module.
```

---

### Task 4: Update PzActuator Python orchestrator

Wire up the fullwave parameter and delegate polarity to pz_pwm.

**Files:**
- Modify: `python/pz_actuator_py.py`

**Step 1: Add init_polarity() call before SPI init**

In `__init__()`, add `pz_pwm.init_polarity()` BEFORE SPI creation. The constructor becomes:

```python
    def __init__(self):
        # Configure polarity GPIOs before SPI (IOMUX conflict on GPIO 10-13)
        pz_pwm.init_polarity()
        # Own the I2C bus
        self.i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100_000)
        # Own the SPI bus
        self.spi = SPI(1, baudrate=1_000_000, polarity=0, phase=0,
                       sck=Pin(9), mosi=Pin(6), miso=Pin(7))
        # Initialize drivers
        self.drv = DRV2665(self.i2c)
        self.sr = ShiftRegister(self.spi, Pin(10, Pin.OUT))

        self._mode = None
        self._waveform = None
        self._gain = 100
```

**Step 2: Add fullwave parameter to set_frequency_analog()**

```python
    def set_frequency_analog(self, hz, resolution=8, amplitude=100, fullwave=False):
        """Configure analog PWM+DDS mode at given frequency.

        Args:
            hz: 0-400 (0 = DC output)
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage, mapped to internal 0-128)
            fullwave: if True, generate |sin| and toggle polarity at zero-crossings
        """
        if hz < 0 or hz > 400:
            raise ValueError("hz must be 0-400")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_pwm.set_frequency(hz, resolution=resolution, amplitude=amp_internal, fullwave=fullwave)
        self._mode = MODE_ANALOG
```

**Step 3: Update toggle_polarity() and get_polarity()**

Replace the existing methods:

```python
    def toggle_polarity(self):
        pz_pwm.set_polarity(not pz_pwm.get_polarity())

    def get_polarity(self):
        return pz_pwm.get_polarity()
```

**Step 4: Commit**

```
feat(pz_actuator): wire up fullwave mode and C-based polarity

set_frequency_analog() now accepts fullwave=False kwarg.
Polarity control delegates to pz_pwm C module.
init_polarity() called before SPI init to avoid IOMUX conflict.
```

---

### Task 5: Flash, test on hardware, verify with scope

**Step 1: Build**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

**Step 2: Enter bootloader and flash**

Use MCP tool `mcp__micropython__enter_bootloader` then flash:
```bash
./scripts/flash.sh COM4   # bootloader port
```

**Step 3: Test basic polarity (non-fullwave)**

Via REPL or MCP exec:
```python
from pz_actuator_py import PzActuator
pa = PzActuator()
pa.set_frequency_analog(250)
pa.set_all(True)
pa.start()
# Verify sine on scope CH1
pa.toggle_polarity()
# Verify polarity flipped on scope
pa.toggle_polarity()
pa.stop()
```

**Step 4: Test fullwave mode**

```python
pa.set_frequency_analog(250, fullwave=True)
pa.set_all(True)
pa.start()
# On scope: CH1 should show full-wave rectified sine (all positive bumps at 500 Hz)
# On scope: CH2/logic analyzer should show polarity toggling at 250 Hz (every other bump)
# Net result: actuator sees 250 Hz sine at ~200V pk-pk
```

**Step 5: Verify stop resets polarity**

```python
pa.stop()
# Polarity GPIOs should be LOW
print(pa.get_polarity())  # Should print False
```

**Step 6: Test different frequencies**

```python
for f in [50, 100, 200, 250]:
    pa.set_frequency_analog(f, fullwave=True)
    pa.start()
    import time; time.sleep(2)
    pa.stop()
```
