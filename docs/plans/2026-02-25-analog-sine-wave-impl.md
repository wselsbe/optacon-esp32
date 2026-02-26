# Analog Sine Wave Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add PWM-based analog sine wave output to the pz_actuator module, plus refactor digital sine generation from Python into C.

**Architecture:** New `sine.c`/`sine.h` files handle the shared sine LUT and DDS phase accumulator. New `pwm.c`/`pwm.h` files handle LEDC + GPTimer for analog output. The existing `task.c` is extended to accept a C-generated sine buffer for `set_frequency_digital()`. The MicroPython binding in `pz_actuator.c` gets new functions. All new files are added to `micropython.cmake`.

**Tech Stack:** ESP-IDF v5.5.1 (LEDC, GPTimer), MicroPython v1.27.0 C user module, FreeRTOS.

**Design doc:** `docs/plans/2026-02-25-analog-sine-wave-design.md`

---

### Task 1: Sine Lookup Table (`sine.h` / `sine.c`)

Shared sine LUT used by both analog (PWM) and digital (FIFO) paths.

**Files:**
- Create: `modules/pz_actuator/sine.h`
- Create: `modules/pz_actuator/sine.c`

**Step 1: Create `sine.h`**

```c
#ifndef PZ_SINE_H
#define PZ_SINE_H

#include <stdint.h>
#include <stddef.h>

// 256-entry sine table, values 0-255 (one full cycle, peak at index 64)
#define SINE_LUT_SIZE 256

extern const uint8_t sine_lut_8bit[SINE_LUT_SIZE];

// Generate a signed sine waveform for the DRV2665 digital FIFO.
// Writes one full period into `buf`. Returns number of samples written.
// buf must be at least (sample_rate / freq_hz) bytes.
// Output range: -127 to +127 (signed 8-bit, trough at index 0).
size_t sine_generate_digital(int8_t *buf, size_t buf_max, uint16_t freq_hz, uint16_t sample_rate);

#endif // PZ_SINE_H
```

**Step 2: Create `sine.c`**

```c
#include "sine.h"
#include <math.h>

// Pre-computed 256-entry sine table: 0-255 range (unsigned, for PWM duty cycle)
// sin(0)=128, sin(π/2)=255, sin(π)=128, sin(3π/2)=0
const uint8_t sine_lut_8bit[SINE_LUT_SIZE] = {
    // Generate with: round(127.5 + 127.5 * sin(2π * i / 256)) for i in 0..255
    128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
    176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
    218,220,222,224,226,228,230,232,234,235,237,239,240,241,243,244,
    245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,
    255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,
    245,244,243,241,240,239,237,235,234,232,230,228,226,224,222,220,
    218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,
    176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,
    128,124,121,118,115,112,109,106,103,100, 97, 93, 90, 88, 85, 82,
     79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40,
     37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 15, 14, 12, 11,
     10,  9,  7,  6,  5,  5,  4,  3,  2,  2,  1,  1,  1,  0,  0,  0,
      0,  0,  0,  0,  1,  1,  1,  2,  2,  3,  4,  5,  5,  6,  7,  9,
     10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35,
     37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76,
     79, 82, 85, 88, 90, 93, 97,100,103,106,109,112,115,118,121,124
};

size_t sine_generate_digital(int8_t *buf, size_t buf_max, uint16_t freq_hz, uint16_t sample_rate) {
    if (freq_hz == 0 || sample_rate == 0) return 0;

    size_t period = sample_rate / freq_hz;
    if (period < 2) period = 2;
    if (period > buf_max) period = buf_max;

    // Generate signed sine: -127 to +127, trough at index 0
    for (size_t i = 0; i < period; i++) {
        // Phase offset: -π/2 so trough is at index 0 (matches existing Python behavior)
        double phase = -M_PI / 2.0 + (2.0 * M_PI * i) / period;
        buf[i] = (int8_t)(sin(phase) * 127.0);
    }
    return period;
}
```

**Step 3: Build and verify**

Add `sine.c` to `micropython.cmake` (done in Task 4), then build.

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: clean compile.

**Step 4: Commit**

```bash
git add modules/pz_actuator/sine.h modules/pz_actuator/sine.c
git commit -m "feat: add shared sine lookup table and digital waveform generator"
```

---

### Task 2: PWM + DDS Analog Output (`pwm.h` / `pwm.c`)

LEDC PWM on GPIO5 with GPTimer-driven DDS for sine wave generation.

**Files:**
- Create: `modules/pz_actuator/pwm.h`
- Create: `modules/pz_actuator/pwm.c`

**Step 1: Create `pwm.h`**

```c
#ifndef PZ_PWM_H
#define PZ_PWM_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Compile-time configuration
#define PWM_GPIO                5
#define PWM_DEFAULT_RESOLUTION  8       // 8 or 10
#define PWM_SAMPLE_RATE_HZ      32000   // DDS update rate

// Initialize LEDC and GPTimer (call once from pz_actuator init)
esp_err_t pwm_init(void);

// Deinitialize PWM and timer
esp_err_t pwm_deinit(void);

// Start sine wave output at given frequency (50-400 Hz).
// freq_hz == 0 means DC fully on (100% duty).
esp_err_t pwm_start_sine(uint16_t freq_hz);

// Stop PWM output (sets duty to 0)
esp_err_t pwm_stop(void);

// Change resolution at runtime (8 or 10 bits).
// Stops and restarts output if currently running.
esp_err_t pwm_set_resolution(uint8_t bits);

// Returns true if PWM sine is currently running
bool pwm_is_running(void);

#endif // PZ_PWM_H
```

**Step 2: Create `pwm.c`**

```c
#include "pwm.h"
#include "sine.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "pwm";

// State
static gptimer_handle_t s_timer = NULL;
static bool s_initialized = false;
static bool s_running = false;
static uint8_t s_resolution = PWM_DEFAULT_RESOLUTION;

// DDS state (accessed from ISR)
static volatile uint32_t s_phase_acc = 0;
static volatile uint32_t s_phase_step = 0;

#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

static uint32_t ledc_max_duty(void) {
    return (1U << s_resolution) - 1;
}

// GPTimer ISR: update LEDC duty from sine LUT
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer,
                                          const gptimer_alarm_event_data_t *edata,
                                          void *user_data) {
    s_phase_acc += s_phase_step;

    // Map phase accumulator top 8 bits to LUT index
    uint8_t index = (uint8_t)(s_phase_acc >> 24);
    uint32_t duty = sine_lut_8bit[index];

    // Scale 8-bit LUT value to current resolution
    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6);  // scale 0-255 to 0-1023
    }

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);

    return false;  // no need to yield
}

static esp_err_t configure_ledc(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = s_resolution,
        .freq_hz = 80000000U >> s_resolution,  // 312.5kHz@8bit, 78.1kHz@10bit
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&ch_cfg);
}

static esp_err_t configure_timer(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz (1 µs per tick)
    };
    esp_err_t err = gptimer_new_timer(&cfg, &s_timer);
    if (err != ESP_OK) return err;

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000000 / PWM_SAMPLE_RATE_HZ,  // period in µs
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    err = gptimer_set_alarm_action(s_timer, &alarm_cfg);
    if (err != ESP_OK) return err;

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    return gptimer_register_event_callbacks(s_timer, &cbs, NULL);
}

esp_err_t pwm_init(void) {
    if (s_initialized) return ESP_OK;

    esp_err_t err = configure_ledc();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC config failed: %d", err);
        return err;
    }

    err = configure_timer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPTimer config failed: %d", err);
        return err;
    }

    err = gptimer_enable(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPTimer enable failed: %d", err);
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t pwm_deinit(void) {
    if (!s_initialized) return ESP_OK;

    pwm_stop();

    if (s_timer) {
        gptimer_disable(s_timer);
        gptimer_del_timer(s_timer);
        s_timer = NULL;
    }

    ledc_stop(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    s_initialized = false;
    return ESP_OK;
}

esp_err_t pwm_start_sine(uint16_t freq_hz) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    // Stop current output first
    if (s_running) {
        gptimer_stop(s_timer);
        s_running = false;
    }

    if (freq_hz == 0) {
        // DC mode: 100% duty, no timer
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, ledc_max_duty());
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        return ESP_OK;
    }

    // Clamp frequency
    if (freq_hz < 50) freq_hz = 50;
    if (freq_hz > 400) freq_hz = 400;

    // Calculate DDS phase step: step = (freq / sample_rate) * 2^32
    s_phase_acc = 0;
    s_phase_step = (uint32_t)(((uint64_t)freq_hz << 32) / PWM_SAMPLE_RATE_HZ);

    // Start timer
    esp_err_t err = gptimer_start(s_timer);
    if (err != ESP_OK) return err;

    s_running = true;
    return ESP_OK;
}

esp_err_t pwm_stop(void) {
    if (s_running) {
        gptimer_stop(s_timer);
        s_running = false;
    }

    // Set duty to 0 (no output)
    if (s_initialized) {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    }

    return ESP_OK;
}

esp_err_t pwm_set_resolution(uint8_t bits) {
    if (bits != 8 && bits != 10) return ESP_ERR_INVALID_ARG;
    if (bits == s_resolution) return ESP_OK;

    bool was_running = s_running;
    uint16_t saved_freq = 0;

    if (was_running) {
        // Save current frequency from phase_step
        saved_freq = (uint16_t)(((uint64_t)s_phase_step * PWM_SAMPLE_RATE_HZ) >> 32);
        pwm_stop();
    }

    s_resolution = bits;
    esp_err_t err = configure_ledc();
    if (err != ESP_OK) return err;

    if (was_running && saved_freq > 0) {
        return pwm_start_sine(saved_freq);
    }

    return ESP_OK;
}

bool pwm_is_running(void) {
    return s_running;
}
```

**Step 3: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: clean compile.

**Step 4: Commit**

```bash
git add modules/pz_actuator/pwm.h modules/pz_actuator/pwm.c
git commit -m "feat: add LEDC PWM + DDS analog sine wave output"
```

---

### Task 3: DRV2665 Analog Mode Support

Add `drv2665_enable_analog()` alongside the existing `drv2665_enable_digital()`.

**Files:**
- Modify: `modules/pz_actuator/drv2665.h`
- Modify: `modules/pz_actuator/drv2665.c`

**Step 1: Add declaration to `drv2665.h`**

Add after `drv2665_enable_digital` declaration (line 56):

```c
esp_err_t drv2665_enable_analog(drv2665_t *dev, uint8_t gain);
```

**Step 2: Implement in `drv2665.c`**

Add after `drv2665_enable_digital` (after line 98):

```c
esp_err_t drv2665_enable_analog(drv2665_t *dev, uint8_t gain) {
    dev->gain = gain & 0x03;

    // Exit standby
    esp_err_t err = drv2665_write_register(dev, DRV2665_REG_CTRL2,
                                            DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    // Configure analog input mode + gain
    err = drv2665_write_register(dev, DRV2665_REG_CTRL1,
                                  DRV2665_INPUT_ANALOG | dev->gain);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
```

**Step 3: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: clean compile.

**Step 4: Commit**

```bash
git add modules/pz_actuator/drv2665.h modules/pz_actuator/drv2665.c
git commit -m "feat: add DRV2665 analog input mode support"
```

---

### Task 4: Update CMake Build

Register the new source files so they compile into the user module.

**Files:**
- Modify: `modules/pz_actuator/micropython.cmake`

**Step 1: Add new source files**

Replace the `target_sources` block (lines 3-8) with:

```cmake
target_sources(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_actuator.c
    ${CMAKE_CURRENT_LIST_DIR}/drv2665.c
    ${CMAKE_CURRENT_LIST_DIR}/shift_register.c
    ${CMAKE_CURRENT_LIST_DIR}/task.c
    ${CMAKE_CURRENT_LIST_DIR}/sine.c
    ${CMAKE_CURRENT_LIST_DIR}/pwm.c
)
```

**Step 2: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: clean compile with all 6 source files.

**Step 3: Commit**

```bash
git add modules/pz_actuator/micropython.cmake
git commit -m "build: add sine.c and pwm.c to user module"
```

---

### Task 5: Refactor `set_frequency_digital()` in C

Move sine generation from Python (`main.py:make_sine`) into C. Add `set_frequency_digital(hz)` which generates the waveform internally and starts the FIFO task.

**Files:**
- Modify: `modules/pz_actuator/pz_actuator.c`
- Modify: `modules/pz_actuator/task.h` (add internal sine buffer to state)

**Step 1: Add sine buffer to task state**

In `task.h`, add a field to `pz_task_state_t` for the internally-generated buffer (after line 21):

```c
    // Internally generated sine buffer (for set_frequency_digital)
    int8_t *internal_sine_buf;
    size_t  internal_sine_len;
```

**Step 2: Add `set_frequency_digital` to `pz_actuator.c`**

Add after the `pz_actuator_set_waveform` function (after line 105):

```c
// ─── set_frequency_digital(hz) ───────────────────────────────────────────────

static mp_obj_t pz_actuator_set_frequency_digital(mp_obj_t freq_obj) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));

    int freq_hz = mp_obj_get_int(freq_obj);
    if (freq_hz < 50 || freq_hz > 4000) {
        mp_raise_ValueError(MP_ERROR_TEXT("frequency must be 50-4000 Hz"));
    }

    // Stop if running
    if (g_state.running) {
        pz_task_stop(&g_state);
    }

    // Free previous internal buffer
    if (g_state.internal_sine_buf != NULL) {
        free(g_state.internal_sine_buf);
        g_state.internal_sine_buf = NULL;
    }

    // Allocate and generate sine waveform
    size_t max_period = DRV2665_SAMPLE_RATE / freq_hz + 1;
    g_state.internal_sine_buf = (int8_t *)malloc(max_period);
    if (g_state.internal_sine_buf == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("failed to allocate sine buffer"));
    }

    g_state.internal_sine_len = sine_generate_digital(
        g_state.internal_sine_buf, max_period, freq_hz, DRV2665_SAMPLE_RATE);

    // Point waveform_buf at the internal buffer
    g_state.waveform_buf = g_state.internal_sine_buf;
    g_state.waveform_len = g_state.internal_sine_len;
    g_state.write_index = 0;

    mp_printf(&mp_plat_print, "set_frequency_digital: %dHz, %d samples/period\n",
              freq_hz, (int)g_state.waveform_len);

    // Start the FIFO task
    esp_err_t err = pz_task_start(&g_state);
    if (err != ESP_OK) mp_raise_OSError(err);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_frequency_digital_obj, pz_actuator_set_frequency_digital);
```

**Step 3: Add `#include "sine.h"` at the top of `pz_actuator.c`**

After the existing includes (after line 11):

```c
#include "sine.h"
#include "pwm.h"
```

**Step 4: Register in module globals table**

Add to `pz_actuator_module_globals_table` (before the closing `};`):

```c
    { MP_ROM_QSTR(MP_QSTR_set_frequency_digital), MP_ROM_PTR(&pz_actuator_set_frequency_digital_obj) },
```

**Step 5: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: clean compile.

**Step 6: Flash and test on device**

```bash
./scripts/flash.sh COM5
```

Then via MCP micropython exec:

```python
import pz_actuator
pz_actuator.init()
pz_actuator.set_frequency_digital(250)
# Should hear/see 250Hz vibration
pz_actuator.stop()
```

**Step 7: Commit**

```bash
git add modules/pz_actuator/pz_actuator.c modules/pz_actuator/task.h
git commit -m "feat: add set_frequency_digital() — move sine gen from Python to C"
```

---

### Task 6: Add `set_frequency_analog()` MicroPython Binding

Wire up the PWM analog path to MicroPython.

**Files:**
- Modify: `modules/pz_actuator/pz_actuator.c`

**Step 1: Add `set_frequency_analog` function**

Add after the `set_frequency_digital` function:

```c
// ─── set_frequency_analog(hz) ────────────────────────────────────────────────

static mp_obj_t pz_actuator_set_frequency_analog(mp_obj_t freq_obj) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));

    int freq_hz = mp_obj_get_int(freq_obj);

    // Stop digital path if running
    if (g_state.running) {
        pz_task_stop(&g_state);
    }

    // Switch DRV2665 to analog input mode
    esp_err_t err = drv2665_enable_analog(&g_state.drv, g_state.drv.gain);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "set_frequency_analog: DRV2665 analog mode failed: %d\n", err);
        mp_raise_OSError(err);
    }

    // Start PWM
    err = pwm_start_sine((uint16_t)freq_hz);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "set_frequency_analog: PWM start failed: %d\n", err);
        mp_raise_OSError(err);
    }

    if (freq_hz == 0) {
        mp_printf(&mp_plat_print, "set_frequency_analog: DC mode (100%% duty)\n");
    } else {
        mp_printf(&mp_plat_print, "set_frequency_analog: %dHz sine\n", freq_hz);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_frequency_analog_obj, pz_actuator_set_frequency_analog);
```

**Step 2: Add `set_pwm_resolution` function**

```c
// ─── set_pwm_resolution(bits) ────────────────────────────────────────────────

static mp_obj_t pz_actuator_set_pwm_resolution(mp_obj_t bits_obj) {
    int bits = mp_obj_get_int(bits_obj);
    if (bits != 8 && bits != 10) {
        mp_raise_ValueError(MP_ERROR_TEXT("resolution must be 8 or 10"));
    }
    esp_err_t err = pwm_set_resolution((uint8_t)bits);
    if (err != ESP_OK) mp_raise_OSError(err);
    mp_printf(&mp_plat_print, "PWM resolution: %d-bit\n", bits);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_pwm_resolution_obj, pz_actuator_set_pwm_resolution);
```

**Step 3: Update `stop()` to also stop PWM**

Modify the existing `pz_actuator_stop` function (lines 68-72):

```c
static mp_obj_t pz_actuator_stop(void) {
    pz_task_stop(&g_state);
    pwm_stop();
    return mp_const_none;
}
```

**Step 4: Add `pwm_init()` to `pz_actuator_init`**

In the `pz_actuator_init` function, after the shift register init succeeds and before `drv2665_enable_digital` (after line 38), add:

```c
    mp_printf(&mp_plat_print, "pz_actuator: initializing PWM...\n");
    err = pwm_init();
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_actuator: PWM init failed: %d (0x%03x)\n", err, err);
        shift_register_deinit(&g_state.sr);
        drv2665_deinit(&g_state.drv);
        mp_raise_OSError(err);
    }
```

**Step 5: Register new functions in module globals table**

Add to `pz_actuator_module_globals_table`:

```c
    { MP_ROM_QSTR(MP_QSTR_set_frequency_analog),  MP_ROM_PTR(&pz_actuator_set_frequency_analog_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pwm_resolution),    MP_ROM_PTR(&pz_actuator_set_pwm_resolution_obj) },
```

**Step 6: Build and verify**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: clean compile.

**Step 7: Flash and test analog path**

```bash
./scripts/flash.sh COM5
```

Then via MCP micropython exec:

```python
import pz_actuator
pz_actuator.init()

# Test analog sine at 250 Hz
pz_actuator.set_frequency_analog(250)
# Verify on scope: ~250 Hz sine at PWM_FILTERED (TP2)

# Test DC mode
pz_actuator.set_frequency_analog(0)
# Verify on scope: flat 3.3V DC

# Test resolution switch
pz_actuator.set_pwm_resolution(10)
pz_actuator.set_frequency_analog(250)
# Verify: same sine but PWM carrier now at ~78 kHz

# Stop
pz_actuator.stop()
```

**Step 8: Commit**

```bash
git add modules/pz_actuator/pz_actuator.c
git commit -m "feat: add set_frequency_analog() and set_pwm_resolution() API"
```

---

### Task 7: Update Python `main.py`

Remove the Python `make_sine()` function (now in C) and update the demo to showcase both paths.

**Files:**
- Modify: `python/main.py`

**Step 1: Replace `main.py` contents**

```python
import pz_actuator

# Reset DRV2665 to standby on boot — its state persists across ESP32 reboots
pz_actuator.reset_drv()

def demo_analog():
    """Demo: analog sine wave via PWM + RC filter."""
    import time
    pz_actuator.init()
    pz_actuator.set_gain(100)

    try:
        # Sweep frequencies
        for freq in [50, 100, 200, 250, 300, 400]:
            print(f"Analog: {freq}Hz")
            pz_actuator.set_frequency_analog(freq)
            pz_actuator.set_all(True)
            time.sleep(2)

        # DC mode
        print("Analog: DC")
        pz_actuator.set_frequency_analog(0)
        time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()

def demo_digital():
    """Demo: digital sine wave via I2C FIFO."""
    import time
    pz_actuator.init()
    pz_actuator.set_gain(100)

    try:
        for freq in [50, 100, 200, 250, 300]:
            print(f"Digital: {freq}Hz")
            pz_actuator.set_frequency_digital(freq)
            pz_actuator.set_all(True)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()

def demo():
    """Demo: cycle through pins with analog output."""
    import time
    pz_actuator.init()
    pz_actuator.set_gain(100)
    pz_actuator.set_frequency_analog(250)

    try:
        while True:
            for i in range(20):
                pz_actuator.set_all(False)
                pz_actuator.set_pin(i, True)
                time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        pz_actuator.stop()
```

**Step 2: Build, flash, and test**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`
Flash: `./scripts/flash.sh COM5`

Test via MCP: `import main; main.demo_analog()`

**Step 3: Commit**

```bash
git add python/main.py
git commit -m "refactor: update main.py to use C-based sine generation for both paths"
```

---

### Task 8: Update CLAUDE.md and Memory

Update project documentation to reflect the new API.

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Update the MicroPython API section in CLAUDE.md**

Replace the API section with the updated function list including:
- `set_frequency_analog(hz)` — Analog sine wave (50-400 Hz) or DC (0)
- `set_frequency_digital(hz)` — Digital sine wave via I2C FIFO (50-4000 Hz)
- `set_pwm_resolution(bits)` — PWM resolution (8 or 10 bits)

Add GPIO5 to the hardware pins table.

**Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update API docs for analog/digital sine wave modes"
```

---

## Task Summary

| Task | Description | New Files | Modified Files |
|------|-------------|-----------|----------------|
| 1 | Sine LUT + digital generator | `sine.h`, `sine.c` | — |
| 2 | PWM + DDS analog output | `pwm.h`, `pwm.c` | — |
| 3 | DRV2665 analog mode | — | `drv2665.h`, `drv2665.c` |
| 4 | CMake build update | — | `micropython.cmake` |
| 5 | `set_frequency_digital()` in C | — | `pz_actuator.c`, `task.h` |
| 6 | `set_frequency_analog()` + `set_pwm_resolution()` | — | `pz_actuator.c` |
| 7 | Update `main.py` | — | `main.py` |
| 8 | Update docs | — | `CLAUDE.md` |

## Dependency Order

```
Task 1 (sine LUT) ──┬── Task 4 (cmake) ── Task 5 (digital) ── Task 7 (main.py)
Task 2 (PWM+DDS) ───┤                  └── Task 6 (analog) ───┘
Task 3 (DRV2665) ───┘                                          └── Task 8 (docs)
```

Tasks 1, 2, 3 can be done in parallel. Task 4 must follow. Tasks 5 and 6 depend on Task 4. Task 7 depends on 5+6. Task 8 is last.
