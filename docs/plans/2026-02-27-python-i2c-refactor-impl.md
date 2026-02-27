# Python I2C Refactor Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Move DRV2665 configuration and shift register control from C to Python, keeping only the FIFO fill task and analog DDS in C as minimal modules.

**Architecture:** Two small C modules (`pz_fifo` for real-time FIFO filling, `pz_pwm` for 32kHz analog DDS) plus three Python files (`drv2665.py`, `shift_register.py`, `pz_actuator.py`). Python owns `machine.I2C` and `machine.SPI` buses. C extracts the I2C `bus_handle` from the MicroPython object when starting the FIFO task. ESP-IDF's built-in I2C bus lock handles concurrency.

**Tech Stack:** MicroPython v1.27.0, ESP-IDF v5.5.1 (new I2C master driver), ESP32-S3, FreeRTOS, LEDC + GPTimer for DDS.

**Design doc:** `docs/plans/2026-02-27-python-i2c-refactor-design.md`

---

## Task 1: Enable New I2C Driver in Build

**Files:**
- Modify: `modules/pz_actuator/micropython.cmake`

This is the foundation — enable `MICROPY_HW_ESP_NEW_I2C_DRIVER=1` so MicroPython's `machine.I2C` uses the same `i2c_master.h` API as our C code.

**Step 1: Add compile definition to cmake**

In `modules/pz_actuator/micropython.cmake`, add after the `target_include_directories` block:

```cmake
target_compile_definitions(usermod_pz_actuator INTERFACE
    MICROPY_HW_ESP_NEW_I2C_DRIVER=1
)
```

This overrides the `#ifndef` default in `mpconfigport.h`.

**Step 2: Build the firmware**

Run:
```bash
MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"
```
Expected: Build succeeds. The I2C driver switch is compile-time only; existing C code still works because it creates its own I2C bus via `drv2665_init()`.

**Step 3: Flash and verify**

```bash
./scripts/flash.sh COM5
```

Then on the board via MCP micropython exec:
```python
from machine import I2C, Pin
i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100000)
print(i2c.scan())
```
Expected: `[89]` (0x59 = DRV2665). This confirms the new I2C driver works.

Note: This will fail if `pz_actuator.init()` was already called (bus conflict — two drivers on same pins). Test with a fresh boot, no `pz_actuator.init()`.

**Step 4: Commit**

```bash
git add modules/pz_actuator/micropython.cmake
git commit -m "build: enable new I2C master driver for MicroPython"
```

---

## Task 2: Create `pz_pwm` C Module

**Files:**
- Create: `modules/pz_pwm/micropython.cmake`
- Create: `modules/pz_pwm/pz_pwm.c`
- Modify: `modules/micropython.cmake`

Extract the PWM/DDS code into a standalone module. This module has zero I2C dependency — it only does LEDC + GPTimer.

**Step 1: Create the cmake file**

Create `modules/pz_pwm/micropython.cmake`:

```cmake
add_library(usermod_pz_pwm INTERFACE)

target_sources(usermod_pz_pwm INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_pwm.c
)

target_include_directories(usermod_pz_pwm INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_pz_pwm)
```

**Step 2: Create the module C file**

Create `modules/pz_pwm/pz_pwm.c`. This is a self-contained file combining the sine LUT, DDS ISR, LEDC config, and MicroPython bindings.

The module needs these components from the current codebase:
- `sine_lut_8bit[256]` from `sine.c` (the 8-bit unsigned LUT, values 0-255)
- The entire GPTimer ISR + LEDC setup from `pwm.c`
- Three Python-callable functions: `set_frequency()`, `start()`, `stop()`, `is_running()`

```c
#include "py/runtime.h"
#include "py/mphal.h"

#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_err.h"

#include <math.h>

// ── Configuration ──────────────────────────────────────────────────────────
#define PWM_GPIO               5
#define PWM_SAMPLE_RATE_HZ     32000
#define LEDC_SPEED_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_CHANNEL           LEDC_CHANNEL_0

// ── 8-bit sine LUT (256 entries, unsigned 0-255) ───────────────────────────
static const uint8_t sine_lut_8bit[256] = {
    128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
    176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
    218,220,222,224,226,228,230,232,234,235,237,239,240,241,243,244,
    245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,
    255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,
    245,244,243,241,240,239,237,235,234,232,230,228,226,224,222,220,
    218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,
    176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,
    128,125,122,119,116,113,110,107,104,101, 98, 94, 91, 89, 86, 83,
     80, 77, 74, 71, 68, 66, 63, 60, 58, 55, 53, 50, 48, 45, 43, 41,
     38, 36, 34, 32, 30, 28, 26, 24, 22, 21, 19, 17, 16, 15, 13, 12,
     11, 10,  8,  7,  6,  6,  5,  4,  3,  3,  2,  2,  2,  1,  1,  1,
      1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  5,  6,  6,  7,  8, 10,
     11, 12, 13, 15, 16, 17, 19, 21, 22, 24, 26, 28, 30, 32, 34, 36,
     38, 41, 43, 45, 48, 50, 53, 55, 58, 60, 63, 66, 68, 71, 74, 77,
     80, 83, 86, 89, 91, 94, 98,101,104,107,110,113,116,119,122,125,
};

// ── DDS state (ISR-accessed, volatile) ─────────────────────────────────────
static gptimer_handle_t s_timer = NULL;
static bool s_initialized = false;
static bool s_running = false;
static uint8_t s_resolution = 8;

static volatile uint32_t s_phase_acc = 0;
static volatile uint32_t s_phase_step = 0;
static volatile uint8_t s_amplitude = 128;  // 0-128

// ── GPTimer ISR (32 kHz) ───────────────────────────────────────────────────
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_data) {
    s_phase_acc += s_phase_step;
    uint8_t index = (uint8_t)(s_phase_acc >> 24);
    int32_t raw = (int32_t)sine_lut_8bit[index] - 128;
    int32_t scaled = 128 + ((raw * (int32_t)s_amplitude) >> 7);
    uint32_t duty = (uint32_t)scaled;

    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6);
    }

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    return false;
}

// ── Internal helpers ───────────────────────────────────────────────────────
static esp_err_t pwm_init_hw(void) {
    if (s_initialized) return ESP_OK;

    // LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = (s_resolution == 10) ? LEDC_TIMER_10_BIT : LEDC_TIMER_8_BIT,
        .freq_hz = (s_resolution == 10) ? 78125 : 312500,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) return err;

    // LEDC channel
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) return err;

    // GPTimer at 32 kHz
    gptimer_config_t gpt_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz base
    };
    err = gptimer_new_timer(&gpt_conf, &s_timer);
    if (err != ESP_OK) return err;

    gptimer_alarm_config_t alarm_conf = {
        .alarm_count = 1000000 / PWM_SAMPLE_RATE_HZ,  // = 31 ticks
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    err = gptimer_set_alarm_action(s_timer, &alarm_conf);
    if (err != ESP_OK) return err;

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    err = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) return err;

    err = gptimer_enable(s_timer);
    if (err != ESP_OK) return err;

    s_initialized = true;
    return ESP_OK;
}

static esp_err_t pwm_deinit_hw(void) {
    if (!s_initialized) return ESP_OK;
    if (s_running) {
        gptimer_stop(s_timer);
        s_running = false;
    }
    gptimer_disable(s_timer);
    gptimer_del_timer(s_timer);
    s_timer = NULL;

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);

    s_initialized = false;
    return ESP_OK;
}

// ── Python API ─────────────────────────────────────────────────────────────

// pz_pwm.set_frequency(hz, resolution=8, amplitude=128)
static mp_obj_t pz_pwm_set_frequency(size_t n_args, const mp_obj_t *pos_args,
                                      mp_map_t *kw_args) {
    enum { ARG_hz, ARG_resolution, ARG_amplitude };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_hz,         MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_resolution, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8}},
        {MP_QSTR_amplitude,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int hz = args[ARG_hz].u_int;
    int resolution = args[ARG_resolution].u_int;
    int amplitude = args[ARG_amplitude].u_int;

    // Validate
    if (hz < 0 || hz > 400) {
        mp_raise_ValueError(MP_ERROR_TEXT("hz must be 0-400"));
    }
    if (resolution != 8 && resolution != 10) {
        mp_raise_ValueError(MP_ERROR_TEXT("resolution must be 8 or 10"));
    }
    if (amplitude < 0) amplitude = 0;
    if (amplitude > 128) amplitude = 128;

    // If resolution changed, deinit and reinit
    if (s_initialized && resolution != s_resolution) {
        pwm_deinit_hw();
    }
    s_resolution = (uint8_t)resolution;
    s_amplitude = (uint8_t)amplitude;

    esp_err_t err = pwm_init_hw();
    if (err != ESP_OK) {
        mp_raise_OSError(err);
    }

    // Set DDS phase step
    if (hz == 0) {
        s_phase_step = 0;  // DC mode
    } else {
        s_phase_step = (uint32_t)(((uint64_t)hz << 32) / PWM_SAMPLE_RATE_HZ);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_pwm_set_frequency_obj, 1, pz_pwm_set_frequency);

// pz_pwm.start()
static mp_obj_t pz_pwm_start(void) {
    if (!s_initialized) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("call set_frequency() first"));
    }
    if (s_running) return mp_const_none;

    // If DC mode (phase_step == 0), set full duty
    if (s_phase_step == 0) {
        uint32_t max_duty = (s_resolution == 10) ? 1023 : 255;
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, max_duty);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        s_running = true;
        return mp_const_none;
    }

    s_phase_acc = 0;
    esp_err_t err = gptimer_start(s_timer);
    if (err != ESP_OK) {
        mp_raise_OSError(err);
    }
    s_running = true;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_start_obj, pz_pwm_start);

// pz_pwm.stop()
static mp_obj_t pz_pwm_stop(void) {
    if (!s_running) return mp_const_none;
    if (s_phase_step != 0 && s_timer) {
        gptimer_stop(s_timer);
    }
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    s_running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_stop_obj, pz_pwm_stop);

// pz_pwm.is_running()
static mp_obj_t pz_pwm_is_running(void) {
    return mp_obj_new_bool(s_running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_is_running_obj, pz_pwm_is_running);

// ── Module definition ──────────────────────────────────────────────────────
static const mp_rom_map_elem_t pz_pwm_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_pz_pwm)},
    {MP_ROM_QSTR(MP_QSTR_set_frequency),  MP_ROM_PTR(&pz_pwm_set_frequency_obj)},
    {MP_ROM_QSTR(MP_QSTR_start),          MP_ROM_PTR(&pz_pwm_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop),           MP_ROM_PTR(&pz_pwm_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_is_running),     MP_ROM_PTR(&pz_pwm_is_running_obj)},
};
static MP_DEFINE_CONST_DICT(pz_pwm_globals, pz_pwm_globals_table);

const mp_obj_module_t pz_pwm_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&pz_pwm_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_pwm, pz_pwm_module);
```

**Step 3: Register in top-level cmake**

Modify `modules/micropython.cmake`:

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/pz_actuator/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/pz_pwm/micropython.cmake)
```

**Step 4: Build and verify**

Build the firmware. The `pz_pwm` module should coexist with the old `pz_actuator` module for now.

```bash
MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"
```
Expected: Build succeeds.

**Step 5: Flash and test on hardware**

Flash, then run via MCP micropython exec:
```python
import pz_pwm
pz_pwm.set_frequency(250, resolution=8, amplitude=128)
pz_pwm.start()
# Verify on oscilloscope CH4 (filtered PWM) — should see 250 Hz sine
pz_pwm.stop()
```

Note: The DRV2665 must be configured for analog mode separately (EN_OVERRIDE etc.) for the signal to reach VPP. For now just verify the PWM output on CH4.

**Step 6: Commit**

```bash
git add modules/pz_pwm/ modules/micropython.cmake
git commit -m "feat: add standalone pz_pwm C module for analog DDS"
```

---

## Task 3: Create `pz_fifo` C Module

**Files:**
- Create: `modules/pz_fifo/micropython.cmake`
- Create: `modules/pz_fifo/pz_fifo.c`
- Modify: `modules/micropython.cmake`

This module extracts the FIFO fill task. It accepts a MicroPython `machine.I2C` object, extracts the `bus_handle`, adds a DRV2665 device, and runs the background fill task.

**Step 1: Create the cmake file**

Create `modules/pz_fifo/micropython.cmake`:

```cmake
add_library(usermod_pz_fifo INTERFACE)

target_sources(usermod_pz_fifo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_fifo.c
)

target_include_directories(usermod_pz_fifo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_pz_fifo)
```

**Step 2: Create the module C file**

Create `modules/pz_fifo/pz_fifo.c`. This file needs to:
1. Include MicroPython's I2C struct definition (copy the struct, don't include the file)
2. Add a DRV2665 device on the bus from the MP I2C object
3. Run the FIFO fill task (hybrid bulk + byte-by-byte strategy from current `task.c`)

Key reference points:
- MicroPython I2C struct: `ports/esp32/machine_i2c.c` lines 51-62 (when `MICROPY_HW_ESP_NEW_I2C_DRIVER=1` and `ESP_IDF_VERSION >= 5.5.0`)
- FIFO fill loop: current `task.c` lines 58-121
- DRV2665 constants: `DRV2665_REG_DATA=0x0B`, `DRV2665_FIFO_SIZE=100`, `DRV2665_FIFO_FULL=0x01`, etc.

```c
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/obj.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "pz_fifo";

// ── DRV2665 constants ──────────────────────────────────────────────────────
#define DRV2665_ADDR        0x59
#define DRV2665_REG_STATUS  0x00
#define DRV2665_REG_CTRL2   0x02
#define DRV2665_REG_DATA    0x0B
#define DRV2665_FIFO_SIZE   100
#define DRV2665_FIFO_FULL   0x01
#define DRV2665_STANDBY     (1 << 6)
#define SAMPLE_PERIOD_US    125     // 1/8000 Hz

// ── MicroPython I2C struct (copied from ports/esp32/machine_i2c.c) ─────────
// This must match the struct layout when MICROPY_HW_ESP_NEW_I2C_DRIVER=1
// and ESP_IDF_VERSION >= 5.5.0
typedef struct {
    mp_obj_base_t base;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint8_t port;
    int8_t scl;
    int8_t sda;
    uint32_t freq;
    uint32_t timeout_us;
} machine_hw_i2c_obj_t;

// ── Task state ─────────────────────────────────────────────────────────────
typedef struct {
    i2c_master_dev_handle_t dev;
    int8_t *waveform_buf;
    size_t waveform_len;
    size_t write_index;
    TaskHandle_t task_handle;
    volatile bool running;
} fifo_state_t;

static fifo_state_t g_state = {0};
static i2c_master_bus_handle_t g_bus_handle = NULL;

// ── I2C helpers ────────────────────────────────────────────────────────────
static esp_err_t fifo_write_bulk(fifo_state_t *st, const int8_t *data, size_t len) {
    if (len > DRV2665_FIFO_SIZE) len = DRV2665_FIFO_SIZE;
    uint8_t buf[DRV2665_FIFO_SIZE + 1];
    buf[0] = DRV2665_REG_DATA;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(st->dev, buf, 1 + len, 100);
}

static esp_err_t fifo_write_byte(fifo_state_t *st, int8_t sample) {
    uint8_t buf[2] = {DRV2665_REG_DATA, (uint8_t)sample};
    return i2c_master_transmit(st->dev, buf, 2, 100);
}

static esp_err_t fifo_read_status(fifo_state_t *st, uint8_t *status) {
    uint8_t reg = DRV2665_REG_STATUS;
    return i2c_master_transmit_receive(st->dev, &reg, 1, status, 1, 100);
}

static esp_err_t fifo_standby(fifo_state_t *st) {
    uint8_t buf[2] = {DRV2665_REG_CTRL2, DRV2665_STANDBY};
    return i2c_master_transmit(st->dev, buf, 2, 100);
}

// ── Waveform helpers ───────────────────────────────────────────────────────
static int8_t next_sample(fifo_state_t *st) {
    int8_t s = st->waveform_buf[st->write_index];
    st->write_index++;
    if (st->write_index >= st->waveform_len) st->write_index = 0;
    return s;
}

static void fill_from_waveform(fifo_state_t *st, int8_t *fill_buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        fill_buf[i] = next_sample(st);
    }
}

static int fill_until_full(fifo_state_t *st) {
    int written = 0;
    while (1) {
        int8_t sample = next_sample(st);
        esp_err_t err = fifo_write_byte(st, sample);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "fifo_write_byte failed: %d", err);
            break;
        }
        written++;

        uint8_t status = 0;
        err = fifo_read_status(st, &status);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "fifo_read_status failed: %d", err);
            break;
        }
        if (status & DRV2665_FIFO_FULL) break;
    }
    return written;
}

// ── Background task ────────────────────────────────────────────────────────
static void fifo_background_task(void *arg) {
    fifo_state_t *st = (fifo_state_t *)arg;

    // Initial fill: 100 bytes
    int8_t fill_buf[DRV2665_FIFO_SIZE];
    fill_from_waveform(st, fill_buf, DRV2665_FIFO_SIZE);
    fifo_write_bulk(st, fill_buf, DRV2665_FIFO_SIZE);

    int64_t sync_time = esp_timer_get_time();
    int fifo_level = DRV2665_FIFO_SIZE;

    while (st->running) {
        // Sleep for half the estimated remaining FIFO time
        int remaining_us = fifo_level * SAMPLE_PERIOD_US;
        int sleep_us = remaining_us / 2;
        if (sleep_us < 1000) sleep_us = 1000;  // minimum 1ms
        vTaskDelay(pdMS_TO_TICKS(sleep_us / 1000));
        if (pdMS_TO_TICKS(sleep_us / 1000) == 0) {
            vTaskDelay(1);  // ensure at least 1 tick
        }

        if (!st->running) break;

        // Estimate consumed samples
        int64_t now = esp_timer_get_time();
        int elapsed_us = (int)(now - sync_time);
        int consumed = elapsed_us / SAMPLE_PERIOD_US;
        if (consumed > fifo_level) consumed = fifo_level;

        // Bulk write half the estimated room
        int bulk = consumed / 2;
        if (bulk > 0) {
            if (bulk > DRV2665_FIFO_SIZE) bulk = DRV2665_FIFO_SIZE;
            fill_from_waveform(st, fill_buf, bulk);
            fifo_write_bulk(st, fill_buf, bulk);
        }

        // Single-byte fill until FIFO full
        fill_until_full(st);

        // Sync point: FIFO is full
        sync_time = esp_timer_get_time();
        fifo_level = DRV2665_FIFO_SIZE;
    }

    // Put DRV2665 in standby
    fifo_standby(st);

    st->task_handle = NULL;
    vTaskDelete(NULL);
}

// ── Python API ─────────────────────────────────────────────────────────────

// pz_fifo.start(i2c, waveform)
static mp_obj_t pz_fifo_start(mp_obj_t i2c_obj, mp_obj_t waveform_obj) {
    if (g_state.running) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("FIFO task already running"));
    }

    // Extract bus_handle from MicroPython I2C object
    machine_hw_i2c_obj_t *i2c = (machine_hw_i2c_obj_t *)MP_OBJ_TO_PTR(i2c_obj);
    g_bus_handle = i2c->bus_handle;
    if (!g_bus_handle) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("I2C bus not initialized"));
    }

    // Get waveform buffer
    mp_buffer_info_t buf_info;
    mp_get_buffer_raise(waveform_obj, &buf_info, MP_BUFFER_READ);
    if (buf_info.len < 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("waveform must have >= 2 samples"));
    }

    // Add DRV2665 device on the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2665_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(g_bus_handle, &dev_cfg, &g_state.dev);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_OSError,
                          MP_ERROR_TEXT("failed to add I2C device: %d"), err);
    }

    // Set up state
    g_state.waveform_buf = (int8_t *)buf_info.buf;
    g_state.waveform_len = buf_info.len;
    g_state.write_index = 0;
    g_state.running = true;

    // Spawn task at high priority
    BaseType_t ret = xTaskCreate(fifo_background_task, "pz_fifo", 8192,
                                 &g_state, configMAX_PRIORITIES - 2,
                                 &g_state.task_handle);
    if (ret != pdPASS) {
        g_state.running = false;
        i2c_master_bus_rm_device(g_state.dev);
        g_state.dev = NULL;
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("failed to create task"));
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(pz_fifo_start_obj, pz_fifo_start);

// pz_fifo.stop()
static mp_obj_t pz_fifo_stop(void) {
    if (!g_state.running) return mp_const_none;

    g_state.running = false;

    // Wait for task to exit
    while (g_state.task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Remove device from bus
    if (g_state.dev) {
        i2c_master_bus_rm_device(g_state.dev);
        g_state.dev = NULL;
    }
    g_bus_handle = NULL;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_fifo_stop_obj, pz_fifo_stop);

// pz_fifo.is_running()
static mp_obj_t pz_fifo_is_running(void) {
    return mp_obj_new_bool(g_state.running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_fifo_is_running_obj, pz_fifo_is_running);

// ── Module definition ──────────────────────────────────────────────────────
static const mp_rom_map_elem_t pz_fifo_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_pz_fifo)},
    {MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&pz_fifo_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&pz_fifo_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_is_running),  MP_ROM_PTR(&pz_fifo_is_running_obj)},
};
static MP_DEFINE_CONST_DICT(pz_fifo_globals, pz_fifo_globals_table);

const mp_obj_module_t pz_fifo_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&pz_fifo_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_fifo, pz_fifo_module);
```

**Important note on struct layout:** The `machine_hw_i2c_obj_t` struct is copied from MicroPython's `ports/esp32/machine_i2c.c` (lines 51-62). The bit-field widths for `port`, `scl`, `sda` are `uint8_t port : 8`, `gpio_num_t scl : 8`, `gpio_num_t sda : 8`. We use `uint8_t`/`int8_t` with no bit-fields to match the packed size (both are 1 byte). If MicroPython changes this struct, our code will silently misread. Add a compile-time check:

```c
// Verify struct layout hasn't changed (bus_handle should be right after base)
_Static_assert(offsetof(machine_hw_i2c_obj_t, bus_handle) == sizeof(mp_obj_base_t),
               "machine_hw_i2c_obj_t layout mismatch");
```

**Step 3: Register in top-level cmake**

Modify `modules/micropython.cmake`:

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/pz_actuator/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/pz_pwm/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/pz_fifo/micropython.cmake)
```

**Step 4: Build**

```bash
MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"
```
Expected: Build succeeds. There may be a warning about the `_Static_assert` — if so, the struct layout needs adjustment.

**Step 5: Flash and test on hardware**

This test requires the DRV2665 to be configured for digital mode first. Use the existing `pz_actuator` module for setup, then test `pz_fifo` for the FIFO fill:

```python
import pz_actuator
pz_actuator.init()
pz_actuator.set_frequency_digital(250)
# Don't start via pz_actuator — we'll test pz_fifo directly

# Now test pz_fifo
from machine import I2C, Pin
import pz_fifo, math

i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100000)
# This will fail if pz_actuator already owns the bus — need to test
# without pz_actuator.init() in a separate boot
```

Actually, the old `pz_actuator` creates its own I2C bus, so we can't also create a `machine.I2C` on the same pins. We need to test `pz_fifo` after removing the old module (Task 6), or test it standalone without `pz_actuator.init()`.

**Better test approach for this task:** Just verify it builds. Full hardware testing happens in Task 7 after the Python drivers are in place.

**Step 6: Commit**

```bash
git add modules/pz_fifo/ modules/micropython.cmake
git commit -m "feat: add standalone pz_fifo C module for digital FIFO fill"
```

---

## Task 4: Create Python DRV2665 Driver

**Files:**
- Create: `python/drv2665.py`

Pure Python driver for DRV2665 register access, using `machine.I2C`.

**Step 1: Create the driver file**

Create `python/drv2665.py`:

```python
class DRV2665:
    """DRV2665 piezo driver IC — I2C register interface."""

    ADDR = 0x59

    # Registers
    REG_STATUS = 0x00
    REG_CTRL1  = 0x01
    REG_CTRL2  = 0x02
    REG_DATA   = 0x0B

    # Status bits
    FIFO_FULL  = 0x01
    FIFO_EMPTY = 0x02

    # CTRL1: input mode (bit 2) + gain (bits 1:0)
    INPUT_DIGITAL = 0x00  # bit 2 = 0
    INPUT_ANALOG  = 0x04  # bit 2 = 1
    GAIN_25  = 0x00
    GAIN_50  = 0x01
    GAIN_75  = 0x02
    GAIN_100 = 0x03

    # CTRL2 bits
    RESET          = 0x80  # bit 7
    STANDBY        = 0x40  # bit 6
    TIMEOUT_5MS    = 0x00  # bits 3:2
    TIMEOUT_10MS   = 0x04
    TIMEOUT_15MS   = 0x08
    TIMEOUT_20MS   = 0x0C
    EN_OVERRIDE    = 0x02  # bit 1

    def __init__(self, i2c):
        self.i2c = i2c
        self._buf1 = bytearray(1)
        self._buf2 = bytearray(2)
        # Verify device is present
        status = self.read_reg(self.REG_STATUS)
        if status is None:
            raise OSError("DRV2665 not found at 0x{:02X}".format(self.ADDR))

    def read_reg(self, reg):
        self._buf1[0] = reg
        self.i2c.writeto(self.ADDR, self._buf1)
        self.i2c.readfrom_into(self.ADDR, self._buf1)
        return self._buf1[0]

    def write_reg(self, reg, val):
        self._buf2[0] = reg
        self._buf2[1] = val
        self.i2c.writeto(self.ADDR, self._buf2)

    def init_digital(self, gain=GAIN_100):
        """Configure for digital FIFO mode (datasheet 8.3.1)."""
        # Exit standby, set timeout
        self.write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)
        # Set digital mode + gain
        self.write_reg(self.REG_CTRL1, self.INPUT_DIGITAL | gain)
        # Rewrite CTRL2 (per datasheet sequence)
        self.write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)

    def init_analog(self, gain=GAIN_100):
        """Configure for analog input mode (datasheet 8.3.1)."""
        # Exit standby, set timeout
        self.write_reg(self.REG_CTRL2, self.TIMEOUT_20MS)
        # Set analog mode + gain
        self.write_reg(self.REG_CTRL1, self.INPUT_ANALOG | gain)
        # Enable boost + amplifier override
        self.write_reg(self.REG_CTRL2, self.EN_OVERRIDE | self.TIMEOUT_20MS)

    def standby(self):
        """Enter standby mode."""
        self.write_reg(self.REG_CTRL2, self.STANDBY)

    def status(self):
        """Read STATUS register."""
        return self.read_reg(self.REG_STATUS)
```

**Step 2: Commit**

```bash
git add python/drv2665.py
git commit -m "feat: add Python DRV2665 I2C register driver"
```

---

## Task 5: Create Python Shift Register Driver

**Files:**
- Create: `python/shift_register.py`

Pure Python driver for the HV509 dual daisy-chained shift registers via `machine.SPI`.

**Step 1: Create the driver file**

Create `python/shift_register.py`:

```python
from machine import Pin


class ShiftRegister:
    """HV509 dual daisy-chained shift register — SPI interface.

    32-bit word layout:
      bits [31:26] = unused (always 0)
      bits [25:6]  = pins 0-19 (pin N → bit 25-N)
      bits [5:0]   = unused (always 0)
    """

    NUM_PINS = 20
    _COMMON_MASK = 0xFC00003F  # bits that must stay 0
    _ALL_PINS_MASK = 0x03FFFFC0  # all 20 pin bits

    def __init__(self, spi, cs_pin, pol_a_pin=12, pol_b_pin=13):
        self.spi = spi
        self.cs = cs_pin
        self._state = 0x00000000
        self._tx_buf = bytearray(4)

        # Polarity pins (active-low: LOW = inverted mode = normal operation)
        self._pol_a = Pin(pol_a_pin, Pin.OUT, value=0)
        self._pol_b = Pin(pol_b_pin, Pin.OUT, value=0)
        self._polarity = False  # False = low (inverted/normal)

        # Commit initial state (all off)
        self.flush()

    def _pin_bit(self, pin):
        return 1 << (25 - pin)

    def set_pin(self, pin, value):
        if pin < 0 or pin >= self.NUM_PINS:
            raise ValueError("pin must be 0-19")
        bit = self._pin_bit(pin)
        if value:
            self._state |= bit
        else:
            self._state &= ~bit

    def get_pin(self, pin):
        if pin < 0 or pin >= self.NUM_PINS:
            raise ValueError("pin must be 0-19")
        return bool(self._state & self._pin_bit(pin))

    def set_all(self, value):
        if value:
            self._state = self._ALL_PINS_MASK
        else:
            self._state = 0

    def get_all(self):
        return tuple(self.get_pin(i) for i in range(self.NUM_PINS))

    def set_pins(self, values):
        if len(values) != self.NUM_PINS:
            raise ValueError("expected 20 values")
        self._state = 0
        for i, v in enumerate(values):
            if v:
                self._state |= self._pin_bit(i)

    def flush(self):
        """Commit pending state to shift registers via SPI."""
        state = self._state & ~self._COMMON_MASK
        self._tx_buf[0] = (state >> 24) & 0xFF
        self._tx_buf[1] = (state >> 16) & 0xFF
        self._tx_buf[2] = (state >> 8) & 0xFF
        self._tx_buf[3] = state & 0xFF
        self.cs.value(0)
        self.spi.write(self._tx_buf)
        self.cs.value(1)

    def toggle_polarity(self):
        """Toggle HV509 polarity pins."""
        self._polarity = not self._polarity
        val = 1 if self._polarity else 0
        self._pol_a.value(val)
        self._pol_b.value(val)

    def get_polarity(self):
        return self._polarity
```

**Important SPI note:** MicroPython's `machine.SPI` does NOT manage CS automatically for `spi.write()`. We manually toggle `cs` low before write and high after. The HV509 latches data on CS rising edge.

**Step 2: Commit**

```bash
git add python/shift_register.py
git commit -m "feat: add Python shift register (HV509) SPI driver"
```

---

## Task 6: Create Python PzActuator Orchestrator

**Files:**
- Create: `python/pz_actuator_py.py` (named to avoid collision with C module during transition)
- Modify: `python/manifest.py`

This is the high-level API that ties everything together.

**Step 1: Create the orchestrator**

Create `python/pz_actuator_py.py`:

```python
import math
import pz_fifo
import pz_pwm
from machine import I2C, SPI, Pin
from drv2665 import DRV2665
from shift_register import ShiftRegister


MODE_DIGITAL = 'digital'
MODE_ANALOG = 'analog'


class PzActuator:
    """High-level piezo actuator controller.

    Wraps DRV2665 (I2C), HV509 shift registers (SPI), and
    real-time C modules (pz_fifo, pz_pwm).
    """

    GAINS = {
        25: DRV2665.GAIN_25,
        50: DRV2665.GAIN_50,
        75: DRV2665.GAIN_75,
        100: DRV2665.GAIN_100,
    }

    def __init__(self):
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

    def set_frequency_digital(self, hz):
        """Configure digital FIFO mode at given frequency.

        Generates one period of signed sine samples for 8 kHz playback.
        """
        if hz < 1 or hz > 4000:
            raise ValueError("hz must be 1-4000")
        n_samples = round(8000 / hz)
        if n_samples < 2:
            n_samples = 2
        # Generate sine: trough at index 0 (phase offset -pi/2)
        self._waveform = bytearray(
            (int(127 * math.sin(-math.pi / 2 + 2 * math.pi * i / n_samples)) & 0xFF)
            for i in range(n_samples)
        )
        self._mode = MODE_DIGITAL

    def set_frequency_analog(self, hz, resolution=8, amplitude=100):
        """Configure analog PWM+DDS mode at given frequency.

        Args:
            hz: 0-400 (0 = DC output)
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage, mapped to internal 0-128)
        """
        if hz < 0 or hz > 400:
            raise ValueError("hz must be 0-400")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_pwm.set_frequency(hz, resolution=resolution, amplitude=amp_internal)
        self._mode = MODE_ANALOG

    def start(self, gain=100):
        """Start output in the configured mode."""
        if self._mode is None:
            raise RuntimeError("call set_frequency_digital() or set_frequency_analog() first")
        if gain not in self.GAINS:
            raise ValueError("gain must be 25, 50, 75, or 100")
        gain_bits = self.GAINS[gain]

        if self._mode == MODE_DIGITAL:
            self.drv.init_digital(gain_bits)
            pz_fifo.start(self.i2c, self._waveform)
        elif self._mode == MODE_ANALOG:
            self.drv.init_analog(gain_bits)
            pz_pwm.start()

    def stop(self):
        """Stop output and put DRV2665 in standby."""
        if pz_fifo.is_running():
            pz_fifo.stop()
        if pz_pwm.is_running():
            pz_pwm.stop()
        self.drv.standby()

    def is_running(self):
        return pz_fifo.is_running() or pz_pwm.is_running()

    # ── Shift register pass-through ─────────────────────────────────────
    def set_pin(self, pin, value, flush=True):
        self.sr.set_pin(pin, value)
        if flush:
            self.sr.flush()

    def get_pin(self, pin):
        return self.sr.get_pin(pin)

    def set_pins(self, values, flush=True):
        self.sr.set_pins(values)
        if flush:
            self.sr.flush()

    def get_all(self):
        return self.sr.get_all()

    def set_all(self, value, flush=True):
        self.sr.set_all(value)
        if flush:
            self.sr.flush()

    def flush(self):
        self.sr.flush()

    def toggle_polarity(self):
        self.sr.toggle_polarity()

    def get_polarity(self):
        return self.sr.get_polarity()
```

**Step 2: Update manifest to freeze new Python files**

Modify `python/manifest.py`:

```python
include("$(PORT_DIR)/boards/manifest.py")
freeze(".")
```

This already freezes all `.py` files in the `python/` directory. The new files (`drv2665.py`, `shift_register.py`, `pz_actuator_py.py`) will be picked up automatically. No change needed unless `freeze(".")` doesn't include subdirectories (it doesn't — all files are in `python/` root).

Verify the manifest still works — no changes needed.

**Step 3: Commit**

```bash
git add python/pz_actuator_py.py
git commit -m "feat: add Python PzActuator orchestrator"
```

---

## Task 7: Integration Test — All New Modules Together

**Files:** None (testing only)

Test the new Python + C modules end-to-end on hardware, with the old `pz_actuator` C module still present but unused.

**Step 1: Build and flash**

```bash
MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"
./scripts/flash.sh COM5
```

**Step 2: Test Python I2C + DRV2665 driver**

Via MCP micropython exec:
```python
from machine import I2C, Pin
i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100000)
print("I2C scan:", i2c.scan())

from drv2665 import DRV2665
drv = DRV2665(i2c)
print("STATUS:", hex(drv.status()))
print("CTRL1:", hex(drv.read_reg(0x01)))
print("CTRL2:", hex(drv.read_reg(0x02)))
```
Expected: scan finds `[89]`, register reads return valid values.

**Step 3: Test shift register via Python SPI**

```python
from machine import SPI, Pin
from shift_register import ShiftRegister

spi = SPI(1, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(9), mosi=Pin(6), miso=Pin(7))
sr = ShiftRegister(spi, Pin(10, Pin.OUT))

sr.set_all(True)
sr.flush()
# Verify on oscilloscope: all pins should be high

sr.set_all(False)
sr.flush()
# Verify: all pins low

sr.set_pin(4, True)
sr.flush()
# Verify on scope CH2 (connected to pin 4)
```

**Step 4: Test analog mode (pz_pwm + DRV2665 Python config)**

```python
from machine import I2C, Pin
from drv2665 import DRV2665
import pz_pwm

i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100000)
drv = DRV2665(i2c)

# Configure analog mode
drv.init_analog(DRV2665.GAIN_100)

# Start PWM DDS at 250 Hz
pz_pwm.set_frequency(250, resolution=8, amplitude=128)
pz_pwm.start()

# Check oscilloscope CH4 (filtered PWM) and CH1 (VPP)
# Should see 250 Hz sine on both

pz_pwm.stop()
drv.standby()
```

**Step 5: Test digital mode (pz_fifo + DRV2665 Python config)**

```python
from machine import I2C, Pin
from drv2665 import DRV2665
import pz_fifo, math

i2c = I2C(0, sda=Pin(47), scl=Pin(21), freq=100000)
drv = DRV2665(i2c)

# Configure digital mode
drv.init_digital(DRV2665.GAIN_100)

# Generate 250 Hz waveform (32 samples at 8 kHz)
n = round(8000 / 250)
waveform = bytearray(
    (int(127 * math.sin(-math.pi/2 + 2*math.pi*i/n)) & 0xFF)
    for i in range(n)
)

# Start FIFO task
pz_fifo.start(i2c, waveform)
print("Running:", pz_fifo.is_running())

# Check oscilloscope CH1 (VPP) — should see 250 Hz sine

# Test bus sharing: read registers while FIFO task is running
print("STATUS during playback:", hex(drv.status()))
print("CTRL1 during playback:", hex(drv.read_reg(0x01)))

pz_fifo.stop()
drv.standby()
```

**Step 6: Test full PzActuator orchestrator**

```python
from pz_actuator_py import PzActuator

pa = PzActuator()
pa.set_all(True)

# Digital mode
pa.set_frequency_digital(250)
pa.start(gain=100)
# Verify on scope
pa.stop()

# Analog mode
pa.set_frequency_analog(250, amplitude=100)
pa.start(gain=100)
# Verify on scope
pa.stop()

pa.set_all(False)
```

**Step 7: Commit test results**

No code to commit — this is verification only.

---

## Task 8: Remove Old `pz_actuator` C Module

**Files:**
- Delete: `modules/pz_actuator/` (entire directory)
- Modify: `modules/micropython.cmake` — remove `pz_actuator` include
- Modify: `modules/pz_pwm/micropython.cmake` — move the `MICROPY_HW_ESP_NEW_I2C_DRIVER=1` define here (it was on `pz_actuator`)
- Rename: `python/pz_actuator_py.py` → `python/pz_actuator.py`
- Modify: `python/main.py` — update imports to use new Python module

**Step 1: Move the I2C driver define**

The `MICROPY_HW_ESP_NEW_I2C_DRIVER=1` compile definition was added to `modules/pz_actuator/micropython.cmake` in Task 1. Since we're deleting that module, move it to `modules/pz_fifo/micropython.cmake` (it's the module that needs it):

Add to `modules/pz_fifo/micropython.cmake`:
```cmake
target_compile_definitions(usermod_pz_fifo INTERFACE
    MICROPY_HW_ESP_NEW_I2C_DRIVER=1
)
```

**Step 2: Remove old module**

Delete the entire `modules/pz_actuator/` directory.

**Step 3: Update top-level cmake**

Modify `modules/micropython.cmake` to:
```cmake
include(${CMAKE_CURRENT_LIST_DIR}/pz_pwm/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/pz_fifo/micropython.cmake)
```

**Step 4: Rename Python orchestrator**

Rename `python/pz_actuator_py.py` to `python/pz_actuator.py`. This means `import pz_actuator` now gets the Python version instead of the old C module.

**Step 5: Update main.py**

Update `python/main.py` to import from the new Python module. The API is the same at the `PzActuator` class level, but instantiation changes:

Old:
```python
import pz_actuator
pz_actuator.init()
pz_actuator.set_frequency_analog(250)
pz_actuator.start()
```

New:
```python
from pz_actuator import PzActuator
pa = PzActuator()
pa.set_frequency_analog(250)
pa.start()
```

Update `main.py` to use the new class-based API. The demos (`demo_analog`, `demo_digital`, `demo`) need to be rewritten to use `pa.set_frequency_digital()`, `pa.start()`, `pa.stop()`, `pa.set_pin()`, etc.

**Step 6: Build and verify**

```bash
MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"
```
Expected: Build succeeds without the old C module.

**Step 7: Flash and test**

```bash
./scripts/flash.sh COM5
```

Run the same tests from Task 7, but now using `from pz_actuator import PzActuator` directly.

**Step 8: Commit**

```bash
git add -A
git commit -m "refactor: remove old pz_actuator C module, use Python drivers

Replace monolithic C module with:
- pz_fifo: C module for real-time FIFO fill task
- pz_pwm: C module for 32kHz analog DDS
- drv2665.py: Python DRV2665 register driver
- shift_register.py: Python HV509 SPI driver
- pz_actuator.py: Python orchestrator"
```

---

## Task 9: Add `enter_bootloader()` to a C Module

**Files:**
- Modify: `modules/pz_fifo/pz_fifo.c` (or create a small utility module)

The `enter_bootloader()` function from the old module is needed for flashing via MCP. It does USB disconnect + RTC boot flag + restart. This can't be done from Python.

**Step 1: Add to pz_fifo module**

Add to `modules/pz_fifo/pz_fifo.c`:

```c
#include "esp_private/usb_phy.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"

// enter_bootloader() — switch to USB download mode for flashing
static mp_obj_t pz_fifo_enter_bootloader(void) {
    // Disconnect USB-CDC so host detects re-enumeration
    usb_phy_handle_t phy;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
    };
    if (usb_new_phy(&phy_conf, &phy) == ESP_OK) {
        usb_phy_action(phy, USB_PHY_ACTION_HOST_FORCE_DISCONN);
    }
    mp_hal_delay_ms(100);

    // Set boot mode to download and restart
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();

    return mp_const_none;  // unreachable
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_fifo_enter_bootloader_obj, pz_fifo_enter_bootloader);
```

Add to the module globals table:
```c
{MP_ROM_QSTR(MP_QSTR_enter_bootloader), MP_ROM_PTR(&pz_fifo_enter_bootloader_obj)},
```

**Step 2: Build and verify**

Build, flash, test `pz_fifo.enter_bootloader()` from REPL.

**Step 3: Commit**

```bash
git add modules/pz_fifo/pz_fifo.c
git commit -m "feat: add enter_bootloader() to pz_fifo module"
```

---

## Task 10: Final Cleanup and Verification

**Files:**
- Possibly modify any remaining references to old module

**Step 1: Search for stale references**

Search the codebase for any remaining references to the old `pz_actuator` C module:

```bash
grep -r "pz_actuator" --include="*.py" --include="*.c" --include="*.h" --include="*.cmake" --include="*.md"
```

Update any stale references in docs, CLAUDE.md, scripts, etc.

**Step 2: Verify MCP micropython tools still work**

The MCP `mcp_micropython.py` server should work unchanged since it just executes Python on the board. But verify:
- `mcp__micropython__exec` works
- `mcp__micropython__enter_bootloader` works (now calls `pz_fifo.enter_bootloader()` instead of `pz_actuator.enter_bootloader()`)

If `mcp_micropython.py` has hardcoded references to `pz_actuator.enter_bootloader()`, update them.

**Step 3: Update CLAUDE.md**

Update the API documentation in `CLAUDE.md` to reflect the new Python-based API:

```markdown
### MicroPython API

#### Python modules (frozen into firmware)
- `pz_actuator.PzActuator` — High-level orchestrator
- `drv2665.DRV2665` — DRV2665 I2C register driver
- `shift_register.ShiftRegister` — HV509 SPI shift register driver

#### C modules (real-time only)
- `pz_fifo` — Digital FIFO fill background task
- `pz_pwm` — Analog 32kHz DDS + LEDC PWM
```

**Step 4: Full regression test**

Run through all modes on hardware:
1. Digital mode: multiple frequencies (100, 250, 500 Hz)
2. Analog mode: multiple frequencies (100, 250 Hz) + DC
3. Shift register: individual pins, all on/off, polarity toggle
4. Bus sharing: register reads during digital playback
5. Stop/restart cycles

**Step 5: Final commit**

```bash
git add -A
git commit -m "docs: update API documentation for Python I2C refactor"
```
