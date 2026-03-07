# Waveform Task Redesign — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace C-side waveform generation with Python-provided buffers. Rewrite the FreeRTOS task loop to verify FIFO status, synchronize SR flush to waveform trough with half-buffer cap, and use adaptive sleep.

**Architecture:** Python generates a one-period waveform as a `bytearray` and passes it to C via `set_waveform(buf)`. The C task copies circularly from this buffer into the DRV2665 FIFO, verifies FIFO_FULL, predicts trough timing from write position, and flushes the HV509 shift register at the trough (or skips if too far away). Sleep time adapts to actual bytes written.

**Tech Stack:** ESP-IDF (FreeRTOS, SPI, GPIO), MicroPython C modules, soft I2C

---

### Task 1: Update `task.h` — replace waveform fields with buffer pointer

**Files:**
- Modify: `modules/pz_actuator/task.h`

**Step 1: Edit task.h**

Replace the `waveform.h` include and `waveform_t` member with buffer pointer fields. Remove frequency-change fields.

New `task.h`:
```c
#ifndef PZ_TASK_H
#define PZ_TASK_H

#include "drv2665.h"
#include "shift_register.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    drv2665_t drv;
    shift_register_t sr;
    TaskHandle_t task_handle;
    bool running;
    bool sync_trough;           // if true, wait for waveform trough before SR commit

    // Python-provided waveform buffer (one period, trough at index 0)
    int8_t *waveform_buf;       // pointer into Python bytearray data
    size_t  waveform_len;       // period length in samples
    size_t  write_index;        // circular position in waveform
} pz_task_state_t;

// Start the background task.
esp_err_t pz_task_start(pz_task_state_t *state);

// Stop the background task. Puts DRV2665 in standby.
esp_err_t pz_task_stop(pz_task_state_t *state);

#endif // PZ_TASK_H
```

**Step 2: Build to check for compile errors**

Run: `MSYS_NO_PATHCONV=1 docker compose run --rm dev bash /workspace/scripts/build.sh`

Expected: Compile errors in `task.c` and `pz_actuator.c` referencing removed fields (`waveform`, `target_frequency`, `frequency_changed`). This is expected — we fix them in the next tasks.

---

### Task 2: Rewrite `task.c` — new task loop

**Files:**
- Modify: `modules/pz_actuator/task.c`

**Step 1: Write the new task.c**

The new task loop:
1. FILL — copy from `waveform_buf[write_index..]` circularly, write to DRV2665 FIFO
2. VERIFY — read FIFO_FULL status register
3. TROUGH — if SR pending: predict trough, hybrid wait or skip
4. SLEEP — adaptive, half of bytes_written time

New `task.c`:
```c
#include "task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "py/mpprint.h"
#include <string.h>

static const char *TAG = "pz_task";

#define FIFO_FILL_CHUNK 100  // try to fill entire FIFO each iteration
#define DEBUG_EVERY 500      // print debug every N iterations

static void pz_background_task(void *arg) {
    pz_task_state_t *state = (pz_task_state_t *)arg;
    int8_t fill_buf[FIFO_FILL_CHUNK];
    uint32_t iter = 0;

    mp_printf(&mp_plat_print, "pz_task: started (waveform_len=%d, freq=%dHz)\n",
              (int)state->waveform_len,
              (int)(8000 / state->waveform_len));

    while (state->running) {
        bool debug = (iter % DEBUG_EVERY == 0);

        // ── Step 1: FILL ─────────────────────────────────────────────
        // Record write_index before fill (needed for trough calculation)
        size_t write_index_before = state->write_index;

        // Copy from waveform_buf into fill_buf, wrapping circularly
        for (int i = 0; i < FIFO_FILL_CHUNK; i++) {
            fill_buf[i] = state->waveform_buf[state->write_index];
            state->write_index = (state->write_index + 1) % state->waveform_len;
        }

        // Write to DRV2665 FIFO
        int bytes_written = drv2665_write_fifo(&state->drv, fill_buf, FIFO_FILL_CHUNK);
        int64_t fill_timestamp = esp_timer_get_time();
        if (bytes_written < 0) bytes_written = 0;

        // Only advance write_index by what was actually accepted
        if (bytes_written < FIFO_FILL_CHUNK) {
            // Rewind write_index: we advanced by FIFO_FILL_CHUNK but only bytes_written were accepted
            state->write_index = (write_index_before + bytes_written) % state->waveform_len;
        }

        if (debug) {
            mp_printf(&mp_plat_print, "pz_task[%d]: fifo_write=%d/%d write_idx=%d\n",
                      (int)iter, bytes_written, FIFO_FILL_CHUNK,
                      (int)state->write_index);
        }

        // ── Step 2: VERIFY ───────────────────────────────────────────
        uint8_t status;
        esp_err_t err = drv2665_read_status(&state->drv, &status);
        if (err == ESP_OK && debug) {
            mp_printf(&mp_plat_print, "pz_task[%d]: status=0x%02x FIFO_FULL=%d FIFO_EMPTY=%d\n",
                      (int)iter, status,
                      (status & DRV2665_FIFO_FULL) ? 1 : 0,
                      (status & DRV2665_FIFO_EMPTY) ? 1 : 0);
        }

        // ── Step 3: TROUGH ───────────────────────────────────────────
        if ((state->sr.pending_commit || state->sr.pending_polarity) && state->sync_trough) {
            // Approximate playback head: where we started writing this iteration.
            // The FIFO was nearly empty when we woke (we sleep until half-drained).
            size_t playback_idx = write_index_before % state->waveform_len;

            // Samples from playback head to next trough (index 0)
            size_t samples_to_trough = (state->waveform_len - playback_idx) % state->waveform_len;

            // Convert to microseconds (125us per sample at 8kHz)
            int64_t wait_us = (int64_t)samples_to_trough * 125;

            // Half-buffer cap: don't wait longer than half the time the FIFO content will last
            int64_t max_wait_us = ((int64_t)bytes_written * 125) / 2;

            if (samples_to_trough == 0) {
                // We're at the trough right now — commit immediately
                shift_register_commit(&state->sr);
            } else if (wait_us <= max_wait_us) {
                // Hybrid wait: FreeRTOS sleep for bulk, busy-wait for precision
                int64_t target_time = fill_timestamp + wait_us;

                if (wait_us > 2000) {
                    TickType_t ticks = pdMS_TO_TICKS((wait_us / 1000) - 1);
                    if (ticks > 0) vTaskDelay(ticks);
                }

                // Busy-wait for precise timing
                while (esp_timer_get_time() < target_time) {
                    // spin
                }

                shift_register_commit(&state->sr);
            }
            // else: trough too far away, skip — catch it next iteration
        } else if (state->sr.pending_commit || state->sr.pending_polarity) {
            // sync_trough disabled: commit immediately
            shift_register_commit(&state->sr);
        }

        // ── Step 4: SLEEP ────────────────────────────────────────────
        if (bytes_written == 0) {
            // FIFO was already full, retry quickly
            vTaskDelay(1);
        } else {
            // Sleep until FIFO is roughly half-drained
            int64_t sleep_us = ((int64_t)bytes_written * 125) / 2;
            TickType_t ticks = pdMS_TO_TICKS(sleep_us / 1000);
            vTaskDelay(ticks > 0 ? ticks : 1);
        }

        iter++;
    }

    mp_printf(&mp_plat_print, "pz_task: stopping, entering standby\n");
    drv2665_standby(&state->drv);
    state->task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t pz_task_start(pz_task_state_t *state) {
    if (state->running) {
        return ESP_OK;
    }

    state->running = true;
    state->write_index = 0;  // reset playback position

    mp_printf(&mp_plat_print, "pz_task: creating task (priority=%d)\n", configMAX_PRIORITIES - 2);

    BaseType_t ret = xTaskCreate(
        pz_background_task,
        "pz_task",
        8192,
        state,
        configMAX_PRIORITIES - 2,
        &state->task_handle
    );

    if (ret != pdPASS) {
        state->running = false;
        mp_printf(&mp_plat_print, "pz_task: xTaskCreate FAILED\n");
        return ESP_FAIL;
    }

    mp_printf(&mp_plat_print, "pz_task: task created OK\n");
    return ESP_OK;
}

esp_err_t pz_task_stop(pz_task_state_t *state) {
    if (!state->running) {
        return ESP_OK;
    }

    mp_printf(&mp_plat_print, "pz_task: requesting stop...\n");
    state->running = false;

    // Wait for task to exit
    while (state->task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    mp_printf(&mp_plat_print, "pz_task: stopped\n");
    return ESP_OK;
}
```

**Step 2: Build**

Run: `MSYS_NO_PATHCONV=1 docker compose run --rm dev bash /workspace/scripts/build.sh`

Expected: Compile errors in `pz_actuator.c` referencing `waveform_init`, `waveform_set_frequency`, `g_state.waveform.frequency`, `g_state.target_frequency`, `g_state.frequency_changed`. This is expected — fixed in Task 3.

---

### Task 3: Update `pz_actuator.c` — add `set_waveform()`, remove frequency functions

**Files:**
- Modify: `modules/pz_actuator/pz_actuator.c`

**Step 1: Add `set_waveform(buf)` and remove `set_frequency` / `get_frequency`**

Changes to `pz_actuator.c`:

1. Remove the `waveform_init()` call from `pz_actuator_init()`. The init function should no longer set up a default waveform — the user must call `set_waveform()` before `start()`.

2. Add a `set_waveform(buf)` function that takes a `bytearray`:
```c
// ─── set_waveform(buf) ──────────────────────────────────────────────────

static mp_obj_t pz_actuator_set_waveform(mp_obj_t buf_obj) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_obj, &bufinfo, MP_BUFFER_READ);

    if (bufinfo.len < 2 || bufinfo.len > 160) {
        mp_raise_ValueError(MP_ERROR_TEXT("waveform must be 2-160 samples"));
    }

    // Stop task if running (buffer pointer is about to change)
    if (g_state.running) {
        pz_task_stop(&g_state);
    }

    g_state.waveform_buf = (int8_t *)bufinfo.buf;
    g_state.waveform_len = bufinfo.len;
    g_state.write_index = 0;

    mp_printf(&mp_plat_print, "set_waveform: %d samples (%dHz)\n",
              (int)bufinfo.len, (int)(8000 / bufinfo.len));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_waveform_obj, pz_actuator_set_waveform);
```

3. Remove the `pz_actuator_set_frequency` and `pz_actuator_get_frequency` functions entirely.

4. Update `pz_actuator_start()` to check that `waveform_buf` is set:
```c
static mp_obj_t pz_actuator_start(void) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));
    if (g_state.waveform_buf == NULL) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("call set_waveform() first"));
    esp_err_t err = pz_task_start(&g_state);
    if (err != ESP_OK) mp_raise_OSError(err);
    return mp_const_none;
}
```

5. Update `pz_actuator_init()` — remove the `waveform_init()` call and `sync_trough` default (keep it false by default via the zero-init of `g_state`):
```c
// In pz_actuator_init, remove these two lines:
//     waveform_init(&g_state.waveform, 250);
//     g_state.sync_trough = false;
```

6. Update the module globals table:
   - Remove `set_frequency` and `get_frequency` entries
   - Add `set_waveform` entry

**Step 2: Build**

Run: `MSYS_NO_PATHCONV=1 docker compose run --rm dev bash /workspace/scripts/build.sh`

Expected: Linker errors for `waveform_init`, `waveform_set_frequency`, `waveform_fill_buffer`, `waveform_samples_until_trough` — all from `waveform.c` which is still in cmake but no longer called. Should compile clean since we removed all references.

Actually — at this point no code calls anything from `waveform.c`/`waveform.h`, so the build should succeed. The linker won't complain about unused symbols in a static library. But `waveform.c` still compiles (just unused). We clean it up in Task 4.

---

### Task 4: Remove `waveform.c` / `waveform.h`, update cmake

**Files:**
- Delete: `modules/pz_actuator/waveform.c`
- Delete: `modules/pz_actuator/waveform.h`
- Modify: `modules/pz_actuator/micropython.cmake` — remove `waveform.c` from `target_sources`

**Step 1: Delete waveform files**

Remove `waveform.c` and `waveform.h`.

**Step 2: Update micropython.cmake**

Remove the `${CMAKE_CURRENT_LIST_DIR}/waveform.c` line from `target_sources`:

```cmake
add_library(usermod_pz_actuator INTERFACE)

target_sources(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_actuator.c
    ${CMAKE_CURRENT_LIST_DIR}/drv2665.c
    ${CMAKE_CURRENT_LIST_DIR}/soft_i2c.c
    ${CMAKE_CURRENT_LIST_DIR}/shift_register.c
    ${CMAKE_CURRENT_LIST_DIR}/task.c
)

target_include_directories(usermod_pz_actuator INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_pz_actuator)
```

**Step 3: Build**

Run: `MSYS_NO_PATHCONV=1 docker compose run --rm dev bash /workspace/scripts/build.sh`

Expected: Clean build, no errors.

**Step 4: Commit**

```bash
git add modules/pz_actuator/task.h modules/pz_actuator/task.c modules/pz_actuator/pz_actuator.c modules/pz_actuator/micropython.cmake
git rm modules/pz_actuator/waveform.c modules/pz_actuator/waveform.h
git commit -m "feat: python-provided waveform buffer, adaptive FIFO fill, trough-synced SR flush"
```

---

### Task 5: Flash and test with Python 250Hz sinewave

**Step 1: Flash**

```bash
python -m esptool --chip esp32s3 -p COM7 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m 0x0 build/bootloader.bin 0x8000 build/partition-table.bin 0x10000 build/micropython.bin
```

(May need `enter_bootloader()` first if board is running MicroPython, then flash with `--before no_reset` on COM4.)

**Step 2: Test basic init + waveform + start**

Via MicroPython MCP `exec`:
```python
import math
import pz_actuator

# Generate 250Hz sine (32 samples at 8kHz)
# Phase starts at -pi/2 so index 0 is the trough
SAMPLE_RATE = 8000
freq = 250
period = SAMPLE_RATE // freq  # 32

buf = bytearray(
    int(math.sin(-math.pi/2 + 2*math.pi*i/period) * 127) & 0xFF
    for i in range(period)
)

pz_actuator.init()
pz_actuator.set_waveform(buf)
pz_actuator.start()
```

Expected output:
- `set_waveform: 32 samples (250Hz)`
- `pz_task: started (waveform_len=32, freq=250Hz)`
- Periodic debug lines showing `fifo_write=100/100` and `status=0x01` (FIFO_FULL)

**Step 3: Test trough sync**

```python
pz_actuator.set_sync_trough(True)
pz_actuator.set_pin(0, True)
```

Expected: SR commit happens at trough, debug output shows trough wait timing.

**Step 4: Stop and verify cleanup**

```python
pz_actuator.stop()
pz_actuator.is_running()  # should return False
```

**Step 5: Test error case — start without waveform**

```python
import machine
machine.soft_reset()

import pz_actuator
pz_actuator.init()
pz_actuator.start()  # should raise RuntimeError: "call set_waveform() first"
```

**Step 6: Test different frequencies**

```python
import math
import pz_actuator

SAMPLE_RATE = 8000

def make_waveform(freq):
    period = SAMPLE_RATE // freq
    return bytearray(
        int(math.sin(-math.pi/2 + 2*math.pi*i/period) * 127) & 0xFF
        for i in range(period)
    )

pz_actuator.init()

# 100Hz (80 samples)
pz_actuator.set_waveform(make_waveform(100))
pz_actuator.start()
# observe output...
pz_actuator.stop()

# 500Hz (16 samples)
pz_actuator.set_waveform(make_waveform(500))
pz_actuator.start()
# observe output...
pz_actuator.stop()
```
