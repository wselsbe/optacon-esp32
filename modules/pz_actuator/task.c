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

        // Write to DRV2665 FIFO (timestamp before write — I2C takes ~8ms)
        int64_t fill_timestamp = esp_timer_get_time();
        int bytes_written = drv2665_write_fifo(&state->drv, fill_buf, FIFO_FILL_CHUNK);
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
        if (err == ESP_OK) {
            if (debug) {
                mp_printf(&mp_plat_print, "pz_task[%d]: status=0x%02x FIFO_FULL=%d FIFO_EMPTY=%d\n",
                          (int)iter, status,
                          (status & DRV2665_FIFO_FULL) ? 1 : 0,
                          (status & DRV2665_FIFO_EMPTY) ? 1 : 0);
            }
            // Underrun detection: FIFO empty after we just wrote data
            if ((status & DRV2665_FIFO_EMPTY) && bytes_written > 0) {
                mp_printf(&mp_plat_print, "pz_task[%d]: WARNING FIFO underrun\n", (int)iter);
            }
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
