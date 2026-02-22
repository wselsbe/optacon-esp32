#include "task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "py/mpprint.h"
#include <string.h>

static const char *TAG = "pz_task";

#define FIFO_FILL_CHUNK 100  // try to fill entire FIFO
#define DEBUG_EVERY 500      // print debug every N iterations

static void pz_background_task(void *arg) {
    pz_task_state_t *state = (pz_task_state_t *)arg;
    int8_t fill_buf[FIFO_FILL_CHUNK];
    uint32_t iter = 0;

    mp_printf(&mp_plat_print, "pz_task: started (freq=%dHz, period=%d samples)\n",
              state->waveform.frequency, state->waveform.period_len);

    while (state->running) {
        bool debug = (iter % DEBUG_EVERY == 0);

        // Check for frequency change
        if (state->frequency_changed) {
            mp_printf(&mp_plat_print, "pz_task: freq change -> %dHz\n", state->target_frequency);
            waveform_set_frequency(&state->waveform, state->target_frequency);
            state->frequency_changed = false;
        }

        // Step 1: Generate sine data and write to FIFO
        waveform_fill_buffer(&state->waveform, fill_buf, FIFO_FILL_CHUNK);
        int64_t fill_timestamp = esp_timer_get_time();
        int bytes_written = drv2665_write_fifo(&state->drv, fill_buf, FIFO_FILL_CHUNK);
        if (bytes_written < 0) bytes_written = 0;

        if (debug) {
            mp_printf(&mp_plat_print, "pz_task[%d]: fifo_write=%d/%d write_idx=%d first=%d\n",
                      (int)iter, bytes_written, FIFO_FILL_CHUNK,
                      (int)state->waveform.write_index, (int)fill_buf[0]);
        }

        // After fill, FIFO is approximately full (100 bytes)
        size_t fifo_depth = DRV2665_FIFO_SIZE;

        // Step 2: Check for pending shift register / polarity updates
        if (state->sr.pending_commit || state->sr.pending_polarity) {
            size_t samples_to_trough = waveform_samples_until_trough(
                &state->waveform, fifo_depth);

            // Convert to microseconds (125us per sample at 8kHz)
            int64_t wait_us = (int64_t)samples_to_trough * 125;
            int64_t target_time = fill_timestamp + wait_us;

            mp_printf(&mp_plat_print, "pz_task: SR commit pending, wait %lld us (%d samples to trough)\n",
                      wait_us, (int)samples_to_trough);

            // Hybrid wait: coarse FreeRTOS sleep, then busy-wait for precision
            if (wait_us > 2000) {
                vTaskDelay(pdMS_TO_TICKS((wait_us / 1000) - 1));
            }

            // Busy-wait for precise timing
            while (esp_timer_get_time() < target_time) {
                // spin
            }

            // FIRE: commit shift register + polarity at trough
            shift_register_commit(&state->sr);
        }

        // Step 3: Sleep until roughly half FIFO has drained
        // 50 bytes at 8kHz = 6.25ms. Minimum 1 tick to avoid starving other tasks.
        TickType_t delay = pdMS_TO_TICKS(6);
        vTaskDelay(delay > 0 ? delay : 1);

        iter++;
    }

    mp_printf(&mp_plat_print, "pz_task: stopping, entering standby\n");

    // Stopped: put DRV2665 in standby
    drv2665_standby(&state->drv);
    state->task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t pz_task_start(pz_task_state_t *state) {
    if (state->running) {
        return ESP_OK;
    }

    state->running = true;
    mp_printf(&mp_plat_print, "pz_task: creating task (priority=%d)\n", configMAX_PRIORITIES - 2);

    BaseType_t ret = xTaskCreate(
        pz_background_task,
        "pz_task",
        4096,
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
