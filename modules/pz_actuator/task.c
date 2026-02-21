#include "task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "pz_task";

#define FIFO_FILL_CHUNK 100  // try to fill entire FIFO

static void pz_background_task(void *arg) {
    pz_task_state_t *state = (pz_task_state_t *)arg;
    int8_t fill_buf[FIFO_FILL_CHUNK];

    while (state->running) {
        // Check for frequency change
        if (state->frequency_changed) {
            waveform_set_frequency(&state->waveform, state->target_frequency);
            state->frequency_changed = false;
        }

        // Step 1: Generate sine data and write to FIFO until NACK (full)
        waveform_fill_buffer(&state->waveform, fill_buf, FIFO_FILL_CHUNK);
        int64_t fill_timestamp = esp_timer_get_time();
        int bytes_written = drv2665_write_fifo(&state->drv, fill_buf, FIFO_FILL_CHUNK);
        if (bytes_written < 0) bytes_written = 0;

        // After fill, FIFO is approximately full (100 bytes)
        size_t fifo_depth = DRV2665_FIFO_SIZE;

        // Step 2: Check for pending shift register / polarity updates
        if (state->sr.pending_commit || state->sr.pending_polarity) {
            size_t samples_to_trough = waveform_samples_until_trough(
                &state->waveform, fifo_depth);

            // Convert to microseconds (125us per sample at 8kHz)
            int64_t wait_us = (int64_t)samples_to_trough * 125;
            int64_t target_time = fill_timestamp + wait_us;

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
        // 50 bytes at 8kHz = 6.25ms
        vTaskDelay(pdMS_TO_TICKS(6));
    }

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
    BaseType_t ret = xTaskCreate(
        pz_background_task,
        "pz_task",
        4096,
        state,
        configMAX_PRIORITIES - 1,  // high priority for real-time
        &state->task_handle
    );

    if (ret != pdPASS) {
        state->running = false;
        ESP_LOGE(TAG, "Failed to create background task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Background task started");
    return ESP_OK;
}

esp_err_t pz_task_stop(pz_task_state_t *state) {
    if (!state->running) {
        return ESP_OK;
    }

    state->running = false;

    // Wait for task to exit
    while (state->task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Background task stopped");
    return ESP_OK;
}
