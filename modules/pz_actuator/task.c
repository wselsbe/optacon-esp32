#include "task.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include <string.h>

// ── Hybrid FIFO fill strategy ──────────────────────────────────────────
// Goal: keep FIFO fed (no silence) while knowing exact FIFO level.
//
// The DRV2665 does NOT NACK when the FIFO is full — it silently drops
// data. So we use the FIFO_FULL status bit (polled via I2C) instead.
//
// After initial fill (100 bytes), the loop:
//   1. Sleep ~half the FIFO drain time (conservative headroom)
//   2. Estimate how many samples were consumed based on elapsed time
//   3. Bulk-write half the estimated room (fast, guaranteed to fit)
//   4. Single-byte writes, polling FIFO_FULL after each — stop when full
//   5. SYNC POINT: FIFO is full = exactly 100 bytes queued
//   6. Calculate trough timing from known position, commit shift register
//   7. Repeat

#define SAMPLE_PERIOD_US  125   // 1/8000 Hz = 125us per sample

// Get next waveform sample and advance write_index.
static inline int8_t next_sample(pz_task_state_t *state) {
    int8_t s = state->waveform_buf[state->write_index];
    state->write_index = (state->write_index + 1) % state->waveform_len;
    return s;
}

// Bulk-fill fill_buf from waveform, advancing write_index.
static void fill_from_waveform(pz_task_state_t *state, int8_t *fill_buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        fill_buf[i] = next_sample(state);
    }
}

// Write single bytes until FIFO_FULL status bit is set.
// Returns number of bytes successfully written.
static int fill_until_full(pz_task_state_t *state) {
    int written = 0;
    uint8_t status;

    while (state->running) {
        // Write one sample
        int8_t s = next_sample(state);
        drv2665_write_fifo_byte(&state->drv, s);
        written++;

        // Check if FIFO is now full
        drv2665_read_status(&state->drv, &status);
        if (status & DRV2665_FIFO_FULL) {
            break;
        }
    }
    return written;
}

static void pz_background_task(void *arg) {
    pz_task_state_t *state = (pz_task_state_t *)arg;
    int8_t fill_buf[DRV2665_FIFO_SIZE];

    drv2665_enable_digital(&state->drv, state->drv.gain);

    // ── INITIAL FILL ─────────────────────────────────────────────
    // FIFO is empty after enable_digital. Fill it completely.
    fill_from_waveform(state, fill_buf, DRV2665_FIFO_SIZE);
    drv2665_write_fifo(&state->drv, fill_buf, DRV2665_FIFO_SIZE);

    // Sync point: FIFO has exactly 100 samples.
    int64_t sync_time = esp_timer_get_time();
    size_t fifo_level = DRV2665_FIFO_SIZE;

    while (state->running) {
        // ── SLEEP ───────────────────────────────────────────────
        // Sleep for roughly half the remaining FIFO time.
        int64_t remaining_us = (int64_t)fifo_level * SAMPLE_PERIOD_US;
        int64_t sleep_us = remaining_us / 2;
        if (sleep_us > 1000) {
            TickType_t t = pdMS_TO_TICKS(sleep_us / 1000);
            if (t > 0) vTaskDelay(t);
        } else {
            vTaskDelay(1);
        }

        if (!state->running) break;

        // ── ESTIMATE CONSUMED ───────────────────────────────────
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - sync_time;
        size_t consumed = (size_t)(elapsed / SAMPLE_PERIOD_US);
        if (consumed > fifo_level) consumed = fifo_level;
        size_t room = consumed;

        // ── BULK WRITE (half the estimated room) ────────────────
        size_t bulk = room / 2;
        if (bulk > 0) {
            if (bulk > DRV2665_FIFO_SIZE) bulk = DRV2665_FIFO_SIZE;
            fill_from_waveform(state, fill_buf, bulk);
            drv2665_write_fifo(&state->drv, fill_buf, bulk);
        }

        // ── BYTE-BY-BYTE UNTIL FIFO_FULL ────────────────────────
        // Write one sample, poll FIFO_FULL, repeat until full.
        // Each iteration costs ~2 I2C transactions (~400us at 100kHz).
        fill_until_full(state);

        if (!state->running) break;

        // ── SYNC POINT ─────────────────────────────────────────
        // FIFO is full (100 bytes). write_index is exactly 100 samples
        // ahead of the current playback position.
        sync_time = esp_timer_get_time();
        fifo_level = DRV2665_FIFO_SIZE;

        // Shift register commits are handled directly by set_pin/flush calls
    }

    drv2665_standby(&state->drv);
    state->task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t pz_task_start(pz_task_state_t *state) {
    if (state->running) {
        return ESP_OK;
    }

    state->running = true;
    state->write_index = 0;

    BaseType_t ret = xTaskCreate(
        pz_background_task,
        "pz_task",
        8192,
        state,
        configMAX_PRIORITIES / 2,
        &state->task_handle
    );

    if (ret != pdPASS) {
        state->running = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pz_task_stop(pz_task_state_t *state) {
    if (!state->running) {
        return ESP_OK;
    }

    state->running = false;

    while (state->task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}
