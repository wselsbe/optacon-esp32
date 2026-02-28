// modules/pz_drive/fifo.c — Digital FIFO background task for DRV2665
//
// Runs a FreeRTOS background task that fills the DRV2665's 100-byte FIFO
// with waveform samples at 8 kHz.  Uses drv2665.c functions for I2C access
// instead of raw bus handles.
//
// At each period boundary (waveform wrap), the task:
//   - Toggles polarity via hv509_pol_toggle() if fullwave mode is active
//   - Latches any pending shift register data via hv509_sr_latch_if_pending()

#include "pz_drive.h"

#include "py/runtime.h"
#include "py/mpprint.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

// ─── DRV2665 constants ──────────────────────────────────────────────────────

#define DRV2665_REG_STATUS      0x00
#define DRV2665_REG_CTRL1       0x01
#define DRV2665_REG_CTRL2       0x02
#define DRV2665_FIFO_SIZE       100
#define DRV2665_FIFO_FULL       0x01
#define DRV2665_TIMEOUT_20MS    (3 << 2)    // bits [3:2] of CTRL2
#define DRV2665_STANDBY         (1 << 6)
#define DRV2665_INPUT_DIGITAL   (0 << 2)
#define SAMPLE_PERIOD_US        125         // 1/8000 Hz = 125 us per sample

// ─── FIFO state ─────────────────────────────────────────────────────────────

static int8_t *s_waveform_buf = NULL;
static size_t s_waveform_len = 0;
static size_t s_write_index = 0;
static uint8_t s_gain = 3;
static volatile bool s_fullwave = false;
static volatile bool s_running = false;
static TaskHandle_t s_task_handle = NULL;

// ─── Waveform helpers ───────────────────────────────────────────────────────

static inline int8_t next_sample(void) {
    int8_t s = s_waveform_buf[s_write_index];
    s_write_index++;
    if (s_write_index >= s_waveform_len) {
        s_write_index = 0;
        // Period boundary — latch shift register + toggle polarity
        if (s_fullwave) {
            hv509_pol_toggle();
        }
        hv509_sr_latch_if_pending();
    }
    return s;
}

static void fill_from_waveform(uint8_t *fill_buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        fill_buf[i] = (uint8_t)next_sample();
    }
}

// Write single bytes until FIFO_FULL status bit is set.
static int fill_until_full(void) {
    int written = 0;

    while (s_running) {
        int8_t s = next_sample();
        drv2665_write_fifo_byte((uint8_t)s);
        written++;

        uint8_t status = drv2665_read_status();
        if (status & DRV2665_FIFO_FULL) {
            break;
        }
    }
    return written;
}

// ─── DRV2665 mode helpers ───────────────────────────────────────────────────

static void fifo_enable_digital(uint8_t gain) {
    // Datasheet 8.3.1: exit standby, set digital mode + gain, set timeout
    drv2665_write_reg(DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    drv2665_write_reg(DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | (gain & 0x03));
    drv2665_write_reg(DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
}

static void fifo_standby(void) {
    drv2665_write_reg(DRV2665_REG_CTRL2, DRV2665_STANDBY);
}

// ─── Background task ────────────────────────────────────────────────────────
//
// Hybrid fill strategy:
//   1. Initial fill: bulk-write 100 bytes to FIFO
//   2. Main loop:
//      a. Sleep ~half the remaining FIFO drain time
//      b. Estimate consumed samples from elapsed time
//      c. Bulk-write half the estimated room
//      d. Single-byte writes polling FIFO_FULL until full
//      e. Update sync time, repeat

static void fifo_background_task(void *arg) {
    (void)arg;
    uint8_t fill_buf[DRV2665_FIFO_SIZE];

    fifo_enable_digital(s_gain);

    // ── INITIAL FILL ─────────────────────────────────────────────
    fill_from_waveform(fill_buf, DRV2665_FIFO_SIZE);
    drv2665_write_fifo_bulk(fill_buf, DRV2665_FIFO_SIZE);

    // Sync point: FIFO has exactly 100 samples
    int64_t sync_time = esp_timer_get_time();
    size_t fifo_level = DRV2665_FIFO_SIZE;

    while (s_running) {
        // ── SLEEP ───────────────────────────────────────────────
        int64_t remaining_us = (int64_t)fifo_level * SAMPLE_PERIOD_US;
        int64_t sleep_us = remaining_us / 2;
        if (sleep_us > 1000) {
            TickType_t t = pdMS_TO_TICKS(sleep_us / 1000);
            if (t > 0) vTaskDelay(t);
        } else {
            vTaskDelay(1);
        }

        if (!s_running) break;

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
            fill_from_waveform(fill_buf, bulk);
            drv2665_write_fifo_bulk(fill_buf, bulk);
        }

        // ── BYTE-BY-BYTE UNTIL FIFO_FULL ────────────────────────
        fill_until_full();

        if (!s_running) break;

        // ── SYNC POINT ─────────────────────────────────────────
        sync_time = esp_timer_get_time();
        fifo_level = DRV2665_FIFO_SIZE;
    }

    fifo_standby();
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── Public API (called by pz_drive.c bindings) ─────────────────────────────

bool pzd_fifo_is_running(void) {
    return s_running;
}

void pzd_fifo_start(const uint8_t *buf, size_t len, int gain, bool fullwave) {
    if (s_running) {
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("pz_drive: FIFO already running"));
    }

    // Set up state
    s_waveform_buf = (int8_t *)buf;
    s_waveform_len = len;
    s_write_index = 0;
    s_gain = (uint8_t)gain;
    s_fullwave = fullwave;
    s_running = true;

    // Spawn background task
    BaseType_t ret = xTaskCreate(fifo_background_task, "pz_fifo",
                                 8192, NULL,
                                 configMAX_PRIORITIES - 2,
                                 &s_task_handle);
    if (ret != pdPASS) {
        s_running = false;
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("pz_drive: failed to create FIFO task"));
    }

    mp_printf(&mp_plat_print,
              "pz_drive: fifo started (waveform=%u samples, gain=%d, fullwave=%d)\n",
              (unsigned)len, gain, (int)fullwave);
}

void pzd_fifo_stop(void) {
    if (!s_running) {
        return;
    }

    // Signal the task to stop
    s_running = false;

    // Wait for the task to exit (it sets task_handle = NULL before deleting)
    while (s_task_handle != NULL) {
        TickType_t t = pdMS_TO_TICKS(10);
        vTaskDelay(t > 0 ? t : 1);
    }

    // Clear waveform pointers (Python owns the buffer)
    s_waveform_buf = NULL;
    s_waveform_len = 0;
    s_write_index = 0;

    mp_printf(&mp_plat_print, "pz_drive: fifo stopped\n");
}
