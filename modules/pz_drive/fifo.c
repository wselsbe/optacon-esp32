// modules/pz_drive/fifo.c — Digital FIFO background task for DRV2665
//
// Runs a FreeRTOS background task that fills the DRV2665's 100-byte FIFO
// with waveform samples at 8 kHz.  Uses drv2665.c functions for I2C access.
//
// Timing: esp_timer fires every 8ms, notifies the task via xTaskNotifyGive.
// Task wakes, estimates consumed samples from elapsed time, refills exactly
// that count via bulk I2C write.  This keeps the FIFO topped up without
// polling FIFO_FULL (which was too slow and caused underruns).
//
// Polarity toggling (fullwave) and shift register latching are driven by a
// separate "boundary timer" that fires at the waveform playback rate
// (waveform_len * 125µs).  This keeps the polarity toggle synchronized to
// actual DRV2665 playback rather than the write-ahead cursor.

#include "pz_drive.h"

#include "py/runtime.h"
#include "py/mpprint.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"

#include <string.h>

// ─── DRV2665 constants ──────────────────────────────────────────────────────

#define DRV2665_REG_STATUS    0x00
#define DRV2665_REG_CTRL1     0x01
#define DRV2665_REG_CTRL2     0x02
#define DRV2665_FIFO_SIZE     100
#define DRV2665_TIMEOUT_20MS  (3 << 2) // bits [3:2] of CTRL2
#define DRV2665_STANDBY       (1 << 6)
#define DRV2665_INPUT_DIGITAL (0 << 2)
#define SAMPLE_PERIOD_US      125 // 1/8000 Hz = 125 us per sample

#define REFILL_PERIOD_US 8000 // 8 ms between refills
// At 8kHz, 8ms = 64 samples consumed.  FIFO is 100, so we refill well
// before underrun (100 - 64 = 36 samples headroom).

// ─── FIFO state ─────────────────────────────────────────────────────────────

static int8_t *s_waveform_buf = NULL;
static size_t s_waveform_len = 0;
static size_t s_write_index = 0;
static uint8_t s_gain = 3;
static volatile bool s_fullwave = false;
static volatile bool s_running = false;
static TaskHandle_t s_task_handle = NULL;
static esp_timer_handle_t s_timer_handle = NULL;
static esp_timer_handle_t s_boundary_timer = NULL;

// ─── Waveform helpers ───────────────────────────────────────────────────────

static inline int8_t next_sample(void) {
    int8_t s = s_waveform_buf[s_write_index];
    s_write_index++;
    if (s_write_index >= s_waveform_len) {
        s_write_index = 0;
    }
    return s;
}

static void fill_from_waveform(uint8_t *fill_buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        fill_buf[i] = (uint8_t)next_sample();
    }
}

// ─── DRV2665 mode helpers ───────────────────────────────────────────────────

static void fifo_enable_digital(uint8_t gain) {
    // Datasheet 8.3.1: exit standby, set digital mode + gain, set timeout
    drv2665_write_reg(DRV2665_REG_CTRL2, 0x00); // exit standby, clear timeout

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    drv2665_write_reg(DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | (gain & 0x03));
    drv2665_write_reg(DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
}

static void fifo_standby(void) {
    drv2665_write_reg(DRV2665_REG_CTRL2, DRV2665_STANDBY);
}

// ─── Timer callbacks ────────────────────────────────────────────────────────

// Refill timer: wakes the FIFO task every 8ms to top up the FIFO
static void fifo_timer_callback(void *arg) {
    if (s_task_handle != NULL) {
        xTaskNotifyGive(s_task_handle);
    }
}

// Boundary timer: fires at each waveform period boundary (playback rate).
// Toggles polarity for fullwave mode and latches pending SR data.
// Runs in esp_timer task context — gpio_set_level() is safe here.
static void boundary_timer_callback(void *arg) {
    if (!s_running) return;
    if (s_fullwave) {
        hv509_pol_toggle();
    }
    hv509_sr_latch_if_pending();
}

// ─── Background task ────────────────────────────────────────────────────────

static void fifo_background_task(void *arg) {
    (void)arg;
    uint8_t fill_buf[DRV2665_FIFO_SIZE];

    // Let USB-CDC stabilize before heavy I2C traffic
    vTaskDelay(pdMS_TO_TICKS(20) > 0 ? pdMS_TO_TICKS(20) : 2);

    fifo_enable_digital(s_gain);

    // ── INITIAL FILL ─────────────────────────────────────────────
    fill_from_waveform(fill_buf, DRV2665_FIFO_SIZE);
    drv2665_write_fifo_bulk(fill_buf, DRV2665_FIFO_SIZE);

    // DRV2665 starts playing from sample 0 as soon as FIFO data arrives.
    // Start the boundary timer now — first callback fires at exactly one
    // waveform period after playback begins, toggling polarity in sync.
    uint64_t boundary_us = (uint64_t)s_waveform_len * SAMPLE_PERIOD_US;
    esp_timer_start_periodic(s_boundary_timer, boundary_us);

    // Track time for consumed sample estimation
    int64_t fill_time = esp_timer_get_time();

    // Start periodic timer for refill wakeups
    esp_timer_start_periodic(s_timer_handle, REFILL_PERIOD_US);

    while (s_running) {
        // Wait for timer notification (blocks until timer fires)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)); // 50ms safety timeout
        if (!s_running) break;

        // Estimate how many samples the FIFO has consumed since last fill
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - fill_time;
        size_t consumed = (size_t)(elapsed / SAMPLE_PERIOD_US);
        if (consumed > DRV2665_FIFO_SIZE) consumed = DRV2665_FIFO_SIZE;
        if (consumed == 0) continue;

        // Refill with exactly the consumed count
        fill_from_waveform(fill_buf, consumed);
        drv2665_write_fifo_bulk(fill_buf, consumed);

        fill_time = now;
    }

    esp_timer_stop(s_timer_handle);
    esp_timer_stop(s_boundary_timer);
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
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pz_drive: FIFO already running"));
    }

    // Set up state
    s_waveform_buf = (int8_t *)buf;
    s_waveform_len = len;
    s_write_index = 0;
    s_gain = (uint8_t)gain;
    s_fullwave = fullwave;
    s_running = true;

    // Create esp_timer for periodic FIFO refill wakeups
    if (s_timer_handle == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = fifo_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pz_fifo_timer",
        };
        esp_err_t err = esp_timer_create(&timer_args, &s_timer_handle);
        if (err != ESP_OK) {
            s_running = false;
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pz_drive: failed to create timer"));
        }
    }

    // Create esp_timer for playback-synchronized boundary events
    if (s_boundary_timer == NULL) {
        esp_timer_create_args_t bnd_args = {
            .callback = boundary_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pz_fifo_boundary",
        };
        esp_err_t err = esp_timer_create(&bnd_args, &s_boundary_timer);
        if (err != ESP_OK) {
            s_running = false;
            esp_timer_delete(s_timer_handle);
            s_timer_handle = NULL;
            mp_raise_msg(&mp_type_RuntimeError,
                         MP_ERROR_TEXT("pz_drive: failed to create boundary timer"));
        }
    }

    // Pin to core 1 so I2C traffic doesn't starve USB-CDC (TinyUSB on core 0)
    BaseType_t ret = xTaskCreatePinnedToCore(fifo_background_task, "pz_fifo", 8192, NULL,
                                             tskIDLE_PRIORITY + 2, &s_task_handle, 1);
    if (ret != pdPASS) {
        s_running = false;
        esp_timer_delete(s_timer_handle);
        s_timer_handle = NULL;
        esp_timer_delete(s_boundary_timer);
        s_boundary_timer = NULL;
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pz_drive: failed to create FIFO task"));
    }

    mp_printf(&mp_plat_print,
              "pz_drive: fifo started (waveform=%u samples, gain=%d, fullwave=%d)\n", (unsigned)len,
              gain, (int)fullwave);
}

void pzd_fifo_stop(void) {
    if (!s_running) {
        return;
    }

    // Signal the task to stop
    s_running = false;

    // Wait for the task to exit (it sets task_handle = NULL before deleting)
    int retries = 50;
    while (s_task_handle != NULL && retries-- > 0) {
        TickType_t t = pdMS_TO_TICKS(10);
        vTaskDelay(t > 0 ? t : 1);
    }
    if (s_task_handle != NULL) {
        mp_printf(&mp_plat_print, "pz_drive: warning - FIFO task did not exit, deleting\n");
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }

    // Clean up timers
    if (s_timer_handle != NULL) {
        esp_timer_delete(s_timer_handle);
        s_timer_handle = NULL;
    }
    if (s_boundary_timer != NULL) {
        esp_timer_delete(s_boundary_timer);
        s_boundary_timer = NULL;
    }

    // Reset polarity to safe default
    hv509_pol_init();

    // Clear waveform pointers (Python owns the buffer)
    s_waveform_buf = NULL;
    s_waveform_len = 0;
    s_write_index = 0;

    mp_printf(&mp_plat_print, "pz_drive: fifo stopped\n");
}
