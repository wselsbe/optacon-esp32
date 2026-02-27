// drv_fifo — Standalone MicroPython C module for DRV2665 digital FIFO fill
//
// Accepts a MicroPython machine.I2C object, extracts the native I2C bus handle,
// adds a DRV2665 device on that bus, and runs a FreeRTOS background task that
// fills the DRV2665's 100-byte FIFO with waveform samples at 8 kHz.
//
// Python is responsible for all DRV2665 register configuration (mode, gain,
// standby). This module ONLY fills the FIFO.
//
// Python API:
//   drv_fifo.start(i2c, waveform)  — start FIFO fill task
//   drv_fifo.stop()                — stop task, remove device from bus
//   drv_fifo.is_running()          — return bool

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mpprint.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include <string.h>
#include <stddef.h>

// ─── MicroPython I2C struct layout (from ports/esp32/machine_i2c.c) ─────────
//
// We replicate this struct to extract the native bus_handle from a Python
// machine.I2C object. The _Static_assert below catches layout drift.

typedef struct _machine_hw_i2c_obj_t {
    mp_obj_base_t base;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle; // present on IDF >= 5.5.0
    uint8_t port : 8;
    gpio_num_t scl : 8;
    gpio_num_t sda : 8;
    uint32_t freq;
    uint32_t timeout_us;
} machine_hw_i2c_obj_t;

_Static_assert(offsetof(machine_hw_i2c_obj_t, bus_handle) == sizeof(mp_obj_base_t),
               "I2C struct layout mismatch — check MicroPython machine_i2c.c");

// ─── DRV2665 constants ──────────────────────────────────────────────────────

#define DRV2665_ADDR       0x59
#define DRV2665_I2C_CLK_HZ 100000
#define DRV2665_REG_DATA   0x0B
#define DRV2665_FIFO_SIZE  100

#define SAMPLE_PERIOD_US 125 // 1/8000 Hz = 125 us per sample
#define I2C_TIMEOUT_MS   100

// ─── FIFO state ─────────────────────────────────────────────────────────────

typedef struct {
    i2c_master_dev_handle_t dev;
    int8_t *waveform_buf;
    size_t waveform_len;
    size_t write_index;
    TaskHandle_t task_handle;
    esp_timer_handle_t timer_handle;
    volatile bool running;
} fifo_state_t;

static fifo_state_t s_state = {0};

// ─── I2C helpers ────────────────────────────────────────────────────────────
//
// We share MicroPython's dev_handle. Before each transaction, ensure the
// handle's address is set to DRV2665 (0x59) in case Python changed it
// (e.g. during i2c.scan()). Since both sides only talk to 0x59 in normal
// operation, the address is almost always already correct.

static inline void ensure_addr(i2c_master_dev_handle_t dev) {
    i2c_master_device_change_address(dev, DRV2665_ADDR, I2C_TIMEOUT_MS);
}

// fifo_read_status removed — no longer polling FIFO_FULL bit

static esp_err_t fifo_write_bulk(i2c_master_dev_handle_t dev, const int8_t *data, size_t len) {
    if (len > DRV2665_FIFO_SIZE) len = DRV2665_FIFO_SIZE;
    uint8_t buf[DRV2665_FIFO_SIZE + 1];
    buf[0] = DRV2665_REG_DATA;
    memcpy(&buf[1], data, len);
    ensure_addr(dev);
    return i2c_master_transmit(dev, buf, 1 + len, I2C_TIMEOUT_MS);
}

// fifo_write_byte removed — bulk writes only now

// ─── Waveform helpers ───────────────────────────────────────────────────────

static inline int8_t next_sample(fifo_state_t *state) {
    int8_t s = state->waveform_buf[state->write_index];
    state->write_index = (state->write_index + 1) % state->waveform_len;
    return s;
}

static void fill_from_waveform(fifo_state_t *state, int8_t *fill_buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        fill_buf[i] = next_sample(state);
    }
}

// ─── Background task + timer ─────────────────────────────────────────────────
//
// Strategy: esp_timer fires a periodic callback every REFILL_PERIOD_US.
// The callback sends a task notification to wake the FreeRTOS task, which
// does the actual I2C bulk write (I2C can't run from ISR/timer context).
//
// At 8 kHz sample rate and 100-byte FIFO, the FIFO drains in 12.5 ms.
// We refill every 5 ms (~40 samples consumed), well before underrun.

#define REFILL_PERIOD_US 5000 // 5 ms between refills
#define REFILL_SAMPLES   ((REFILL_PERIOD_US + SAMPLE_PERIOD_US - 1) / SAMPLE_PERIOD_US) // 40

static void fifo_timer_callback(void *arg) {
    fifo_state_t *state = (fifo_state_t *)arg;
    if (state->task_handle != NULL) {
        xTaskNotifyGive(state->task_handle);
    }
}

static void fifo_background_task(void *arg) {
    fifo_state_t *state = (fifo_state_t *)arg;
    int8_t fill_buf[DRV2665_FIFO_SIZE];

    // ── INITIAL FILL ─────────────────────────────────────────────
    fill_from_waveform(state, fill_buf, DRV2665_FIFO_SIZE);
    esp_err_t err = fifo_write_bulk(state->dev, fill_buf, DRV2665_FIFO_SIZE);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "drv_fifo: initial fill failed (%d)\n", err);
        state->running = false;
    }

    // Track time for consumed sample estimation
    int64_t fill_time = esp_timer_get_time();

    // Start periodic timer to wake us for refills
    esp_timer_start_periodic(state->timer_handle, REFILL_PERIOD_US);

    while (state->running) {
        // Wait for timer notification (blocks until timer fires)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)); // 50ms timeout as safety net
        if (!state->running) break;

        // Estimate how many samples the FIFO has consumed since last fill
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - fill_time;
        size_t consumed = (size_t)(elapsed / SAMPLE_PERIOD_US);
        if (consumed > DRV2665_FIFO_SIZE) consumed = DRV2665_FIFO_SIZE;
        if (consumed == 0) continue;

        // Refill with exactly the consumed count
        fill_from_waveform(state, fill_buf, consumed);
        err = fifo_write_bulk(state->dev, fill_buf, consumed);
        if (err != ESP_OK) {
            mp_printf(&mp_plat_print, "drv_fifo: write error (%d)\n", err);
        }

        fill_time = now;
    }

    esp_timer_stop(state->timer_handle);
    state->task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── MicroPython bindings ────────────────────────────────────────────────────

// drv_fifo.start(i2c, waveform)
static mp_obj_t drv_fifo_start(mp_obj_t i2c_obj, mp_obj_t waveform_obj) {
    // Check not already running
    if (s_state.running) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("drv_fifo already running"));
    }

    // Extract handles from MicroPython I2C object
    machine_hw_i2c_obj_t *i2c = (machine_hw_i2c_obj_t *)MP_OBJ_TO_PTR(i2c_obj);
    if (i2c->bus_handle == NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("I2C bus not initialized"));
    }

    // Get waveform buffer (must be bytearray)
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(waveform_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("waveform must not be empty"));
    }

    // Reuse MicroPython's device handle instead of creating a second one.
    // ESP-IDF's i2c_master_probe() (called by MicroPython before every transaction)
    // fails with ENODEV when two device handles exist for the same address.
    // Since both Python and the FIFO task only talk to 0x59, sharing the handle
    // is safe — the bus semaphore serializes all transactions.
    i2c_master_dev_handle_t dev = i2c->dev_handle;
    if (dev == NULL) {
        mp_raise_msg(
            &mp_type_RuntimeError,
            MP_ERROR_TEXT("I2C device not initialized - do i2c.writeto(0x59, b'\\x00') first"));
    }
    s_state.dev = dev;

    // Set up state
    s_state.waveform_buf = (int8_t *)bufinfo.buf;
    s_state.waveform_len = bufinfo.len;
    s_state.write_index = 0;
    s_state.running = true;

    // Create esp_timer for precise periodic wakeups
    if (s_state.timer_handle == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = fifo_timer_callback,
            .arg = &s_state,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "drv_fifo_timer",
        };
        esp_err_t terr = esp_timer_create(&timer_args, &s_state.timer_handle);
        if (terr != ESP_OK) {
            s_state.running = false;
            s_state.dev = NULL;
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create timer"));
        }
    }

    // Spawn background task at elevated priority for timely FIFO refills.
    // Must be higher than MicroPython's main task (priority 1) to respond
    // promptly to timer notifications.  The I2C bus semaphore serializes
    // transactions, so Python can still access the bus between refills.
    BaseType_t ret = xTaskCreate(fifo_background_task, "drv_fifo", 4096, &s_state,
                                 tskIDLE_PRIORITY + 3, &s_state.task_handle);
    if (ret != pdPASS) {
        s_state.running = false;
        esp_timer_delete(s_state.timer_handle);
        s_state.timer_handle = NULL;
        s_state.dev = NULL;
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create FIFO task"));
    }

    mp_printf(&mp_plat_print, "drv_fifo: started (waveform=%u samples)\n", (unsigned)bufinfo.len);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(drv_fifo_start_obj, drv_fifo_start);

// drv_fifo.stop()
static mp_obj_t drv_fifo_stop(void) {
    if (!s_state.running) {
        return mp_const_none;
    }

    // Signal the task to stop
    s_state.running = false;

    // Wait for the task to exit (it sets task_handle = NULL before deleting)
    // Timeout after 500 ms to avoid hanging forever
    int retries = 50;
    while (s_state.task_handle != NULL && retries-- > 0) {
        vTaskDelay(1); // ~10 ms per tick at 100 Hz
    }
    if (s_state.task_handle != NULL) {
        mp_printf(&mp_plat_print, "drv_fifo: warning - task did not exit, deleting\n");
        vTaskDelete(s_state.task_handle);
        s_state.task_handle = NULL;
    }

    // Clean up timer
    if (s_state.timer_handle != NULL) {
        esp_timer_delete(s_state.timer_handle);
        s_state.timer_handle = NULL;
    }

    // Don't remove the device — we borrowed MicroPython's dev_handle
    s_state.dev = NULL;

    // Clear waveform pointers (Python owns the buffer)
    s_state.waveform_buf = NULL;
    s_state.waveform_len = 0;
    s_state.write_index = 0;

    mp_printf(&mp_plat_print, "drv_fifo: stopped\n");

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(drv_fifo_stop_obj, drv_fifo_stop);

// drv_fifo.is_running()
static mp_obj_t drv_fifo_is_running(void) {
    return mp_obj_new_bool(s_state.running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(drv_fifo_is_running_obj, drv_fifo_is_running);

// drv_fifo.read_reg(reg) — bypass MicroPython's probe for bus-sharing
// The DRV2665 NACKs probes while FIFO data is being processed. This function
// uses the shared device handle directly, skipping the probe.
static mp_obj_t drv_fifo_read_reg(mp_obj_t reg_obj) {
    if (s_state.dev == NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("drv_fifo not started"));
    }
    uint8_t reg = (uint8_t)mp_obj_get_int(reg_obj);
    uint8_t val;
    ensure_addr(s_state.dev);
    esp_err_t err = i2c_master_transmit_receive(s_state.dev, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        mp_raise_OSError(err);
    }
    return mp_obj_new_int(val);
}
static MP_DEFINE_CONST_FUN_OBJ_1(drv_fifo_read_reg_obj, drv_fifo_read_reg);

// drv_fifo.write_reg(reg, val) — bypass MicroPython's probe for bus-sharing
static mp_obj_t drv_fifo_write_reg(mp_obj_t reg_obj, mp_obj_t val_obj) {
    if (s_state.dev == NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("drv_fifo not started"));
    }
    uint8_t buf[2] = {(uint8_t)mp_obj_get_int(reg_obj), (uint8_t)mp_obj_get_int(val_obj)};
    ensure_addr(s_state.dev);
    esp_err_t err = i2c_master_transmit(s_state.dev, buf, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        mp_raise_OSError(err);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(drv_fifo_write_reg_obj, drv_fifo_write_reg);

// ─── Module registration ─────────────────────────────────────────────────────

static const mp_rom_map_elem_t drv_fifo_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_drv_fifo)},
    {MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&drv_fifo_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&drv_fifo_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_is_running), MP_ROM_PTR(&drv_fifo_is_running_obj)},
    {MP_ROM_QSTR(MP_QSTR_read_reg), MP_ROM_PTR(&drv_fifo_read_reg_obj)},
    {MP_ROM_QSTR(MP_QSTR_write_reg), MP_ROM_PTR(&drv_fifo_write_reg_obj)},
};
static MP_DEFINE_CONST_DICT(drv_fifo_globals, drv_fifo_globals_table);

const mp_obj_module_t drv_fifo_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&drv_fifo_globals,
};

MP_REGISTER_MODULE(MP_QSTR_drv_fifo, drv_fifo_module);
