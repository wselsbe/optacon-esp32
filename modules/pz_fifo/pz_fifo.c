// pz_fifo — Standalone MicroPython C module for DRV2665 digital FIFO fill
//
// Accepts a MicroPython machine.I2C object, extracts the native I2C bus handle,
// adds a DRV2665 device on that bus, and runs a FreeRTOS background task that
// fills the DRV2665's 100-byte FIFO with waveform samples at 8 kHz.
//
// Python API:
//   pz_fifo.start(i2c, waveform, gain=3)  — start FIFO fill task
//   pz_fifo.stop()                         — stop task, standby, remove device
//   pz_fifo.is_running()                   — return bool

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
    i2c_master_dev_handle_t dev_handle;  // present on IDF >= 5.5.0
    uint8_t port : 8;
    gpio_num_t scl : 8;
    gpio_num_t sda : 8;
    uint32_t freq;
    uint32_t timeout_us;
} machine_hw_i2c_obj_t;

_Static_assert(offsetof(machine_hw_i2c_obj_t, bus_handle) == sizeof(mp_obj_base_t),
               "I2C struct layout mismatch — check MicroPython machine_i2c.c");

// ─── DRV2665 constants ──────────────────────────────────────────────────────

#define DRV2665_ADDR            0x59
#define DRV2665_I2C_CLK_HZ     100000
#define DRV2665_REG_STATUS      0x00
#define DRV2665_REG_CTRL1       0x01
#define DRV2665_REG_CTRL2       0x02
#define DRV2665_REG_DATA        0x0B
#define DRV2665_FIFO_SIZE       100
#define DRV2665_FIFO_FULL       0x01

#define DRV2665_INPUT_DIGITAL   (0 << 2)
#define DRV2665_STANDBY         (1 << 6)
#define DRV2665_TIMEOUT_20MS    (3 << 2)

#define SAMPLE_PERIOD_US        125     // 1/8000 Hz = 125 us per sample
#define I2C_TIMEOUT_MS          100

// ─── FIFO state ─────────────────────────────────────────────────────────────

typedef struct {
    i2c_master_dev_handle_t dev;
    int8_t *waveform_buf;
    size_t waveform_len;
    size_t write_index;
    uint8_t gain;
    TaskHandle_t task_handle;
    volatile bool running;
} fifo_state_t;

static fifo_state_t s_state = {0};

// ─── I2C helpers ────────────────────────────────────────────────────────────

static esp_err_t fifo_write_register(i2c_master_dev_handle_t dev,
                                     uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t fifo_read_status(i2c_master_dev_handle_t dev, uint8_t *status) {
    uint8_t reg = DRV2665_REG_STATUS;
    return i2c_master_transmit_receive(dev, &reg, 1, status, 1, I2C_TIMEOUT_MS);
}

static esp_err_t fifo_write_bulk(i2c_master_dev_handle_t dev,
                                 const int8_t *data, size_t len) {
    if (len > DRV2665_FIFO_SIZE) len = DRV2665_FIFO_SIZE;
    uint8_t buf[DRV2665_FIFO_SIZE + 1];
    buf[0] = DRV2665_REG_DATA;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev, buf, 1 + len, I2C_TIMEOUT_MS);
}

static esp_err_t fifo_write_byte(i2c_master_dev_handle_t dev, int8_t sample) {
    uint8_t buf[2] = {DRV2665_REG_DATA, (uint8_t)sample};
    return i2c_master_transmit(dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t fifo_standby(i2c_master_dev_handle_t dev) {
    return fifo_write_register(dev, DRV2665_REG_CTRL2, DRV2665_STANDBY);
}

static esp_err_t fifo_enable_digital(i2c_master_dev_handle_t dev, uint8_t gain) {
    // Datasheet 8.3.1: exit standby, set digital mode + gain, set timeout
    esp_err_t err = fifo_write_register(dev, DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    err = fifo_write_register(dev, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | (gain & 0x03));
    if (err != ESP_OK) return err;

    err = fifo_write_register(dev, DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

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

// Write single bytes until FIFO_FULL status bit is set.
static int fill_until_full(fifo_state_t *state) {
    int written = 0;
    uint8_t status;

    while (state->running) {
        int8_t s = next_sample(state);
        fifo_write_byte(state->dev, s);
        written++;

        fifo_read_status(state->dev, &status);
        if (status & DRV2665_FIFO_FULL) {
            break;
        }
    }
    return written;
}

// ─── Background task ────────────────────────────────────────────────────────
//
// Hybrid fill strategy (from pz_actuator/task.c):
//   1. Initial fill: bulk-write 100 bytes
//   2. Main loop:
//      a. Sleep ~half the remaining FIFO drain time
//      b. Estimate consumed samples from elapsed time
//      c. Bulk-write half the estimated room
//      d. Single-byte writes polling FIFO_FULL until full
//      e. Update sync time, repeat

static void fifo_background_task(void *arg) {
    fifo_state_t *state = (fifo_state_t *)arg;
    int8_t fill_buf[DRV2665_FIFO_SIZE];

    fifo_enable_digital(state->dev, state->gain);

    // ── INITIAL FILL ─────────────────────────────────────────────
    fill_from_waveform(state, fill_buf, DRV2665_FIFO_SIZE);
    fifo_write_bulk(state->dev, fill_buf, DRV2665_FIFO_SIZE);

    // Sync point: FIFO has exactly 100 samples
    int64_t sync_time = esp_timer_get_time();
    size_t fifo_level = DRV2665_FIFO_SIZE;

    while (state->running) {
        // ── SLEEP ───────────────────────────────────────────────
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
            fifo_write_bulk(state->dev, fill_buf, bulk);
        }

        // ── BYTE-BY-BYTE UNTIL FIFO_FULL ────────────────────────
        fill_until_full(state);

        if (!state->running) break;

        // ── SYNC POINT ─────────────────────────────────────────
        sync_time = esp_timer_get_time();
        fifo_level = DRV2665_FIFO_SIZE;
    }

    fifo_standby(state->dev);
    state->task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── MicroPython bindings ────────────────────────────────────────────────────

// pz_fifo.start(i2c, waveform, gain=3)
static mp_obj_t pz_fifo_start(size_t n_args, const mp_obj_t *pos_args,
                               mp_map_t *kw_args) {
    enum { ARG_i2c, ARG_waveform, ARG_gain };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_i2c,      MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_waveform, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_gain,     MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 3}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Check not already running
    if (s_state.running) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pz_fifo already running"));
    }

    // Extract I2C bus handle from MicroPython object
    mp_obj_t i2c_obj = args[ARG_i2c].u_obj;
    machine_hw_i2c_obj_t *i2c = (machine_hw_i2c_obj_t *)MP_OBJ_TO_PTR(i2c_obj);
    i2c_master_bus_handle_t bus = i2c->bus_handle;
    if (bus == NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("I2C bus not initialized"));
    }

    // Get waveform buffer (must be bytearray)
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_waveform].u_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("waveform must not be empty"));
    }

    // Validate gain
    int gain = args[ARG_gain].u_int;
    if (gain < 0 || gain > 3) {
        mp_raise_ValueError(MP_ERROR_TEXT("gain must be 0-3"));
    }

    // Add DRV2665 device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2665_ADDR,
        .scl_speed_hz = DRV2665_I2C_CLK_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_state.dev);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_fifo: add device failed: %d\n", err);
        mp_raise_OSError(err);
    }

    // Set up state
    s_state.waveform_buf = (int8_t *)bufinfo.buf;
    s_state.waveform_len = bufinfo.len;
    s_state.write_index = 0;
    s_state.gain = (uint8_t)gain;
    s_state.running = true;

    // Spawn background task
    BaseType_t ret = xTaskCreate(fifo_background_task, "pz_fifo",
                                 8192, &s_state,
                                 configMAX_PRIORITIES - 2,
                                 &s_state.task_handle);
    if (ret != pdPASS) {
        s_state.running = false;
        i2c_master_bus_rm_device(s_state.dev);
        s_state.dev = NULL;
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create FIFO task"));
    }

    mp_printf(&mp_plat_print, "pz_fifo: started (waveform=%u samples, gain=%d)\n",
              (unsigned)bufinfo.len, gain);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_fifo_start_obj, 2, pz_fifo_start);

// pz_fifo.stop()
static mp_obj_t pz_fifo_stop(void) {
    if (!s_state.running) {
        return mp_const_none;
    }

    // Signal the task to stop
    s_state.running = false;

    // Wait for the task to exit (it sets task_handle = NULL before deleting)
    while (s_state.task_handle != NULL) {
        TickType_t t = pdMS_TO_TICKS(10);
        vTaskDelay(t > 0 ? t : 1);
    }

    // Remove the DRV2665 device from the bus
    if (s_state.dev != NULL) {
        i2c_master_bus_rm_device(s_state.dev);
        s_state.dev = NULL;
    }

    // Clear waveform pointers (Python owns the buffer)
    s_state.waveform_buf = NULL;
    s_state.waveform_len = 0;
    s_state.write_index = 0;

    mp_printf(&mp_plat_print, "pz_fifo: stopped\n");

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_fifo_stop_obj, pz_fifo_stop);

// pz_fifo.is_running()
static mp_obj_t pz_fifo_is_running(void) {
    return mp_obj_new_bool(s_state.running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_fifo_is_running_obj, pz_fifo_is_running);

// ─── Module registration ─────────────────────────────────────────────────────

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
