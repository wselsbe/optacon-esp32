#include "py/runtime.h"
#include "py/obj.h"
#include "py/objlist.h"
#include "task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include <stdlib.h>
#include <string.h>
#include "sine.h"
#include "pwm.h"

// Defined in ports/esp32/usb.c — switches USB PHY from OTG to Serial/JTAG
extern void usb_usj_mode(void);

static pz_task_state_t g_state = {0};
static bool g_initialized = false;

typedef enum { DRV_MODE_NONE, DRV_MODE_ANALOG, DRV_MODE_DIGITAL } drv_mode_t;
static drv_mode_t g_mode = DRV_MODE_NONE;
static uint16_t g_freq_hz = 0;

// ─── 1. init() ───────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_init(void) {
    if (g_initialized) return mp_const_none;

    mp_printf(&mp_plat_print, "pz_actuator: initializing DRV2665 (I2C)...\n");
    esp_err_t err = drv2665_init(&g_state.drv);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_actuator: DRV2665 init failed: %d (0x%03x)\n", err, err);
        mp_raise_OSError(err);
    }

    mp_printf(&mp_plat_print, "pz_actuator: initializing shift register (SPI)...\n");
    err = shift_register_init(&g_state.sr);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_actuator: shift register init failed: %d (0x%03x)\n", err,
                  err);
        drv2665_deinit(&g_state.drv);
        mp_raise_OSError(err);
    }

    mp_printf(&mp_plat_print, "pz_actuator: initializing PWM...\n");
    err = pwm_init();
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_actuator: PWM init failed: %d (0x%03x)\n", err, err);
        shift_register_deinit(&g_state.sr);
        drv2665_deinit(&g_state.drv);
        mp_raise_OSError(err);
    }

    mp_printf(&mp_plat_print, "pz_actuator: initialized successfully\n");
    g_initialized = true;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_init_obj, pz_actuator_init);

// ─── 2. start(gain=100) ──────────────────────────────────────────────────────

static mp_obj_t pz_actuator_start(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_gain };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_gain, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 100}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));
    if (g_mode == DRV_MODE_NONE)
        mp_raise_msg(&mp_type_RuntimeError,
                     MP_ERROR_TEXT("call set_frequency_analog() or set_frequency_digital() first"));

    // Resolve gain
    uint8_t gain_val;
    switch (args[ARG_gain].u_int) {
    case 25:
        gain_val = DRV2665_GAIN_25V;
        break;
    case 50:
        gain_val = DRV2665_GAIN_50V;
        break;
    case 75:
        gain_val = DRV2665_GAIN_75V;
        break;
    case 100:
        gain_val = DRV2665_GAIN_100V;
        break;
    default:
        mp_raise_ValueError(MP_ERROR_TEXT("gain must be 25, 50, 75, or 100"));
    }
    g_state.drv.gain = gain_val;

    esp_err_t err;
    if (g_mode == DRV_MODE_ANALOG) {
        err = drv2665_enable_analog(&g_state.drv, gain_val);
        if (err != ESP_OK) mp_raise_OSError(err);
        err = pwm_start_sine(g_freq_hz);
        if (err != ESP_OK) mp_raise_OSError(err);
    } else {
        err = drv2665_enable_digital(&g_state.drv, gain_val);
        if (err != ESP_OK) mp_raise_OSError(err);
        err = pz_task_start(&g_state);
        if (err != ESP_OK) mp_raise_OSError(err);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_actuator_start_obj, 0, pz_actuator_start);

// ─── 3. stop() ───────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_stop(void) {
    if (g_state.running) pz_task_stop(&g_state);
    if (pwm_is_running()) pwm_stop();
    drv2665_standby(&g_state.drv);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_stop_obj, pz_actuator_stop);

// ─── set_frequency_digital(hz) ───────────────────────────────────────────────

static mp_obj_t pz_actuator_set_frequency_digital(mp_obj_t freq_obj) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));

    int freq_hz = mp_obj_get_int(freq_obj);
    if (freq_hz < 50 || freq_hz > 4000) {
        mp_raise_ValueError(MP_ERROR_TEXT("frequency must be 50-4000 Hz"));
    }

    // Stop if running (digital FIFO task or analog PWM)
    if (g_state.running) {
        pz_task_stop(&g_state);
    }
    pwm_stop();

    // Free previous internal buffer
    if (g_state.internal_sine_buf != NULL) {
        free(g_state.internal_sine_buf);
        g_state.internal_sine_buf = NULL;
    }

    // Allocate and generate sine waveform
    size_t max_period = DRV2665_SAMPLE_RATE / freq_hz + 1;
    g_state.internal_sine_buf = (int8_t *)malloc(max_period);
    if (g_state.internal_sine_buf == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("failed to allocate sine buffer"));
    }

    g_state.internal_sine_len =
        sine_generate_digital(g_state.internal_sine_buf, max_period, freq_hz, DRV2665_SAMPLE_RATE);

    // Point waveform_buf at the internal buffer
    g_state.waveform_buf = g_state.internal_sine_buf;
    g_state.waveform_len = g_state.internal_sine_len;
    g_state.write_index = 0;

    g_mode = DRV_MODE_DIGITAL;
    g_freq_hz = (uint16_t)freq_hz;

    mp_printf(&mp_plat_print, "set_frequency_digital: %dHz, %d samples/period\n", freq_hz,
              (int)g_state.waveform_len);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_frequency_digital_obj,
                                 pz_actuator_set_frequency_digital);

// ─── set_frequency_analog(hz, resolution=8) ─────────────────────────────────

static mp_obj_t pz_actuator_set_frequency_analog(size_t n_args, const mp_obj_t *pos_args,
                                                 mp_map_t *kw_args) {
    enum { ARG_hz, ARG_resolution };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_hz, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_resolution, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));

    int freq_hz = args[ARG_hz].u_int;
    int resolution = args[ARG_resolution].u_int;

    if (resolution != 8 && resolution != 10) {
        mp_raise_ValueError(MP_ERROR_TEXT("resolution must be 8 or 10"));
    }

    // Stop if running
    if (g_state.running) pz_task_stop(&g_state);
    if (pwm_is_running()) pwm_stop();

    // Configure PWM resolution
    esp_err_t err = pwm_set_resolution((uint8_t)resolution);
    if (err != ESP_OK) mp_raise_OSError(err);

    g_mode = DRV_MODE_ANALOG;
    g_freq_hz = (uint16_t)freq_hz;

    if (freq_hz == 0) {
        mp_printf(&mp_plat_print, "set_frequency_analog: DC mode\n");
    } else {
        mp_printf(&mp_plat_print, "set_frequency_analog: %dHz, %d-bit\n", freq_hz, resolution);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_actuator_set_frequency_analog_obj, 1,
                                  pz_actuator_set_frequency_analog);

// ─── 6. set_pin(pin, value, flush=True) ──────────────────────────────────────

static mp_obj_t pz_actuator_set_pin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_pin, ARG_value, ARG_flush };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_pin, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_value, MP_ARG_REQUIRED | MP_ARG_BOOL, {0}},
        {MP_QSTR_flush, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t pin = args[ARG_pin].u_int;
    bool value = args[ARG_value].u_bool;
    bool flush = args[ARG_flush].u_bool;

    if (pin >= SHIFTREG_NUM_PINS) {
        mp_raise_ValueError(MP_ERROR_TEXT("pin must be 0-19"));
    }

    shift_register_set_pin(&g_state.sr, pin, value);
    if (flush) {
        shift_register_commit(&g_state.sr);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_actuator_set_pin_obj, 2, pz_actuator_set_pin);

// ─── 7. get_pin(pin) ─────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_get_pin(mp_obj_t pin_obj) {
    uint8_t pin = mp_obj_get_int(pin_obj);
    if (pin >= SHIFTREG_NUM_PINS) {
        mp_raise_ValueError(MP_ERROR_TEXT("pin must be 0-19"));
    }
    return mp_obj_new_bool(shift_register_get_pin(&g_state.sr, pin));
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_get_pin_obj, pz_actuator_get_pin);

// ─── 8. set_pins(list, flush=True) ───────────────────────────────────────────

static mp_obj_t pz_actuator_set_pins(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_pins, ARG_flush };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_pins, MP_ARG_REQUIRED | MP_ARG_OBJ, {0}},
        {MP_QSTR_flush, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t pins_obj = args[ARG_pins].u_obj;
    bool flush = args[ARG_flush].u_bool;

    // Get list/tuple items
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(pins_obj, &len, &items);

    if (len != SHIFTREG_NUM_PINS) {
        mp_raise_ValueError(MP_ERROR_TEXT("pins must be a list of 20 bools"));
    }

    bool values[SHIFTREG_NUM_PINS];
    for (size_t i = 0; i < SHIFTREG_NUM_PINS; i++) {
        values[i] = mp_obj_is_true(items[i]);
    }
    shift_register_set_pins(&g_state.sr, values);

    if (flush) {
        shift_register_commit(&g_state.sr);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_actuator_set_pins_obj, 1, pz_actuator_set_pins);

// ─── 9. get_all() ────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_get_all(void) {
    mp_obj_t items[SHIFTREG_NUM_PINS];
    for (uint8_t i = 0; i < SHIFTREG_NUM_PINS; i++) {
        items[i] = mp_obj_new_bool(shift_register_get_pin(&g_state.sr, i));
    }
    return mp_obj_new_tuple(SHIFTREG_NUM_PINS, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_get_all_obj, pz_actuator_get_all);

// ─── 10. set_all(value, flush=True) ──────────────────────────────────────────

static mp_obj_t pz_actuator_set_all(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_value, ARG_flush };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_value, MP_ARG_REQUIRED | MP_ARG_BOOL, {0}},
        {MP_QSTR_flush, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    shift_register_set_all(&g_state.sr, args[ARG_value].u_bool);
    if (args[ARG_flush].u_bool) {
        shift_register_commit(&g_state.sr);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_actuator_set_all_obj, 1, pz_actuator_set_all);

// ─── 11. flush() ─────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_flush(void) {
    shift_register_commit(&g_state.sr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_flush_obj, pz_actuator_flush);

// ─── 12. toggle_polarity() ───────────────────────────────────────────────────

static mp_obj_t pz_actuator_toggle_polarity(void) {
    shift_register_request_polarity_toggle(&g_state.sr);
    shift_register_commit(&g_state.sr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_toggle_polarity_obj, pz_actuator_toggle_polarity);

// ─── 13. get_polarity() ──────────────────────────────────────────────────────

static mp_obj_t pz_actuator_get_polarity(void) {
    return mp_obj_new_bool(g_state.sr.polarity);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_get_polarity_obj, pz_actuator_get_polarity);

// set_gain() removed — gain is now a kwarg of start()

// ─── 15. is_running() ────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_is_running(void) {
    return mp_obj_new_bool(g_state.running || pwm_is_running());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_is_running_obj, pz_actuator_is_running);

// ─── 16. set_sync_trough(value) ──────────────────────────────────────────────

static mp_obj_t pz_actuator_set_sync_trough(mp_obj_t val_obj) {
    g_state.sync_trough = mp_obj_is_true(val_obj);
    mp_printf(&mp_plat_print, "sync_trough = %s\n", g_state.sync_trough ? "True" : "False");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_sync_trough_obj, pz_actuator_set_sync_trough);

// ─── 17. test_i2c() ──────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_test_i2c(void) {
    mp_printf(&mp_plat_print, "=== I2C diagnostic (native) ===\n");

    drv2665_t dev = {0};
    esp_err_t err = drv2665_init(&dev);
    mp_printf(&mp_plat_print, "  init: err=%d\n", err);
    if (err == ESP_OK) {
        uint8_t status;
        err = drv2665_read_register(&dev, DRV2665_REG_STATUS, &status);
        mp_printf(&mp_plat_print, "  read STATUS: err=%d val=0x%02x\n", err, status);
        drv2665_deinit(&dev);
    }

    mp_printf(&mp_plat_print, "=== done ===\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_test_i2c_obj, pz_actuator_test_i2c);

// ─── I2C test helpers ────────────────────────────────────────────────────────

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} test_i2c_ctx_t;

static esp_err_t test_i2c_setup(test_i2c_ctx_t *ctx) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = DRV2665_I2C_SDA_PIN,
        .scl_io_num = DRV2665_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &ctx->bus);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "  setup: bus create failed err=%d\n", err);
        return err;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2665_I2C_ADDR,
        .scl_speed_hz = DRV2665_I2C_CLK_HZ,
    };
    err = i2c_master_bus_add_device(ctx->bus, &dev_cfg, &ctx->dev);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "  setup: add device failed err=%d\n", err);
        i2c_del_master_bus(ctx->bus);
        return err;
    }
    mp_printf(&mp_plat_print, "  setup: OK (native i2c, sda=%d, scl=%d)\n", DRV2665_I2C_SDA_PIN,
              DRV2665_I2C_SCL_PIN);
    return ESP_OK;
}

static void test_i2c_cleanup(test_i2c_ctx_t *ctx) {
    if (ctx->dev) i2c_master_bus_rm_device(ctx->dev);
    if (ctx->bus) i2c_del_master_bus(ctx->bus);
    mp_printf(&mp_plat_print, "  cleanup: done\n");
}

// Helper: read + log a register
static esp_err_t test_read_reg(test_i2c_ctx_t *ctx, uint8_t reg, const char *label) {
    uint8_t val = 0xFF;
    esp_err_t err = i2c_master_transmit_receive(ctx->dev, &reg, 1, &val, 1, 100);
    mp_printf(&mp_plat_print, "  %s: reg=0x%02x err=%d val=0x%02x\n", label, reg, err, val);
    return err;
}

// Helper: write + log a register
static esp_err_t test_write_reg(test_i2c_ctx_t *ctx, uint8_t reg, uint8_t val, const char *label) {
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(ctx->dev, buf, 2, 100);
    mp_printf(&mp_plat_print, "  %s: reg=0x%02x val=0x%02x err=%d\n", label, reg, val, err);
    return err;
}

// Helper: delay with minimum 1 tick
static void test_delay_ms(uint32_t ms) {
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0) ticks = 1;
    mp_printf(&mp_plat_print, "  delay: %lums (%lu ticks)\n", (unsigned long)ms,
              (unsigned long)ticks);
    vTaskDelay(ticks);
}

// ─── read_reg(reg) / write_reg(reg, val) — debug DRV2665 registers ───────────

static mp_obj_t pz_actuator_read_reg(mp_obj_t reg_obj) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));
    uint8_t reg = mp_obj_get_int(reg_obj);
    uint8_t val;
    esp_err_t err = drv2665_read_register(&g_state.drv, reg, &val);
    if (err != ESP_OK) mp_raise_OSError(err);
    return mp_obj_new_int(val);
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_read_reg_obj, pz_actuator_read_reg);

static mp_obj_t pz_actuator_write_reg(mp_obj_t reg_obj, mp_obj_t val_obj) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));
    uint8_t reg = mp_obj_get_int(reg_obj);
    uint8_t val = mp_obj_get_int(val_obj);
    esp_err_t err = drv2665_write_register(&g_state.drv, reg, val);
    if (err != ESP_OK) mp_raise_OSError(err);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(pz_actuator_write_reg_obj, pz_actuator_write_reg);

// ─── 18. reset_drv() ─────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_reset_drv(void) {
    test_i2c_ctx_t ctx = {0};
    esp_err_t err = test_i2c_setup(&ctx);
    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C setup failed"));
    }

    uint8_t buf[2] = {DRV2665_REG_CTRL2, DRV2665_RESET};
    err = i2c_master_transmit(ctx.dev, buf, 2, 100);
    test_i2c_cleanup(&ctx);

    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("DRV2665 reset failed"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_reset_drv_obj, pz_actuator_reset_drv);

// ─── 19. test_init_no_reset() ────────────────────────────────────────────────

static mp_obj_t pz_actuator_test_init_no_reset(void) {
    mp_printf(&mp_plat_print, "=== test_init_no_reset (baseline) ===\n");

    test_i2c_ctx_t ctx = {0};
    if (test_i2c_setup(&ctx) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C setup failed"));
    }

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS pre");
    test_read_reg(&ctx, DRV2665_REG_CTRL1, "CTRL1 default");
    test_read_reg(&ctx, DRV2665_REG_CTRL2, "CTRL2 default");

    test_write_reg(&ctx, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | DRV2665_GAIN_100V,
                   "write CTRL1");
    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS,
                   "write CTRL2");

    test_delay_ms(2);

    test_read_reg(&ctx, DRV2665_REG_CTRL1, "CTRL1 readback");
    test_read_reg(&ctx, DRV2665_REG_CTRL2, "CTRL2 readback");
    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS post");

    test_i2c_cleanup(&ctx);
    mp_printf(&mp_plat_print, "=== done ===\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_test_init_no_reset_obj,
                                 pz_actuator_test_init_no_reset);

// ─── 19. test_init_reset_no_delay() ──────────────────────────────────────────

static mp_obj_t pz_actuator_test_init_reset_no_delay(void) {
    mp_printf(&mp_plat_print, "=== test_init_reset_no_delay ===\n");

    test_i2c_ctx_t ctx = {0};
    if (test_i2c_setup(&ctx) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C setup failed"));
    }

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS pre-reset");

    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_RESET, "write DEV_RST");

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS immed after reset");

    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS,
                   "write CTRL2");
    test_write_reg(&ctx, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | DRV2665_GAIN_100V,
                   "write CTRL1");

    test_delay_ms(2);

    test_read_reg(&ctx, DRV2665_REG_CTRL1, "CTRL1 readback");
    test_read_reg(&ctx, DRV2665_REG_CTRL2, "CTRL2 readback");
    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS post");

    test_i2c_cleanup(&ctx);
    mp_printf(&mp_plat_print, "=== done ===\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_test_init_reset_no_delay_obj,
                                 pz_actuator_test_init_reset_no_delay);

// ─── 20. test_init_reset_5ms() ───────────────────────────────────────────────

static mp_obj_t pz_actuator_test_init_reset_5ms(void) {
    mp_printf(&mp_plat_print, "=== test_init_reset_5ms ===\n");

    test_i2c_ctx_t ctx = {0};
    if (test_i2c_setup(&ctx) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C setup failed"));
    }

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS pre-reset");

    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_RESET, "write DEV_RST");

    test_delay_ms(5);

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS after 5ms");

    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS,
                   "write CTRL2");
    test_write_reg(&ctx, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | DRV2665_GAIN_100V,
                   "write CTRL1");

    test_delay_ms(2);

    test_read_reg(&ctx, DRV2665_REG_CTRL1, "CTRL1 readback");
    test_read_reg(&ctx, DRV2665_REG_CTRL2, "CTRL2 readback");
    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS post");

    test_i2c_cleanup(&ctx);
    mp_printf(&mp_plat_print, "=== done ===\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_test_init_reset_5ms_obj,
                                 pz_actuator_test_init_reset_5ms);

// ─── 21. test_init_reset_20ms() ──────────────────────────────────────────────

static mp_obj_t pz_actuator_test_init_reset_20ms(void) {
    mp_printf(&mp_plat_print, "=== test_init_reset_20ms ===\n");

    test_i2c_ctx_t ctx = {0};
    if (test_i2c_setup(&ctx) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C setup failed"));
    }

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS pre-reset");

    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_RESET, "write DEV_RST");

    test_delay_ms(20);

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS after 20ms");

    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS,
                   "write CTRL2");
    test_write_reg(&ctx, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | DRV2665_GAIN_100V,
                   "write CTRL1");

    test_delay_ms(2);

    test_read_reg(&ctx, DRV2665_REG_CTRL1, "CTRL1 readback");
    test_read_reg(&ctx, DRV2665_REG_CTRL2, "CTRL2 readback");
    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS post");

    test_i2c_cleanup(&ctx);
    mp_printf(&mp_plat_print, "=== done ===\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_test_init_reset_20ms_obj,
                                 pz_actuator_test_init_reset_20ms);

// ─── 22. test_fifo_nack() ─────────────────────────────────────────────────────
// Test: does DRV2665 NACK when FIFO is full?

static mp_obj_t pz_actuator_test_fifo_nack(void) {
    mp_printf(&mp_plat_print, "=== test_fifo_nack ===\n");

    test_i2c_ctx_t ctx = {0};
    if (test_i2c_setup(&ctx) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C setup failed"));
    }

    // Reset
    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_RESET, "reset");
    test_delay_ms(5);

    // Enable digital mode
    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS,
                   "enable");
    test_delay_ms(5);
    test_write_reg(&ctx, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | DRV2665_GAIN_100V,
                   "digital+gain");

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS before fill");

    // Fill FIFO with 100 bytes (bulk write)
    uint8_t buf[101];
    buf[0] = DRV2665_REG_DATA;
    for (int i = 1; i <= 100; i++)
        buf[i] = 0x40;
    esp_err_t err = i2c_master_transmit(ctx.dev, buf, 101, 100);
    mp_printf(&mp_plat_print, "  bulk 100 bytes: err=%d\n", err);

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS after 100");

    // Now try writing single bytes past FIFO capacity
    for (int i = 0; i < 5; i++) {
        uint8_t single[2] = {DRV2665_REG_DATA, 0x40};
        err = i2c_master_transmit(ctx.dev, single, 2, 100);
        mp_printf(&mp_plat_print, "  byte %d past full: err=%d (0x%03x)\n", i + 1, err, err);
    }

    test_read_reg(&ctx, DRV2665_REG_STATUS, "STATUS final");

    // Standby
    test_write_reg(&ctx, DRV2665_REG_CTRL2, DRV2665_STANDBY, "standby");

    test_i2c_cleanup(&ctx);
    mp_printf(&mp_plat_print, "=== done ===\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_test_fifo_nack_obj, pz_actuator_test_fifo_nack);

// ─── 23. enter_bootloader() ───────────────────────────────────────────────────

static mp_obj_t pz_actuator_enter_bootloader(void) {
    mp_printf(&mp_plat_print, "Entering bootloader...\n");

    if (tud_connected()) {
        tud_disconnect();
    }
    esp_rom_delay_us(100000); // 100ms for host to process disconnect

    // Switch USB PHY from OTG to Serial/JTAG so ROM uses same path as physical BOOT+RST
    usb_usj_mode();

    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);

    esp_restart();
    return mp_const_none; // never reached
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_enter_bootloader_obj, pz_actuator_enter_bootloader);

// ─── Module globals table ────────────────────────────────────────────────────

static const mp_rom_map_elem_t pz_actuator_module_globals_table[] = {
    {MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_pz_actuator)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&pz_actuator_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&pz_actuator_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&pz_actuator_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_pin), MP_ROM_PTR(&pz_actuator_set_pin_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_pin), MP_ROM_PTR(&pz_actuator_get_pin_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_pins), MP_ROM_PTR(&pz_actuator_set_pins_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_all), MP_ROM_PTR(&pz_actuator_get_all_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_all), MP_ROM_PTR(&pz_actuator_set_all_obj)},
    {MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&pz_actuator_flush_obj)},
    {MP_ROM_QSTR(MP_QSTR_toggle_polarity), MP_ROM_PTR(&pz_actuator_toggle_polarity_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_polarity), MP_ROM_PTR(&pz_actuator_get_polarity_obj)},
    {MP_ROM_QSTR(MP_QSTR_is_running), MP_ROM_PTR(&pz_actuator_is_running_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_sync_trough), MP_ROM_PTR(&pz_actuator_set_sync_trough_obj)},
    {MP_ROM_QSTR(MP_QSTR_reset_drv), MP_ROM_PTR(&pz_actuator_reset_drv_obj)},
    {MP_ROM_QSTR(MP_QSTR_test_i2c), MP_ROM_PTR(&pz_actuator_test_i2c_obj)},
    {MP_ROM_QSTR(MP_QSTR_test_init_no_reset), MP_ROM_PTR(&pz_actuator_test_init_no_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_test_init_reset_no_delay),
     MP_ROM_PTR(&pz_actuator_test_init_reset_no_delay_obj)},
    {MP_ROM_QSTR(MP_QSTR_test_init_reset_5ms), MP_ROM_PTR(&pz_actuator_test_init_reset_5ms_obj)},
    {MP_ROM_QSTR(MP_QSTR_test_init_reset_20ms), MP_ROM_PTR(&pz_actuator_test_init_reset_20ms_obj)},
    {MP_ROM_QSTR(MP_QSTR_test_fifo_nack), MP_ROM_PTR(&pz_actuator_test_fifo_nack_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_frequency_digital),
     MP_ROM_PTR(&pz_actuator_set_frequency_digital_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_frequency_analog), MP_ROM_PTR(&pz_actuator_set_frequency_analog_obj)},
    {MP_ROM_QSTR(MP_QSTR_read_reg), MP_ROM_PTR(&pz_actuator_read_reg_obj)},
    {MP_ROM_QSTR(MP_QSTR_write_reg), MP_ROM_PTR(&pz_actuator_write_reg_obj)},
    {MP_ROM_QSTR(MP_QSTR_enter_bootloader), MP_ROM_PTR(&pz_actuator_enter_bootloader_obj)},
};
static MP_DEFINE_CONST_DICT(pz_actuator_module_globals, pz_actuator_module_globals_table);

const mp_obj_module_t pz_actuator_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&pz_actuator_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_actuator, pz_actuator_module);
