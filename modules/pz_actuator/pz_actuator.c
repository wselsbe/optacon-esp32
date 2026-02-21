#include "py/runtime.h"
#include "py/obj.h"
#include "py/objlist.h"
#include "task.h"

static pz_task_state_t g_state = {0};
static bool g_initialized = false;

// ─── 1. init() ───────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_init(void) {
    if (g_initialized) return mp_const_none;

    esp_err_t err = drv2665_init(&g_state.drv);
    if (err != ESP_OK) mp_raise_OSError(err);

    err = shift_register_init(&g_state.sr);
    if (err != ESP_OK) mp_raise_OSError(err);

    waveform_init(&g_state.waveform, 250);  // default 250Hz

    err = drv2665_enable_digital(&g_state.drv, DRV2665_GAIN_100V);  // default 100Vpp
    if (err != ESP_OK) mp_raise_OSError(err);

    g_initialized = true;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_init_obj, pz_actuator_init);

// ─── 2. start() ──────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_start(void) {
    if (!g_initialized) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("not initialized"));
    esp_err_t err = pz_task_start(&g_state);
    if (err != ESP_OK) mp_raise_OSError(err);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_start_obj, pz_actuator_start);

// ─── 3. stop() ───────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_stop(void) {
    pz_task_stop(&g_state);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_stop_obj, pz_actuator_stop);

// ─── 4. set_frequency(hz) ────────────────────────────────────────────────────

static mp_obj_t pz_actuator_set_frequency(mp_obj_t hz_obj) {
    uint16_t hz = mp_obj_get_int(hz_obj);
    if (hz < 50 || hz > 4000) {
        mp_raise_ValueError(MP_ERROR_TEXT("frequency must be 50-4000 Hz"));
    }
    if (g_state.running) {
        g_state.target_frequency = hz;
        g_state.frequency_changed = true;
    } else {
        waveform_set_frequency(&g_state.waveform, hz);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_frequency_obj, pz_actuator_set_frequency);

// ─── 5. get_frequency() ──────────────────────────────────────────────────────

static mp_obj_t pz_actuator_get_frequency(void) {
    return mp_obj_new_int(g_state.waveform.frequency);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_get_frequency_obj, pz_actuator_get_frequency);

// ─── 6. set_pin(pin, value, flush=True) ──────────────────────────────────────

static mp_obj_t pz_actuator_set_pin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_pin, ARG_value, ARG_flush };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin,   MP_ARG_REQUIRED | MP_ARG_INT, {0} },
        { MP_QSTR_value, MP_ARG_REQUIRED | MP_ARG_BOOL, {0} },
        { MP_QSTR_flush, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
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
        shift_register_request_commit(&g_state.sr);
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
        { MP_QSTR_pins,  MP_ARG_REQUIRED | MP_ARG_OBJ, {0} },
        { MP_QSTR_flush, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
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
        shift_register_request_commit(&g_state.sr);
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
        { MP_QSTR_value, MP_ARG_REQUIRED | MP_ARG_BOOL, {0} },
        { MP_QSTR_flush, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    shift_register_set_all(&g_state.sr, args[ARG_value].u_bool);
    if (args[ARG_flush].u_bool) {
        shift_register_request_commit(&g_state.sr);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_actuator_set_all_obj, 1, pz_actuator_set_all);

// ─── 11. flush() ─────────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_flush(void) {
    shift_register_request_commit(&g_state.sr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_flush_obj, pz_actuator_flush);

// ─── 12. toggle_polarity() ───────────────────────────────────────────────────

static mp_obj_t pz_actuator_toggle_polarity(void) {
    shift_register_request_polarity_toggle(&g_state.sr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_toggle_polarity_obj, pz_actuator_toggle_polarity);

// ─── 13. get_polarity() ──────────────────────────────────────────────────────

static mp_obj_t pz_actuator_get_polarity(void) {
    return mp_obj_new_bool(g_state.sr.polarity);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_get_polarity_obj, pz_actuator_get_polarity);

// ─── 14. set_gain(gain) ──────────────────────────────────────────────────────

static mp_obj_t pz_actuator_set_gain(mp_obj_t gain_obj) {
    int gain = mp_obj_get_int(gain_obj);
    uint8_t gain_val;
    switch (gain) {
        case 25:  gain_val = DRV2665_GAIN_25V; break;
        case 50:  gain_val = DRV2665_GAIN_50V; break;
        case 75:  gain_val = DRV2665_GAIN_75V; break;
        case 100: gain_val = DRV2665_GAIN_100V; break;
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("gain must be 25, 50, 75, or 100"));
    }
    esp_err_t err = drv2665_write_register(&g_state.drv, DRV2665_REG_CTRL1,
                                            DRV2665_INPUT_DIGITAL | gain_val);
    if (err != ESP_OK) mp_raise_OSError(err);
    g_state.drv.gain = gain_val;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_set_gain_obj, pz_actuator_set_gain);

// ─── 15. is_running() ────────────────────────────────────────────────────────

static mp_obj_t pz_actuator_is_running(void) {
    return mp_obj_new_bool(g_state.running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_is_running_obj, pz_actuator_is_running);

// ─── Module globals table ────────────────────────────────────────────────────

static const mp_rom_map_elem_t pz_actuator_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),         MP_OBJ_NEW_QSTR(MP_QSTR_pz_actuator) },
    { MP_ROM_QSTR(MP_QSTR_init),                  MP_ROM_PTR(&pz_actuator_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),                 MP_ROM_PTR(&pz_actuator_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),                  MP_ROM_PTR(&pz_actuator_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_frequency),         MP_ROM_PTR(&pz_actuator_set_frequency_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_frequency),         MP_ROM_PTR(&pz_actuator_get_frequency_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pin),               MP_ROM_PTR(&pz_actuator_set_pin_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pin),               MP_ROM_PTR(&pz_actuator_get_pin_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pins),              MP_ROM_PTR(&pz_actuator_set_pins_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_all),               MP_ROM_PTR(&pz_actuator_get_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_all),               MP_ROM_PTR(&pz_actuator_set_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush),                 MP_ROM_PTR(&pz_actuator_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_toggle_polarity),       MP_ROM_PTR(&pz_actuator_toggle_polarity_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_polarity),          MP_ROM_PTR(&pz_actuator_get_polarity_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gain),              MP_ROM_PTR(&pz_actuator_set_gain_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_running),            MP_ROM_PTR(&pz_actuator_is_running_obj) },
};
static MP_DEFINE_CONST_DICT(pz_actuator_module_globals, pz_actuator_module_globals_table);

const mp_obj_module_t pz_actuator_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pz_actuator_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_actuator, pz_actuator_module);
