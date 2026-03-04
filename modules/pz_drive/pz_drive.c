// modules/pz_drive/pz_drive.c
#include "py/runtime.h"
#include "py/obj.h"
#include "pz_drive.h"

// ── sr_stage(word32) ────────────────────────────────────────────────────
static mp_obj_t pz_drive_sr_stage(mp_obj_t word_obj) {
    uint32_t word32 = mp_obj_get_int(word_obj);
    hv509_sr_stage(word32);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_drive_sr_stage_obj, pz_drive_sr_stage);

// ── sr_write(word32) ────────────────────────────────────────────────────
static mp_obj_t pz_drive_sr_write(mp_obj_t word_obj) {
    uint32_t word32 = mp_obj_get_int(word_obj);
    hv509_sr_write(word32);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_drive_sr_write_obj, pz_drive_sr_write);

// ── pol_init() ──────────────────────────────────────────────────────────
static mp_obj_t pz_drive_pol_init(void) {
    hv509_pol_init();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pol_init_obj, pz_drive_pol_init);

// ── pol_set(val) ────────────────────────────────────────────────────────
static mp_obj_t pz_drive_pol_set(mp_obj_t val_obj) {
    hv509_pol_set(mp_obj_is_true(val_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_drive_pol_set_obj, pz_drive_pol_set);

// ── pol_get() ───────────────────────────────────────────────────────────
static mp_obj_t pz_drive_pol_get(void) {
    return mp_obj_new_bool(hv509_pol_get());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pol_get_obj, pz_drive_pol_get);

// ── reset() ─────────────────────────────────────────────────────────────
static mp_obj_t pz_drive_reset(void) {
    drv2665_reset();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_reset_obj, pz_drive_reset);

// ── i2c_read(reg) ───────────────────────────────────────────────────────
static mp_obj_t pz_drive_i2c_read(mp_obj_t reg_obj) {
    uint8_t reg = mp_obj_get_int(reg_obj);
    int val = drv2665_read_reg(reg);
    return mp_obj_new_int(val);
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_drive_i2c_read_obj, pz_drive_i2c_read);

// ── i2c_write(reg, val) ────────────────────────────────────────────────
static mp_obj_t pz_drive_i2c_write(mp_obj_t reg_obj, mp_obj_t val_obj) {
    uint8_t reg = mp_obj_get_int(reg_obj);
    uint8_t val = mp_obj_get_int(val_obj);
    drv2665_write_reg(reg, val);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(pz_drive_i2c_write_obj, pz_drive_i2c_write);

// ── i2c_write_bytes(reg, data) → esp_err_t ───────────────────────────
static mp_obj_t pz_drive_i2c_write_bytes(mp_obj_t reg_obj, mp_obj_t data_obj) {
    uint8_t reg = mp_obj_get_int(reg_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    esp_err_t err = drv2665_write_bulk(reg, bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(err);
}
static MP_DEFINE_CONST_FUN_OBJ_2(pz_drive_i2c_write_bytes_obj, pz_drive_i2c_write_bytes);

// ── i2c_read_bytes(reg, count) → bytes ──────────────────────────────
static mp_obj_t pz_drive_i2c_read_bytes(mp_obj_t reg_obj, mp_obj_t count_obj) {
    uint8_t reg = mp_obj_get_int(reg_obj);
    int count = mp_obj_get_int(count_obj);
    if (count <= 0 || count > 256) {
        mp_raise_ValueError(MP_ERROR_TEXT("count must be 1-256"));
    }
    uint8_t buf[256];
    esp_err_t err = drv2665_read_bulk(reg, buf, count);
    if (err != 0) {
        return mp_obj_new_int(-1);
    }
    return mp_obj_new_bytes(buf, count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(pz_drive_i2c_read_bytes_obj, pz_drive_i2c_read_bytes);

// ── pwm_set_frequency(hz, resolution=8, amplitude=128, fullwave=False,
//                      dead_time=0, phase_advance=0, waveform=0) ────────
static mp_obj_t pz_drive_pwm_set_frequency(size_t n_args, const mp_obj_t *pos_args,
                                           mp_map_t *kw_args) {
    enum {
        ARG_hz,
        ARG_resolution,
        ARG_amplitude,
        ARG_fullwave,
        ARG_dead_time,
        ARG_phase_advance,
        ARG_waveform
    };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_hz, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_resolution, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8}},
        {MP_QSTR_amplitude, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_fullwave, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_dead_time, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_phase_advance, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_waveform, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pzd_pwm_set_frequency(args[ARG_hz].u_int, args[ARG_resolution].u_int, args[ARG_amplitude].u_int,
                          args[ARG_fullwave].u_bool, args[ARG_dead_time].u_int,
                          args[ARG_phase_advance].u_int, args[ARG_waveform].u_int);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_set_frequency_obj, 1, pz_drive_pwm_set_frequency);

// ── pwm_start() ────────────────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_start(void) {
    pzd_pwm_start();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_start_obj, pz_drive_pwm_start);

// ── pwm_stop() ─────────────────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_stop(void) {
    pzd_pwm_stop();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_stop_obj, pz_drive_pwm_stop);

// ── pwm_is_running() ──────────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_is_running(void) {
    return mp_obj_new_bool(pzd_pwm_is_running());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_is_running_obj, pz_drive_pwm_is_running);

// ── fifo_start(waveform_buf, gain=3, fullwave=False) ────────────────────
static mp_obj_t pz_drive_fifo_start(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_waveform, ARG_gain, ARG_fullwave };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_waveform, MP_ARG_REQUIRED | MP_ARG_OBJ},
        {MP_QSTR_gain, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 3}},
        {MP_QSTR_fullwave, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_waveform].u_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("waveform must not be empty"));
    }
    int gain = args[ARG_gain].u_int;
    if (gain < 0 || gain > 3) {
        mp_raise_ValueError(MP_ERROR_TEXT("gain must be 0-3"));
    }

    pzd_fifo_start((const uint8_t *)bufinfo.buf, bufinfo.len, gain, args[ARG_fullwave].u_bool);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_fifo_start_obj, 1, pz_drive_fifo_start);

// ── fifo_stop() ─────────────────────────────────────────────────────────
static mp_obj_t pz_drive_fifo_stop(void) {
    pzd_fifo_stop();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_fifo_stop_obj, pz_drive_fifo_stop);

// ── fifo_is_running() ───────────────────────────────────────────────────
static mp_obj_t pz_drive_fifo_is_running(void) {
    return mp_obj_new_bool(pzd_fifo_is_running());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_fifo_is_running_obj, pz_drive_fifo_is_running);

// ── pwm_play_samples(buf, sample_rate, loop=False) ──────────────────────
static mp_obj_t pz_drive_pwm_play_samples(size_t n_args, const mp_obj_t *pos_args,
                                          mp_map_t *kw_args) {
    enum { ARG_buf, ARG_sample_rate, ARG_loop };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_buf, MP_ARG_REQUIRED | MP_ARG_OBJ},
        {MP_QSTR_sample_rate, MP_ARG_REQUIRED | MP_ARG_INT},
        {MP_QSTR_loop, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer must not be empty"));
    }
    uint32_t sample_rate = (uint32_t)args[ARG_sample_rate].u_int;
    if (sample_rate == 0 || sample_rate > 48000) {
        mp_raise_ValueError(MP_ERROR_TEXT("sample_rate must be 1-48000"));
    }

    pzd_pwm_play_samples((const uint8_t *)bufinfo.buf, bufinfo.len, sample_rate,
                         args[ARG_loop].u_bool);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_play_samples_obj, 2, pz_drive_pwm_play_samples);

// ── pwm_is_sample_done() ────────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_is_sample_done(void) {
    return mp_obj_new_bool(pzd_pwm_is_sample_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_is_sample_done_obj, pz_drive_pwm_is_sample_done);

// ── pwm_set_sweep(target_step, increment, logarithmic=False) ─────────
static mp_obj_t pz_drive_pwm_set_sweep(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_target_step, ARG_increment, ARG_logarithmic };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_target_step, MP_ARG_REQUIRED | MP_ARG_INT},
        {MP_QSTR_increment, MP_ARG_REQUIRED | MP_ARG_INT},
        {MP_QSTR_logarithmic, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pzd_pwm_set_sweep(args[ARG_target_step].u_int, args[ARG_increment].u_int,
                      args[ARG_logarithmic].u_bool);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_set_sweep_obj, 2, pz_drive_pwm_set_sweep);

// ── pwm_is_sweep_done() ─────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_is_sweep_done(void) {
    return mp_obj_new_bool(pzd_pwm_is_sweep_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_is_sweep_done_obj, pz_drive_pwm_is_sweep_done);

// ── Module table ────────────────────────────────────────────────────────
static const mp_rom_map_elem_t pz_drive_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pz_drive)},
    {MP_ROM_QSTR(MP_QSTR_sr_stage), MP_ROM_PTR(&pz_drive_sr_stage_obj)},
    {MP_ROM_QSTR(MP_QSTR_sr_write), MP_ROM_PTR(&pz_drive_sr_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_pol_init), MP_ROM_PTR(&pz_drive_pol_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_pol_set), MP_ROM_PTR(&pz_drive_pol_set_obj)},
    {MP_ROM_QSTR(MP_QSTR_pol_get), MP_ROM_PTR(&pz_drive_pol_get_obj)},
    {MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&pz_drive_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_i2c_read), MP_ROM_PTR(&pz_drive_i2c_read_obj)},
    {MP_ROM_QSTR(MP_QSTR_i2c_write), MP_ROM_PTR(&pz_drive_i2c_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_i2c_write_bytes), MP_ROM_PTR(&pz_drive_i2c_write_bytes_obj)},
    {MP_ROM_QSTR(MP_QSTR_i2c_read_bytes), MP_ROM_PTR(&pz_drive_i2c_read_bytes_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_set_frequency), MP_ROM_PTR(&pz_drive_pwm_set_frequency_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_start), MP_ROM_PTR(&pz_drive_pwm_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_stop), MP_ROM_PTR(&pz_drive_pwm_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_is_running), MP_ROM_PTR(&pz_drive_pwm_is_running_obj)},
    {MP_ROM_QSTR(MP_QSTR_fifo_start), MP_ROM_PTR(&pz_drive_fifo_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_fifo_stop), MP_ROM_PTR(&pz_drive_fifo_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_fifo_is_running), MP_ROM_PTR(&pz_drive_fifo_is_running_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_play_samples), MP_ROM_PTR(&pz_drive_pwm_play_samples_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_is_sample_done), MP_ROM_PTR(&pz_drive_pwm_is_sample_done_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_set_sweep), MP_ROM_PTR(&pz_drive_pwm_set_sweep_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_is_sweep_done), MP_ROM_PTR(&pz_drive_pwm_is_sweep_done_obj)},
};
static MP_DEFINE_CONST_DICT(pz_drive_module_globals, pz_drive_module_globals_table);

const mp_obj_module_t pz_drive_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&pz_drive_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_pz_drive, pz_drive_module);
