// modules/sam/mod_sam.c — MicroPython bindings for SAM speech synthesizer
#include <string.h>

#include "py/mphal.h"
#include "py/runtime.h"

#include "pz_drive.h"
#include "sam.h"

// Helper: run SAM synthesis with given parameters, returns PCM buffer info.
// Caller must free(buf) after use. Returns buf_len via out param.
static char *sam_synthesize(const char *text, unsigned char speed,
                            unsigned char pitch, unsigned char mouth,
                            unsigned char throat, int *out_len) {
    size_t len = strlen(text);
    if (len > 254) {
        mp_raise_ValueError(MP_ERROR_TEXT("text too long (max 254 chars)"));
    }
    char input[256];
    memcpy(input, text, len + 1);

    SetInput(input);
    SetSpeed(speed);
    SetPitch(pitch);
    SetMouth(mouth);
    SetThroat(throat);

    if (!SAMMain()) {
        mp_raise_ValueError(MP_ERROR_TEXT("SAM synthesis failed"));
    }

    char *buf = GetBuffer();
    int buf_len = GetBufferLength();

    if (buf == NULL || buf_len <= 0) {
        free(buf);
        mp_raise_ValueError(MP_ERROR_TEXT("SAM produced no output"));
    }

    *out_len = buf_len;
    return buf;
}

// sam.render(text, speed=72, pitch=64, mouth=128, throat=128)
// Returns bytearray of 8-bit unsigned PCM at 22050 Hz
static mp_obj_t mod_sam_render(size_t n_args, const mp_obj_t *pos_args,
                               mp_map_t *kw_args) {
    enum { ARG_text, ARG_speed, ARG_pitch, ARG_mouth, ARG_throat };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_text, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_speed, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 72}},
        {MP_QSTR_pitch, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64}},
        {MP_QSTR_mouth, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_throat, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args),
                     allowed_args, args);

    int buf_len;
    char *buf = sam_synthesize(
        mp_obj_str_get_str(args[ARG_text].u_obj),
        (unsigned char)args[ARG_speed].u_int,
        (unsigned char)args[ARG_pitch].u_int,
        (unsigned char)args[ARG_mouth].u_int,
        (unsigned char)args[ARG_throat].u_int, &buf_len);

    mp_obj_t result = mp_obj_new_bytearray(buf_len, (const byte *)buf);
    free(buf);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_sam_render_obj, 1, mod_sam_render);

// sam.say(text, speed=72, pitch=64, mouth=128, throat=128, gain=100)
// Renders and plays through pz_drive, blocks until done
static mp_obj_t mod_sam_say(size_t n_args, const mp_obj_t *pos_args,
                            mp_map_t *kw_args) {
    enum { ARG_text, ARG_speed, ARG_pitch, ARG_mouth, ARG_throat, ARG_gain };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_text, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_speed, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 72}},
        {MP_QSTR_pitch, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64}},
        {MP_QSTR_mouth, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_throat, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_gain, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 100}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args),
                     allowed_args, args);

    int buf_len;
    char *buf = sam_synthesize(
        mp_obj_str_get_str(args[ARG_text].u_obj),
        (unsigned char)args[ARG_speed].u_int,
        (unsigned char)args[ARG_pitch].u_int,
        (unsigned char)args[ARG_mouth].u_int,
        (unsigned char)args[ARG_throat].u_int, &buf_len);

    // Play via pz_drive
    pzd_pwm_play_samples((const uint8_t *)buf, buf_len, 22050, false);

    // Poll until done, allowing Ctrl-C
    while (!pzd_pwm_is_sample_done()) {
        mp_handle_pending(true);
        mp_hal_delay_ms(10);
    }

    free(buf);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_sam_say_obj, 1, mod_sam_say);

// Module globals table
static const mp_rom_map_elem_t sam_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sam)},
    {MP_ROM_QSTR(MP_QSTR_render), MP_ROM_PTR(&mod_sam_render_obj)},
    {MP_ROM_QSTR(MP_QSTR_say), MP_ROM_PTR(&mod_sam_say_obj)},
};
static MP_DEFINE_CONST_DICT(sam_module_globals, sam_module_globals_table);

const mp_obj_module_t sam_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&sam_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_sam, sam_module);
