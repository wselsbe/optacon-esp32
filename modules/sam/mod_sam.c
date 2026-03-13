// modules/sam/mod_sam.c — MicroPython bindings for SAM speech synthesizer
#include <string.h>

#include "py/mphal.h"
#include "py/runtime.h"

#include "pz_drive.h"
#include "reciter.h"
#include "render.h"
#include "sam.h"

// debug flag expected by SAM (originally in main.c)
int debug = 0;

// SAM globals we need to access
extern int bufferpos;
extern char *buffer;

// Helper: run SAM synthesis, returns PCM buffer info.
// Caller must free the returned buffer with m_free().
static char *sam_synthesize(const char *text, int phonetic, unsigned char speed,
                            unsigned char pitch, unsigned char mouth, unsigned char throat,
                            int *out_len) {
    size_t len = strlen(text);
    if (len > 254) {
        mp_raise_ValueError(MP_ERROR_TEXT("text too long (max 254 chars)"));
    }

    // Pre-allocate output buffer with MicroPython allocator (10 sec @ 22050 Hz)
    int buf_size = 22050 * 10;
    char *buf = m_malloc(buf_size);
    if (!buf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("SAM buffer alloc failed"));
    }
    memset(buf, 128, buf_size);

    // Set SAM globals — buffer is pre-allocated so Init() won't malloc
    buffer = buf;
    bufferpos = 0;

    // Reset render state for deterministic output
    RenderReset();

    // Configure SAM parameters
    SetSpeed(speed);
    SetPitch(pitch);
    SetMouth(mouth);
    SetThroat(throat);

    // Prepare input: copy text, uppercase, convert to phonemes if needed
    char input_buf[256];
    memset(input_buf, 0, sizeof(input_buf));
    strncpy(input_buf, text, 254);

    // Uppercase the input
    for (int i = 0; input_buf[i] != 0; i++) {
        if (input_buf[i] >= 'a' && input_buf[i] <= 'z') input_buf[i] -= 32;
    }

    if (!phonetic) {
        // English text → phonemes via reciter
        strncat(input_buf, "[", sizeof(input_buf) - strlen(input_buf) - 1);
        if (!TextToPhonemes((unsigned char *)input_buf)) {
            buffer = NULL;
            m_free(buf);
            mp_raise_ValueError(MP_ERROR_TEXT("SAM text-to-phoneme conversion failed"));
        }
    } else {
        // Direct phonetic input — append end marker
        strncat(input_buf, "\x9b", sizeof(input_buf) - strlen(input_buf) - 1);
    }

    SetInput(input_buf);

    if (!SAMMain()) {
        buffer = NULL;
        m_free(buf);
        mp_raise_ValueError(MP_ERROR_TEXT("SAM synthesis failed"));
    }

    int result_len = GetBufferLength() / 50;
    if (result_len <= 0) {
        buffer = NULL;
        m_free(buf);
        mp_raise_ValueError(MP_ERROR_TEXT("SAM produced no output"));
    }

    // Detach buffer from SAM globals so Init() won't free it
    buffer = NULL;

    *out_len = result_len;
    return buf;
}

// sam.render(text, speed=72, pitch=64, mouth=128, throat=128, phonetic=False)
// Returns bytearray of 8-bit unsigned PCM at 22050 Hz
static mp_obj_t mod_sam_render(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_text, ARG_speed, ARG_pitch, ARG_mouth, ARG_throat, ARG_phonetic };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_text, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_speed, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 72}},
        {MP_QSTR_pitch, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64}},
        {MP_QSTR_mouth, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_throat, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_phonetic, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int buf_len;
    char *buf = sam_synthesize(
        mp_obj_str_get_str(args[ARG_text].u_obj), args[ARG_phonetic].u_bool,
        (unsigned char)args[ARG_speed].u_int, (unsigned char)args[ARG_pitch].u_int,
        (unsigned char)args[ARG_mouth].u_int, (unsigned char)args[ARG_throat].u_int, &buf_len);

    mp_obj_t result = mp_obj_new_bytearray(buf_len, (const byte *)buf);
    m_free(buf);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_sam_render_obj, 1, mod_sam_render);

// sam.say(text, speed=72, pitch=64, mouth=128, throat=128, gain=100, phonetic=False)
// Renders and plays through pz_drive, blocks until done
static mp_obj_t mod_sam_say(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_text, ARG_speed, ARG_pitch, ARG_mouth, ARG_throat, ARG_gain, ARG_phonetic };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_text, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_speed, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 72}},
        {MP_QSTR_pitch, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64}},
        {MP_QSTR_mouth, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_throat, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
        {MP_QSTR_gain, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 100}},
        {MP_QSTR_phonetic, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int buf_len;
    char *buf = sam_synthesize(
        mp_obj_str_get_str(args[ARG_text].u_obj), args[ARG_phonetic].u_bool,
        (unsigned char)args[ARG_speed].u_int, (unsigned char)args[ARG_pitch].u_int,
        (unsigned char)args[ARG_mouth].u_int, (unsigned char)args[ARG_throat].u_int, &buf_len);

    // Map gain percentage to DRV2665 register value
    int gain = args[ARG_gain].u_int;
    uint8_t gain_bits;
    if (gain <= 25) gain_bits = 0x00;
    else if (gain <= 50) gain_bits = 0x01;
    else if (gain <= 75) gain_bits = 0x02;
    else gain_bits = 0x03;

    // Scale samples to amplitude=55 equivalent (avoid overdriving DRV2665
    // 1.8Vpp differential input — there is unexplained coupling on IN-)
    for (int i = 0; i < buf_len; i++) {
        int val = (int)buf[i] - 128;
        val = (val * 55) / 100;
        buf[i] = (char)(val + 128);
    }

    // Configure DRV2665 for analog input before playback
    drv2665_write_reg(0x02, 0x0C);             // CTRL2: clear timeout, set TIMEOUT_20MS
    drv2665_write_reg(0x01, 0x04 | gain_bits); // CTRL1: INPUT_ANALOG | gain
    drv2665_write_reg(0x02, 0x02 | 0x0C);      // CTRL2: EN_OVERRIDE | TIMEOUT_20MS

    // Play via pz_drive
    pzd_pwm_play_samples((const uint8_t *)buf, buf_len, 22050, false);

    // Poll until done, allowing Ctrl-C.
    while (!pzd_pwm_is_sample_done()) {
        mp_handle_pending(true);
        mp_hal_delay_ms(10);
    }

    // Stop PWM ISR (otherwise it continues in DDS mode after samples end)
    pzd_pwm_stop();

    // Put DRV2665 in standby
    drv2665_write_reg(0x02, 0x40);

    m_free(buf);
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
