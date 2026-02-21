#include "py/runtime.h"
#include "py/obj.h"
#include "waveform.h"

static mp_obj_t pz_actuator_init(void) {
    mp_printf(&mp_plat_print, "pz_actuator: init\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_init_obj, pz_actuator_init);

static waveform_t g_waveform;

static mp_obj_t pz_actuator_test_waveform(mp_obj_t freq_obj) {
    uint16_t freq = mp_obj_get_int(freq_obj);
    waveform_init(&g_waveform, freq);

    mp_printf(&mp_plat_print, "freq=%d period=%d trough_idx=%d\n",
              g_waveform.frequency, g_waveform.period_len, g_waveform.trough_index);

    // Print first period
    for (size_t i = 0; i < g_waveform.period_len; i++) {
        mp_printf(&mp_plat_print, "  [%d] = %d\n", i, g_waveform.lut[i]);
    }

    // Test samples_until_trough with full FIFO scenario
    // After writing one full period, with fifo_depth = period_len,
    // playback head is at index 0, trough is at index 0, so distance should be 0
    g_waveform.write_index = 0;  // reset after fill
    int8_t tmp_buf[WAVEFORM_MAX_PERIOD];
    waveform_fill_buffer(&g_waveform, tmp_buf, g_waveform.period_len);
    size_t samples = waveform_samples_until_trough(&g_waveform, g_waveform.period_len);
    mp_printf(&mp_plat_print, "samples_until_trough (full period in fifo): %d\n", samples);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(pz_actuator_test_waveform_obj, pz_actuator_test_waveform);

static const mp_rom_map_elem_t pz_actuator_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_pz_actuator) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&pz_actuator_init_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_waveform), MP_ROM_PTR(&pz_actuator_test_waveform_obj) },
};
static MP_DEFINE_CONST_DICT(pz_actuator_module_globals, pz_actuator_module_globals_table);

const mp_obj_module_t pz_actuator_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pz_actuator_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_actuator, pz_actuator_module);
