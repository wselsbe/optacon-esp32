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

// ── Module table ────────────────────────────────────────────────────────
static const mp_rom_map_elem_t pz_drive_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pz_drive) },
    { MP_ROM_QSTR(MP_QSTR_sr_stage), MP_ROM_PTR(&pz_drive_sr_stage_obj) },
    { MP_ROM_QSTR(MP_QSTR_sr_write), MP_ROM_PTR(&pz_drive_sr_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_pol_init), MP_ROM_PTR(&pz_drive_pol_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_pol_set), MP_ROM_PTR(&pz_drive_pol_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_pol_get), MP_ROM_PTR(&pz_drive_pol_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_i2c_read), MP_ROM_PTR(&pz_drive_i2c_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_i2c_write), MP_ROM_PTR(&pz_drive_i2c_write_obj) },
};
static MP_DEFINE_CONST_DICT(pz_drive_module_globals, pz_drive_module_globals_table);

const mp_obj_module_t pz_drive_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pz_drive_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_pz_drive, pz_drive_module);
