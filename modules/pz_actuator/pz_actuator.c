#include "py/runtime.h"
#include "py/obj.h"

static mp_obj_t pz_actuator_init(void) {
    mp_printf(&mp_plat_print, "pz_actuator: init\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_actuator_init_obj, pz_actuator_init);

static const mp_rom_map_elem_t pz_actuator_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_pz_actuator) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&pz_actuator_init_obj) },
};
static MP_DEFINE_CONST_DICT(pz_actuator_module_globals, pz_actuator_module_globals_table);

const mp_obj_module_t pz_actuator_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pz_actuator_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_actuator, pz_actuator_module);
