// modules/pz_drive/pz_drive.c
#include "py/runtime.h"
#include "py/obj.h"
#include "pz_drive.h"

// ── Module table (populated in later tasks) ─────────────────────────────
static const mp_rom_map_elem_t pz_drive_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_pz_drive) },
};
static MP_DEFINE_CONST_DICT(pz_drive_module_globals, pz_drive_module_globals_table);

const mp_obj_module_t pz_drive_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pz_drive_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_pz_drive, pz_drive_module);
