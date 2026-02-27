// board_utils — MicroPython C module for board-level utilities
//
// Python API:
//   board_utils.enter_bootloader()  — switch to USB download mode for flashing

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mpprint.h"
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"
#include "esp_rom_sys.h"

// Defined in ports/esp32/usb.c - switches USB PHY from OTG to Serial/JTAG
extern void usb_usj_mode(void);

// board_utils.enter_bootloader()
// Disconnects USB, switches PHY to Serial/JTAG, sets download boot flag,
// and restarts into ROM bootloader. The host sees a new USB device
// (VID:PID 303A:1001) on a different COM port.
static mp_obj_t board_utils_enter_bootloader(void) {
    mp_printf(&mp_plat_print, "Entering bootloader...\n");

    if (tud_connected()) {
        tud_disconnect();
    }
    esp_rom_delay_us(100000); // 100 ms for host to process disconnect

    // Switch USB PHY from OTG to Serial/JTAG so ROM uses same path as physical BOOT+RST
    usb_usj_mode();

    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);

    esp_restart();
    return mp_const_none; // never reached
}
static MP_DEFINE_CONST_FUN_OBJ_0(board_utils_enter_bootloader_obj, board_utils_enter_bootloader);

// ─── Module registration ─────────────────────────────────────────────────────

static const mp_rom_map_elem_t board_utils_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_board_utils)},
    {MP_ROM_QSTR(MP_QSTR_enter_bootloader),  MP_ROM_PTR(&board_utils_enter_bootloader_obj)},
};
static MP_DEFINE_CONST_DICT(board_utils_globals, board_utils_globals_table);

const mp_obj_module_t board_utils_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&board_utils_globals,
};

MP_REGISTER_MODULE(MP_QSTR_board_utils, board_utils_module);
