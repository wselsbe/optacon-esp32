# Software Bootloader Entry — Implementation Plan

## Goal

Implement a `pz_actuator.enter_bootloader()` C function that reboots the ESP32-S3 into download/bootloader mode from MicroPython, replacing the broken `machine.bootloader()`.

## Why `machine.bootloader()` Doesn't Work

The ESP32-S3 board uses **USB-CDC via TinyUSB** (VID:PID 303A:4001), not the USB-Serial/JTAG peripheral. MicroPython's `machine.bootloader()` calls ROM persist functions (`usb_dc_prepare_persist`, `chip_usb_set_persist_flags`) that expect the ROM's own CDC stack, not TinyUSB. This leaves USB in a broken state — the host sees "device not functioning" and the board must be physically reset.

### Broken code path (modmachine.c:275–288)

```c
// This is exactly what machine.bootloader() does on ESP32-S3 with MICROPY_HW_USB_CDC:
usb_usj_mode();              // switches USB PHY to Serial/JTAG mode (wrong for TinyUSB CDC boards)
usb_dc_prepare_persist();    // ROM USB persist — expects ROM CDC stack, not TinyUSB
chip_usb_set_persist_flags(USBDC_BOOT_DFU);  // ROM persist flags — meaningless for TinyUSB
REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
esp_restart();
```

`usb_usj_mode()` (in `ports/esp32/usb.c`) calls `usb_del_phy()` then creates a new PHY with `USB_PHY_CTRL_SERIAL_JTAG` — this tears down the OTG PHY but the ROM persist functions still expect their own USB stack, so the host sees a broken device.

Testing confirmed:
- `machine.bootloader()` via MCP: timeout, board stuck on COM10 in broken USB state
- `mem32` register write to force download boot: same broken USB result
- Both approaches fail because they don't cleanly tear down TinyUSB before resetting

## Proposed Solution

A custom C function in `pz_actuator.c` that:

1. **Disconnects TinyUSB cleanly** — `tud_disconnect()` signals to the USB host that the device is going away
2. **Waits for host to process** — ~100ms delay for the host OS to handle the disconnect
3. **Resets USB peripheral** — clear any stale USB state via `periph_module_reset(PERIPH_USB_MODULE)`
4. **Sets force-download-boot flag** — write to RTC register so ROM bootloader enters download mode
5. **Restarts** — `esp_restart()` triggers the actual reboot

```c
#include "tusb.h"                        // tud_disconnect, tud_connected
#include "esp_private/periph_ctrl.h"     // periph_module_reset
#include "soc/rtc_cntl_reg.h"           // RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT
#include "soc/periph_defs.h"            // PERIPH_USB_MODULE
#include "esp_system.h"                  // esp_restart
#include "esp_rom_sys.h"                 // esp_rom_delay_us

static mp_obj_t pz_actuator_enter_bootloader(void) {
    mp_printf(&mp_plat_print, "Entering bootloader...\n");

    // 1. Disconnect TinyUSB cleanly
    if (tud_connected()) {
        tud_disconnect();
    }
    esp_rom_delay_us(100000);  // 100ms for host to process

    // 2. Reset USB OTG peripheral
    periph_module_reset(PERIPH_USB_MODULE);

    // 3. Force download boot on next reset
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);

    // 4. Restart
    esp_restart();
    return mp_const_none;  // never reached
}
```

## Header/Symbol Availability (ESP-IDF v5.5.1) — VERIFIED

All symbols and headers confirmed available in the build environment.

| Symbol/Header | Status | Location |
|---|---|---|
| `tusb.h` | **Available** | `managed_components/espressif__tinyusb/src/tusb.h` (pulled at build time via `idf_component.yml`) |
| `tud_disconnect()` | **Linked** at `0x420ab3d4` | `libespressif__tinyusb.a(usbd.c.obj)` — declared in `src/device/usbd.h:106` |
| `tud_connected()` | **Linked** at `0x420ab368` | `libespressif__tinyusb.a(usbd.c.obj)` — declared in `src/device/usbd.h:87` |
| `PERIPH_USB_MODULE` | **Available** | `/opt/esp/idf/components/soc/esp32s3/include/soc/periph_defs.h:20` |
| `RTC_CNTL_OPTION1_REG` | **Available** | `rtc_cntl_reg.h:3209` — `DR_REG_RTCCNTL_BASE + 0x12C` |
| `RTC_CNTL_FORCE_DOWNLOAD_BOOT` | **Available** | `rtc_cntl_reg.h:3212` — `BIT(0)` |
| `esp_restart()` | **Available** | `esp_system.h:79` |
| `periph_module_reset()` | **Available** | `esp_private/periph_ctrl.h` |
| `esp_rom_delay_us()` | **Available** | `esp_rom_sys.h` |

### TinyUSB integration details

- MicroPython ESP32 port includes TinyUSB via `esp32_common.cmake:84–104`
- The `espressif__tinyusb` managed component is declared in `idf_component.yml` and pinned to git commit `e4c0ec3` from `micropython/tinyusb-espressif`
- MicroPython wraps TinyUSB via `shared/tinyusb/mp_usbd.h` which `#include "tusb.h"` (line 59)
- The Espressif TinyUSB component is linked as `espressif__tinyusb` (not `idf::tinyusb`)

### Fallback registers (also verified)

| Symbol | Location |
|---|---|
| `USB_WRAP_OTG_CONF_REG` | `soc/usb_wrap_reg.h:15` — `DR_REG_USB_WRAP_BASE + 0x0` |
| `USB_WRAP_USB_PAD_ENABLE` | `soc/usb_wrap_reg.h:48` — `BIT(18)` |

### Reference: how MicroPython uses tud_disconnect()

In `shared/tinyusb/mp_usbd_runtime.c:488`:
```c
tud_disconnect();
// followed by 50ms delay for host bus reset
```

In `ports/alif/modmachine.c:84`:
```c
tud_disconnect();
tud_deinit(TUD_OPT_RHPORT);
dcd_uninit();
```

### usb_console.h — NOT useful for our case

`esp_private/usb_console.h` provides ROM USB CDC console functions (init, read, write, flush). It does NOT provide any USB teardown/before-restart helpers. It's for the ROM CDC driver path, not TinyUSB.

## Build System Changes

The `micropython.cmake` for pz_actuator is an INTERFACE library. TinyUSB headers should be available via the existing build since MicroPython's ESP32 port already links `espressif__tinyusb` globally. If `#include "tusb.h"` fails, add to `micropython.cmake`:

```cmake
target_link_libraries(usermod_pz_actuator INTERFACE idf::espressif__tinyusb)
```

## Fallback Approaches

### Approach B: Direct Register Manipulation (no TinyUSB headers needed)

Skip TinyUSB API entirely and manipulate USB registers directly:
```c
#include "soc/usb_wrap_reg.h"
// Disable USB pad to signal disconnect to host
REG_CLR_BIT(USB_WRAP_OTG_CONF_REG, USB_WRAP_USB_PAD_ENABLE);
esp_rom_delay_us(100000);
// Then force download boot + restart as before
```

### Approach C: Weak Symbol

Call `tud_disconnect` via weak extern if header include is problematic:
```c
extern bool tud_disconnect(void) __attribute__((weak));
extern bool tud_connected(void) __attribute__((weak));
if (tud_disconnect) {
    if (tud_connected()) tud_disconnect();
}
```

## Expected Behavior After Implementation

1. User calls `pz_actuator.enter_bootloader()` from REPL or MCP
2. MCP connection drops (expected — USB disconnects)
3. Board appears on COM7 (download mode)
4. `flash.cmd` can flash without manual button press
5. After flash + watchdog-reset, board reboots to MicroPython on COM10

## Port Mapping

- **COM10** = MicroPython REPL (USB-CDC, TinyUSB)
- **COM7** = Download/bootloader mode (ROM USB)

## Related Files

- `modules/pz_actuator/pz_actuator.c` — add the function here
- `modules/pz_actuator/micropython.cmake` — may need TinyUSB dependency
- `python/main.py` — no changes needed (function is called manually)

## DevContainer Fix

The Docker image had `git config --global core.autocrlf true` which caused the `espressif__tinyusb` managed component to get CRLF line endings after `git clone`, breaking the IDF component hash check (`ee1c96...` expected vs `0eb60a...` computed). Fixed by setting `autocrlf=false` globally and `autocrlf=true` only for the workspace repo in `.devcontainer/devcontainer.json`.
