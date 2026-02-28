# pz_drive Unified Module Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Consolidate `pz_pwm`, `pz_fifo`, and Python SPI/I2C layers into a single `pz_drive` C module. Add waveform types and fullwave to the digital path. Synchronize shift register latching to waveform zero-crossings.

**Architecture:** Single C module (`modules/pz_drive/`) with 4 implementation files (hv509.c, drv2665.c, pwm.c, fifo.c) plus a registration file (pz_drive.c) and shared header (pz_drive.h). Python classes (DRV2665, ShiftRegister, PzActuator) delegate all hardware access to `pz_drive.*` functions. The old `pz_pwm`, `pz_fifo`, and `pz_actuator` C modules are removed.

**Tech Stack:** ESP-IDF v5.5.1 (spi_master, i2c_master, gptimer, ledc), MicroPython v1.27.0 C API, FreeRTOS

**Design doc:** `docs/plans/2026-02-28-pz-drive-unified-module-design.md`

---

## Task 1: Create pz_drive skeleton — header, cmake, registration

**Files:**
- Create: `modules/pz_drive/pz_drive.h`
- Create: `modules/pz_drive/pz_drive.c`
- Create: `modules/pz_drive/micropython.cmake`
- Modify: `modules/micropython.cmake`

**Step 1: Create shared header `pz_drive.h`**

```c
// modules/pz_drive/pz_drive.h
#ifndef PZ_DRIVE_H
#define PZ_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

// ── hv509.c — SPI shift register + polarity GPIOs ──────────────────────
void hv509_init(void);
void hv509_sr_stage(uint32_t word32);
void hv509_sr_write(uint32_t word32);
void hv509_sr_latch_if_pending(void);
void hv509_pol_init(void);
void hv509_pol_set(bool val);
bool hv509_pol_get(void);
void hv509_pol_toggle(void);

// ── drv2665.c — I2C bus + register access ───────────────────────────────
void drv2665_bus_init(void);
int drv2665_read_reg(uint8_t reg);
void drv2665_write_reg(uint8_t reg, uint8_t val);
void drv2665_write_fifo_bulk(const uint8_t *data, size_t len);
void drv2665_write_fifo_byte(uint8_t val);
uint8_t drv2665_read_status(void);

// ── pwm.c — analog DDS ISR ─────────────────────────────────────────────
bool pwm_is_running(void);

// ── fifo.c — digital FIFO background task ───────────────────────────────
bool fifo_is_running(void);

#endif // PZ_DRIVE_H
```

**Step 2: Create cmake build file `micropython.cmake`**

```cmake
# modules/pz_drive/micropython.cmake
add_library(usermod_pz_drive INTERFACE)

target_sources(usermod_pz_drive INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_drive.c
    ${CMAKE_CURRENT_LIST_DIR}/hv509.c
    ${CMAKE_CURRENT_LIST_DIR}/drv2665.c
    ${CMAKE_CURRENT_LIST_DIR}/pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/fifo.c
)

target_include_directories(usermod_pz_drive INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(usermod_pz_drive INTERFACE
    MICROPY_HW_ESP_NEW_I2C_DRIVER=1
)

target_link_libraries(usermod INTERFACE usermod_pz_drive)
```

**Step 3: Create empty stub files so cmake doesn't fail**

Create these files with minimal content (empty function bodies):

`modules/pz_drive/hv509.c`:
```c
#include "pz_drive.h"
void hv509_init(void) {}
void hv509_sr_stage(uint32_t word32) { (void)word32; }
void hv509_sr_write(uint32_t word32) { (void)word32; }
void hv509_sr_latch_if_pending(void) {}
void hv509_pol_init(void) {}
void hv509_pol_set(bool val) { (void)val; }
bool hv509_pol_get(void) { return false; }
void hv509_pol_toggle(void) {}
```

`modules/pz_drive/drv2665.c`:
```c
#include "pz_drive.h"
void drv2665_bus_init(void) {}
int drv2665_read_reg(uint8_t reg) { (void)reg; return 0; }
void drv2665_write_reg(uint8_t reg, uint8_t val) { (void)reg; (void)val; }
void drv2665_write_fifo_bulk(const uint8_t *data, size_t len) { (void)data; (void)len; }
void drv2665_write_fifo_byte(uint8_t val) { (void)val; }
uint8_t drv2665_read_status(void) { return 0; }
```

`modules/pz_drive/pwm.c`:
```c
#include "pz_drive.h"
bool pwm_is_running(void) { return false; }
```

`modules/pz_drive/fifo.c`:
```c
#include "pz_drive.h"
bool fifo_is_running(void) { return false; }
```

**Step 4: Create module registration `pz_drive.c`**

Start with just the module shell (no Python bindings yet — those are added per-task):

```c
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
```

**Step 5: Update top-level cmake**

In `modules/micropython.cmake`, replace the `pz_pwm` and `pz_fifo` includes with `pz_drive`:

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/pz_actuator/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/pz_drive/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/board_utils/micropython.cmake)
```

Note: `pz_actuator` stays for now (removed in Task 10). The old `pz_pwm` and `pz_fifo` lines are removed.

**Step 6: Build to verify skeleton compiles**

Run: local build
Expected: clean build, `import pz_drive` available (empty module)

**Step 7: Flash and verify**

Flash to board, open REPL:
```python
import pz_drive
print(dir(pz_drive))
# Should show ['__name__']
```

**Step 8: Commit**

```bash
git add modules/pz_drive/ modules/micropython.cmake
git commit -m "feat(pz_drive): add module skeleton with cmake and stubs"
```

---

## Task 2: Implement hv509.c — SPI shift register + polarity GPIOs

**Files:**
- Modify: `modules/pz_drive/hv509.c` (replace stub with real implementation)
- Modify: `modules/pz_drive/pz_drive.c` (add sr_stage, sr_write, pol_init, pol_set, pol_get bindings)

**Reference:** `modules/pz_actuator/shift_register.c` (SPI config, GPIO init, polarity handling) and `modules/pz_pwm/pz_pwm.c:218-233` (polarity GPIO init)

**Step 1: Implement hv509.c**

Key design points:
- **GPIO 12/13 (polarity) must be configured BEFORE SPI bus init** (IOMUX conflict on GPIO 10-13)
- SPI1_HOST (SPI2 in ESP-IDF terms), 1 MHz, mode 0, CS on GPIO 10
- 32-bit word: `[31:26]=0, [25:6]=pins, [5:0]=0`
- Stage/latch mechanism: `s_pending_word` + `s_latch_pending` flag. When staged:
  - If neither PWM ISR nor FIFO task is running → immediate SPI write
  - Otherwise → set flag, ISR/task calls `hv509_sr_latch_if_pending()` at the right moment

```c
// modules/pz_drive/hv509.c
#include "pz_drive.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "py/mpprint.h"

#include <string.h>

#define SPI_MOSI_GPIO   6
#define SPI_MISO_GPIO   7
#define SPI_SCK_GPIO    9
#define SPI_CS_GPIO     10
#define POL_A_GPIO      12
#define POL_B_GPIO      13

#define SPI_CLK_HZ      1000000

static spi_device_handle_t s_spi_dev;
static bool s_spi_inited = false;
static bool s_pol_inited = false;
static bool s_pol_value = false;

// Stage/latch state
static volatile uint32_t s_pending_word = 0;
static volatile bool s_latch_pending = false;

// ── Polarity GPIOs ──────────────────────────────────────────────────────

void hv509_pol_init(void) {
    if (s_pol_inited) return;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POL_A_GPIO) | (1ULL << POL_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(POL_A_GPIO, 0);
    gpio_set_level(POL_B_GPIO, 0);
    s_pol_value = false;
    s_pol_inited = true;
}

void hv509_pol_set(bool val) {
    s_pol_value = val;
    gpio_set_level(POL_A_GPIO, val ? 1 : 0);
    gpio_set_level(POL_B_GPIO, val ? 1 : 0);
}

bool hv509_pol_get(void) {
    return s_pol_value;
}

void hv509_pol_toggle(void) {
    hv509_pol_set(!s_pol_value);
}

// ── SPI shift register ─────────────────────────────────────────────────

static void spi_write_word(uint32_t word32) {
    // Mask off common bits (only pins 6-25 are valid)
    word32 &= 0x03FFFFC0U;
    // Send big-endian (MSB first)
    uint8_t buf[4];
    buf[0] = (word32 >> 24) & 0xFF;
    buf[1] = (word32 >> 16) & 0xFF;
    buf[2] = (word32 >> 8) & 0xFF;
    buf[3] = word32 & 0xFF;
    spi_transaction_t txn = {
        .length = 32,
        .tx_buffer = buf,
    };
    spi_device_transmit(s_spi_dev, &txn);
}

void hv509_init(void) {
    if (s_spi_inited) return;
    // Polarity GPIOs MUST be configured before SPI (IOMUX conflict)
    hv509_pol_init();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = SPI_MISO_GPIO,
        .sclk_io_num = SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED);

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLK_HZ,
        .mode = 0,
        .spics_io_num = SPI_CS_GPIO,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi_dev);

    // Clear all outputs
    spi_write_word(0);
    s_spi_inited = true;
}

void hv509_sr_write(uint32_t word32) {
    if (!s_spi_inited) hv509_init();
    spi_write_word(word32);
}

void hv509_sr_stage(uint32_t word32) {
    s_pending_word = word32;
    // If neither ISR nor task is running, write immediately
    if (!pwm_is_running() && !fifo_is_running()) {
        spi_write_word(word32);
        s_latch_pending = false;
    } else {
        s_latch_pending = true;
    }
}

void hv509_sr_latch_if_pending(void) {
    if (s_latch_pending) {
        spi_write_word(s_pending_word);
        s_latch_pending = false;
    }
}
```

**Step 2: Add Python bindings in pz_drive.c**

Add these functions and module table entries to `pz_drive.c`:

```c
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
```

Module table entries:
```c
{ MP_ROM_QSTR(MP_QSTR_sr_stage), MP_ROM_PTR(&pz_drive_sr_stage_obj) },
{ MP_ROM_QSTR(MP_QSTR_sr_write), MP_ROM_PTR(&pz_drive_sr_write_obj) },
{ MP_ROM_QSTR(MP_QSTR_pol_init), MP_ROM_PTR(&pz_drive_pol_init_obj) },
{ MP_ROM_QSTR(MP_QSTR_pol_set), MP_ROM_PTR(&pz_drive_pol_set_obj) },
{ MP_ROM_QSTR(MP_QSTR_pol_get), MP_ROM_PTR(&pz_drive_pol_get_obj) },
```

**Step 3: Build**

Run: local build
Expected: clean compile

**Step 4: Flash and verify on hardware**

```python
import pz_drive
pz_drive.pol_init()
pz_drive.sr_write(0x03FFFFC0)  # all pins on
pz_drive.sr_write(0)            # all pins off
pz_drive.pol_set(True)
print(pz_drive.pol_get())       # True
pz_drive.pol_set(False)
```

**Step 5: Commit**

```bash
git add modules/pz_drive/hv509.c modules/pz_drive/pz_drive.c
git commit -m "feat(pz_drive): implement hv509 SPI shift register + polarity GPIOs"
```

---

## Task 3: Implement drv2665.c — I2C bus + register access

**Files:**
- Modify: `modules/pz_drive/drv2665.c` (replace stub with real implementation)
- Modify: `modules/pz_drive/pz_drive.h` (already has prototypes)
- Modify: `modules/pz_drive/pz_drive.c` (add i2c_read, i2c_write bindings)

**Reference:** `modules/pz_actuator/drv2665.c` (I2C bus creation, device add, register ops), `modules/pz_fifo/pz_fifo.c:37-73` (MicroPython I2C struct layout)

**Step 1: Implement drv2665.c**

Key design points:
- Create I2C bus and add DRV2665 device at 0x59 during `drv2665_bus_init()`
- Keep bus handle persistent — shared by FIFO task and Python register access
- ESP-IDF `i2c_master` driver is thread-safe (mutex per transaction)
- Timeout: 100ms (milliseconds, not ticks!)
- FIFO helpers: `drv2665_write_fifo_bulk()`, `drv2665_write_fifo_byte()`, `drv2665_read_status()`

```c
// modules/pz_drive/drv2665.c
#include "pz_drive.h"

#include "driver/i2c_master.h"
#include "py/mpprint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

#define I2C_SDA_GPIO    47
#define I2C_SCL_GPIO    21
#define I2C_FREQ_HZ     100000
#define DRV2665_ADDR    0x59
#define I2C_TIMEOUT_MS  100

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_inited = false;

void drv2665_bus_init(void) {
    if (s_inited) return;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2665_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    // Wait for device ready (datasheet: 1ms after power-on)
    vTaskDelay(pdMS_TO_TICKS(2) > 0 ? pdMS_TO_TICKS(2) : 1);

    // Verify device present by reading STATUS register
    uint8_t reg = 0x00;
    uint8_t val = 0;
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "drv2665: I2C probe failed (0x%02x)\n", err);
    }
    s_inited = true;
}

int drv2665_read_reg(uint8_t reg) {
    if (!s_inited) drv2665_bus_init();
    uint8_t val = 0;
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return -1;
    return val;
}

void drv2665_write_reg(uint8_t reg, uint8_t val) {
    if (!s_inited) drv2665_bus_init();
    uint8_t buf[2] = { reg, val };
    i2c_master_transmit(s_dev, buf, 2, I2C_TIMEOUT_MS);
}

void drv2665_write_fifo_bulk(const uint8_t *data, size_t len) {
    // Prepend FIFO register address (0x0B)
    uint8_t buf[101]; // max 100 data bytes + 1 reg byte
    if (len > 100) len = 100;
    buf[0] = 0x0B;
    memcpy(buf + 1, data, len);
    i2c_master_transmit(s_dev, buf, len + 1, I2C_TIMEOUT_MS);
}

void drv2665_write_fifo_byte(uint8_t val) {
    uint8_t buf[2] = { 0x0B, val };
    i2c_master_transmit(s_dev, buf, 2, I2C_TIMEOUT_MS);
}

uint8_t drv2665_read_status(void) {
    uint8_t reg = 0x00;
    uint8_t val = 0;
    i2c_master_transmit_receive(s_dev, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
    return val;
}
```

**Step 2: Add Python bindings in pz_drive.c**

```c
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
```

Module table entries:
```c
{ MP_ROM_QSTR(MP_QSTR_i2c_read), MP_ROM_PTR(&pz_drive_i2c_read_obj) },
{ MP_ROM_QSTR(MP_QSTR_i2c_write), MP_ROM_PTR(&pz_drive_i2c_write_obj) },
```

**Step 3: Build**

Run: local build
Expected: clean compile

**Step 4: Flash and verify on hardware**

```python
import pz_drive
val = pz_drive.i2c_read(0x00)  # STATUS register
print(hex(val))                 # Should be 0x60 (standby, not in digital mode)
pz_drive.i2c_write(0x02, 0x00) # Clear standby
val = pz_drive.i2c_read(0x02)
print(hex(val))                 # Should be 0x00
pz_drive.i2c_write(0x02, 0x40) # Back to standby
```

**Step 5: Commit**

```bash
git add modules/pz_drive/drv2665.c modules/pz_drive/pz_drive.c
git commit -m "feat(pz_drive): implement drv2665 I2C bus + register access"
```

---

## Task 4: Implement pwm.c — analog DDS ISR with latch + polarity

**Files:**
- Modify: `modules/pz_drive/pwm.c` (replace stub with full implementation)
- Modify: `modules/pz_drive/pz_drive.c` (add pwm_set_frequency, pwm_start, pwm_stop, pwm_is_running bindings)

**Reference:** `modules/pz_pwm/pz_pwm.c` (the entire file — this is a port with latch added)

**Step 1: Implement pwm.c**

Port the existing `pz_pwm.c` (495 lines) into `modules/pz_drive/pwm.c`. Key changes from the original:
- Remove polarity GPIO code (now in hv509.c)
- Replace direct `gpio_set_level(POL_A/B)` calls with `hv509_pol_set()`/`hv509_pol_toggle()`
- Add `hv509_sr_latch_if_pending()` call at latch points:
  - **No fullwave**: when `s_phase_acc` wraps past 0 (new cycle start)
  - **Fullwave**: at polarity toggle moment (zero-crossing)
- Keep all existing features: sine/triangle/square waveforms, fullwave, dead_time, phase_advance, 8/10-bit resolution

The ISR latch logic (inside `timer_isr_callback`):

```c
// After phase accumulator update:
uint32_t prev_phase = s_phase_acc;
s_phase_acc += s_phase_step;

if (s_fullwave) {
    // ... existing fullwave duty inversion + dead_time ...

    // Polarity toggle with phase advance
    uint8_t pol_half = (uint8_t)((s_phase_acc + s_pol_advance) >> 31);
    if (pol_half != s_prev_half) {
        s_prev_half = pol_half;
        hv509_pol_set(pol_half ? true : false);
        hv509_sr_latch_if_pending();    // <── NEW: latch at zero-crossing
    }
} else {
    // No fullwave — latch at cycle start (phase wrap)
    if (s_phase_acc < prev_phase) {     // phase wrapped
        hv509_sr_latch_if_pending();    // <── NEW: latch at cycle start
    }
}
```

Full implementation: copy `modules/pz_pwm/pz_pwm.c` lines 1-309 (everything except MicroPython bindings), then apply the changes above. The sine LUT, LEDC config, GPTimer config, and all state variables carry over.

Remove from pwm.c:
- `ensure_pol_gpio_init()` — replaced by `hv509_pol_init()` called from hv509.c
- `init_polarity` Python binding — replaced by `pol_init` binding already in pz_drive.c
- Direct `gpio_set_level(POL_A_GPIO/POL_B_GPIO)` — replaced by `hv509_pol_set()`/`hv509_pol_toggle()`

Keep the same `#include` for `driver/ledc.h`, `driver/gptimer.h`, `esp_timer.h`.

**Step 2: Add Python bindings in pz_drive.c**

```c
// ── pwm_set_frequency(hz, ...) ──────────────────────────────────────────
// Same kwargs as pz_pwm: hz, resolution=8, amplitude=128, fullwave=False,
//                         dead_time=0, phase_advance=0, waveform=0
// (Copy the STATIC mp_obj_t function + allowed_args from pz_pwm.c:312-410,
//  but call the new internal functions instead of the pz_pwm_ prefixed ones)

// ── pwm_start() ─────────────────────────────────────────────────────────
// ── pwm_stop() ──────────────────────────────────────────────────────────
// ── pwm_is_running() ────────────────────────────────────────────────────
```

Module table entries:
```c
{ MP_ROM_QSTR(MP_QSTR_pwm_set_frequency), MP_ROM_PTR(&pz_drive_pwm_set_frequency_obj) },
{ MP_ROM_QSTR(MP_QSTR_pwm_start), MP_ROM_PTR(&pz_drive_pwm_start_obj) },
{ MP_ROM_QSTR(MP_QSTR_pwm_stop), MP_ROM_PTR(&pz_drive_pwm_stop_obj) },
{ MP_ROM_QSTR(MP_QSTR_pwm_is_running), MP_ROM_PTR(&pz_drive_pwm_is_running_obj) },
```

**Step 3: Build**

Run: local build
Expected: clean compile

**Step 4: Flash and verify analog path on hardware**

```python
import pz_drive
pz_drive.pol_init()
pz_drive.pwm_set_frequency(250, fullwave=True, dead_time=8, phase_advance=3)
# Need to init DRV2665 analog mode manually for now
pz_drive.i2c_write(0x02, 0x00)  # clear standby
pz_drive.i2c_write(0x01, 0x00)  # analog mode, gain 100
pz_drive.i2c_write(0x02, 0x02)  # EN_OVERRIDE
pz_drive.pwm_start()
# Verify on scope: 250 Hz fullwave sine
pz_drive.pwm_stop()
```

Also test latch:
```python
pz_drive.pwm_set_frequency(250)
pz_drive.pwm_start()
pz_drive.sr_stage(0x02000000)  # pin 0 on — should latch at next cycle start
import time; time.sleep_ms(100)
pz_drive.sr_stage(0)            # pin 0 off
pz_drive.pwm_stop()
```

**Step 5: Commit**

```bash
git add modules/pz_drive/pwm.c modules/pz_drive/pz_drive.c
git commit -m "feat(pz_drive): implement analog DDS ISR with shift register latch"
```

---

## Task 5: Implement fifo.c — digital FIFO task with fullwave + latch

**Files:**
- Modify: `modules/pz_drive/fifo.c` (replace stub with real implementation)
- Modify: `modules/pz_drive/pz_drive.h` (add fifo function prototypes if needed)
- Modify: `modules/pz_drive/pz_drive.c` (add fifo_start, fifo_stop, fifo_is_running bindings)

**Reference:** `modules/pz_fifo/pz_fifo.c` (hybrid fill algorithm), `modules/pz_actuator/task.c` (background task structure)

**Step 1: Implement fifo.c**

Port `modules/pz_fifo/pz_fifo.c` with these changes:
- **Remove MicroPython I2C struct extraction** — use `drv2665_*` functions from drv2665.c instead
- **Add fullwave support**: track `write_index`, toggle polarity at period boundary (index wraps to 0)
- **Add shift register latch**: call `hv509_sr_latch_if_pending()` at period boundary
- **DRV2665 init/standby**: call `drv2665_write_reg()` directly instead of owning the I2C object

Key additions for fullwave:

```c
static volatile bool s_fullwave = false;
static size_t s_waveform_len = 0;
static size_t s_write_index = 0;

// In the fill loop, track write_index:
static uint8_t next_sample(void) {
    uint8_t val = s_waveform[s_write_index];
    s_write_index++;
    if (s_write_index >= s_waveform_len) {
        s_write_index = 0;
        // Period boundary — latch + polarity toggle
        if (s_fullwave) {
            hv509_pol_toggle();
        }
        hv509_sr_latch_if_pending();
    }
    return val;
}
```

The `fifo_start()` internal function takes `(waveform_buf, len, gain, fullwave)`.

DRV2665 digital init sequence (inside `fifo_start`):
```c
drv2665_write_reg(0x02, 0x00);  // clear standby
drv2665_write_reg(0x01, 0x04 | (gain & 0x03));  // digital mode + gain
// No EN_OVERRIDE for digital mode — auto-wakes on FIFO write
```

DRV2665 standby (inside `fifo_stop`):
```c
drv2665_write_reg(0x02, 0x40);  // standby
```

**Step 2: Add Python bindings in pz_drive.c**

```c
// ── fifo_start(waveform_buf, gain=3, fullwave=False) ────────────────────
static mp_obj_t pz_drive_fifo_start(size_t n_args, const mp_obj_t *pos_args,
                                     mp_map_t *kw_args) {
    enum { ARG_waveform, ARG_gain, ARG_fullwave };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_waveform, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_gain, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 3} },
        { MP_QSTR_fullwave, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_waveform].u_obj, &bufinfo, MP_BUFFER_READ);

    fifo_start_internal(bufinfo.buf, bufinfo.len,
                        args[ARG_gain].u_int,
                        args[ARG_fullwave].u_bool);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_fifo_start_obj, 1, pz_drive_fifo_start);

// ── fifo_stop() ─────────────────────────────────────────────────────────
// ── fifo_is_running() ───────────────────────────────────────────────────
```

Module table entries:
```c
{ MP_ROM_QSTR(MP_QSTR_fifo_start), MP_ROM_PTR(&pz_drive_fifo_start_obj) },
{ MP_ROM_QSTR(MP_QSTR_fifo_stop), MP_ROM_PTR(&pz_drive_fifo_stop_obj) },
{ MP_ROM_QSTR(MP_QSTR_fifo_is_running), MP_ROM_PTR(&pz_drive_fifo_is_running_obj) },
```

**Step 3: Build**

Run: local build
Expected: clean compile

**Step 4: Flash and verify digital path on hardware**

```python
import pz_drive, math
pz_drive.pol_init()
# Generate 250 Hz sine (8000/250 = 32 samples)
n = 32
wav = bytearray(
    (int(127 * math.sin(-math.pi/2 + 2*math.pi*i/n)) & 0xFF)
    for i in range(n)
)
pz_drive.sr_write(0x03FFFFC0)  # all pins on
pz_drive.fifo_start(wav, gain=3, fullwave=False)
import time; time.sleep(2)
pz_drive.fifo_stop()
```

Test fullwave:
```python
# Half-period |sin|
n = 32
wav = bytearray(
    (int(127 * abs(math.sin(math.pi * i / n))) & 0xFF)
    for i in range(n)
)
pz_drive.fifo_start(wav, gain=3, fullwave=True)
import time; time.sleep(2)
pz_drive.fifo_stop()
```

**Step 5: Commit**

```bash
git add modules/pz_drive/fifo.c modules/pz_drive/pz_drive.c modules/pz_drive/pz_drive.h
git commit -m "feat(pz_drive): implement digital FIFO task with fullwave + latch"
```

---

## Task 6: Update Python DRV2665 class

**Files:**
- Modify: `python/drv2665.py`

**Step 1: Rewrite drv2665.py to delegate to pz_drive**

```python
# python/drv2665.py
import pz_drive

# WARNING: Heavy I2C access (tight loops of _read_reg/_write_reg)
# during active FIFO playback may cause audio glitches. The DRV2665
# FIFO holds 100 samples (12.5 ms at 8 kHz), so occasional reads
# are fine, but avoid burst access patterns.

# Register addresses
_STATUS = 0x00
_CTRL1  = 0x01
_CTRL2  = 0x02

# Gain settings (CTRL1 bits [1:0])
GAIN_25  = 0x00
GAIN_50  = 0x01
GAIN_75  = 0x02
GAIN_100 = 0x03

# CTRL1 bits
_INPUT_DIGITAL = 0x04  # bit 2

# CTRL2 bits
_STANDBY     = 0x40  # bit 6
_EN_OVERRIDE = 0x02  # bit 1


class DRV2665:
    GAIN_25  = GAIN_25
    GAIN_50  = GAIN_50
    GAIN_75  = GAIN_75
    GAIN_100 = GAIN_100

    def __init__(self):
        # I2C bus is owned by pz_drive C module
        val = self._read_reg(_STATUS)
        if val < 0:
            raise RuntimeError("DRV2665 not responding")

    def init_analog(self, gain=GAIN_100):
        self._write_reg(_CTRL2, 0x00)                # clear standby
        self._write_reg(_CTRL1, gain & 0x03)          # analog mode + gain
        self._write_reg(_CTRL2, _EN_OVERRIDE)         # enable boost + amp

    def init_digital(self, gain=GAIN_100):
        self._write_reg(_CTRL2, 0x00)                 # clear standby
        self._write_reg(_CTRL1, _INPUT_DIGITAL | (gain & 0x03))  # digital + gain

    def standby(self):
        self._write_reg(_CTRL2, _STANDBY)

    def status(self):
        return self._read_reg(_STATUS)

    def _read_reg(self, reg):
        return pz_drive.i2c_read(reg)

    def _write_reg(self, reg, val):
        pz_drive.i2c_write(reg, val)
```

**Step 2: Build and flash**

Run: local build + flash
Expected: `from drv2665 import DRV2665; d = DRV2665(); print(hex(d.status()))` prints status

**Step 3: Commit**

```bash
git add python/drv2665.py
git commit -m "refactor(drv2665): delegate I2C to pz_drive C module"
```

---

## Task 7: Update Python ShiftRegister class

**Files:**
- Modify: `python/shift_register.py`

**Step 1: Rewrite shift_register.py to delegate to pz_drive**

```python
# python/shift_register.py
import pz_drive


class ShiftRegister:
    """HV509 daisy-chained shift register driver.

    Maintains a Python-side 32-bit buffer with pin-to-bit mapping.
    Delegates SPI writes to pz_drive C module for ISR-synchronized latching.
    """

    def __init__(self):
        self._state = 0

    @staticmethod
    def _pin_bit(pin):
        if pin < 0 or pin > 19:
            raise ValueError("pin must be 0-19")
        return 1 << (25 - pin)

    def set_pin(self, pin, value, latch=True):
        bit = self._pin_bit(pin)
        if value:
            self._state |= bit
        else:
            self._state &= ~bit
        if latch:
            self.latch()

    def get_pin(self, pin):
        return bool(self._state & self._pin_bit(pin))

    def set_all(self, value, latch=True):
        if value:
            self._state = 0x03FFFFC0
        else:
            self._state = 0
        if latch:
            self.latch()

    def get_all(self):
        return tuple(
            1 if (self._state & (1 << (25 - i))) else 0
            for i in range(20)
        )

    def set_pins(self, values, latch=True):
        state = 0
        for i, v in enumerate(values):
            if i > 19:
                break
            if v:
                state |= 1 << (25 - i)
        self._state = state
        if latch:
            self.latch()

    def latch(self):
        pz_drive.sr_stage(self._state)

    def _direct_write(self, word32):
        pz_drive.sr_write(word32)
```

**Step 2: Build and flash**

Run: local build + flash
Expected:
```python
from shift_register import ShiftRegister
sr = ShiftRegister()
sr.set_all(1)          # all pins on
print(sr.get_all())    # (1, 1, 1, ..., 1)
sr.set_all(0)          # all pins off
sr.set_pin(4, 1)       # pin 4 on
print(sr.get_pin(4))   # True
```

**Step 3: Commit**

```bash
git add python/shift_register.py
git commit -m "refactor(shift_register): delegate SPI to pz_drive, rename flush to latch"
```

---

## Task 8: Update Python PzActuator class

**Files:**
- Modify: `python/pz_actuator_py.py`

**Step 1: Rewrite pz_actuator_py.py**

Key changes:
- Remove `machine.I2C`, `machine.SPI`, `machine.Pin` imports — pz_drive owns buses
- Remove `import pz_fifo`, `import pz_pwm` — replaced by `import pz_drive`
- `__init__` no longer creates I2C/SPI objects
- `start()` for digital mode calls `pz_drive.fifo_start()` directly
- `start()` for analog mode uses `self.drv.init_analog()` + `pz_drive.pwm_start()`
- `stop()` uses `pz_drive.fifo_stop()` / `pz_drive.pwm_stop()` + `self.drv.standby()`
- Rename `flush` to `latch` in pass-through methods
- `toggle_polarity()` / `get_polarity()` use `pz_drive.pol_set()`/`pz_drive.pol_get()`
- `set_frequency_digital()` gets `fullwave` and `waveform` parameters

```python
# python/pz_actuator_py.py
import math
import pz_drive
from drv2665 import DRV2665
from shift_register import ShiftRegister


MODE_DIGITAL = 'digital'
MODE_ANALOG = 'analog'


class PzActuator:
    """High-level piezo actuator controller.

    Wraps DRV2665 (I2C), HV509 shift registers (SPI), and
    real-time C module (pz_drive).
    """

    GAINS = {
        25: DRV2665.GAIN_25,
        50: DRV2665.GAIN_50,
        75: DRV2665.GAIN_75,
        100: DRV2665.GAIN_100,
    }

    WAVEFORMS = {'sine': 0, 'triangle': 1, 'square': 2}

    def __init__(self):
        # pz_drive owns I2C and SPI buses — init happens on first use
        self.drv = DRV2665()
        self.sr = ShiftRegister()

        self._mode = None
        self._waveform = None
        self._waveform_name = 'sine'
        self._gain = 100
        self._fullwave = False

    def set_frequency_digital(self, hz, fullwave=False, waveform='sine'):
        """Configure digital FIFO mode at given frequency.

        Args:
            hz: 1-4000
            fullwave: if True, generate |waveform| half-period buffer,
                      fifo.c toggles polarity at period boundary
            waveform: 'sine', 'triangle', or 'square'
        """
        if hz < 1 or hz > 4000:
            raise ValueError("hz must be 1-4000")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        n_samples = round(8000 / hz)
        if n_samples < 2:
            n_samples = 2

        if fullwave:
            # Half-period |waveform| — fifo.c handles polarity toggle
            if waveform == 'sine':
                self._waveform = bytearray(
                    int(127 * math.sin(math.pi * i / n_samples)) & 0xFF
                    for i in range(n_samples)
                )
            elif waveform == 'triangle':
                self._waveform = bytearray(
                    int(127 * (2 * i / n_samples if i < n_samples // 2
                               else 2 * (n_samples - i) / n_samples)) & 0xFF
                    for i in range(n_samples)
                )
            elif waveform == 'square':
                self._waveform = bytearray(127 for _ in range(n_samples))
        else:
            # Full-period waveform (trough at index 0 for sine)
            if waveform == 'sine':
                self._waveform = bytearray(
                    (int(127 * math.sin(-math.pi / 2 + 2 * math.pi * i / n_samples)) & 0xFF)
                    for i in range(n_samples)
                )
            elif waveform == 'triangle':
                self._waveform = bytearray(
                    (int(127 * (4 * i / n_samples - 1 if i < n_samples // 2
                                else 3 - 4 * i / n_samples)) & 0xFF)
                    for i in range(n_samples)
                )
            elif waveform == 'square':
                half = n_samples // 2
                self._waveform = bytearray(
                    (127 if i < half else ((-128) & 0xFF))
                    for i in range(n_samples)
                )

        self._fullwave = fullwave
        self._waveform_name = waveform
        self._mode = MODE_DIGITAL

    def set_frequency_analog(self, hz, resolution=8, amplitude=100, fullwave=False,
                             dead_time=0, phase_advance=0, waveform='sine'):
        """Configure analog PWM+DDS mode at given frequency.

        Args:
            hz: 0-400 (0 = DC output)
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage, mapped to internal 0-128)
            fullwave: if True, generate |waveform| and toggle polarity at zero-crossings
            dead_time: ISR ticks (at 32 kHz) to force zero output near each
                       zero-crossing, giving the DRV2665 output time to settle
            phase_advance: ISR ticks to advance polarity toggle, compensating
                           for DRV2665 output lag (~3 ticks at 250 Hz)
            waveform: 'sine', 'triangle', or 'square'
        """
        if hz < 0 or hz > 400:
            raise ValueError("hz must be 0-400")
        if waveform not in self.WAVEFORMS:
            raise ValueError("waveform must be 'sine', 'triangle', or 'square'")
        amp_internal = (amplitude * 128 + 50) // 100
        pz_drive.pwm_set_frequency(hz, resolution=resolution, amplitude=amp_internal,
                                   fullwave=fullwave, dead_time=dead_time,
                                   phase_advance=phase_advance,
                                   waveform=self.WAVEFORMS[waveform])
        self._fullwave = fullwave
        self._mode = MODE_ANALOG

    def start(self, gain=100):
        """Start output in the configured mode."""
        if self._mode is None:
            raise RuntimeError("call set_frequency_digital() or set_frequency_analog() first")
        if gain not in self.GAINS:
            raise ValueError("gain must be 25, 50, 75, or 100")
        self._gain = gain
        gain_bits = self.GAINS[gain]

        if self._mode == MODE_DIGITAL:
            pz_drive.fifo_start(self._waveform, gain=gain_bits,
                                fullwave=self._fullwave)
        elif self._mode == MODE_ANALOG:
            self.drv.init_analog(gain_bits)
            pz_drive.pwm_start()

    def stop(self):
        """Stop output and put DRV2665 in standby."""
        if pz_drive.fifo_is_running():
            pz_drive.fifo_stop()
        if pz_drive.pwm_is_running():
            pz_drive.pwm_stop()
        self.drv.standby()

    def is_running(self):
        return pz_drive.fifo_is_running() or pz_drive.pwm_is_running()

    # ── Shift register pass-through ─────────────────────────────────────
    def set_pin(self, pin, value, latch=True):
        self.sr.set_pin(pin, value, latch=latch)

    def get_pin(self, pin):
        return self.sr.get_pin(pin)

    def set_pins(self, values, latch=True):
        self.sr.set_pins(values, latch=latch)

    def get_all(self):
        return self.sr.get_all()

    def set_all(self, value, latch=True):
        self.sr.set_all(value, latch=latch)

    def latch(self):
        self.sr.latch()

    def toggle_polarity(self):
        pz_drive.pol_set(not pz_drive.pol_get())

    def get_polarity(self):
        return pz_drive.pol_get()
```

**Step 2: Build and flash**

Run: local build + flash

**Step 3: Verify on hardware — analog mode**

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
pa.set_frequency_analog(250, fullwave=True, dead_time=8, phase_advance=3)
pa.set_all(1)
pa.start(gain=100)
# Verify on scope: 250 Hz fullwave sine, all pins on
pa.stop()
```

**Step 4: Verify on hardware — digital mode**

```python
pa.set_frequency_digital(250)
pa.set_all(1)
pa.start(gain=100)
# Verify on scope: 250 Hz digital sine
pa.stop()
```

**Step 5: Verify on hardware — digital fullwave**

```python
pa.set_frequency_digital(250, fullwave=True, waveform='sine')
pa.set_all(1)
pa.start(gain=100)
# Verify on scope: 500 Hz output (fullwave), polarity toggling
pa.stop()
```

**Step 6: Commit**

```bash
git add python/pz_actuator_py.py
git commit -m "refactor(pz_actuator): use pz_drive, add digital fullwave/waveform, rename flush to latch"
```

---

## Task 9: Update main.py

**Files:**
- Modify: `python/main.py`

**Step 1: Read current main.py**

Read the file to understand demo functions and imports.

**Step 2: Update imports and usage**

- Replace `import pz_pwm` / `import pz_fifo` with `import pz_drive`
- Replace `pz_pwm.init_polarity()` calls — no longer needed (pz_drive auto-inits)
- Update any `flush()` calls to `latch()`
- Update demo functions to use new API

**Step 3: Build, flash, verify**

**Step 4: Commit**

```bash
git add python/main.py
git commit -m "refactor(main): update imports for pz_drive module"
```

---

## Task 10: Remove old modules

**Files:**
- Delete: `modules/pz_pwm/` (entire directory)
- Delete: `modules/pz_fifo/` (entire directory)
- Delete: `modules/pz_actuator/` (entire directory)
- Modify: `modules/micropython.cmake` (remove old includes)

**Step 1: Update modules/micropython.cmake**

Final content:
```cmake
include(${CMAKE_CURRENT_LIST_DIR}/pz_drive/micropython.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/board_utils/micropython.cmake)
```

**Step 2: Delete old module directories**

```bash
rm -rf modules/pz_pwm/ modules/pz_fifo/ modules/pz_actuator/
```

**Step 3: Build**

Run: local build
Expected: clean compile with only pz_drive + board_utils

**Step 4: Flash and verify**

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
# Quick smoke test of all modes
pa.set_frequency_analog(250, fullwave=True, dead_time=8, phase_advance=3)
pa.set_all(1)
pa.start()
import time; time.sleep(1)
pa.stop()

pa.set_frequency_digital(250, fullwave=True, waveform='triangle')
pa.start()
time.sleep(1)
pa.stop()
```

**Step 5: Commit**

```bash
git add -A modules/
git commit -m "refactor: remove old pz_pwm, pz_fifo, pz_actuator modules"
```

---

## Task 11: Update CLAUDE.md

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Update architecture and API docs**

Replace references to `pz_pwm`, `pz_fifo`, `drv_fifo` with `pz_drive`. Update:
- Project structure section
- Python API section
- C Module APIs section
- Key design patterns

**Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md for pz_drive unified module"
```
