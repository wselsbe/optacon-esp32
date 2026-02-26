#ifndef SHIFT_REGISTER_H
#define SHIFT_REGISTER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define SHIFTREG_NUM_PINS   20
#define SHIFTREG_DATA_BYTES 4

// SPI configuration
#define SHIFTREG_SPI_HOST   SPI2_HOST
#define SHIFTREG_SPI_MOSI   6
#define SHIFTREG_SPI_MISO   7
#define SHIFTREG_SPI_SCK    9
#define SHIFTREG_SPI_CS     10
#define SHIFTREG_SPI_CLK_HZ (1 * 1000 * 1000)

// Polarity GPIO
#define SHIFTREG_POLARITY_PIN 34

// 32-bit layout: [31..26]=common_U1(always 0) [25..6]=pins_0..19 [5..0]=common_U2(always 0)
// Pin N (0-indexed) maps to bit (25 - N).
#define SHIFTREG_PIN_BIT(pin)  (1U << (25 - (pin)))
#define SHIFTREG_COMMON_MASK   0xFC00003FU // bits that must always be 0
#define SHIFTREG_ALL_PINS_MASK 0x03FFFFC0U // all 20 actuator pins

typedef struct {
    void *spi_dev;          // spi_device_handle_t
    uint32_t active_state;  // currently latched state
    uint32_t pending_state; // pending state (buffered mode)
    bool pending_commit;    // flag: commit pending state at next trough
    bool pending_polarity;  // flag: toggle polarity at next trough
    bool polarity;          // current polarity (true = high)
} shift_register_t;

esp_err_t shift_register_init(shift_register_t *sr);
esp_err_t shift_register_deinit(shift_register_t *sr);
void shift_register_set_pin(shift_register_t *sr, uint8_t pin, bool value);
bool shift_register_get_pin(const shift_register_t *sr, uint8_t pin);
void shift_register_set_all(shift_register_t *sr, bool value);
void shift_register_set_pins(shift_register_t *sr, const bool *values);
void shift_register_request_commit(shift_register_t *sr);
void shift_register_request_polarity_toggle(shift_register_t *sr);
esp_err_t shift_register_commit(shift_register_t *sr);

#endif // SHIFT_REGISTER_H
