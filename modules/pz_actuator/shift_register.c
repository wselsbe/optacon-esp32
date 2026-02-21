#include "shift_register.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "shiftreg";

// Pin-to-bit mapping: [byte_index, bit_mask]
static const uint8_t pin_map[SHIFTREG_NUM_PINS][2] = {
    {0, 1 << 1}, // pin 0
    {0, 1 << 0}, // pin 1
    {1, 1 << 7}, // pin 2
    {1, 1 << 6}, // pin 3
    {1, 1 << 5}, // pin 4
    {1, 1 << 4}, // pin 5
    {1, 1 << 3}, // pin 6
    {1, 1 << 2}, // pin 7
    {1, 1 << 1}, // pin 8
    {1, 1 << 0}, // pin 9
    {2, 1 << 7}, // pin 10
    {2, 1 << 6}, // pin 11
    {2, 1 << 5}, // pin 12
    {2, 1 << 4}, // pin 13
    {2, 1 << 3}, // pin 14
    {2, 1 << 2}, // pin 15
    {2, 1 << 1}, // pin 16
    {2, 1 << 0}, // pin 17
    {3, 1 << 7}, // pin 18
    {3, 1 << 6}, // pin 19
};

static void clear_common_bits(uint8_t *data) {
    data[0] &= ~SHIFTREG_COMMON_MASK_BYTE0;
    data[3] &= ~SHIFTREG_COMMON_MASK_BYTE3;
}

esp_err_t shift_register_init(shift_register_t *sr) {
    memset(sr, 0, sizeof(shift_register_t));
    sr->polarity = true;  // default high

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SHIFTREG_SPI_MOSI,
        .miso_io_num = SHIFTREG_SPI_MISO,
        .sclk_io_num = SHIFTREG_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SHIFTREG_DATA_BYTES,
    };
    esp_err_t err = spi_bus_initialize(SHIFTREG_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SHIFTREG_SPI_CLK_HZ,
        .mode = 0,
        .spics_io_num = SHIFTREG_SPI_CS,
        .queue_size = 1,
    };
    spi_device_handle_t spi;
    err = spi_bus_add_device(SHIFTREG_SPI_HOST, &dev_cfg, &spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
        return err;
    }
    sr->spi_dev = spi;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SHIFTREG_POLARITY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(SHIFTREG_POLARITY_PIN, sr->polarity ? 1 : 0);

    // Clear all outputs
    shift_register_set_all(sr, false);
    sr->pending_commit = true;
    shift_register_commit(sr);

    ESP_LOGI(TAG, "Shift register initialized");
    return ESP_OK;
}

esp_err_t shift_register_deinit(shift_register_t *sr) {
    if (sr->spi_dev) {
        spi_bus_remove_device(sr->spi_dev);
        sr->spi_dev = NULL;
    }
    spi_bus_free(SHIFTREG_SPI_HOST);
    return ESP_OK;
}

void shift_register_set_pin(shift_register_t *sr, uint8_t pin, bool value) {
    if (pin >= SHIFTREG_NUM_PINS) return;
    uint8_t byte_idx = pin_map[pin][0];
    uint8_t bit_mask = pin_map[pin][1];
    if (value) {
        sr->pending_state[byte_idx] |= bit_mask;
    } else {
        sr->pending_state[byte_idx] &= ~bit_mask;
    }
}

bool shift_register_get_pin(const shift_register_t *sr, uint8_t pin) {
    if (pin >= SHIFTREG_NUM_PINS) return false;
    uint8_t byte_idx = pin_map[pin][0];
    uint8_t bit_mask = pin_map[pin][1];
    return (sr->pending_state[byte_idx] & bit_mask) != 0;
}

void shift_register_set_all(shift_register_t *sr, bool value) {
    if (value) {
        sr->pending_state[0] = 0x03;  // bits 0-1 only
        sr->pending_state[1] = 0xFF;
        sr->pending_state[2] = 0xFF;
        sr->pending_state[3] = 0xC0;  // bits 6-7 only
    } else {
        memset(sr->pending_state, 0, SHIFTREG_DATA_BYTES);
    }
}

void shift_register_set_pins(shift_register_t *sr, const bool *values) {
    memset(sr->pending_state, 0, SHIFTREG_DATA_BYTES);
    for (uint8_t i = 0; i < SHIFTREG_NUM_PINS; i++) {
        if (values[i]) {
            sr->pending_state[pin_map[i][0]] |= pin_map[i][1];
        }
    }
}

void shift_register_request_commit(shift_register_t *sr) {
    sr->pending_commit = true;
}

void shift_register_request_polarity_toggle(shift_register_t *sr) {
    sr->pending_polarity = true;
}

esp_err_t shift_register_commit(shift_register_t *sr) {
    if (!sr->pending_commit && !sr->pending_polarity) {
        return ESP_OK;
    }

    if (sr->pending_commit) {
        memcpy(sr->active_state, sr->pending_state, SHIFTREG_DATA_BYTES);
        clear_common_bits(sr->active_state);

        spi_transaction_t trans = {
            .length = SHIFTREG_DATA_BYTES * 8,
            .tx_buffer = sr->active_state,
        };
        esp_err_t err = spi_device_polling_transmit(sr->spi_dev, &trans);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(err));
            return err;
        }
        sr->pending_commit = false;
    }

    if (sr->pending_polarity) {
        sr->polarity = !sr->polarity;
        gpio_set_level(SHIFTREG_POLARITY_PIN, sr->polarity ? 1 : 0);
        sr->pending_polarity = false;
    }

    return ESP_OK;
}
