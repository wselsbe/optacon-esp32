#include "shift_register.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "py/mpprint.h"
#include <string.h>

static const char *TAG = "shiftreg";

esp_err_t shift_register_init(shift_register_t *sr) {
    memset(sr, 0, sizeof(shift_register_t));
    sr->polarity = true; // default high

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
    uint32_t mask = SHIFTREG_PIN_BIT(pin);
    if (value) {
        sr->pending_state |= mask;
    } else {
        sr->pending_state &= ~mask;
    }
}

bool shift_register_get_pin(const shift_register_t *sr, uint8_t pin) {
    if (pin >= SHIFTREG_NUM_PINS) return false;
    return (sr->pending_state & SHIFTREG_PIN_BIT(pin)) != 0;
}

void shift_register_set_all(shift_register_t *sr, bool value) {
    sr->pending_state = value ? SHIFTREG_ALL_PINS_MASK : 0;
}

void shift_register_set_pins(shift_register_t *sr, const bool *values) {
    sr->pending_state = 0;
    for (uint8_t i = 0; i < SHIFTREG_NUM_PINS; i++) {
        if (values[i]) {
            sr->pending_state |= SHIFTREG_PIN_BIT(i);
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
    sr->active_state = sr->pending_state & ~SHIFTREG_COMMON_MASK;

    // Convert uint32_t to big-endian bytes for SPI (MSB first)
    uint8_t tx_buf[SHIFTREG_DATA_BYTES] = {
        (sr->active_state >> 24) & 0xFF,
        (sr->active_state >> 16) & 0xFF,
        (sr->active_state >> 8) & 0xFF,
        sr->active_state & 0xFF,
    };

    spi_transaction_t trans = {
        .length = SHIFTREG_DATA_BYTES * 8,
        .tx_buffer = tx_buf,
    };
    esp_err_t err = spi_device_polling_transmit(sr->spi_dev, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(err));
        return err;
    }

    if (sr->pending_polarity) {
        sr->polarity = !sr->polarity;
        gpio_set_level(SHIFTREG_POLARITY_PIN, sr->polarity ? 1 : 0);
        sr->pending_polarity = false;
    }

    return ESP_OK;
}
