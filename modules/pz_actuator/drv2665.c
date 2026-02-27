#include "drv2665.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "py/mpprint.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "drv2665";

esp_err_t drv2665_init(drv2665_t *dev) {
    // Create I2C master bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1, // auto-select
        .sda_io_num = DRV2665_I2C_SDA_PIN,
        .scl_io_num = DRV2665_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &dev->bus);
    if (err != ESP_OK) return err;

    // Add DRV2665 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2665_I2C_ADDR,
        .scl_speed_hz = DRV2665_I2C_CLK_HZ,
    };
    err = i2c_master_bus_add_device(dev->bus, &dev_cfg, &dev->dev);
    if (err != ESP_OK) {
        i2c_del_master_bus(dev->bus);
        return err;
    }

    dev->gain = DRV2665_GAIN_100V;

    // Datasheet 8.3.1: step 2 — wait 1ms after power-up before I2C writes
    TickType_t ticks = pdMS_TO_TICKS(2);
    vTaskDelay(ticks > 0 ? ticks : 1);

    // Verify communication by reading status register
    uint8_t status;
    err = drv2665_read_register(dev, DRV2665_REG_STATUS, &status);
    if (err != ESP_OK) {
        drv2665_deinit(dev);
        return err;
    }

    return ESP_OK;
}

esp_err_t drv2665_deinit(drv2665_t *dev) {
    if (dev->dev) {
        i2c_master_bus_rm_device(dev->dev);
        dev->dev = NULL;
    }
    if (dev->bus) {
        i2c_del_master_bus(dev->bus);
        dev->bus = NULL;
    }
    return ESP_OK;
}

esp_err_t drv2665_write_register(drv2665_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev->dev, buf, 2, 100);
}

esp_err_t drv2665_read_register(drv2665_t *dev, uint8_t reg, uint8_t *value) {
    return i2c_master_transmit_receive(dev->dev, &reg, 1, value, 1, 100);
}

esp_err_t drv2665_enable_digital(drv2665_t *dev, uint8_t gain) {
    dev->gain = gain & 0x03;

    // Datasheet 8.3.1: step 3 — exit standby (clear STANDBY bit)
    esp_err_t err = drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    // Datasheet 8.3.1: steps 4-5 — set digital input mode + gain
    err = drv2665_write_register(dev, DRV2665_REG_CTRL1, DRV2665_INPUT_DIGITAL | dev->gain);
    if (err != ESP_OK) return err;

    // Datasheet 8.3.1: step 6 — set timeout
    err = drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t drv2665_enable_analog(drv2665_t *dev, uint8_t gain) {
    dev->gain = gain & 0x03;

    // Datasheet 8.3.1: step 3 — exit standby (clear STANDBY bit)
    esp_err_t err = drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    // Datasheet 8.3.1: steps 4-5 — set analog input mode + gain
    err = drv2665_write_register(dev, DRV2665_REG_CTRL1, DRV2665_INPUT_ANALOG | dev->gain);
    if (err != ESP_OK) return err;

    // Datasheet 8.3.1: step 7 — for analog, set EN_OVERRIDE to enable boost + amplifier
    err = drv2665_write_register(dev, DRV2665_REG_CTRL2,
                                 DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t drv2665_standby(drv2665_t *dev) {
    return drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_STANDBY);
}

int drv2665_write_fifo(drv2665_t *dev, const int8_t *data, size_t len) {
    // Build I2C message: register address byte + data bytes
    if (len > DRV2665_FIFO_SIZE) len = DRV2665_FIFO_SIZE;
    uint8_t buf[DRV2665_FIFO_SIZE + 1];
    size_t buf_len = 1 + len;

    buf[0] = DRV2665_REG_DATA;
    memcpy(&buf[1], data, len);

    esp_err_t err = i2c_master_transmit(dev->dev, buf, buf_len, 100);

    if (err != ESP_OK) return 0;
    // Native I2C doesn't tell us partial write count on NACK.
    // We assume all bytes accepted if no error. Use status register
    // to check FIFO_FULL after the write.
    return (int)len;
}

esp_err_t drv2665_write_fifo_byte(drv2665_t *dev, int8_t sample) {
    uint8_t buf[2] = {DRV2665_REG_DATA, (uint8_t)sample};
    return i2c_master_transmit(dev->dev, buf, 2, 100);
}

esp_err_t drv2665_read_status(drv2665_t *dev, uint8_t *status) {
    return drv2665_read_register(dev, DRV2665_REG_STATUS, status);
}
