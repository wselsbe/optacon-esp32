#include "drv2665.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "py/mpprint.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "drv2665";

esp_err_t drv2665_init(drv2665_t *dev) {
    mp_printf(&mp_plat_print, "  drv2665: dev=%p i2c_bus=%p i2c_dev=%p\n", dev, dev->i2c_bus, dev->i2c_dev);

    // Clean up any previous state (e.g. after soft reset)
    if (dev->i2c_dev) {
        mp_printf(&mp_plat_print, "  drv2665: cleaning up old i2c_dev\n");
        i2c_master_bus_rm_device(dev->i2c_dev);
        dev->i2c_dev = NULL;
    }
    if (dev->i2c_bus) {
        mp_printf(&mp_plat_print, "  drv2665: cleaning up old i2c_bus\n");
        i2c_del_master_bus(dev->i2c_bus);
        dev->i2c_bus = NULL;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = DRV2665_I2C_SCL_PIN,
        .sda_io_num = DRV2665_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    mp_printf(&mp_plat_print, "  drv2665: calling i2c_new_master_bus(port=%d, sda=%d, scl=%d)...\n",
              bus_cfg.i2c_port, bus_cfg.sda_io_num, bus_cfg.scl_io_num);
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    mp_printf(&mp_plat_print, "  drv2665: i2c_new_master_bus returned %d (0x%03x), bus=%p\n", err, err, bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return err;
    }
    dev->i2c_bus = bus;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2665_I2C_ADDR,
        .scl_speed_hz = DRV2665_I2C_CLK_HZ,
    };
    i2c_master_dev_handle_t i2c_dev;
    err = i2c_master_bus_add_device(bus, &dev_cfg, &i2c_dev);
    mp_printf(&mp_plat_print, "  drv2665: add_device(0x%02x): err=%d\n", DRV2665_I2C_ADDR, err);
    if (err != ESP_OK) {
        i2c_del_master_bus(bus);
        dev->i2c_bus = NULL;
        return err;
    }
    dev->i2c_dev = i2c_dev;
    dev->gain = DRV2665_GAIN_100V;

    // Verify communication by reading status register
    uint8_t status;
    err = drv2665_read_register(dev, DRV2665_REG_STATUS, &status);
    mp_printf(&mp_plat_print, "  drv2665: read status: err=%d val=0x%02x\n", err, status);
    if (err != ESP_OK) {
        drv2665_deinit(dev);
        return err;
    }

    mp_printf(&mp_plat_print, "  drv2665: init complete\n");
    return ESP_OK;
}

esp_err_t drv2665_deinit(drv2665_t *dev) {
    if (dev->i2c_dev) {
        i2c_master_bus_rm_device(dev->i2c_dev);
        dev->i2c_dev = NULL;
    }
    if (dev->i2c_bus) {
        i2c_del_master_bus(dev->i2c_bus);
        dev->i2c_bus = NULL;
    }
    return ESP_OK;
}

esp_err_t drv2665_write_register(drv2665_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev->i2c_dev, buf, 2, 100);
}

esp_err_t drv2665_read_register(drv2665_t *dev, uint8_t reg, uint8_t *value) {
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, value, 1, 100);
}

esp_err_t drv2665_enable_digital(drv2665_t *dev, uint8_t gain) {
    dev->gain = gain & 0x03;
    esp_err_t err = drv2665_write_register(dev, DRV2665_REG_CTRL1,
                                            DRV2665_INPUT_DIGITAL | dev->gain);
    if (err != ESP_OK) return err;

    err = drv2665_write_register(dev, DRV2665_REG_CTRL2,
                                  DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

esp_err_t drv2665_standby(drv2665_t *dev) {
    return drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_STANDBY);
}

int drv2665_write_fifo(drv2665_t *dev, const int8_t *data, size_t len) {
    // Build I2C message: register address byte + data bytes
    // Try to write all `len` bytes at once. If FIFO full, DRV2665 NACKs.
    size_t buf_len = 1 + len;
    uint8_t *buf = malloc(buf_len);
    if (!buf) return -1;

    buf[0] = DRV2665_REG_DATA;
    memcpy(&buf[1], data, len);

    esp_err_t err = i2c_master_transmit(dev->i2c_dev, buf, buf_len, 100);
    free(buf);

    if (err == ESP_OK) {
        return (int)len;  // all bytes accepted
    }

    // If NACK, try writing one byte at a time to find how much fits
    int written = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t single_buf[2] = {DRV2665_REG_DATA, (uint8_t)data[i]};
        err = i2c_master_transmit(dev->i2c_dev, single_buf, 2, 100);
        if (err != ESP_OK) {
            break;
        }
        written++;
    }

    return written;
}

esp_err_t drv2665_read_status(drv2665_t *dev, uint8_t *status) {
    return drv2665_read_register(dev, DRV2665_REG_STATUS, status);
}
