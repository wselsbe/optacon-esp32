#include "drv2665.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "py/mpprint.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "drv2665";

esp_err_t drv2665_init(drv2665_t *dev) {
    mp_printf(&mp_plat_print, "  drv2665: init (sda=%d, scl=%d, addr=0x%02x)\n",
              DRV2665_I2C_SDA_PIN, DRV2665_I2C_SCL_PIN, DRV2665_I2C_ADDR);

    soft_i2c_init(&dev->i2c, DRV2665_I2C_SDA_PIN, DRV2665_I2C_SCL_PIN, DRV2665_I2C_CLK_HZ);
    dev->gain = DRV2665_GAIN_100V;

    // Reset device to known state
    esp_err_t err = drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_RESET);
    mp_printf(&mp_plat_print, "  drv2665: reset: err=%d\n", err);
    if (err != ESP_OK) {
        drv2665_deinit(dev);
        return err;
    }

    // Wait for device to come out of reset (enters standby after reset)
    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

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
    soft_i2c_deinit(&dev->i2c);
    return ESP_OK;
}

esp_err_t drv2665_write_register(drv2665_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int ret = soft_i2c_write(&dev->i2c, DRV2665_I2C_ADDR, buf, 2);
    return (ret == 2) ? ESP_OK : ESP_FAIL;
}

esp_err_t drv2665_read_register(drv2665_t *dev, uint8_t reg, uint8_t *value) {
    int ret = soft_i2c_write_read(&dev->i2c, DRV2665_I2C_ADDR, &reg, 1, value, 1);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t drv2665_enable_digital(drv2665_t *dev, uint8_t gain) {
    dev->gain = gain & 0x03;

    // Exit standby first — device needs time before CTRL1 can be configured
    esp_err_t err = drv2665_write_register(dev, DRV2665_REG_CTRL2,
                                            DRV2665_ENABLE_OVERRIDE | DRV2665_TIMEOUT_20MS);
    if (err != ESP_OK) return err;

    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);

    // Configure input mode + gain (after device is fully active)
    err = drv2665_write_register(dev, DRV2665_REG_CTRL1,
                                  DRV2665_INPUT_DIGITAL | dev->gain);
    if (err != ESP_OK) return err;

    // Read back to verify
    uint8_t ctrl1_rb;
    drv2665_read_register(dev, DRV2665_REG_CTRL1, &ctrl1_rb);
    mp_printf(&mp_plat_print, "  drv2665: CTRL1 readback=0x%02x (expected 0x%02x)\n",
              ctrl1_rb, DRV2665_INPUT_DIGITAL | dev->gain);

    return ESP_OK;
}

esp_err_t drv2665_standby(drv2665_t *dev) {
    return drv2665_write_register(dev, DRV2665_REG_CTRL2, DRV2665_STANDBY);
}

int drv2665_write_fifo(drv2665_t *dev, const int8_t *data, size_t len) {
    // Build I2C message: register address byte + data bytes
    // Use stack buffer (max 101 bytes) to avoid malloc from task context.
    if (len > DRV2665_FIFO_SIZE) len = DRV2665_FIFO_SIZE;
    uint8_t buf[DRV2665_FIFO_SIZE + 1];
    size_t buf_len = 1 + len;

    buf[0] = DRV2665_REG_DATA;
    memcpy(&buf[1], data, len);

    int written = soft_i2c_write(&dev->i2c, DRV2665_I2C_ADDR, buf, buf_len);

    if (written < 0) return 0;       // address NACK or timeout
    if (written <= 1) return 0;       // only register addr byte ACKed, no data
    return written - 1;               // subtract the register address byte
}

esp_err_t drv2665_read_status(drv2665_t *dev, uint8_t *status) {
    return drv2665_read_register(dev, DRV2665_REG_STATUS, status);
}
