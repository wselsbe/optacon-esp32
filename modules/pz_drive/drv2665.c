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

__attribute__((weak)) void drv2665_write_fifo_byte(uint8_t val) {
    uint8_t buf[2] = { 0x0B, val };
    i2c_master_transmit(s_dev, buf, 2, I2C_TIMEOUT_MS);
}

__attribute__((weak)) uint8_t drv2665_read_status(void) {
    uint8_t reg = 0x00;
    uint8_t val = 0;
    i2c_master_transmit_receive(s_dev, &reg, 1, &val, 1, I2C_TIMEOUT_MS);
    return val;
}
