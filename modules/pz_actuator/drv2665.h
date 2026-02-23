#ifndef DRV2665_H
#define DRV2665_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// I2C configuration
#define DRV2665_I2C_ADDR    0x59
#define DRV2665_I2C_CLK_HZ  100000
#define DRV2665_I2C_SDA_PIN 47
#define DRV2665_I2C_SCL_PIN 21

// Registers
#define DRV2665_REG_STATUS  0x00
#define DRV2665_REG_CTRL1   0x01
#define DRV2665_REG_CTRL2   0x02
#define DRV2665_REG_DATA    0x0B

// Status register bits
#define DRV2665_FIFO_FULL   0x01
#define DRV2665_FIFO_EMPTY  0x02

// Control register 1
#define DRV2665_INPUT_DIGITAL  (0 << 2)
#define DRV2665_INPUT_ANALOG   (1 << 2)
#define DRV2665_GAIN_25V       0
#define DRV2665_GAIN_50V       1
#define DRV2665_GAIN_75V       2
#define DRV2665_GAIN_100V      3

// Control register 2
#define DRV2665_RESET          (1 << 7)
#define DRV2665_STANDBY        (1 << 6)
#define DRV2665_TIMEOUT_5MS    (0 << 2)
#define DRV2665_TIMEOUT_10MS   (1 << 2)
#define DRV2665_TIMEOUT_15MS   (2 << 2)
#define DRV2665_TIMEOUT_20MS   (3 << 2)
#define DRV2665_ENABLE_AUTO    (0 << 1)
#define DRV2665_ENABLE_OVERRIDE (1 << 1)

#define DRV2665_FIFO_SIZE      100
#define DRV2665_SAMPLE_RATE    8000

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t gain;     // current gain setting (0-3)
} drv2665_t;

esp_err_t drv2665_init(drv2665_t *dev);
esp_err_t drv2665_deinit(drv2665_t *dev);
esp_err_t drv2665_write_register(drv2665_t *dev, uint8_t reg, uint8_t value);
esp_err_t drv2665_read_register(drv2665_t *dev, uint8_t reg, uint8_t *value);
esp_err_t drv2665_enable_digital(drv2665_t *dev, uint8_t gain);
esp_err_t drv2665_standby(drv2665_t *dev);
int drv2665_write_fifo(drv2665_t *dev, const int8_t *data, size_t len);
esp_err_t drv2665_read_status(drv2665_t *dev, uint8_t *status);

#endif // DRV2665_H
