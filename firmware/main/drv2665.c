
#include "driver/i2c.h"
#include "drv2665.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "esp_system.h"


static const char *TAG = "drv2665";

void drv_init() {
    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = I2C_SDA_PIN;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num = I2C_SCL_PIN;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = I2C_MASTER_CLK;
    i2c_conf.clk_flags = 0;

    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0));

    vTaskDelay(5*portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(drv_write_register(DRV_REGISTER_2, DRV_RESET));
}

esp_err_t drv_write_register(uint8_t register_address, uint8_t data) {
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (DRV2665_ADDR<<1)| I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, register_address, true);
    i2c_master_write_byte(i2c_cmd, data, true);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_cmd, pdMS_TO_TICKS(5));
    i2c_cmd_link_delete(i2c_cmd);
    return ret;
}


uint8_t drv_read_register(uint8_t register_address) {
    uint8_t data;
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (DRV2665_ADDR<<1)| I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, register_address, true);
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (DRV2665_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(i2c_cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_cmd, pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    i2c_cmd_link_delete(i2c_cmd);
    return data;
}

void drv_write_fifo(uint8_t* data, size_t data_len) {
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (DRV2665_ADDR<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, DRV_REGISTER_DATA, true);
    i2c_master_write(i2c_cmd, data, data_len, true);
    i2c_master_stop(i2c_cmd);    
    i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_cmd, pdMS_TO_TICKS(1));
    i2c_cmd_link_delete(i2c_cmd);
}

void drv_enable_digital() {    
    ESP_ERROR_CHECK(drv_write_register(DRV_REGISTER_1, INPUT_DIGITAL | GAIN_100V));    
    ESP_ERROR_CHECK(drv_write_register(DRV_REGISTER_2,  _STANDBY_FALSE | ENABLE_AUTO));    
    vTaskDelay(2*portTICK_PERIOD_MS);
}


void drv_enable_analog() {    
    ESP_ERROR_CHECK(drv_write_register(DRV_REGISTER_1, INPUT_ANALOG | GAIN_100V));    
    ESP_ERROR_CHECK(drv_write_register(DRV_REGISTER_2,  _STANDBY_FALSE | ENABLE_OVERRIDE));
}
