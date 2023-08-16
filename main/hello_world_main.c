/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include <math.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SCL_PIN   23
#define I2C_SDA_PIN   24

#define DRV2665_ADDR    0x59
#define DRV_REGISTER_0  0x00
#define DRV_REGISTER_1  0x01
#define DRV_REGISTER_2  0x02
#define DRV_RESET       1 << 7

#define FIFO_BUFFER_SIZE 100
#define SAMPLE_RATE 8000 // 8 kHz


size_t sine_wave_size;
uint8_t sine_wave[8000]; // Pre-calculated sine wave for one period


void generate_sine_wave(float frequency) {
    float phase = -M_PI / 2; // Shift phase by 90 degrees to start at minimum value
    float sine_wave_step = 2 * M_PI / sine_wave_size;

    for (size_t i = 0; i < sine_wave_size; i++) {
        float value = sin(phase);
        int twos_comp_value = (int)(value * 127);
        sine_wave[i] = (uint8_t)(twos_comp_value + 0x80);

        phase += sine_wave_step;
        if (phase >= 2 * M_PI) {
            phase -= 2 * M_PI;
        }
    }
}


void writer_task(void *pvParameters) {
    size_t sine_wave_index = 0;

    while (1) {
        size_t write_size = 50;
        uint8_t buffer[write_size];
        for (size_t i = 0; i < write_size; i++) {
            buffer[i] = sine_wave[sine_wave_index];
            sine_wave_index = (sine_wave_index + 1) % sine_wave_size;
        }

        // Calculate write delay ticks based on write_size and sample rate
        TickType_t write_delay_ticks = ((write_size-1) * portTICK_PERIOD_MS * 1000) / SAMPLE_RATE;
        vTaskDelay(write_delay_ticks);          
    }
}

static void drv_init_task(void *arg) {
    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = I2C_SDA_PIN;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num = I2C_SCL_PIN;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = 100000; // 100 kHz

    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (DRV2665_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, DRV_REGISTER_2, true);
    i2c_master_write_byte(i2c_cmd, DRV_RESET, true);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, i2c_cmd, pdMS_TO_TICKS(1000));
    vTaskDelay(5 / portTICK_PERIOD_MS);
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());


    float frequency = 250.0; // Desired frequency in Hz
    sine_wave_size = (size_t)((SAMPLE_RATE / frequency) + 0.5); // Rounded up

    generate_sine_wave(frequency);

    
    xTaskCreate(drv_init_task, "drv_init_task", 4096, NULL, 10, NULL);
    

}
